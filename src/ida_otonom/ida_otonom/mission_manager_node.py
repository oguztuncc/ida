import json
from typing import Any, Dict, List, Optional, Tuple

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int32, Bool

from .common import default_mission_file, resolve_mission_file, to_json


class MissionManagerNode(Node):
    def __init__(self) -> None:
        super().__init__("mission_manager_node")

        # Tek mission dosyası veya çoklu mission dosyaları desteği
        self.declare_parameter("mission_file", default_mission_file())
        self.declare_parameter("mission_files", "")  # Virgülle ayrılmış mission dosyaları
        self.declare_parameter("auto_start", False)
        self.declare_parameter("enable_multi_mission", False)  # Multi-mission modu
        self.declare_parameter("transition_delay_s", 2.0)  # Missionlar arası bekleme
        self.declare_parameter("disable_waypoint_completion", False)
        self.declare_parameter("complete_on_parkur3", False)
        self.declare_parameter("allow_runtime_mission_load", True)
        self.declare_parameter("allow_mission_reload_while_started", False)
        self.declare_parameter("require_runtime_mission_load", False)
        self.declare_parameter("min_runtime_waypoints", 1)
        self.declare_parameter("max_runtime_waypoints", 100)

        self.enable_multi_mission = bool(
            self.get_parameter("enable_multi_mission").value
        )
        self.transition_delay_s = float(
            self.get_parameter("transition_delay_s").value
        )
        self.disable_waypoint_completion = bool(
            self.get_parameter("disable_waypoint_completion").value
        )
        self.complete_on_parkur3 = bool(
            self.get_parameter("complete_on_parkur3").value
        )
        self.allow_runtime_mission_load = bool(
            self.get_parameter("allow_runtime_mission_load").value
        )
        self.allow_mission_reload_while_started = bool(
            self.get_parameter("allow_mission_reload_while_started").value
        )
        self.require_runtime_mission_load = bool(
            self.get_parameter("require_runtime_mission_load").value
        )
        self.min_runtime_waypoints = int(
            self.get_parameter("min_runtime_waypoints").value
        )
        self.max_runtime_waypoints = int(
            self.get_parameter("max_runtime_waypoints").value
        )

        # Mission dosyalarını hazırla
        self.mission_files = []
        if self.enable_multi_mission:
            # Çoklu mission modu aktif - virgülle ayrılmış string'i parse et
            mission_files_str = str(self.get_parameter("mission_files").value)
            if mission_files_str:
                # Virgülle ayır ve boşlukları temizle
                self.mission_files = [f.strip() for f in mission_files_str.split(",") if f.strip()]

            if not self.mission_files:
                # Eğer mission_files boşsa, mission_file parametresini kullan
                single_file = str(self.get_parameter("mission_file").value)
                if single_file:
                    self.mission_files = [single_file]
        else:
            # Tek mission modu
            single_file = str(self.get_parameter("mission_file").value)
            if single_file:
                self.mission_files = [single_file]

        self.auto_start = bool(self.get_parameter("auto_start").value)
        self.current_mission_index = 0
        self.active_waypoint_index = 0
        self.mission_loaded = False
        self.mission_started = False
        self.mission_completed = False
        self.all_missions_completed = False
        self.waypoints = []
        self.transition_timer = None

        # Publishers
        self.active_wp_pub = self.create_publisher(
            Int32,
            "/mission/active_waypoint",
            10,
        )
        self.status_pub = self.create_publisher(String, "/mission/status", 10)
        self.load_status_pub = self.create_publisher(
            String,
            "/mission/load_status",
            10,
        )
        self.started_pub = self.create_publisher(Bool, "/mission/started", 10)
        self.done_pub = self.create_publisher(Bool, "/mission/completed", 10)
        self.all_done_pub = self.create_publisher(
            Bool, "/mission/all_completed", 10
        )
        self.waypoints_pub = self.create_publisher(
            String,
            "/mission/waypoints",
            10,
        )
        self.current_mission_pub = self.create_publisher(
            Int32,
            "/mission/current_mission_index",
            10,
        )

        # Waypoint değişikliği bildirimi
        self.waypoints_changed_pub = self.create_publisher(
            Bool,
            "/mission/waypoints_changed",
            10,
        )
        self._previous_waypoints_hash = None

        # Subscribers
        self.create_subscription(
            Int32,
            "/guidance/advance_waypoint",
            self.advance_cb,
            10,
        )
        self.create_subscription(Bool, "/mission/start", self.start_cb, 10)
        self.create_subscription(
            String,
            "/mission/load",
            self.load_runtime_mission_cb,
            10,
        )
        self.create_subscription(
            Bool,
            "/parkur3/complete",
            self.parkur3_complete_cb,
            10,
        )

        self.timer = self.create_timer(1.0, self.loop)

        # İlk mission'ı yükle
        if self.require_runtime_mission_load:
            self.get_logger().info(
                "Waiting for runtime mission load from YKI"
            )
        else:
            self.load_current_mission()

    def load_current_mission(self) -> bool:
        """Mevcut mission index'indeki mission'ı yükler."""
        if self.current_mission_index >= len(self.mission_files):
            self.all_missions_completed = True
            self.get_logger().info("All missions completed!")
            return False

        mission_file = self.mission_files[self.current_mission_index]
        path = resolve_mission_file(mission_file)

        if not path.exists():
            self.get_logger().error(f"Mission file not found: {path}")
            return False

        try:
            data = json.loads(path.read_text(encoding="utf-8"))
            waypoints = self._validate_waypoints(
                data.get("waypoints", []),
                min_count=1,
                max_count=max(self.max_runtime_waypoints, 1),
            )
            self.waypoints = waypoints

            self.mission_loaded = True
            self.mission_started = self.auto_start
            self.mission_completed = False
            self.active_waypoint_index = 0

            mission_name = data.get("mission_name", f"mission_{self.current_mission_index}")
            self.get_logger().info(
                f"Loaded mission {self.current_mission_index + 1}/{len(self.mission_files)}: "
                f"{mission_name} ({len(self.waypoints)} waypoint(s)) from {path}"
            )

            if self.mission_started:
                self.get_logger().info("Mission auto-started")

            return True

        except Exception as exc:
            self.get_logger().error(f"Mission load failed: {exc}")
            return False

    def load_runtime_mission_cb(self, msg: String) -> None:
        if not self.allow_runtime_mission_load:
            self._publish_load_status(False, "runtime mission load disabled")
            return

        if self.mission_started and not self.allow_mission_reload_while_started:
            self._publish_load_status(
                False,
                "mission load rejected after mission start",
            )
            return

        try:
            data = json.loads(msg.data)
            waypoints = self._validate_waypoints(
                data.get("waypoints", []),
                min_count=self.min_runtime_waypoints,
                max_count=self.max_runtime_waypoints,
            )
        except Exception as exc:
            self._publish_load_status(False, f"invalid mission payload: {exc}")
            return

        self._apply_runtime_mission(data, waypoints)

    def _validate_waypoints(
        self,
        raw_waypoints: Any,
        *,
        min_count: int,
        max_count: int,
    ) -> List[Dict[str, Any]]:
        if not isinstance(raw_waypoints, list):
            raise ValueError("waypoints must be a list")
        if len(raw_waypoints) < min_count:
            raise ValueError(
                f"expected at least {min_count} waypoint(s), got "
                f"{len(raw_waypoints)}"
            )
        if len(raw_waypoints) > max_count:
            raise ValueError(
                f"expected at most {max_count} waypoint(s), got "
                f"{len(raw_waypoints)}"
            )

        waypoints = []
        for index, raw_wp in enumerate(raw_waypoints):
            if not isinstance(raw_wp, dict):
                raise ValueError(f"waypoint {index} must be an object")
            lat, lon = self._parse_lat_lon(raw_wp, index)
            wp = dict(raw_wp)
            wp["lat"] = lat
            wp["lon"] = lon
            waypoints.append(wp)

        return waypoints

    def _parse_lat_lon(
        self,
        waypoint: Dict[str, Any],
        index: int,
    ) -> Tuple[float, float]:
        if "lat" not in waypoint or "lon" not in waypoint:
            raise ValueError(f"waypoint {index} missing lat/lon")

        lat = float(waypoint["lat"])
        lon = float(waypoint["lon"])
        if not -90.0 <= lat <= 90.0:
            raise ValueError(f"waypoint {index} latitude out of range")
        if not -180.0 <= lon <= 180.0:
            raise ValueError(f"waypoint {index} longitude out of range")

        return lat, lon

    def _apply_runtime_mission(
        self,
        data: Dict[str, Any],
        waypoints: List[Dict[str, Any]],
    ) -> None:
        if self.transition_timer:
            self.transition_timer.cancel()
            self.transition_timer = None

        self.enable_multi_mission = False
        self.mission_files = []
        self.current_mission_index = 0
        self.active_waypoint_index = 0
        self.mission_loaded = True
        self.mission_started = False
        self.mission_completed = False
        self.all_missions_completed = False
        self.waypoints = waypoints
        self._previous_waypoints_hash = None

        self.waypoints_pub.publish(
            String(data=to_json({"waypoints": self.waypoints}))
        )
        self.waypoints_changed_pub.publish(Bool(data=True))
        self._previous_waypoints_hash = self._get_waypoints_hash()

        source = str(data.get("source", "runtime"))
        mission_name = str(data.get("mission_name", "runtime_mission"))
        self.get_logger().info(
            f"Runtime mission loaded from {source}: {mission_name} "
            f"({len(self.waypoints)} waypoint(s))"
        )
        self._publish_load_status(
            True,
            "mission loaded",
            {
                "source": source,
                "mission_name": mission_name,
                "waypoint_count": len(self.waypoints),
            },
        )

    def start_next_mission(self) -> None:
        """Bir sonraki mission'a geçer."""
        self.current_mission_index += 1
        if self.current_mission_index < len(self.mission_files):
            self.get_logger().info(
                f"Transitioning to mission {self.current_mission_index + 1}/"
                f"{len(self.mission_files)} "
                f"in {self.transition_delay_s}s..."
            )
            # Transition timer oluştur
            if self.transition_timer:
                self.transition_timer.cancel()
            self.transition_timer = self.create_timer(
                self.transition_delay_s,
                self._transition_callback,
            )
        else:
            self.all_missions_completed = True
            self.get_logger().info("All missions completed! No more missions.")

    def _transition_callback(self) -> None:
        """Transition timer callback - yeni mission'ı yükler."""
        if self.transition_timer:
            self.transition_timer.cancel()
            self.transition_timer = None

        # Önceki mission'ın durumlarını sıfırla
        self.mission_completed = False
        self.mission_started = False
        self.active_waypoint_index = 0
        self._previous_waypoints_hash = None  # Hash'i sıfırla

        # Yeni mission'ı yükle
        success = self.load_current_mission()

        if success and self.auto_start:
            self.mission_started = True
            self.get_logger().info("Next mission auto-started")

        # Yeni waypoints'i hemen publish et
        if self.waypoints:
            self.waypoints_pub.publish(
                String(data=to_json({"waypoints": self.waypoints}))
            )
            self._previous_waypoints_hash = self._get_waypoints_hash()

    def start_cb(self, msg: Bool) -> None:
        if not msg.data:
            return
        if not self.mission_loaded:
            self.get_logger().warn(
                "Mission start ignored: mission is not loaded"
            )
            return
        if self.mission_completed:
            self.get_logger().warn(
                "Mission start ignored: mission is already completed"
            )
            return
        if not self.mission_started:
            self.mission_started = True
            self.get_logger().info("Mission started")

    def parkur3_complete_cb(self, msg: Bool) -> None:
        if not self.complete_on_parkur3 or not msg.data:
            return
        if not self.mission_loaded or self.mission_completed:
            return
        if not self.mission_started:
            return
        self.mission_completed = True
        self.mission_started = False
        self.get_logger().info("Mission completed by Parkur 3 planner")

    def advance_cb(self, msg: Int32) -> None:
        if (
            not self.mission_loaded
            or not self.mission_started
            or self.mission_completed
            or self.disable_waypoint_completion
        ):
            return

        reached_index = int(msg.data)
        if reached_index != self.active_waypoint_index:
            return

        if self.active_waypoint_index >= len(self.waypoints) - 1:
            self.mission_completed = True
            self.mission_started = False
            self.get_logger().info(
                f"Mission {self.current_mission_index + 1} completed"
            )

            # Multi-mission modunda bir sonraki mission'a geç
            if self.enable_multi_mission:
                self.start_next_mission()
            return

        self.active_waypoint_index += 1

    def _get_waypoints_hash(self) -> str:
        """Waypointlerin hash'ini hesapla."""
        if not self.waypoints:
            return ""
        waypoints_str = json.dumps(self.waypoints, sort_keys=True)
        return str(hash(waypoints_str))

    def _publish_load_status(
        self,
        accepted: bool,
        reason: str,
        extra: Optional[Dict[str, Any]] = None,
    ) -> None:
        payload = {
            "accepted": bool(accepted),
            "reason": reason,
        }
        if extra:
            payload.update(extra)
        self.load_status_pub.publish(String(data=to_json(payload)))
        if accepted:
            self.get_logger().info(f"Mission load accepted: {reason}")
        else:
            self.get_logger().warn(f"Mission load rejected: {reason}")

    def loop(self) -> None:
        # Waypointleri publish et (değişiklik varsa)
        current_hash = self._get_waypoints_hash()
        if self.waypoints and current_hash != self._previous_waypoints_hash:
            self.waypoints_pub.publish(
                String(data=to_json({"waypoints": self.waypoints}))
            )
            self._previous_waypoints_hash = current_hash
            self.waypoints_changed_pub.publish(Bool(data=True))

        self.active_wp_pub.publish(Int32(data=self.active_waypoint_index))
        self.started_pub.publish(Bool(data=self.mission_started))
        self.done_pub.publish(Bool(data=self.mission_completed))
        self.all_done_pub.publish(Bool(data=self.all_missions_completed))
        self.current_mission_pub.publish(
            Int32(data=self.current_mission_index)
        )
        self.status_pub.publish(
            String(
                data=to_json(
                    {
                        "mission_loaded": self.mission_loaded,
                        "mission_started": self.mission_started,
                        "mission_completed": self.mission_completed,
                        "all_missions_completed": self.all_missions_completed,
                        "active_waypoint_index": self.active_waypoint_index,
                        "waypoint_count": len(self.waypoints),
                        "current_mission_index": self.current_mission_index,
                        "total_mission_count": len(self.mission_files),
                        "runtime_mission_load_enabled": (
                            self.allow_runtime_mission_load
                        ),
                        "runtime_mission_load_required": (
                            self.require_runtime_mission_load
                        ),
                    }
                )
            )
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MissionManagerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
