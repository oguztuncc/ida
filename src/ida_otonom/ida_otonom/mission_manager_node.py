import json

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
            Bool,
            "/parkur3/complete",
            self.parkur3_complete_cb,
            10,
        )

        self.timer = self.create_timer(1.0, self.loop)

        # İlk mission'ı yükle
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
            self.waypoints = data.get("waypoints", [])
            if not self.waypoints:
                raise ValueError("No waypoints found in mission file")

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
