import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float32, String

from .common import clamp, from_json, normalize_angle_deg, now_ts, to_json


class Parkur3PlannerNode(Node):
    """
    Parkur 3 renkli duba dokunma planner.

    Hedef renkteki dubaya yönelir ve dokunur.
    Modlar:
    - WAIT_COLOR: IHA'dan renk bekle (veya manuel /parkur3/command 'start')
    - SEARCH: Hedef renkteki dubayı arar (dönerek)
    - APPROACH: Hedefe yaklaşır
    - TOUCH: Dokunma manevrası yapar
    - COMPLETE: Görev tamamlandı
    """

    def __init__(self) -> None:
        super().__init__("parkur3_planner_node")

        # Parametreler
        self.declare_parameter("max_speed_mps", 0.30)
        self.declare_parameter("approach_speed_mps", 0.15)
        self.declare_parameter("touch_speed_mps", 0.08)
        self.declare_parameter("search_turn_speed_radps", 0.4)
        self.declare_parameter("max_relative_bearing_deg", 45.0)
        self.declare_parameter("touch_distance_m", 0.5)  # Dokunma mesafesi
        self.declare_parameter("approach_distance_m", 3.0)  # Yaklaşma mesafesi
        self.declare_parameter("search_timeout_s", 60.0)
        self.declare_parameter("approach_timeout_s", 30.0)
        self.declare_parameter("touch_timeout_s", 10.0)
        self.declare_parameter("sensor_timeout_s", 2.0)
        self.declare_parameter("bearing_rate_limit_degps", 60.0)
        self.declare_parameter("require_color_first", True)
        self.declare_parameter("sim_contact_stop_enabled", True)
        self.declare_parameter("contact_clearance_m", 0.05)

        self.max_speed_mps = float(self.get_parameter("max_speed_mps").value)
        self.approach_speed_mps = float(
            self.get_parameter("approach_speed_mps").value
        )
        self.touch_speed_mps = float(
            self.get_parameter("touch_speed_mps").value
        )
        self.search_turn_speed_radps = float(
            self.get_parameter("search_turn_speed_radps").value
        )
        self.max_relative_bearing_deg = float(
            self.get_parameter("max_relative_bearing_deg").value
        )
        self.touch_distance_m = float(
            self.get_parameter("touch_distance_m").value
        )
        self.approach_distance_m = float(
            self.get_parameter("approach_distance_m").value
        )
        self.search_timeout_s = float(
            self.get_parameter("search_timeout_s").value
        )
        self.approach_timeout_s = float(
            self.get_parameter("approach_timeout_s").value
        )
        self.touch_timeout_s = float(
            self.get_parameter("touch_timeout_s").value
        )
        self.sensor_timeout_s = float(
            self.get_parameter("sensor_timeout_s").value
        )
        self.bearing_rate_limit_degps = float(
            self.get_parameter("bearing_rate_limit_degps").value
        )
        self.require_color_first = bool(
            self.get_parameter("require_color_first").value
        )
        self.sim_contact_stop_enabled = bool(
            self.get_parameter("sim_contact_stop_enabled").value
        )
        self.contact_clearance_m = float(
            self.get_parameter("contact_clearance_m").value
        )

        # Durum
        self.mode = "WAIT_COLOR"  # WAIT_COLOR, SEARCH, APPROACH, TOUCH, COMPLETE
        self.target_color = ""
        self.target_buoy = None
        self.buoy_ts = 0.0
        self.mode_started_ts = 0.0
        self.heading_deg = None
        self.mission_started = False
        self.mission_completed = False
        self.last_target_buoy = None
        self.sim_target_contact = False
        self.sim_target_clearance_m = None

        self.last_relative_bearing = 0.0
        self.last_plan_ts = 0.0
        self.search_start_heading = None
        self.touch_attempted = False

        # Publishers
        self.safe_bearing_pub = self.create_publisher(
            Float32,
            "/planner/safe_bearing_deg",
            10,
        )
        self.speed_limit_pub = self.create_publisher(
            Float32,
            "/planner/speed_limit_mps",
            10,
        )
        self.status_pub = self.create_publisher(
            String,
            "/planner/status",
            10,
        )
        self.complete_pub = self.create_publisher(
            Bool,
            "/parkur3/complete",
            10,
        )

        # Subscribers
        self.create_subscription(
            Float32,
            "/mavros/global_position/compass_hdg",
            self.heading_cb,
            10,
        )
        self.create_subscription(
            String,
            "/parkur3/target_color",
            self.color_cb,
            10,
        )
        self.create_subscription(
            String,
            "/parkur3/target_buoy",
            self.target_cb,
            10,
        )
        self.create_subscription(
            Bool,
            "/mission/started",
            self.mission_started_cb,
            10,
        )
        self.create_subscription(
            String,
            "/parkur3/command",
            self.command_cb,
            10,
        )
        self.create_subscription(
            String,
            "/sim/world",
            self.sim_world_cb,
            10,
        )

        self.timer = self.create_timer(0.1, self.loop)

        self.get_logger().info(
            f"Parkur3PlannerNode started. Mode: {self.mode}"
        )

    def heading_cb(self, msg: Float32) -> None:
        self.heading_deg = float(msg.data)

    def color_cb(self, msg: String) -> None:
        new_color = msg.data.strip().lower()
        if new_color and new_color != self.target_color:
            self.target_color = new_color
            self.get_logger().info(f"Target color: {self.target_color}")
            if self.mode == "WAIT_COLOR":
                self._set_mode("SEARCH")

    def _force_search(self) -> None:
        """Manuel olarak SEARCH moduna geç."""
        if self.require_color_first and not self.target_color:
            self.get_logger().warn("SEARCH ignored: target color is not set")
            return
        if self.mode == "WAIT_COLOR":
            self.get_logger().info("Force SEARCH mode")
            self._set_mode("SEARCH")

    def target_cb(self, msg: String) -> None:
        import json
        try:
            data = json.loads(msg.data)
            if data.get("found"):
                self.target_buoy = {
                    "range_m": float(data["range_m"]),
                    "bearing_deg": float(data["bearing_deg"]),
                    "confidence": float(data.get("confidence", 0.0)),
                }
                self.last_target_buoy = self.target_buoy
                self.buoy_ts = now_ts()
            else:
                if now_ts() - self.buoy_ts > self.sensor_timeout_s:
                    self.target_buoy = None
        except Exception:
            self.target_buoy = None

    def _nav_relative_from_body_left(self, body_left_deg: float) -> float:
        return -body_left_deg

    def mission_started_cb(self, msg: Bool) -> None:
        self.mission_started = bool(msg.data)

    def command_cb(self, msg: String) -> None:
        """Manuel komut: /parkur3/command 'start' ile SEARCH başlat."""
        cmd = msg.data.strip().lower()
        if cmd == "start" and self.mode == "WAIT_COLOR":
            if self.require_color_first and not self.target_color:
                self.get_logger().warn("Command start ignored: no target color")
                return
            self.get_logger().info("Command: start -> SEARCH")
            self._set_mode("SEARCH")
        elif cmd == "reset":
            self.get_logger().info("Command: reset -> WAIT_COLOR")
            self._set_mode("WAIT_COLOR")
            self.mission_completed = False
            self.touch_attempted = False
            self.target_buoy = None
            self.last_target_buoy = None
            self.sim_target_contact = False

    def sim_world_cb(self, msg: String) -> None:
        if not self.sim_contact_stop_enabled or not self.target_color:
            return
        try:
            world = from_json(msg.data)
        except Exception:
            return

        boat = world.get("boat", {})
        closest_id = boat.get("closest_object_id")
        clearance = boat.get("closest_clearance_m")
        if closest_id is None or clearance is None:
            self.sim_target_contact = False
            return

        target_class = f"{self.target_color}_buoy"
        closest_object = None
        for obj in world.get("objects", []):
            if obj.get("id") == closest_id:
                closest_object = obj
                break

        if closest_object is None:
            self.sim_target_contact = False
            return

        is_target = closest_object.get("class_name") == target_class
        self.sim_target_clearance_m = float(clearance)
        self.sim_target_contact = (
            is_target and self.sim_target_clearance_m <= self.contact_clearance_m
        )

    def _set_mode(self, new_mode: str) -> None:
        if new_mode != self.mode:
            self.get_logger().info(f"Mode: {self.mode} -> {new_mode}")
            self.mode = new_mode
            self.mode_started_ts = now_ts()
            if new_mode == "SEARCH":
                self.search_start_heading = self.heading_deg

    def _rate_limit_relative(self, relative_bearing: float) -> float:
        """Bearing değişim hızını sınırla."""
        now = now_ts()
        if self.last_plan_ts <= 0.0:
            self.last_relative_bearing = relative_bearing
            self.last_plan_ts = now
            return relative_bearing

        dt = max(now - self.last_plan_ts, 1e-3)
        max_step = self.bearing_rate_limit_degps * dt
        delta = normalize_angle_deg(relative_bearing - self.last_relative_bearing)
        limited = self.last_relative_bearing + clamp(delta, -max_step, max_step)
        limited = clamp(
            limited,
            -self.max_relative_bearing_deg,
            self.max_relative_bearing_deg,
        )
        self.last_relative_bearing = limited
        self.last_plan_ts = now
        return limited

    def _absolute_from_relative(self, relative_deg: float) -> float:
        if self.heading_deg is None:
            return 0.0
        return (self.heading_deg + relative_deg) % 360.0

    def _publish_plan(
        self,
        mode: str,
        relative_bearing: float,
        speed_limit: float,
        reason: str,
    ) -> None:
        relative_bearing = clamp(
            relative_bearing,
            -self.max_relative_bearing_deg,
            self.max_relative_bearing_deg,
        )
        if mode == "COMPLETE":
            relative_bearing = 0.0
            speed_limit = 0.0
        else:
            relative_bearing = self._rate_limit_relative(relative_bearing)

        safe_bearing = self._absolute_from_relative(relative_bearing)
        self.safe_bearing_pub.publish(Float32(data=float(safe_bearing)))
        self.speed_limit_pub.publish(Float32(data=float(max(0.0, speed_limit))))

        payload = {
            "timestamp": now_ts(),
            "mode": mode,
            "safe_bearing_deg": safe_bearing,
            "relative_bearing_deg": relative_bearing,
            "speed_limit_mps": speed_limit,
            "target_color": self.target_color,
            "target_range_m": (
                self.target_buoy["range_m"] if self.target_buoy else None
            ),
            "target_bearing_deg": (
                self.target_buoy["bearing_deg"] if self.target_buoy else None
            ),
            "reason": reason,
        }
        self.status_pub.publish(String(data=to_json(payload)))

    def _complete_touch(self, reason: str) -> None:
        self.touch_attempted = True
        self.mission_completed = True
        self._set_mode("COMPLETE")
        self._publish_plan("COMPLETE", 0.0, 0.0, reason)
        self.complete_pub.publish(Bool(data=True))

    def loop(self) -> None:
        if self.mission_completed:
            self._publish_plan("COMPLETE", 0.0, 0.0, "mission_completed")
            self.complete_pub.publish(Bool(data=True))
            return

        if self.heading_deg is None:
            self._publish_plan("WAIT_COLOR", 0.0, 0.0, "waiting_heading")
            return

        if self.sim_target_contact:
            self._complete_touch("sim_target_contact")
            return

        now = now_ts()
        mode_age = now - self.mode_started_ts

        # === WAIT_COLOR ===
        if self.mode == "WAIT_COLOR":
            # Auto-start if color already set
            if self.target_color:
                self._set_mode("SEARCH")
            else:
                self._publish_plan("WAIT_COLOR", 0.0, 0.0, "waiting_for_color")
                return

        # === SEARCH ===
        if self.mode == "SEARCH":
            if self.require_color_first and not self.target_color:
                self._set_mode("WAIT_COLOR")
                self._publish_plan("WAIT_COLOR", 0.0, 0.0, "waiting_for_color")
                return

            # Timeout kontrolü
            if mode_age > self.search_timeout_s:
                self.get_logger().error("SEARCH timeout!")
                self._set_mode("COMPLETE")
                return

            # Hedef bulundu mu?
            if self.target_buoy is not None:
                self.get_logger().info(
                    f"Target found! Range: {self.target_buoy['range_m']:.1f}m, "
                    f"Bearing: {self.target_buoy['bearing_deg']:.1f}°"
                )
                self._set_mode("APPROACH")
                return

            # Arama: Yavaşça ilerleyerek dönerek etrafa bak
            # bearing varsa ona göre dön, yoksa sabit açıda dön
            if self.target_buoy:
                relative = clamp(
                    self._nav_relative_from_body_left(
                        self.target_buoy['bearing_deg']
                    ),
                    -self.max_relative_bearing_deg,
                    self.max_relative_bearing_deg,
                )
            else:
                relative = 30.0
            speed = 0.15  # Yavaşça ilerle ki etrafta dönebilsin
            self._publish_plan("SEARCH", relative, speed, "searching_target")
            return

        # === APPROACH ===
        if self.mode == "APPROACH":
            # Timeout kontrolü (daha uzun)
            if mode_age > self.approach_timeout_s:
                self.get_logger().error("APPROACH timeout!")
                self._set_mode("SEARCH")
                return

            # Hedef kayboldu mu?
            if self.target_buoy is None:
                if now - self.buoy_ts > self.sensor_timeout_s:
                    self.get_logger().warn("Target lost, returning to SEARCH")
                    self._set_mode("SEARCH")
                    return

            # Hedef varsa yaklaş
            if self.target_buoy:
                range_m = self.target_buoy["range_m"]
                bearing_deg = self.target_buoy["bearing_deg"]

                # Dokunma mesafesine geldik mi?
                if range_m <= self.touch_distance_m:
                    self.get_logger().info("Touch distance reached!")
                    self._complete_touch("touch_distance_reached")
                    return

                # Yaklaşma hızı - her zaman ilerle
                speed = self.approach_speed_mps

                # Bearing hedefe göre
                relative = clamp(
                    self._nav_relative_from_body_left(bearing_deg),
                    -self.max_relative_bearing_deg,
                    self.max_relative_bearing_deg,
                )

                self._publish_plan(
                    "APPROACH",
                    relative,
                    speed,
                    f"approaching_target_range_{range_m:.1f}",
                )
                return

            # Hedef yoksa bekle
            if self.last_target_buoy is not None:
                relative = clamp(
                    self._nav_relative_from_body_left(
                        self.last_target_buoy["bearing_deg"]
                    ),
                    -self.max_relative_bearing_deg,
                    self.max_relative_bearing_deg,
                )
                self._publish_plan(
                    "APPROACH",
                    relative,
                    min(self.touch_speed_mps, self.approach_speed_mps),
                    "coasting_last_target",
                )
                return
            self._publish_plan("APPROACH", 0.0, 0.0, "waiting_for_target")
            return

        # === TOUCH ===
        if self.mode == "TOUCH":
            self._complete_touch("touch_mode_stop")
            return

        # === COMPLETE ===
        if self.mode == "COMPLETE":
            self.mission_completed = True
            self._publish_plan("COMPLETE", 0.0, 0.0, "parkur3_complete")
            self.complete_pub.publish(Bool(data=True))
            self.get_logger().info("PARKUR 3 COMPLETE!")
            return


def main(args=None) -> None:
    rclpy.init(args=args)
    node = Parkur3PlannerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
