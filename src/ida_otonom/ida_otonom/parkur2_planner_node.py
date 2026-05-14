import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, String

from .common import clamp, from_json, normalize_angle_deg, now_ts, to_json


class Parkur2PlannerNode(Node):
    def __init__(self) -> None:
        super().__init__("parkur2_planner_node")

        self.declare_parameter("max_speed_mps", 0.25)
        self.declare_parameter("approach_speed_mps", 0.15)
        self.declare_parameter("danger_distance_m", 0.60)
        self.declare_parameter("avoid_start_distance_m", 1.20)
        self.declare_parameter("safe_clearance_m", 1.50)
        self.declare_parameter("max_relative_bearing_deg", 55.0)
        self.declare_parameter("corridor_min_confidence", 0.55)
        self.declare_parameter("corridor_keepout_margin_m", 1.10)
        self.declare_parameter("min_corridor_return_angle_deg", 25.0)
        self.declare_parameter("sensor_timeout_s", 1.0)

        self.max_speed_mps = float(self.get_parameter("max_speed_mps").value)
        self.approach_speed_mps = float(
            self.get_parameter("approach_speed_mps").value
        )
        self.danger_distance_m = float(
            self.get_parameter("danger_distance_m").value
        )
        self.avoid_start_distance_m = float(
            self.get_parameter("avoid_start_distance_m").value
        )
        self.safe_clearance_m = float(
            self.get_parameter("safe_clearance_m").value
        )
        self.max_relative_bearing_deg = float(
            self.get_parameter("max_relative_bearing_deg").value
        )
        self.corridor_min_confidence = float(
            self.get_parameter("corridor_min_confidence").value
        )
        self.corridor_keepout_margin_m = float(
            self.get_parameter("corridor_keepout_margin_m").value
        )
        self.min_corridor_return_angle_deg = float(
            self.get_parameter("min_corridor_return_angle_deg").value
        )
        self.sensor_timeout_s = float(
            self.get_parameter("sensor_timeout_s").value
        )

        self.heading_deg = None
        self.target_bearing_deg = None
        self.lidar_summary = None
        self.corridor = None
        self.semantic = None
        self.lidar_ts = 0.0
        self.corridor_ts = 0.0
        self.semantic_ts = 0.0

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

        self.create_subscription(
            Float32,
            "/mavros/global_position/compass_hdg",
            self.heading_cb,
            10,
        )
        self.create_subscription(
            Float32,
            "/guidance/target_bearing_deg",
            self.target_cb,
            10,
        )
        self.create_subscription(
            String,
            "/perception/lidar_summary",
            self.lidar_cb,
            10,
        )
        self.create_subscription(
            String,
            "/planner/corridor",
            self.corridor_cb,
            10,
        )
        self.create_subscription(
            String,
            "/perception/semantic_buoys",
            self.semantic_cb,
            10,
        )

        self.timer = self.create_timer(0.1, self.loop)

    def heading_cb(self, msg: Float32) -> None:
        self.heading_deg = float(msg.data)

    def target_cb(self, msg: Float32) -> None:
        self.target_bearing_deg = float(msg.data)

    def lidar_cb(self, msg: String) -> None:
        try:
            self.lidar_summary = from_json(msg.data)
            self.lidar_ts = now_ts()
        except Exception:
            self.lidar_summary = None

    def corridor_cb(self, msg: String) -> None:
        try:
            self.corridor = from_json(msg.data)
            self.corridor_ts = now_ts()
        except Exception:
            self.corridor = None

    def semantic_cb(self, msg: String) -> None:
        try:
            self.semantic = from_json(msg.data)
            self.semantic_ts = now_ts()
        except Exception:
            self.semantic = None

    def _absolute_from_relative(self, relative_deg: float) -> float:
        if self.heading_deg is None:
            return self.target_bearing_deg or 0.0
        return (self.heading_deg + relative_deg) % 360.0

    def _nav_relative_from_body_left(self, relative_deg: float) -> float:
        return -relative_deg

    def _semantic_front_obstacle(self) -> bool:
        if self.semantic is None:
            return False
        for buoy in self.semantic.get("buoys", []):
            if buoy.get("semantic") not in (
                "obstacle_candidate",
                "unknown",
            ):
                continue
            forward = buoy.get("forward_m")
            left = buoy.get("left_m")
            if forward is None or left is None:
                continue
            if 0.0 < float(forward) < self.avoid_start_distance_m:
                if abs(float(left)) < self.safe_clearance_m / 2.0:
                    return True
        return False

    def _lidar_state(self):
        now = now_ts()
        if self.lidar_summary is None:
            return "missing", None, 0.0
        if now - self.lidar_ts > self.sensor_timeout_s:
            return "timeout", None, 0.0

        front = float(self.lidar_summary.get("front_clearance_m", 999.0))
        best_free = float(self.lidar_summary.get("best_free_angle_deg", 0.0))
        if front < self.danger_distance_m:
            return "danger", front, best_free
        if front < self.avoid_start_distance_m:
            return "avoid", front, best_free
        return "clear", front, best_free

    def _corridor_state(self):
        if self.corridor is None:
            return None, 0.0, "no_corridor", None, None
        if now_ts() - self.corridor_ts > self.sensor_timeout_s:
            return None, 0.0, "corridor_timeout", None, None
        confidence = float(self.corridor.get("confidence", 0.0))
        if confidence < self.corridor_min_confidence:
            return None, confidence, "low_corridor_confidence", None, None
        bearing = float(self.corridor.get("center_bearing_deg", 0.0))
        bearing = clamp(
            bearing,
            -self.max_relative_bearing_deg,
            self.max_relative_bearing_deg,
        )
        center_left = self.corridor.get("center_left_m")
        estimated_width = self.corridor.get("estimated_width_m")
        center_left = None if center_left is None else float(center_left)
        estimated_width = (
            None if estimated_width is None else float(estimated_width)
        )
        return bearing, confidence, "corridor_track", center_left, estimated_width

    def _corridor_limited_body_bearing(
        self,
        body_bearing: float,
        corridor_body_bearing: float | None,
        center_left_m: float | None,
        estimated_width_m: float | None,
    ) -> tuple[float, str | None]:
        if (
            corridor_body_bearing is None
            or center_left_m is None
            or estimated_width_m is None
            or estimated_width_m <= 0.0
        ):
            return body_bearing, None

        half_width = estimated_width_m / 2.0
        left_clearance = center_left_m + half_width
        right_clearance = half_width - center_left_m
        margin = max(self.corridor_keepout_margin_m, self.safe_clearance_m / 2.0)

        if body_bearing < 0.0 and right_clearance < margin:
            return self._corridor_return_angle(corridor_body_bearing, 1.0), (
                "right_boundary_keepout"
            )
        if body_bearing > 0.0 and left_clearance < margin:
            return self._corridor_return_angle(corridor_body_bearing, -1.0), (
                "left_boundary_keepout"
            )
        return body_bearing, None

    def _corridor_return_angle(
        self,
        corridor_body_bearing: float,
        sign: float,
    ) -> float:
        angle = max(
            abs(corridor_body_bearing),
            self.min_corridor_return_angle_deg,
        )
        return clamp(
            sign * angle,
            -self.max_relative_bearing_deg,
            self.max_relative_bearing_deg,
        )

    def loop(self) -> None:
        if self.heading_deg is None or self.target_bearing_deg is None:
            return

        lidar_state, front_clearance, best_free = self._lidar_state()
        (
            corridor_body_bearing,
            corridor_confidence,
            corridor_reason,
            corridor_center_left,
            corridor_width,
        ) = self._corridor_state()
        corridor_bearing = None
        if corridor_body_bearing is not None:
            corridor_bearing = self._nav_relative_from_body_left(
                corridor_body_bearing
            )
        semantic_front_obstacle = self._semantic_front_obstacle()
        corridor_limit_reason = None

        mode = "WAYPOINT"
        relative_bearing = normalize_angle_deg(
            self.target_bearing_deg - self.heading_deg
        )
        speed_limit = self.max_speed_mps
        reason = "target_bearing"

        if lidar_state in ("missing", "timeout"):
            mode = "STOP"
            relative_bearing = 0.0
            speed_limit = 0.0
            reason = f"lidar_{lidar_state}"
        elif lidar_state == "danger":
            mode = "STOP"
            body_bearing, corridor_limit_reason = (
                self._corridor_limited_body_bearing(
                    best_free,
                    corridor_body_bearing,
                    corridor_center_left,
                    corridor_width,
                )
            )
            relative_bearing = clamp(
                self._nav_relative_from_body_left(body_bearing),
                -self.max_relative_bearing_deg,
                self.max_relative_bearing_deg,
            )
            speed_limit = 0.0
            reason = corridor_limit_reason or "front_danger_turn_in_place"
        elif lidar_state == "avoid" or semantic_front_obstacle:
            mode = "AVOID"
            body_bearing, corridor_limit_reason = (
                self._corridor_limited_body_bearing(
                    best_free,
                    corridor_body_bearing,
                    corridor_center_left,
                    corridor_width,
                )
            )
            relative_bearing = clamp(
                self._nav_relative_from_body_left(body_bearing),
                -self.max_relative_bearing_deg,
                self.max_relative_bearing_deg,
            )
            speed_limit = self.approach_speed_mps
            reason = corridor_limit_reason or (
                "local_obstacle_avoidance"
                if lidar_state == "avoid"
                else "semantic_front_obstacle"
            )
        elif corridor_bearing is not None:
            mode = "CORRIDOR_TRACK"
            relative_bearing = corridor_bearing
            speed_limit = self.max_speed_mps
            reason = corridor_reason

        safe_bearing = self._absolute_from_relative(relative_bearing)
        self.safe_bearing_pub.publish(Float32(data=float(safe_bearing)))
        self.speed_limit_pub.publish(Float32(data=float(speed_limit)))
        self.status_pub.publish(
            String(
                data=to_json(
                    {
                        "timestamp": now_ts(),
                        "mode": mode,
                        "safe_bearing_deg": safe_bearing,
                        "relative_bearing_deg": relative_bearing,
                        "speed_limit_mps": speed_limit,
                        "front_clearance_m": front_clearance,
                        "lidar_state": lidar_state,
                        "corridor_confidence": corridor_confidence,
                        "corridor_center_left_m": corridor_center_left,
                        "corridor_width_m": corridor_width,
                        "corridor_limit_reason": corridor_limit_reason,
                        "reason": reason,
                    }
                )
            )
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = Parkur2PlannerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
