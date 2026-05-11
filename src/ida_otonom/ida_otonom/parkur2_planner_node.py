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

    def _corridor_relative_bearing(self):
        if self.corridor is None:
            return None, 0.0, "no_corridor"
        if now_ts() - self.corridor_ts > self.sensor_timeout_s:
            return None, 0.0, "corridor_timeout"
        confidence = float(self.corridor.get("confidence", 0.0))
        if confidence < self.corridor_min_confidence:
            return None, confidence, "low_corridor_confidence"
        bearing = float(self.corridor.get("center_bearing_deg", 0.0))
        bearing = clamp(
            bearing,
            -self.max_relative_bearing_deg,
            self.max_relative_bearing_deg,
        )
        return bearing, confidence, "corridor_track"

    def loop(self) -> None:
        if self.heading_deg is None or self.target_bearing_deg is None:
            return

        lidar_state, front_clearance, best_free = self._lidar_state()
        corridor_bearing, corridor_confidence, corridor_reason = (
            self._corridor_relative_bearing()
        )
        semantic_front_obstacle = self._semantic_front_obstacle()

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
            relative_bearing = 0.0
            speed_limit = 0.0
            reason = "front_danger_distance"
        elif corridor_bearing is not None:
            mode = "CORRIDOR_TRACK"
            relative_bearing = corridor_bearing
            speed_limit = self.max_speed_mps
            reason = corridor_reason
        elif lidar_state == "avoid" or semantic_front_obstacle:
            mode = "AVOID"
            relative_bearing = clamp(
                best_free,
                -self.max_relative_bearing_deg,
                self.max_relative_bearing_deg,
            )
            speed_limit = self.approach_speed_mps
            reason = "local_obstacle_avoidance"

        if lidar_state == "avoid" and mode == "CORRIDOR_TRACK":
            speed_limit = min(speed_limit, self.approach_speed_mps)
            reason = "corridor_track_with_front_obstacle"

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
