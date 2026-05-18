from math import fabs

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float32, String
from geometry_msgs.msg import Twist

from .common import normalize_angle_deg, clamp, from_json, to_json


class ControllerNode(Node):
    def __init__(self) -> None:
        super().__init__("controller_node")

        self.declare_parameter("kp_heading", 0.02)
        self.declare_parameter("max_linear_speed", 0.45)
        self.declare_parameter("max_angular_speed", 0.8)
        self.declare_parameter("use_planner_bearing", False)
        self.declare_parameter("planner_timeout_s", 0.5)
        self.declare_parameter("allow_reverse_planner_speed", False)
        self.declare_parameter("geofence_timeout_s", 1.0)

        self.kp_heading = float(self.get_parameter("kp_heading").value)
        self.max_linear_speed = float(
            self.get_parameter("max_linear_speed").value
        )
        self.max_angular_speed = float(
            self.get_parameter("max_angular_speed").value
        )
        self.use_planner_bearing = bool(
            self.get_parameter("use_planner_bearing").value
        )
        self.planner_timeout_s = float(
            self.get_parameter("planner_timeout_s").value
        )
        self.allow_reverse_planner_speed = bool(
            self.get_parameter("allow_reverse_planner_speed").value
        )
        self.geofence_timeout_s = float(
            self.get_parameter("geofence_timeout_s").value
        )

        self.current_heading = None
        self.target_bearing = None
        self.target_distance = None
        self.geofence_status = None
        self.geofence_ts = 0.0
        self.planner_bearing = None
        self.planner_speed_limit = None
        self.planner_bearing_ts = 0.0
        self.planner_speed_ts = 0.0
        self.vision_heading_bias = 0.0
        self.mission_started = False
        self.mission_completed = False

        self.cmd_pub = self.create_publisher(Twist, "/control/cmd_vel", 10)
        self.setpoint_pub = self.create_publisher(
            String,
            "/control/setpoints",
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
            self.target_bearing_cb,
            10,
        )
        self.create_subscription(
            Float32,
            "/guidance/target_distance_m",
            self.target_distance_cb,
            10,
        )
        self.create_subscription(
            Float32,
            "/planner/safe_bearing_deg",
            self.planner_bearing_cb,
            10,
        )
        self.create_subscription(
            Float32,
            "/planner/speed_limit_mps",
            self.planner_speed_limit_cb,
            10,
        )
        self.create_subscription(
            String,
            "/perception/corridor_hint",
            self.corridor_hint_cb,
            10,
        )
        self.create_subscription(
            String,
            "/geofence/status",
            self.geofence_status_cb,
            10,
        )
        self.create_subscription(
            Bool,
            "/mission/completed",
            self.mission_completed_cb,
            10,
        )
        self.create_subscription(
            Bool,
            "/mission/started",
            self.mission_started_cb,
            10,
        )

        self.timer = self.create_timer(0.1, self.loop)

    def heading_cb(self, msg: Float32) -> None:
        self.current_heading = float(msg.data)

    def target_bearing_cb(self, msg: Float32) -> None:
        self.target_bearing = float(msg.data)

    def target_distance_cb(self, msg: Float32) -> None:
        self.target_distance = float(msg.data)

    def planner_bearing_cb(self, msg: Float32) -> None:
        self.planner_bearing = float(msg.data)
        self.planner_bearing_ts = self.get_clock().now().nanoseconds / 1e9

    def planner_speed_limit_cb(self, msg: Float32) -> None:
        speed_limit = float(msg.data)
        if not self.allow_reverse_planner_speed:
            speed_limit = max(0.0, speed_limit)
        self.planner_speed_limit = speed_limit
        self.planner_speed_ts = self.get_clock().now().nanoseconds / 1e9

    def corridor_hint_cb(self, msg: String) -> None:
        try:
            data = from_json(msg.data)
            self.vision_heading_bias = float(data.get("heading_bias_deg", 0.0))
        except Exception:
            self.vision_heading_bias = 0.0

    def geofence_status_cb(self, msg: String) -> None:
        try:
            self.geofence_status = from_json(msg.data)
            self.geofence_ts = self.get_clock().now().nanoseconds / 1e9
        except Exception:
            self.geofence_status = None

    def mission_completed_cb(self, msg: Bool) -> None:
        self.mission_completed = bool(msg.data)

    def mission_started_cb(self, msg: Bool) -> None:
        self.mission_started = bool(msg.data)

    def publish_stop(self, reason: str) -> None:
        cmd = Twist()
        self.cmd_pub.publish(cmd)
        self.setpoint_pub.publish(
            String(
                data=to_json(
                    {
                        "speed_setpoint": 0.0,
                        "yaw_rate_setpoint": 0.0,
                        "heading_error_deg": 0.0,
                        "vision_heading_bias_deg": self.vision_heading_bias,
                        "stop_reason": reason,
                    }
                )
            )
        )

    def _planner_fresh(self) -> bool:
        if self.planner_bearing is None:
            return False
        now = self.get_clock().now().nanoseconds / 1e9
        return now - self.planner_bearing_ts <= self.planner_timeout_s

    def _speed_limit_fresh(self) -> bool:
        if self.planner_speed_limit is None:
            return False
        now = self.get_clock().now().nanoseconds / 1e9
        return now - self.planner_speed_ts <= self.planner_timeout_s

    def _geofence_override(self):
        if self.geofence_status is None:
            return None
        now = self.get_clock().now().nanoseconds / 1e9
        if now - self.geofence_ts > self.geofence_timeout_s:
            return None
        if not bool(self.geofence_status.get("outside", False)):
            return None
        bearing = self.geofence_status.get("return_bearing_deg")
        if bearing is None:
            return None
        return {
            "bearing_deg": float(bearing),
            "speed_mps": max(
                0.0,
                float(self.geofence_status.get("return_speed_mps", 0.12)),
            ),
            "outside_duration_s": float(
                self.geofence_status.get("outside_duration_s", 0.0)
            ),
            "penalty_equivalent_exit_count": int(
                self.geofence_status.get("penalty_equivalent_exit_count", 0)
            ),
        }

    def loop(self) -> None:
        if self.mission_completed:
            self.publish_stop("mission_completed")
            return

        if not self.mission_started:
            self.publish_stop("mission_not_started")
            return

        if self.current_heading is None:
            return

        # Parkur 3: planner bearing varsa target_bearing şart değil
        using_planner = (
            self.use_planner_bearing
            and self._planner_fresh()
            and self.planner_bearing is not None
        )
        geofence = self._geofence_override()

        if self.target_bearing is None and not using_planner and geofence is None:
            self.publish_stop("no_target_bearing")
            return

        target_source = "guidance"
        if geofence is not None:
            corrected_target = geofence["bearing_deg"]
            target_source = "geofence_return"
        elif using_planner:
            corrected_target = self.planner_bearing
            target_source = "planner"
        else:
            corrected_target = self.target_bearing + self.vision_heading_bias

        heading_error = normalize_angle_deg(
            corrected_target - self.current_heading
        )

        # Parkur 3: planner kullanıyorsa hız limitini planner'dan al
        if geofence is not None:
            linear_speed = min(geofence["speed_mps"], self.max_linear_speed)
        elif using_planner and self._speed_limit_fresh():
            if self.allow_reverse_planner_speed:
                linear_speed = clamp(
                    self.planner_speed_limit,
                    -self.max_linear_speed,
                    self.max_linear_speed,
                )
            else:
                linear_speed = min(self.planner_speed_limit, self.max_linear_speed)
        elif using_planner:
            # Planner kullanılıyor ama speed limit eski, default hız kullan
            if fabs(heading_error) < 10.0:
                linear_speed = self.max_linear_speed
            elif fabs(heading_error) < 25.0:
                linear_speed = 0.28
            elif fabs(heading_error) < 45.0:
                linear_speed = 0.15
            else:
                linear_speed = 0.05
        elif fabs(heading_error) < 10.0:
            linear_speed = self.max_linear_speed
        elif fabs(heading_error) < 25.0:
            linear_speed = 0.28
        elif fabs(heading_error) < 45.0:
            linear_speed = 0.15
        else:
            linear_speed = 0.05

        # Dinamik kazanç: büyük heading error'da daha agresif dönüş
        effective_kp = self.kp_heading
        if fabs(heading_error) > 45.0:
            effective_kp = self.kp_heading * 2.0
        elif fabs(heading_error) > 20.0:
            effective_kp = self.kp_heading * 1.5
        angular_speed = clamp(
            heading_error * effective_kp,
            -self.max_angular_speed,
            self.max_angular_speed,
        )

        cmd = Twist()
        cmd.linear.x = float(linear_speed)
        cmd.angular.z = float(angular_speed)
        self.cmd_pub.publish(cmd)

        self.get_logger().info(
            f"cmd: linear={linear_speed:.2f} angular={angular_speed:.2f} "
            f"source={target_source} planner_fresh={self._planner_fresh()}",
            throttle_duration_sec=1.0,
        )

        self.setpoint_pub.publish(
            String(
                data=to_json(
                    {
                        "speed_setpoint": linear_speed,
                        "yaw_rate_setpoint": angular_speed,
                        "heading_error_deg": heading_error,
                        "vision_heading_bias_deg": self.vision_heading_bias,
                        "target_source": target_source,
                        "planner_speed_limit_mps": self.planner_speed_limit,
                        "geofence_outside_duration_s": None
                        if geofence is None
                        else geofence["outside_duration_s"],
                        "geofence_penalty_equivalent_exit_count": None
                        if geofence is None
                        else geofence["penalty_equivalent_exit_count"],
                        "allow_reverse_planner_speed": (
                            self.allow_reverse_planner_speed
                        ),
                    }
                )
            )
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
