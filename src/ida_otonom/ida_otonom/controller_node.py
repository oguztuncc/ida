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

        self.kp_heading = float(self.get_parameter("kp_heading").value)
        self.max_linear_speed = float(
            self.get_parameter("max_linear_speed").value
        )
        self.max_angular_speed = float(
            self.get_parameter("max_angular_speed").value
        )

        self.current_heading = None
        self.target_bearing = None
        self.target_distance = None
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
            String,
            "/perception/corridor_hint",
            self.corridor_hint_cb,
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

    def corridor_hint_cb(self, msg: String) -> None:
        try:
            data = from_json(msg.data)
            self.vision_heading_bias = float(data.get("heading_bias_deg", 0.0))
        except Exception:
            self.vision_heading_bias = 0.0

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

    def loop(self) -> None:
        if not self.mission_started:
            self.publish_stop("mission_not_started")
            return

        if self.mission_completed:
            self.publish_stop("mission_completed")
            return

        if self.current_heading is None or self.target_bearing is None:
            return

        corrected_target = self.target_bearing + self.vision_heading_bias
        heading_error = normalize_angle_deg(
            corrected_target - self.current_heading
        )

        if fabs(heading_error) < 10.0:
            linear_speed = self.max_linear_speed
        elif fabs(heading_error) < 25.0:
            linear_speed = 0.28
        elif fabs(heading_error) < 45.0:
            linear_speed = 0.15
        else:
            linear_speed = 0.05

        angular_speed = clamp(
            heading_error * self.kp_heading,
            -self.max_angular_speed,
            self.max_angular_speed,
        )

        cmd = Twist()
        cmd.linear.x = float(linear_speed)
        cmd.angular.z = float(angular_speed)
        self.cmd_pub.publish(cmd)

        self.setpoint_pub.publish(
            String(
                data=to_json(
                    {
                        "speed_setpoint": linear_speed,
                        "yaw_rate_setpoint": angular_speed,
                        "heading_error_deg": heading_error,
                        "vision_heading_bias_deg": self.vision_heading_bias,
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
