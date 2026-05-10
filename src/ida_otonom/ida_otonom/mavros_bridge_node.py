import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, String

from .common import clamp, to_json

try:
    from mavros_msgs.msg import ManualControl
except Exception:
    ManualControl = None


class MavrosBridgeNode(Node):
    def __init__(self) -> None:
        super().__init__("mavros_bridge_node")

        self.declare_parameter("enabled", False)
        self.declare_parameter("output_mode", "disabled")
        self.declare_parameter("input_topic", "/control/cmd_vel_safe")
        self.declare_parameter(
            "cmd_vel_output_topic",
            "/mavros/setpoint_velocity/cmd_vel_unstamped",
        )
        self.declare_parameter(
            "manual_control_topic",
            "/mavros/manual_control/send",
        )
        self.declare_parameter("command_timeout_s", 0.5)
        self.declare_parameter("publish_rate_hz", 20.0)
        self.declare_parameter("max_linear_speed_mps", 0.45)
        self.declare_parameter("max_yaw_rate_radps", 0.8)
        self.declare_parameter("manual_throttle_neutral", 500)

        self.enabled = bool(self.get_parameter("enabled").value)
        self.output_mode = str(self.get_parameter("output_mode").value)
        self.command_timeout_s = float(
            self.get_parameter("command_timeout_s").value
        )
        self.max_linear_speed_mps = float(
            self.get_parameter("max_linear_speed_mps").value
        )
        self.max_yaw_rate_radps = float(
            self.get_parameter("max_yaw_rate_radps").value
        )
        self.manual_throttle_neutral = int(
            self.get_parameter("manual_throttle_neutral").value
        )

        self.last_cmd = Twist()
        self.last_cmd_time = 0.0
        self.kill_active = False
        self.mission_completed = False

        input_topic = str(self.get_parameter("input_topic").value)
        cmd_vel_output_topic = str(
            self.get_parameter("cmd_vel_output_topic").value
        )
        manual_control_topic = str(
            self.get_parameter("manual_control_topic").value
        )
        publish_rate_hz = float(self.get_parameter("publish_rate_hz").value)

        self.create_subscription(Twist, input_topic, self.cmd_cb, 10)
        self.create_subscription(Bool, "/safety/kill", self.kill_cb, 10)
        self.create_subscription(
            Bool,
            "/mission/completed",
            self.mission_completed_cb,
            10,
        )

        self.cmd_vel_pub = self.create_publisher(
            Twist,
            cmd_vel_output_topic,
            10,
        )
        self.manual_control_pub = None
        if ManualControl is not None:
            self.manual_control_pub = self.create_publisher(
                ManualControl,
                manual_control_topic,
                10,
            )

        self.status_pub = self.create_publisher(
            String,
            "/mavros_bridge/status",
            10,
        )

        timer_period = 1.0 / max(publish_rate_hz, 1.0)
        self.timer = self.create_timer(timer_period, self.loop)

        self.get_logger().warn(
            "MAVROS bridge starts disabled by default. Set enabled:=true only "
            "after Pixhawk failsafe, kill chain, mode, and topic mapping "
            "are verified."
        )

    def cmd_cb(self, msg: Twist) -> None:
        self.last_cmd = msg
        self.last_cmd_time = time.monotonic()

    def kill_cb(self, msg: Bool) -> None:
        self.kill_active = bool(msg.data)

    def mission_completed_cb(self, msg: Bool) -> None:
        self.mission_completed = bool(msg.data)

    def safe_command(self) -> Twist:
        now = time.monotonic()
        timed_out = (now - self.last_cmd_time) > self.command_timeout_s

        if timed_out or self.kill_active or self.mission_completed:
            return Twist()

        cmd = Twist()
        cmd.linear.x = clamp(
            self.last_cmd.linear.x,
            -self.max_linear_speed_mps,
            self.max_linear_speed_mps,
        )
        cmd.angular.z = clamp(
            self.last_cmd.angular.z,
            -self.max_yaw_rate_radps,
            self.max_yaw_rate_radps,
        )
        return cmd

    def publish_manual_control(self, cmd: Twist) -> bool:
        if self.manual_control_pub is None or ManualControl is None:
            return False

        # TODO: Verify ArduRover MANUAL_CONTROL axis mapping on Cube Orange+.
        # Defaults keep throttle neutral and map surge/yaw into x/r only.
        forward = 0.0
        if self.max_linear_speed_mps > 0.0:
            forward = clamp(
                cmd.linear.x / self.max_linear_speed_mps,
                -1.0,
                1.0,
            )

        yaw = 0.0
        if self.max_yaw_rate_radps > 0.0:
            yaw = clamp(cmd.angular.z / self.max_yaw_rate_radps, -1.0, 1.0)

        manual = ManualControl()
        manual.x = int(forward * 1000.0)
        manual.y = 0
        manual.z = int(clamp(self.manual_throttle_neutral, 0, 1000))
        manual.r = int(yaw * 1000.0)
        manual.buttons = 0
        self.manual_control_pub.publish(manual)
        return True

    def loop(self) -> None:
        cmd = self.safe_command()
        published = False

        if self.enabled and self.output_mode == "cmd_vel":
            self.cmd_vel_pub.publish(cmd)
            published = True
        elif self.enabled and self.output_mode == "manual_control":
            published = self.publish_manual_control(cmd)

        self.status_pub.publish(
            String(
                data=to_json(
                    {
                        "enabled": self.enabled,
                        "output_mode": self.output_mode,
                        "published": published,
                        "kill_active": self.kill_active,
                        "mission_completed": self.mission_completed,
                        "linear_x": cmd.linear.x,
                        "angular_z": cmd.angular.z,
                    }
                )
            )
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MavrosBridgeNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
