import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String

from .common import to_json

try:
    from mavros_msgs.msg import RCIn
except Exception:
    RCIn = None


class RcKillNode(Node):
    def __init__(self) -> None:
        super().__init__("rc_kill_node")

        self.declare_parameter("enabled", True)
        self.declare_parameter("channel_number", 7)
        self.declare_parameter("threshold_pwm", 1800)
        self.declare_parameter("active_high", True)
        self.declare_parameter("failsafe_on_missing", True)
        self.declare_parameter("rc_timeout_s", 0.5)

        self.enabled = bool(self.get_parameter("enabled").value)
        self.channel_number = int(self.get_parameter("channel_number").value)
        self.threshold_pwm = int(self.get_parameter("threshold_pwm").value)
        self.active_high = bool(self.get_parameter("active_high").value)
        self.failsafe_on_missing = bool(
            self.get_parameter("failsafe_on_missing").value
        )
        self.rc_timeout_s = float(self.get_parameter("rc_timeout_s").value)

        self.last_rc_time = 0.0
        self.last_pwm = None
        self.kill_active = self.failsafe_on_missing

        self.kill_pub = self.create_publisher(Bool, "/safety/kill", 10)
        self.status_pub = self.create_publisher(
            String,
            "/safety/rc_kill_status",
            10,
        )

        if RCIn is None:
            self.get_logger().warn(
                "mavros_msgs/RCIn is unavailable; RC kill input is inactive"
            )
        else:
            self.create_subscription(RCIn, "/mavros/rc/in", self.rc_cb, 10)

        self.timer = self.create_timer(0.1, self.loop)

    def rc_cb(self, msg) -> None:
        self.last_rc_time = time.monotonic()
        channel_index = self.channel_number - 1
        if channel_index < 0 or channel_index >= len(msg.channels):
            self.last_pwm = None
            self.kill_active = self.failsafe_on_missing
            return

        self.last_pwm = int(msg.channels[channel_index])
        if self.active_high:
            self.kill_active = self.last_pwm >= self.threshold_pwm
        else:
            self.kill_active = self.last_pwm <= self.threshold_pwm

    def loop(self) -> None:
        timed_out = (time.monotonic() - self.last_rc_time) > self.rc_timeout_s
        if timed_out and self.failsafe_on_missing:
            self.kill_active = True

        publish_kill = self.enabled and self.kill_active
        self.kill_pub.publish(Bool(data=publish_kill))
        self.status_pub.publish(
            String(
                data=to_json(
                    {
                        "enabled": self.enabled,
                        "channel_number": self.channel_number,
                        "last_pwm": self.last_pwm,
                        "threshold_pwm": self.threshold_pwm,
                        "active_high": self.active_high,
                        "timed_out": timed_out,
                        "kill_active": publish_kill,
                    }
                )
            )
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = RcKillNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
