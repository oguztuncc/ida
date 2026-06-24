import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, String

from .common import to_json


class SafetyNode(Node):
    def __init__(self) -> None:
        super().__init__("safety_node")

        self.declare_parameter("input_topic", "/control/cmd_vel")
        self.declare_parameter("output_topic", "/control/cmd_vel_safe")
        self.declare_parameter("latch_kill", True)
        self.declare_parameter("command_timeout_s", 0.5)
        self.declare_parameter("block_reset_on_physical_kill", True)

        self.latch_kill = bool(self.get_parameter("latch_kill").value)
        self.command_timeout_s = float(
            self.get_parameter("command_timeout_s").value
        )
        self.block_reset_on_physical_kill = bool(
            self.get_parameter("block_reset_on_physical_kill").value
        )
        self.kill_active = False
        self.physical_kill_active = False
        self.last_cmd = Twist()
        self.last_cmd_ts = 0.0

        input_topic = str(self.get_parameter("input_topic").value)
        output_topic = str(self.get_parameter("output_topic").value)

        self.create_subscription(Bool, "/safety/kill", self.kill_cb, 10)
        self.create_subscription(
            Bool,
            "/safety/physical_kill",
            self.physical_kill_cb,
            10,
        )
        self.create_subscription(
            Bool,
            "/safety/kill_reset",
            self.kill_reset_cb,
            10,
        )
        self.create_subscription(Twist, input_topic, self.cmd_cb, 10)

        self.safe_pub = self.create_publisher(Twist, output_topic, 10)
        self.status_pub = self.create_publisher(String, "/safety/status", 10)

        self.timer = self.create_timer(0.05, self.loop)

    def kill_cb(self, msg: Bool) -> None:
        if msg.data:
            self.kill_active = True
        elif not self.latch_kill:
            self.kill_active = False

    def physical_kill_cb(self, msg: Bool) -> None:
        self.physical_kill_active = bool(msg.data)
        if self.physical_kill_active:
            self.kill_active = True

    def kill_reset_cb(self, msg: Bool) -> None:
        if msg.data:
            if self.block_reset_on_physical_kill and self.physical_kill_active:
                self.get_logger().warning(
                    "Kill reset ignored while physical/remote kill is active"
                )
                return
            self.kill_active = False

    def cmd_cb(self, msg: Twist) -> None:
        self.last_cmd = msg
        self.last_cmd_ts = self.get_clock().now().nanoseconds / 1e9

    def loop(self) -> None:
        now = self.get_clock().now().nanoseconds / 1e9
        command_timed_out = (
            self.last_cmd_ts <= 0.0
            or now - self.last_cmd_ts > self.command_timeout_s
        )
        if self.kill_active or command_timed_out:
            out = Twist()
            out.linear.x = 0.0
            out.angular.z = 0.0
            self.safe_pub.publish(out)
        else:
            self.safe_pub.publish(self.last_cmd)

        self.status_pub.publish(
            String(
                data=to_json(
                    {
                        "kill_active": self.kill_active,
                        "physical_kill_active": self.physical_kill_active,
                        "latch_kill": self.latch_kill,
                        "block_reset_on_physical_kill": (
                            self.block_reset_on_physical_kill
                        ),
                        "command_timed_out": command_timed_out,
                    }
                )
            )
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SafetyNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
