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

        self.latch_kill = bool(self.get_parameter("latch_kill").value)
        self.kill_active = False
        self.last_cmd = Twist()

        input_topic = str(self.get_parameter("input_topic").value)
        output_topic = str(self.get_parameter("output_topic").value)

        self.create_subscription(Bool, "/safety/kill", self.kill_cb, 10)
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

    def kill_reset_cb(self, msg: Bool) -> None:
        if msg.data:
            self.kill_active = False

    def cmd_cb(self, msg: Twist) -> None:
        self.last_cmd = msg

    def loop(self) -> None:
        if self.kill_active:
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
                        "latch_kill": self.latch_kill,
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
