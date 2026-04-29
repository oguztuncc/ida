import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool


class SafetyNode(Node):
    def __init__(self) -> None:
        super().__init__("safety_node")

        self.kill_active = False
        self.last_cmd = Twist()

        self.create_subscription(Bool, "/safety/kill", self.kill_cb, 10)
        self.create_subscription(Twist, "/cmd_vel", self.cmd_cb, 10)

        self.safe_pub = self.create_publisher(Twist, "/cmd_vel_safe", 10)

        self.timer = self.create_timer(0.05, self.loop)

    def kill_cb(self, msg: Bool) -> None:
        self.kill_active = bool(msg.data)

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


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SafetyNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()