import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String


class MavrosBridgeNode(Node):
    def __init__(self) -> None:
        super().__init__("mavros_bridge_node")

        self.last_cmd = Twist()

        self.create_subscription(Twist, "/control/cmd_vel", self.cmd_cb, 10)

        # Şimdilik geçiş düğümü gibi davranıyor.
        # Gerçek sistemde buradan MAVROS setpoint veya override topic’ine yazacaksınız.
        self.forward_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.status_pub = self.create_publisher(String, "/mavros_bridge/status", 10)

        self.timer = self.create_timer(0.05, self.loop)

    def cmd_cb(self, msg: Twist) -> None:
        self.last_cmd = msg

    def loop(self) -> None:
        self.forward_pub.publish(self.last_cmd)
        self.status_pub.publish(
            String(
                data=f"forwarding linear={self.last_cmd.linear.x:.3f}, angular={self.last_cmd.angular.z:.3f}"
            )
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MavrosBridgeNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()