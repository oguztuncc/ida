import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist


class SimGpsNode(Node):
    def __init__(self):
        super().__init__("sim_gps_node")

        self.lat = 40.1180000
        self.lon = 26.4080000
        self.heading = 0.0

        self.cmd = Twist()

        self.gps_pub = self.create_publisher(NavSatFix, "/mavros/global_position/global", 10)
        self.heading_pub = self.create_publisher(Float32, "/mavros/global_position/compass_hdg", 10)

        self.create_subscription(Twist, "/control/cmd_vel", self.cmd_cb, 10)

        self.timer = self.create_timer(0.1, self.loop)

    def cmd_cb(self, msg):
        self.cmd = msg

    def loop(self):
        dt = 0.1

        self.heading += math.degrees(self.cmd.angular.z) * dt
        self.heading = self.heading % 360.0

        speed = self.cmd.linear.x
        distance = speed * dt

        meters_per_deg_lat = 111_111.0
        meters_per_deg_lon = 111_111.0 * math.cos(math.radians(self.lat))

        self.lat += (distance * math.cos(math.radians(self.heading))) / meters_per_deg_lat
        self.lon += (distance * math.sin(math.radians(self.heading))) / meters_per_deg_lon

        gps = NavSatFix()
        gps.latitude = self.lat
        gps.longitude = self.lon

        self.gps_pub.publish(gps)
        self.heading_pub.publish(Float32(data=float(self.heading)))


def main(args=None):
    rclpy.init(args=args)
    node = SimGpsNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()