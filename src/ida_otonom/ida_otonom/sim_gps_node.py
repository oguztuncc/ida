import math

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float32


class SimGpsNode(Node):
    def __init__(self) -> None:
        super().__init__("sim_gps_node")

        self.declare_parameter("initial_lat", 40.1181000)
        self.declare_parameter("initial_lon", 26.4081000)
        self.declare_parameter("initial_heading_deg", 90.0)
        self.declare_parameter("update_rate_hz", 10.0)
        self.declare_parameter("max_speed_mps", 0.45)
        self.declare_parameter("max_yaw_rate_radps", 0.8)
        self.declare_parameter("cmd_vel_topic", "/control/cmd_vel")

        self.lat = float(self.get_parameter("initial_lat").value)
        self.lon = float(self.get_parameter("initial_lon").value)
        self.heading_deg = float(
            self.get_parameter("initial_heading_deg").value
        )
        self.max_speed_mps = float(self.get_parameter("max_speed_mps").value)
        self.max_yaw_rate_radps = float(
            self.get_parameter("max_yaw_rate_radps").value
        )
        cmd_vel_topic = str(self.get_parameter("cmd_vel_topic").value)

        update_rate_hz = float(self.get_parameter("update_rate_hz").value)
        self.dt = 1.0 / max(update_rate_hz, 1.0)
        self.last_cmd = Twist()

        self.gps_pub = self.create_publisher(
            NavSatFix,
            "/mavros/global_position/global",
            10,
        )
        self.heading_pub = self.create_publisher(
            Float32,
            "/mavros/global_position/compass_hdg",
            10,
        )
        self.create_subscription(Twist, cmd_vel_topic, self.cmd_cb, 10)

        self.timer = self.create_timer(self.dt, self.loop)

    def cmd_cb(self, msg: Twist) -> None:
        self.last_cmd = msg

    def loop(self) -> None:
        speed = max(
            -self.max_speed_mps,
            min(self.max_speed_mps, self.last_cmd.linear.x),
        )
        yaw_rate = max(
            -self.max_yaw_rate_radps,
            min(self.max_yaw_rate_radps, self.last_cmd.angular.z),
        )

        self.heading_deg = (
            self.heading_deg + math.degrees(yaw_rate * self.dt)
        ) % 360.0
        heading_rad = math.radians(self.heading_deg)
        distance_m = speed * self.dt

        north_m = distance_m * math.cos(heading_rad)
        east_m = distance_m * math.sin(heading_rad)
        lat_rad = math.radians(self.lat)

        self.lat += north_m / 111_320.0
        lon_scale = max(111_320.0 * math.cos(lat_rad), 1e-6)
        self.lon += east_m / lon_scale

        gps = NavSatFix()
        gps.header.stamp = self.get_clock().now().to_msg()
        gps.header.frame_id = "map"
        gps.latitude = self.lat
        gps.longitude = self.lon
        gps.altitude = 0.0
        self.gps_pub.publish(gps)
        self.heading_pub.publish(Float32(data=float(self.heading_deg)))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SimGpsNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
