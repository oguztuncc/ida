import socket

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import String, Int32, Bool

from .common import to_json, from_json


class YkiBridgeNode(Node):
    def __init__(self) -> None:
        super().__init__("yki_bridge_node")

        self.declare_parameter("udp_ip", "127.0.0.1")
        self.declare_parameter("udp_port", 5005)

        self.udp_ip = str(self.get_parameter("udp_ip").value)
        self.udp_port = int(self.get_parameter("udp_port").value)

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        self.lat = None
        self.lon = None
        self.active_waypoint = 0
        self.mission_completed = False
        self.mission_status = {}
        self.guidance_status = {}
        self.waypoints = []

        self.create_subscription(NavSatFix, "/mavros/global_position/global", self.gps_cb, 10)
        self.create_subscription(Int32, "/mission/active_waypoint", self.active_wp_cb, 10)
        self.create_subscription(Bool, "/mission/completed", self.done_cb, 10)
        self.create_subscription(String, "/mission/status", self.mission_status_cb, 10)
        self.create_subscription(String, "/guidance/status", self.guidance_status_cb, 10)
        self.create_subscription(String, "/mission/waypoints", self.waypoints_cb, 10)

        self.timer = self.create_timer(0.5, self.loop)

    def gps_cb(self, msg: NavSatFix) -> None:
        self.lat = msg.latitude
        self.lon = msg.longitude

    def active_wp_cb(self, msg: Int32) -> None:
        self.active_waypoint = int(msg.data)

    def done_cb(self, msg: Bool) -> None:
        self.mission_completed = bool(msg.data)

    def mission_status_cb(self, msg: String) -> None:
        self.mission_status = from_json(msg.data)

    def guidance_status_cb(self, msg: String) -> None:
        self.guidance_status = from_json(msg.data)

    def waypoints_cb(self, msg: String) -> None:
        self.waypoints = from_json(msg.data).get("waypoints", [])

    def loop(self) -> None:
        payload = {
            "lat": self.lat,
            "lon": self.lon,
            "active_waypoint": self.active_waypoint,
            "mission_completed": self.mission_completed,
            "mission_status": self.mission_status,
            "guidance_status": self.guidance_status,
            "waypoints": self.waypoints,
        }
        self.sock.sendto(to_json(payload).encode("utf-8"), (self.udp_ip, self.udp_port))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = YkiBridgeNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()