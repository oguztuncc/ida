import socket
from typing import Any, Dict

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Bool, Int32, String

from .common import to_json, from_json


class YkiBridgeNode(Node):
    def __init__(self) -> None:
        super().__init__("yki_bridge_node")

        self.declare_parameter("udp_ip", "127.0.0.1")
        self.declare_parameter("udp_port", 5005)
        self.declare_parameter("enable_command_rx", False)
        self.declare_parameter("command_bind_ip", "0.0.0.0")
        self.declare_parameter("command_bind_port", 5006)

        self.udp_ip = str(self.get_parameter("udp_ip").value)
        self.udp_port = int(self.get_parameter("udp_port").value)
        self.enable_command_rx = bool(
            self.get_parameter("enable_command_rx").value
        )
        self.command_bind_ip = str(self.get_parameter("command_bind_ip").value)
        self.command_bind_port = int(
            self.get_parameter("command_bind_port").value
        )

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.command_sock = None
        if self.enable_command_rx:
            self.command_sock = socket.socket(
                socket.AF_INET,
                socket.SOCK_DGRAM,
            )
            self.command_sock.bind(
                (self.command_bind_ip, self.command_bind_port)
            )
            self.command_sock.setblocking(False)

        self.lat = None
        self.lon = None
        self.active_waypoint = 0
        self.mission_started = False
        self.mission_completed = False
        self.mission_status = {}
        self.guidance_status = {}
        self.waypoints = []
        self.target_color = None

        self.create_subscription(
            NavSatFix,
            "/mavros/global_position/global",
            self.gps_cb,
            10,
        )
        self.create_subscription(
            Int32,
            "/mission/active_waypoint",
            self.active_wp_cb,
            10,
        )
        self.create_subscription(Bool, "/mission/completed", self.done_cb, 10)
        self.create_subscription(Bool, "/mission/started", self.started_cb, 10)
        self.create_subscription(
            String,
            "/mission/status",
            self.mission_status_cb,
            10,
        )
        self.create_subscription(
            String,
            "/guidance/status",
            self.guidance_status_cb,
            10,
        )
        self.create_subscription(
            String,
            "/mission/waypoints",
            self.waypoints_cb,
            10,
        )

        self.start_pub = self.create_publisher(Bool, "/mission/start", 10)
        self.kill_pub = self.create_publisher(Bool, "/safety/kill", 10)
        self.kill_reset_pub = self.create_publisher(
            Bool,
            "/safety/kill_reset",
            10,
        )
        self.target_color_pub = self.create_publisher(
            String,
            "/iha/target_color",
            10,
        )
        self.mission_target_color_pub = self.create_publisher(
            String,
            "/mission/target_color",
            10,
        )

        self.timer = self.create_timer(0.5, self.loop)
        if self.enable_command_rx:
            self.command_timer = self.create_timer(0.1, self.command_loop)

    def gps_cb(self, msg: NavSatFix) -> None:
        self.lat = msg.latitude
        self.lon = msg.longitude

    def active_wp_cb(self, msg: Int32) -> None:
        self.active_waypoint = int(msg.data)

    def done_cb(self, msg: Bool) -> None:
        self.mission_completed = bool(msg.data)

    def started_cb(self, msg: Bool) -> None:
        self.mission_started = bool(msg.data)

    def mission_status_cb(self, msg: String) -> None:
        self.mission_status = from_json(msg.data)

    def guidance_status_cb(self, msg: String) -> None:
        self.guidance_status = from_json(msg.data)

    def waypoints_cb(self, msg: String) -> None:
        self.waypoints = from_json(msg.data).get("waypoints", [])

    def command_loop(self) -> None:
        if self.command_sock is None:
            return

        while True:
            try:
                data, _addr = self.command_sock.recvfrom(4096)
            except BlockingIOError:
                return
            except OSError as exc:
                self.get_logger().warn(f"YKI command socket error: {exc}")
                return

            try:
                command = from_json(data.decode("utf-8"))
                self.handle_command(command)
            except Exception as exc:
                self.get_logger().warn(f"Ignoring invalid YKI command: {exc}")

    def handle_command(self, command: Dict[str, Any]) -> None:
        command_name = str(command.get("command", "")).lower()

        if command_name == "kill":
            active = bool(command.get("active", True))
            self.kill_pub.publish(Bool(data=active))
            return

        if command_name == "reset_kill":
            self.kill_reset_pub.publish(Bool(data=True))
            return

        if self.mission_started:
            self.get_logger().warn(
                "YKI command ignored after mission start; only kill is allowed"
            )
            return

        if command_name == "start_mission":
            self.start_pub.publish(Bool(data=True))
            return

        if command_name == "set_target_color":
            color = str(command.get("color", "")).strip().lower()
            if not color:
                self.get_logger().warn("Target color command has empty color")
                return
            self.target_color = color
            self.target_color_pub.publish(String(data=color))
            self.mission_target_color_pub.publish(String(data=color))
            return

        self.get_logger().warn(f"Unsupported YKI command: {command_name}")

    def loop(self) -> None:
        payload = {
            "lat": self.lat,
            "lon": self.lon,
            "active_waypoint": self.active_waypoint,
            "mission_started": self.mission_started,
            "mission_completed": self.mission_completed,
            "mission_status": self.mission_status,
            "guidance_status": self.guidance_status,
            "waypoints": self.waypoints,
            "target_color": self.target_color,
        }
        self.sock.sendto(
            to_json(payload).encode("utf-8"),
            (self.udp_ip, self.udp_port),
        )

    def destroy_node(self):
        try:
            self.sock.close()
            if self.command_sock is not None:
                self.command_sock.close()
        finally:
            super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = YkiBridgeNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
