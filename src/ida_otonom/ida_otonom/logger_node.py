import csv
import os
from datetime import datetime

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float32, String

from .common import from_json


class LoggerNode(Node):
    def __init__(self) -> None:
        super().__init__("logger_node")

        self.declare_parameter("log_dir", "/home/jetson/records")
        self.log_dir = str(self.get_parameter("log_dir").value)
        os.makedirs(self.log_dir, exist_ok=True)

        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.csv_path = os.path.join(self.log_dir, f"telemetry_{stamp}.csv")

        self.lat = None
        self.lon = None
        self.heading = None
        self.target_bearing = None
        self.target_distance = None
        self.speed_setpoint = 0.0
        self.yaw_rate_setpoint = 0.0

        self.file = open(self.csv_path, "w", newline="", encoding="utf-8")
        self.writer = csv.writer(self.file)
        self.writer.writerow(
            [
                "timestamp",
                "lat",
                "lon",
                "heading_deg",
                "target_bearing_deg",
                "target_distance_m",
                "speed_setpoint",
                "yaw_rate_setpoint",
            ]
        )

        self.create_subscription(NavSatFix, "/mavros/global_position/global", self.gps_cb, 10)
        self.create_subscription(Float32, "/mavros/global_position/compass_hdg", self.heading_cb, 10)
        self.create_subscription(Float32, "/guidance/target_bearing_deg", self.target_bearing_cb, 10)
        self.create_subscription(Float32, "/guidance/target_distance_m", self.target_distance_cb, 10)
        self.create_subscription(String, "/control/setpoints", self.setpoints_cb, 10)

        self.timer = self.create_timer(1.0, self.loop)

    def gps_cb(self, msg: NavSatFix) -> None:
        self.lat = msg.latitude
        self.lon = msg.longitude

    def heading_cb(self, msg: Float32) -> None:
        self.heading = float(msg.data)

    def target_bearing_cb(self, msg: Float32) -> None:
        self.target_bearing = float(msg.data)

    def target_distance_cb(self, msg: Float32) -> None:
        self.target_distance = float(msg.data)

    def setpoints_cb(self, msg: String) -> None:
        data = from_json(msg.data)
        self.speed_setpoint = float(data.get("speed_setpoint", 0.0))
        self.yaw_rate_setpoint = float(data.get("yaw_rate_setpoint", 0.0))

    def loop(self) -> None:
        self.writer.writerow(
            [
                self.get_clock().now().nanoseconds / 1e9,
                self.lat,
                self.lon,
                self.heading,
                self.target_bearing,
                self.target_distance,
                self.speed_setpoint,
                self.yaw_rate_setpoint,
            ]
        )
        self.file.flush()

    def destroy_node(self):
        try:
            self.file.close()
        finally:
            super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = LoggerNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()