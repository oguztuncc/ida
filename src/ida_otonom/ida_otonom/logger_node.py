import csv
import math
import os
from datetime import datetime

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import Imu, NavSatFix
from std_msgs.msg import Float32, String

from .common import default_record_dir, from_json


class LoggerNode(Node):
    def __init__(self) -> None:
        super().__init__("logger_node")

        self.declare_parameter("log_dir", default_record_dir())
        self.log_dir = str(self.get_parameter("log_dir").value)
        os.makedirs(self.log_dir, exist_ok=True)

        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.csv_path = os.path.join(self.log_dir, f"telemetry_{stamp}.csv")

        self.lat = None
        self.lon = None
        self.heading = None
        self.roll = None
        self.pitch = None
        self.ground_speed = None
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
                "ground_speed_mps",
                "roll_deg",
                "pitch_deg",
                "heading_deg",
                "target_bearing_deg",
                "target_distance_m",
                "speed_setpoint",
                "yaw_rate_setpoint",
            ]
        )

        self.create_subscription(
            NavSatFix,
            "/mavros/global_position/global",
            self.gps_cb,
            10,
        )
        self.create_subscription(
            Float32,
            "/mavros/global_position/compass_hdg",
            self.heading_cb,
            10,
        )
        self.create_subscription(
            Imu,
            "/mavros/imu/data",
            self.imu_cb,
            10,
        )
        self.create_subscription(
            TwistStamped,
            "/mavros/local_position/velocity_body",
            self.velocity_cb,
            10,
        )
        self.create_subscription(
            Float32,
            "/guidance/target_bearing_deg",
            self.target_bearing_cb,
            10,
        )
        self.create_subscription(
            Float32,
            "/guidance/target_distance_m",
            self.target_distance_cb,
            10,
        )
        self.create_subscription(
            String,
            "/control/setpoints",
            self.setpoints_cb,
            10,
        )

        self.timer = self.create_timer(1.0, self.loop)

    def gps_cb(self, msg: NavSatFix) -> None:
        self.lat = msg.latitude
        self.lon = msg.longitude

    def heading_cb(self, msg: Float32) -> None:
        self.heading = float(msg.data)

    def imu_cb(self, msg: Imu) -> None:
        q = msg.orientation
        sinr_cosp = 2.0 * (q.w * q.x + q.y * q.z)
        cosr_cosp = 1.0 - 2.0 * (q.x * q.x + q.y * q.y)
        roll = math.atan2(sinr_cosp, cosr_cosp)

        sinp = 2.0 * (q.w * q.y - q.z * q.x)
        if abs(sinp) >= 1.0:
            pitch = math.copysign(math.pi / 2.0, sinp)
        else:
            pitch = math.asin(sinp)

        self.roll = math.degrees(roll)
        self.pitch = math.degrees(pitch)

    def velocity_cb(self, msg: TwistStamped) -> None:
        vx = msg.twist.linear.x
        vy = msg.twist.linear.y
        self.ground_speed = math.hypot(vx, vy)

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
                self.ground_speed,
                self.roll,
                self.pitch,
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
