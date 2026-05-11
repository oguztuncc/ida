import math
from typing import Iterable, List, Tuple

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String

from .common import normalize_angle_deg, now_ts, to_json


class LidarProcessorNode(Node):
    def __init__(self) -> None:
        super().__init__("lidar_processor_node")

        self.declare_parameter("collision_distance_m", 1.2)
        self.declare_parameter("sector_width_deg", 30.0)

        self.collision_distance_m = float(
            self.get_parameter("collision_distance_m").value
        )
        self.sector_width_deg = float(
            self.get_parameter("sector_width_deg").value
        )

        self.create_subscription(LaserScan, "/scan", self.scan_cb, 10)
        self.pub = self.create_publisher(
            String,
            "/perception/lidar_summary",
            10,
        )

    def _valid_ranges(self, msg: LaserScan) -> Iterable[Tuple[float, float]]:
        angle = msg.angle_min
        for distance in msg.ranges:
            if (
                math.isfinite(distance)
                and msg.range_min <= distance <= msg.range_max
            ):
                yield normalize_angle_deg(math.degrees(angle)), distance
            angle += msg.angle_increment

    def _sector_min(self, values: List[Tuple[float, float]]) -> float:
        vals = [
            distance for _, distance in values
            if math.isfinite(distance) and distance > 0.05
        ]
        return min(vals) if vals else 999.0

    def scan_cb(self, msg: LaserScan) -> None:
        n = len(msg.ranges)
        if n < 10:
            return

        half = self.sector_width_deg / 2.0
        values = list(self._valid_ranges(msg))
        front = [(a, r) for a, r in values if abs(a) <= half]
        left = [
            (a, r) for a, r in values
            if half < a <= half + self.sector_width_deg
        ]
        right = [
            (a, r) for a, r in values
            if -half - self.sector_width_deg <= a < -half
        ]

        front_min = self._sector_min(front)
        left_min = self._sector_min(left)
        right_min = self._sector_min(right)

        collision = front_min < self.collision_distance_m
        if left_min > right_min and left_min > front_min:
            best_angle = 25.0
        elif right_min > left_min and right_min > front_min:
            best_angle = -25.0
        else:
            best_angle = 0.0

        payload = {
            "timestamp": now_ts(),
            "collision_imminent": collision,
            "front_clearance_m": front_min,
            "left_clearance_m": left_min,
            "right_clearance_m": right_min,
            "best_free_angle_deg": best_angle,
        }
        self.pub.publish(String(data=to_json(payload)))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = LidarProcessorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
