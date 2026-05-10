import math
from typing import List

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String

from .ida_otonom.common import to_json, now_ts


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

    def _sector_min(
        self,
        ranges: List[float],
        start_idx: int,
        end_idx: int,
    ) -> float:
        vals = [
            r for r in ranges[start_idx:end_idx]
            if math.isfinite(r) and r > 0.05
        ]
        return min(vals) if vals else 999.0

    def scan_cb(self, msg: LaserScan) -> None:
        n = len(msg.ranges)
        if n < 10:
            return

        deg_per_sample = math.degrees(msg.angle_increment)
        sector_samples = max(
            1,
            int(self.sector_width_deg / max(deg_per_sample, 1e-6)),
        )

        center = n // 2
        front_min = self._sector_min(
            msg.ranges,
            max(0, center - sector_samples // 2),
            min(n, center + sector_samples // 2),
        )
        left_min = self._sector_min(
            msg.ranges,
            min(n - 1, center + sector_samples),
            min(n, center + 2 * sector_samples),
        )
        right_min = self._sector_min(
            msg.ranges,
            max(0, center - 2 * sector_samples),
            max(1, center - sector_samples),
        )

        collision = front_min < self.collision_distance_m

        # Basit serbest yön seçimi: en açık tarafı seç
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
