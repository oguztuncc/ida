import csv
import json
import math
import os
from datetime import datetime

import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

from .common import default_record_dir


class LocalCostmapNode(Node):
    def __init__(self) -> None:
        super().__init__("local_costmap_node")

        self.declare_parameter("log_dir", default_record_dir())
        self.declare_parameter("resolution_m", 0.25)
        self.declare_parameter("width_m", 12.0)
        self.declare_parameter("height_m", 12.0)
        self.declare_parameter("publish_rate_hz", 1.0)

        self.log_dir = str(self.get_parameter("log_dir").value)
        self.resolution_m = float(self.get_parameter("resolution_m").value)
        self.width_m = float(self.get_parameter("width_m").value)
        self.height_m = float(self.get_parameter("height_m").value)
        publish_rate_hz = float(self.get_parameter("publish_rate_hz").value)

        os.makedirs(self.log_dir, exist_ok=True)
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.csv_path = os.path.join(
            self.log_dir,
            f"local_costmap_{stamp}.csv",
        )
        self.file = open(self.csv_path, "w", newline="", encoding="utf-8")
        self.writer = csv.writer(self.file)
        self.writer.writerow(
            [
                "timestamp",
                "resolution_m",
                "width",
                "height",
                "origin_x_m",
                "origin_y_m",
                "occupied_cells_json",
            ]
        )

        self.latest_scan = None
        self.costmap_pub = self.create_publisher(
            OccupancyGrid,
            "/local_costmap",
            10,
        )
        self.create_subscription(LaserScan, "/scan", self.scan_cb, 10)

        period = 1.0 / max(publish_rate_hz, 1.0)
        self.timer = self.create_timer(period, self.loop)

    def scan_cb(self, msg: LaserScan) -> None:
        self.latest_scan = msg

    def build_occupied_cells(self):
        if self.latest_scan is None:
            return []

        width = max(1, int(self.width_m / self.resolution_m))
        height = max(1, int(self.height_m / self.resolution_m))
        origin_x = -self.width_m / 2.0
        origin_y = -self.height_m / 2.0
        occupied = set()

        angle = self.latest_scan.angle_min
        for distance in self.latest_scan.ranges:
            if (
                math.isfinite(distance)
                and self.latest_scan.range_min
                <= distance
                <= self.latest_scan.range_max
            ):
                x = distance * math.cos(angle)
                y = distance * math.sin(angle)
                ix = int((x - origin_x) / self.resolution_m)
                iy = int((y - origin_y) / self.resolution_m)
                if 0 <= ix < width and 0 <= iy < height:
                    occupied.add((ix, iy))
            angle += self.latest_scan.angle_increment

        return sorted(occupied)

    def loop(self) -> None:
        width = max(1, int(self.width_m / self.resolution_m))
        height = max(1, int(self.height_m / self.resolution_m))
        origin_x = -self.width_m / 2.0
        origin_y = -self.height_m / 2.0
        occupied = self.build_occupied_cells()

        grid = OccupancyGrid()
        grid.header.stamp = self.get_clock().now().to_msg()
        grid.header.frame_id = "base_link"
        grid.info.resolution = self.resolution_m
        grid.info.width = width
        grid.info.height = height
        grid.info.origin.position.x = origin_x
        grid.info.origin.position.y = origin_y
        grid.data = [0] * (width * height)

        for ix, iy in occupied:
            grid.data[iy * width + ix] = 100

        self.costmap_pub.publish(grid)
        self.writer.writerow(
            [
                self.get_clock().now().nanoseconds / 1e9,
                self.resolution_m,
                width,
                height,
                origin_x,
                origin_y,
                json.dumps(occupied, separators=(",", ":")),
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
    node = LocalCostmapNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
