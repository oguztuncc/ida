#!/usr/bin/env python3
"""Mission JSON'dan tekneye gore sentetik duba algilari uretir."""

import json
import math
from pathlib import Path

import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float32, String


def normalize_angle_deg(angle: float) -> float:
    return (angle + 180.0) % 360.0 - 180.0


class SimBuoyDetector(Node):
    def __init__(self):
        super().__init__('sim_buoy_detector')

        self.declare_parameter('mission_file', '')
        self.declare_parameter('detection_range_max_m', 18.0)
        self.declare_parameter('detection_fov_deg', 180.0)
        self.declare_parameter('publish_rate_hz', 8.0)
        self.declare_parameter('include_boundaries', True)
        self.declare_parameter('include_obstacles', True)
        self.declare_parameter('output_topic', '/perception/buoy_detections_raw')

        self.mission_file = str(self.get_parameter('mission_file').value)
        self.detection_range_max_m = float(
            self.get_parameter('detection_range_max_m').value
        )
        self.detection_fov_deg = float(
            self.get_parameter('detection_fov_deg').value
        )
        self.include_boundaries = bool(
            self.get_parameter('include_boundaries').value
        )
        self.include_obstacles = bool(
            self.get_parameter('include_obstacles').value
        )

        self.origin_lat = 40.1181
        self.origin_lon = 26.4081
        self.objects = []
        self.current_lat = None
        self.current_lon = None
        self.heading_deg = None

        self.load_mission()

        self.pub = self.create_publisher(
            String,
            str(self.get_parameter('output_topic').value),
            10,
        )
        self.create_subscription(
            NavSatFix,
            '/mavros/global_position/global',
            self.gps_cb,
            10,
        )
        self.create_subscription(
            Float32,
            '/mavros/global_position/compass_hdg',
            self.heading_cb,
            10,
        )

        rate_hz = float(self.get_parameter('publish_rate_hz').value)
        self.timer = self.create_timer(1.0 / max(rate_hz, 0.1), self.publish)

    def resolve_mission_file(self) -> Path:
        path = Path(self.mission_file).expanduser()
        if path.is_absolute() or path.exists():
            return path

        share_dir = Path(get_package_share_directory('ida_otonom'))
        return share_dir / 'missions' / path

    def load_mission(self):
        path = self.resolve_mission_file()
        data = json.loads(path.read_text(encoding='utf-8'))
        origin = data.get('origin') or {}
        self.origin_lat = float(origin.get('lat', self.origin_lat))
        self.origin_lon = float(origin.get('lon', self.origin_lon))
        objects = []
        if self.include_boundaries:
            objects.extend(data.get('boundaries', []))
        if self.include_obstacles:
            objects.extend(data.get('obstacles', []))
        self.objects = [obj for obj in objects if 'east_m' in obj and 'north_m' in obj]
        self.get_logger().info(
            f'Sim detector loaded {len(self.objects)} object(s) from {path}'
        )

    def gps_cb(self, msg: NavSatFix):
        self.current_lat = float(msg.latitude)
        self.current_lon = float(msg.longitude)

    def heading_cb(self, msg: Float32):
        self.heading_deg = float(msg.data) % 360.0

    def lat_lon_to_local(self, lat: float, lon: float) -> tuple[float, float]:
        north = (lat - self.origin_lat) * 111_320.0
        lon_scale = max(
            111_320.0 * math.cos(math.radians(self.origin_lat)),
            1e-6,
        )
        east = (lon - self.origin_lon) * lon_scale
        return east, north

    def object_detection(self, obj: dict, own_east: float, own_north: float):
        de = float(obj['east_m']) - own_east
        dn = float(obj['north_m']) - own_north
        range_m = math.hypot(de, dn)
        if range_m < 1e-6 or range_m > self.detection_range_max_m:
            return None

        absolute_bearing = math.degrees(math.atan2(de, dn)) % 360.0
        # Pozitif bearing teknenin sol tarafi olacak sekilde body-left acisi.
        bearing_deg = normalize_angle_deg(self.heading_deg - absolute_bearing)
        if abs(bearing_deg) > self.detection_fov_deg / 2.0:
            return None

        bearing_rad = math.radians(bearing_deg)
        class_name = str(obj.get('class_name', 'course_buoy'))
        kind = str(obj.get('kind', 'course_boundary'))
        hue = 55.0 if class_name == 'obstacle_buoy' or kind == 'obstacle' else 25.0
        return {
            'id': obj.get('id'),
            'class_name': class_name,
            'confidence': 0.95,
            'range_m': range_m,
            'bearing_deg': bearing_deg,
            'forward_m': range_m * math.cos(bearing_rad),
            'left_m': range_m * math.sin(bearing_rad),
            'mean_hsv': [hue, 0.85, 0.95],
            'source': 'sim_ground_truth',
        }

    def publish(self):
        if self.current_lat is None or self.current_lon is None or self.heading_deg is None:
            return

        own_east, own_north = self.lat_lon_to_local(self.current_lat, self.current_lon)
        detections = []
        for obj in self.objects:
            detection = self.object_detection(obj, own_east, own_north)
            if detection is not None:
                detections.append(detection)

        payload = {
            'timestamp': self.get_clock().now().nanoseconds / 1e9,
            'source': 'sim_ground_truth',
            'detections': detections,
        }
        self.pub.publish(String(data=json.dumps(payload)))


def main(args=None):
    rclpy.init(args=args)
    node = SimBuoyDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
