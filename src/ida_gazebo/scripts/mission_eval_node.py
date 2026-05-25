#!/usr/bin/env python3
"""Gazebo görev testini otomatik değerlendiren yardımcı node."""

import json
import math
import os
from pathlib import Path

import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Int32, String, Bool


class MissionEvalNode(Node):
    def __init__(self):
        super().__init__('mission_eval_node')

        self.declare_parameter('mission_file', '')
        self.declare_parameter('timeout_s', 180.0)
        self.declare_parameter('arrival_radius_m', 1.2)
        self.declare_parameter('max_cross_track_m', 5.5)
        self.declare_parameter('max_stall_s', 35.0)
        self.declare_parameter('shutdown_on_finish', True)

        self.timeout_s = float(self.get_parameter('timeout_s').value)
        self.arrival_radius_m = float(
            self.get_parameter('arrival_radius_m').value
        )
        self.max_cross_track_m = float(
            self.get_parameter('max_cross_track_m').value
        )
        self.max_stall_s = float(self.get_parameter('max_stall_s').value)
        self.shutdown_on_finish = bool(
            self.get_parameter('shutdown_on_finish').value
        )

        self.origin_lat = 40.1181
        self.origin_lon = 26.4081
        self.local_waypoints = []
        self.load_mission()

        self.active_wp = 0
        self.completed = False
        self.position = None
        self.min_distances = [999.0 for _ in self.local_waypoints]
        self.max_seen_cross_track = 0.0
        self.last_progress_time = 0.0
        self.start_time = self.now_s()
        self.last_report_time = 0.0

        self.create_subscription(
            NavSatFix,
            '/mavros/global_position/global',
            self.gps_cb,
            10,
        )
        self.create_subscription(
            Int32,
            '/mission/active_waypoint',
            self.active_wp_cb,
            10,
        )
        self.create_subscription(
            Bool,
            '/mission/completed',
            self.completed_cb,
            10,
        )
        self.status_pub = self.create_publisher(
            String,
            '/eval/status',
            10,
        )
        self.timer = self.create_timer(0.5, self.loop)

    def now_s(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    def resolve_mission_file(self) -> Path:
        configured = str(self.get_parameter('mission_file').value)
        path = Path(configured).expanduser()
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
        self.local_waypoints = [
            (float(east), float(north))
            for east, north in data.get('local_waypoints', [])
        ]
        if not self.local_waypoints:
            waypoints = data.get('waypoints', [])
            self.local_waypoints = [self.lat_lon_to_local(wp['lat'], wp['lon'])
                                    for wp in waypoints]
        self.get_logger().info(
            f'Eval mission loaded: {path} ({len(self.local_waypoints)} wp)'
        )

    def lat_lon_to_local(self, lat: float, lon: float):
        north = (float(lat) - self.origin_lat) * 111_320.0
        lon_scale = max(
            111_320.0 * math.cos(math.radians(self.origin_lat)),
            1e-6,
        )
        east = (float(lon) - self.origin_lon) * lon_scale
        return east, north

    def gps_cb(self, msg: NavSatFix):
        self.position = self.lat_lon_to_local(msg.latitude, msg.longitude)

    def active_wp_cb(self, msg: Int32):
        new_wp = int(msg.data)
        if new_wp != self.active_wp:
            self.last_progress_time = self.now_s()
        self.active_wp = new_wp

    def completed_cb(self, msg: Bool):
        self.completed = bool(msg.data)

    def cross_track(self, pos, wp_index: int) -> float:
        if not self.local_waypoints:
            return 0.0
        if wp_index <= 0:
            a = self.local_waypoints[0]
            b = self.local_waypoints[min(1, len(self.local_waypoints) - 1)]
        else:
            a = self.local_waypoints[wp_index - 1]
            b = self.local_waypoints[min(wp_index, len(self.local_waypoints) - 1)]
        ax, ay = a
        bx, by = b
        px, py = pos
        dx = bx - ax
        dy = by - ay
        length = math.hypot(dx, dy)
        if length < 1e-6:
            return math.hypot(px - bx, py - by)
        return abs(dy * px - dx * py + bx * ay - by * ax) / length

    def finish(self, success: bool, reason: str):
        payload = self.payload(reason=reason, success=success)
        self.status_pub.publish(String(data=json.dumps(payload)))
        level = self.get_logger().info if success else self.get_logger().error
        level(f'EVAL {"PASS" if success else "FAIL"}: {reason} {payload}')
        if self.shutdown_on_finish:
            os._exit(0 if success else 2)

    def payload(self, reason='running', success=None):
        elapsed = self.now_s() - self.start_time
        return {
            'success': success,
            'reason': reason,
            'elapsed_s': elapsed,
            'active_waypoint': self.active_wp,
            'completed': self.completed,
            'position': self.position,
            'min_distances_m': self.min_distances,
            'max_cross_track_m': self.max_seen_cross_track,
        }

    def loop(self):
        now = self.now_s()
        if self.position is None or not self.local_waypoints:
            return

        for index, wp in enumerate(self.local_waypoints):
            dist = math.hypot(self.position[0] - wp[0], self.position[1] - wp[1])
            if dist < self.min_distances[index]:
                self.min_distances[index] = dist
                self.last_progress_time = now

        cross_track = self.cross_track(self.position, self.active_wp)
        self.max_seen_cross_track = max(self.max_seen_cross_track, cross_track)

        if self.completed:
            missed = [
                i for i, dist in enumerate(self.min_distances)
                if dist > self.arrival_radius_m
            ]
            if missed:
                self.finish(False, f'missed_waypoints={missed}')
            else:
                self.finish(True, 'mission_completed')
            return

        if cross_track > self.max_cross_track_m:
            self.finish(False, f'cross_track_exceeded={cross_track:.2f}')
            return

        elapsed = now - self.start_time
        if elapsed > self.timeout_s:
            self.finish(False, 'timeout')
            return

        if self.last_progress_time > 0.0 and now - self.last_progress_time > self.max_stall_s:
            self.finish(False, 'stalled')
            return

        if now - self.last_report_time > 2.0:
            self.last_report_time = now
            self.status_pub.publish(String(data=json.dumps(self.payload())))


def main(args=None):
    rclpy.init(args=args)
    node = MissionEvalNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
