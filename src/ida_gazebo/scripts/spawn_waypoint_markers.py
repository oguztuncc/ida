#!/usr/bin/env python3
"""Mission waypoint'lerini Gazebo'da gorunur marker olarak spawn eder."""

import json
import subprocess
from pathlib import Path

import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node


class WaypointMarkerSpawner(Node):
    def __init__(self):
        super().__init__('waypoint_marker_spawner')

        self.declare_parameter('mission_file', '')
        self.declare_parameter('spawn_x', -528.0)
        self.declare_parameter('spawn_y', 193.0)
        self.declare_parameter('marker_z', 0.65)
        self.declare_parameter('world_name', 'sydney_regatta')
        self.declare_parameter('spawn_delay_s', 14.0)

        self.mission_file = str(self.get_parameter('mission_file').value)
        self.spawn_x = float(self.get_parameter('spawn_x').value)
        self.spawn_y = float(self.get_parameter('spawn_y').value)
        self.marker_z = float(self.get_parameter('marker_z').value)
        self.world_name = str(self.get_parameter('world_name').value)
        delay_s = float(self.get_parameter('spawn_delay_s').value)

        self.spawned = False
        self.timer = self.create_timer(max(delay_s, 0.1), self.spawn_once)

    def resolve_mission_file(self) -> Path:
        path = Path(self.mission_file).expanduser()
        if path.is_absolute() or path.exists():
            return path

        share_dir = Path(get_package_share_directory('ida_otonom'))
        return share_dir / 'missions' / path

    def load_waypoints(self) -> list[tuple[float, float]]:
        path = self.resolve_mission_file()
        data = json.loads(path.read_text(encoding='utf-8'))
        local_waypoints = data.get('local_waypoints') or []
        if local_waypoints:
            return [(float(east), float(north)) for east, north in local_waypoints]
        return []

    def marker_sdf(self, index: int, total: int) -> str:
        if index == 0:
            color = '0.1 1.0 0.1 1.0'
        elif index == total - 1:
            color = '1.0 0.1 0.1 1.0'
        else:
            color = '0.1 0.35 1.0 1.0'

        return f'''
<sdf version="1.6">
  <model name="ida_waypoint_marker_{index}">
    <static>true</static>
    <link name="marker_link">
      <visual name="pole_visual">
        <pose>0 0 -0.30 0 0 0</pose>
        <geometry>
          <cylinder>
            <radius>0.035</radius>
            <length>0.60</length>
          </cylinder>
        </geometry>
        <material>
          <diffuse>1.0 1.0 1.0 1.0</diffuse>
          <ambient>1.0 1.0 1.0 1.0</ambient>
        </material>
      </visual>
      <visual name="sphere_visual">
        <pose>0 0 0.08 0 0 0</pose>
        <geometry>
          <sphere>
            <radius>0.28</radius>
          </sphere>
        </geometry>
        <material>
          <diffuse>{color}</diffuse>
          <ambient>{color}</ambient>
        </material>
      </visual>
    </link>
  </model>
</sdf>
'''

    def spawn_marker(self, index: int, total: int, east_m: float, north_m: float):
        x = self.spawn_x + east_m
        y = self.spawn_y + north_m
        cmd = [
            'ros2', 'run', 'ros_gz_sim', 'create',
            '-world', self.world_name,
            '-string', self.marker_sdf(index, total),
            '-name', f'ida_waypoint_{index}',
            '-allow_renaming', 'true',
            '-x', f'{x:.3f}',
            '-y', f'{y:.3f}',
            '-z', f'{self.marker_z:.3f}',
        ]
        subprocess.run(cmd, check=False)

    def spawn_once(self):
        if self.spawned:
            return
        self.spawned = True
        self.timer.cancel()

        try:
            waypoints = self.load_waypoints()
        except Exception as exc:
            self.get_logger().error(f'Waypoint marker load failed: {exc}')
            return

        if not waypoints:
            self.get_logger().warning('No local_waypoints found for markers')
            return

        for index, (east_m, north_m) in enumerate(waypoints):
            self.spawn_marker(index, len(waypoints), east_m, north_m)

        self.get_logger().info(f'Spawned {len(waypoints)} waypoint marker(s)')


def main(args=None):
    rclpy.init(args=args)
    node = WaypointMarkerSpawner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
