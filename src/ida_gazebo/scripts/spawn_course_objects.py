#!/usr/bin/env python3
"""Mission JSON'daki duba ve engelleri Gazebo'ya spawn eder."""

import json
import subprocess
from pathlib import Path

import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node


class CourseObjectSpawner(Node):
    def __init__(self):
        super().__init__('course_object_spawner')

        self.declare_parameter('mission_file', '')
        self.declare_parameter('spawn_x', -528.0)
        self.declare_parameter('spawn_y', 193.0)
        self.declare_parameter('world_name', 'sydney_regatta')
        self.declare_parameter('spawn_delay_s', 14.0)
        self.declare_parameter('buoy_radius_m', 0.15)
        self.declare_parameter('buoy_height_m', 0.90)
        self.declare_parameter('include_boundaries', True)
        self.declare_parameter('include_obstacles', True)

        self.mission_file = str(self.get_parameter('mission_file').value)
        self.spawn_x = float(self.get_parameter('spawn_x').value)
        self.spawn_y = float(self.get_parameter('spawn_y').value)
        self.world_name = str(self.get_parameter('world_name').value)
        self.buoy_radius_m = float(self.get_parameter('buoy_radius_m').value)
        self.buoy_height_m = float(self.get_parameter('buoy_height_m').value)
        self.include_boundaries = bool(
            self.get_parameter('include_boundaries').value
        )
        self.include_obstacles = bool(
            self.get_parameter('include_obstacles').value
        )
        delay_s = float(self.get_parameter('spawn_delay_s').value)

        self.spawned = False
        self.timer = self.create_timer(max(delay_s, 0.1), self.spawn_once)

    def resolve_mission_file(self) -> Path:
        path = Path(self.mission_file).expanduser()
        if path.is_absolute() or path.exists():
            return path

        share_dir = Path(get_package_share_directory('ida_otonom'))
        return share_dir / 'missions' / path

    def load_objects(self) -> list[dict]:
        path = self.resolve_mission_file()
        data = json.loads(path.read_text(encoding='utf-8'))
        objects = []
        if self.include_boundaries:
            objects.extend(data.get('boundaries', []))
        if self.include_obstacles:
            objects.extend(data.get('obstacles', []))
        return objects

    def object_color(self, obj: dict) -> str:
        class_name = str(obj.get('class_name', ''))
        kind = str(obj.get('kind', ''))
        if class_name == 'obstacle_buoy' or kind == 'obstacle':
            return '1.0 0.9 0.05 1.0'
        return '1.0 0.35 0.02 1.0'

    def object_sdf(self, name: str, obj: dict) -> str:
        color = self.object_color(obj)
        radius = float(obj.get('radius_m', self.buoy_radius_m))
        height = self.buoy_height_m
        # Silindir LiDAR hizasından geçtiği için hem görsel hem collision algılanır.
        return f'''
<sdf version="1.6">
  <model name="{name}">
    <static>true</static>
    <link name="buoy_link">
      <visual name="buoy_visual">
        <pose>0 0 {height / 2.0:.3f} 0 0 0</pose>
        <geometry>
          <cylinder>
            <radius>{radius:.3f}</radius>
            <length>{height:.3f}</length>
          </cylinder>
        </geometry>
        <material>
          <diffuse>{color}</diffuse>
          <ambient>{color}</ambient>
        </material>
      </visual>
      <collision name="buoy_collision">
        <pose>0 0 {height / 2.0:.3f} 0 0 0</pose>
        <geometry>
          <cylinder>
            <radius>{radius:.3f}</radius>
            <length>{height:.3f}</length>
          </cylinder>
        </geometry>
      </collision>
    </link>
  </model>
</sdf>
'''

    def spawn_object(self, index: int, obj: dict):
        object_id = str(obj.get('id', f'object_{index}'))
        safe_id = ''.join(ch if ch.isalnum() or ch == '_' else '_' for ch in object_id)
        name = f'ida_course_{safe_id}'
        x = self.spawn_x + float(obj['east_m'])
        y = self.spawn_y + float(obj['north_m'])
        cmd = [
            'ros2', 'run', 'ros_gz_sim', 'create',
            '-world', self.world_name,
            '-string', self.object_sdf(name, obj),
            '-name', name,
            '-allow_renaming', 'true',
            '-x', f'{x:.3f}',
            '-y', f'{y:.3f}',
            '-z', '0.000',
        ]
        subprocess.run(cmd, check=False)

    def spawn_once(self):
        if self.spawned:
            return
        self.spawned = True
        self.timer.cancel()

        try:
            objects = self.load_objects()
        except Exception as exc:
            self.get_logger().error(f'Course object load failed: {exc}')
            return

        for index, obj in enumerate(objects):
            try:
                self.spawn_object(index, obj)
            except Exception as exc:
                self.get_logger().warning(f'Object spawn skipped: {exc}')

        self.get_logger().info(f'Spawned {len(objects)} course object(s)')


def main(args=None):
    rclpy.init(args=args)
    node = CourseObjectSpawner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
