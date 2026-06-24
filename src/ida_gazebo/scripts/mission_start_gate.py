#!/usr/bin/env python3
"""Simulasyon hazir oldugunda mission start komutu yayinlar."""

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import Bool


class MissionStartGate(Node):
    """Gazebo varliklari hazir olmadan gorevi baslatmayan kapi."""

    def __init__(self):
        super().__init__('mission_start_gate')

        self.declare_parameter('auto_start', True)
        self.declare_parameter('require_course_objects', True)
        self.declare_parameter('require_waypoint_markers', False)
        self.declare_parameter('course_objects_ready_topic', '/sim/course_objects_ready')
        self.declare_parameter('waypoint_markers_ready_topic', '/sim/waypoint_markers_ready')
        self.declare_parameter('start_topic', '/mission/start')
        self.declare_parameter('start_delay_s', 0.5)
        self.declare_parameter('start_publish_duration_s', 3.0)

        self.auto_start = bool(self.get_parameter('auto_start').value)
        self.require_course_objects = bool(
            self.get_parameter('require_course_objects').value
        )
        self.require_waypoint_markers = bool(
            self.get_parameter('require_waypoint_markers').value
        )
        self.start_delay_s = max(
            0.0,
            float(self.get_parameter('start_delay_s').value),
        )
        self.start_publish_duration_s = max(
            0.1,
            float(self.get_parameter('start_publish_duration_s').value),
        )

        self.course_objects_ready = not self.require_course_objects
        self.waypoint_markers_ready = not self.require_waypoint_markers
        self.ready_since = None
        self.start_publish_until = None
        self.start_announced = False

        ready_qos = QoSProfile(depth=1)
        ready_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL

        self.start_pub = self.create_publisher(
            Bool,
            str(self.get_parameter('start_topic').value),
            10,
        )
        self.create_subscription(
            Bool,
            str(self.get_parameter('course_objects_ready_topic').value),
            self.course_objects_ready_cb,
            ready_qos,
        )
        self.create_subscription(
            Bool,
            str(self.get_parameter('waypoint_markers_ready_topic').value),
            self.waypoint_markers_ready_cb,
            ready_qos,
        )
        self.timer = self.create_timer(0.2, self.loop)

        if not self.auto_start:
            self.get_logger().info('Auto-start disabled; waiting for manual /mission/start')

    def course_objects_ready_cb(self, msg: Bool):
        self.course_objects_ready = bool(msg.data)
        if self.course_objects_ready:
            self.get_logger().info('Course objects ready')

    def waypoint_markers_ready_cb(self, msg: Bool):
        self.waypoint_markers_ready = bool(msg.data)
        if self.waypoint_markers_ready:
            self.get_logger().info('Waypoint markers ready')

    def all_ready(self) -> bool:
        return self.course_objects_ready and self.waypoint_markers_ready

    def loop(self):
        if not self.auto_start:
            return

        now = self.get_clock().now().nanoseconds / 1e9
        if not self.all_ready():
            self.ready_since = None
            return

        if self.ready_since is None:
            self.ready_since = now
            return

        if now - self.ready_since < self.start_delay_s:
            return

        if self.start_publish_until is None:
            self.start_publish_until = now + self.start_publish_duration_s
            self.get_logger().info('All simulation assets ready; starting mission')

        if now <= self.start_publish_until:
            self.start_pub.publish(Bool(data=True))
            return

        if not self.start_announced:
            self.start_announced = True
            self.get_logger().info('Mission start command published')


def main(args=None):
    rclpy.init(args=args)
    node = MissionStartGate()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
