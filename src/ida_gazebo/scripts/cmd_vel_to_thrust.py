#!/usr/bin/env python3
"""
/control/cmd_vel (Twist) -> sol/sağ thrust komutlarına dönüştürür.
IDA katamaran: diferansiyel itki, arkada 2 adet sabit pervane.

Gazebo Sim Thruster plugin Gazebo Transport topic dinlediği için
ROS2 topic yerine doğrudan gz-transport üzerinden publish eder.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from gz.transport13 import Node as GzNode
from gz.msgs10.double_pb2 import Double


class CmdVelToThrust(Node):
    def __init__(self):
        super().__init__('cmd_vel_to_thrust')

        self.declare_parameter('wheelbase_m', 0.60)
        self.declare_parameter('max_thrust', 50.0)
        self.declare_parameter('command_timeout_s', 0.5)
        self.declare_parameter('yaw_sign', -1.0)
        self.declare_parameter('cmd_vel_topic', '/control/cmd_vel')
        self.declare_parameter(
            'left_thrust_topic',
            '/model/ida_katamaran/joint/left_prop_joint/cmd_thrust',
        )
        self.declare_parameter(
            'right_thrust_topic',
            '/model/ida_katamaran/joint/right_prop_joint/cmd_thrust',
        )

        self.wheelbase = self.get_parameter('wheelbase_m').value
        self.max_thrust = self.get_parameter('max_thrust').value
        self.command_timeout_s = self.get_parameter('command_timeout_s').value
        self.yaw_sign = self.get_parameter('yaw_sign').value
        cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        left_topic = self.get_parameter('left_thrust_topic').value
        right_topic = self.get_parameter('right_thrust_topic').value
        self.last_cmd_ts = 0.0

        # ROS2 subscription
        self.create_subscription(Twist, cmd_vel_topic, self.cmd_vel_cb, 10)

        # Gazebo transport publishers
        self.gz_node = GzNode()
        self.left_pub = self.gz_node.advertise(left_topic, Double)
        self.right_pub = self.gz_node.advertise(right_topic, Double)
        self.create_timer(0.05, self.timeout_cb)

        self.get_logger().info(
            f'CmdVelToThrust basladi: wheelbase={self.wheelbase}m, '
            f'max_thrust={self.max_thrust}N | '
            f'gz topics: {left_topic}, {right_topic}'
        )

    def cmd_vel_cb(self, msg: Twist):
        self.last_cmd_ts = self.get_clock().now().nanoseconds / 1e9
        linear = msg.linear.x
        angular = msg.angular.z * self.yaw_sign

        # cmd_vel (m/s, rad/s) -> thrust (N)
        # Scale: max_thrust N per unit cmd_vel
        v = linear * self.max_thrust
        w = angular * self.max_thrust

        # Diferansiyel itki kinematiği
        # Not: Sağ propeller joint axis'i -1 0 0 (sol propeller ile
        # zıt yönde dönerek reaksiyon torque'larını iptal eder).
        # Aynı cmd_thrust değeri sağda ters yönde ittiği için
        # sağ thrust'ı ters işaretliyoruz.
        left = v - w * self.wheelbase / 2.0
        right = -(v + w * self.wheelbase / 2.0)

        # Thrust sınırlama (Gazebo Thruster plugin N cinsinden bekliyor)
        left = max(-self.max_thrust, min(self.max_thrust, left))
        right = max(-self.max_thrust, min(self.max_thrust, right))

        self.publish_thrust(left, right)

        self.get_logger().info(
            f' thrust -> left={left:.2f}N right={right:.2f}N '
            f'(cmd: lin={linear:.2f} ang={msg.angular.z:.2f} '
            f'effective_ang={angular:.2f})',
            throttle_duration_sec=0.5,
        )

    def publish_thrust(self, left: float, right: float):
        gz_left = Double()
        gz_left.data = float(left)
        gz_right = Double()
        gz_right.data = float(right)

        self.left_pub.publish(gz_left)
        self.right_pub.publish(gz_right)

    def timeout_cb(self):
        now = self.get_clock().now().nanoseconds / 1e9
        command_stale = now - self.last_cmd_ts > self.command_timeout_s
        if self.last_cmd_ts <= 0.0 or command_stale:
            self.publish_thrust(0.0, 0.0)


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelToThrust()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
