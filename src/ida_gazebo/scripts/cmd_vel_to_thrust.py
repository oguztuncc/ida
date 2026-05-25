#!/usr/bin/env python3
"""
/control/cmd_vel (Twist) -> sol/sağ thrust komutlarına dönüştürür.

IDA katamaran: diferansiyel itki, arkada 2 adet sabit pervane.

Gazebo Sim Thruster plugin Gazebo Transport topic dinlediği için itki
komutlarını gz-transport üzerinden publish eder. Aynı değerleri debug için
ROS 2 JSON status topic'i olarak da yayınlar.
"""

import json

import rclpy
from geometry_msgs.msg import Twist
from gz.msgs10.double_pb2 import Double
from gz.transport13 import Node as GzNode
from rclpy.node import Node
from std_msgs.msg import String


class CmdVelToThrust(Node):
    def __init__(self):
        super().__init__('cmd_vel_to_thrust')

        self.declare_parameter('wheelbase_m', 0.60)
        self.declare_parameter('max_thrust', 50.0)
        self.declare_parameter('command_timeout_s', 0.5)
        self.declare_parameter('thrust_rate_limit_nps', 100.0)
        self.declare_parameter('yaw_sign', -1.0)
        self.declare_parameter('cmd_vel_topic', '/control/cmd_vel')
        self.declare_parameter('status_topic', '/sim/motor_thrust')
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
        self.thrust_rate_limit_nps = float(
            self.get_parameter('thrust_rate_limit_nps').value
        )
        self.yaw_sign = self.get_parameter('yaw_sign').value
        cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        status_topic = self.get_parameter('status_topic').value
        left_topic = self.get_parameter('left_thrust_topic').value
        right_topic = self.get_parameter('right_thrust_topic').value
        self.last_cmd_ts = 0.0
        self.last_publish_ts = 0.0
        self.last_left_thrust = 0.0
        self.last_right_thrust = 0.0

        # ROS2 subscription
        self.create_subscription(Twist, cmd_vel_topic, self.cmd_vel_cb, 10)
        self.status_pub = self.create_publisher(String, status_topic, 10)

        # Gazebo transport publishers
        self.gz_node = GzNode()
        self.left_pub = self.gz_node.advertise(left_topic, Double)
        self.right_pub = self.gz_node.advertise(right_topic, Double)
        self.create_timer(0.05, self.timeout_cb)

        self.get_logger().info(
            f'CmdVelToThrust basladi: wheelbase={self.wheelbase}m, '
            f'max_thrust={self.max_thrust}N | '
            f'rate_limit={self.thrust_rate_limit_nps}N/s | '
            f'ros status: {status_topic} | '
            f'gz topics: {left_topic}, {right_topic}'
        )

    def limit_thrust_rate(self, target: float, current: float, dt: float) -> float:
        if self.thrust_rate_limit_nps <= 0.0 or dt <= 0.0:
            return target
        max_delta = self.thrust_rate_limit_nps * dt
        delta = max(-max_delta, min(max_delta, target - current))
        return current + delta

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

        left, right = self.publish_thrust(
            left,
            right,
            stale=False,
            linear=linear,
            angular=msg.angular.z,
            effective_angular=angular,
        )

        self.get_logger().info(
            f' thrust -> left={left:.2f}N right={right:.2f}N '
            f'(cmd: lin={linear:.2f} ang={msg.angular.z:.2f} '
            f'effective_ang={angular:.2f})',
            throttle_duration_sec=0.5,
        )

    def publish_thrust(
        self,
        left: float,
        right: float,
        *,
        stale: bool = False,
        linear: float = 0.0,
        angular: float = 0.0,
        effective_angular: float = 0.0,
    ):
        now = self.get_clock().now().nanoseconds / 1e9
        dt = 0.0 if self.last_publish_ts <= 0.0 else now - self.last_publish_ts

        left = self.limit_thrust_rate(left, self.last_left_thrust, dt)
        right = self.limit_thrust_rate(right, self.last_right_thrust, dt)

        gz_left = Double()
        gz_left.data = float(left)
        gz_right = Double()
        gz_right.data = float(right)

        self.left_pub.publish(gz_left)
        self.right_pub.publish(gz_right)

        self.last_publish_ts = now
        self.last_left_thrust = float(left)
        self.last_right_thrust = float(right)
        self.publish_status(
            left,
            right,
            stale=stale,
            linear=linear,
            angular=angular,
            effective_angular=effective_angular,
        )
        return left, right

    def publish_status(
        self,
        left: float,
        right: float,
        *,
        stale: bool,
        linear: float,
        angular: float,
        effective_angular: float,
    ) -> None:
        max_thrust = max(abs(float(self.max_thrust)), 1e-6)
        left_drive = float(left)
        right_drive = -float(right)
        status = {
            'left_thrust_n': float(left),
            'right_thrust_n': float(right),
            'left_percent': float(left) / max_thrust * 100.0,
            'right_percent': float(right) / max_thrust * 100.0,
            'left_drive_thrust_n': left_drive,
            'right_drive_thrust_n': right_drive,
            'left_drive_percent': left_drive / max_thrust * 100.0,
            'right_drive_percent': right_drive / max_thrust * 100.0,
            'max_thrust_n': float(self.max_thrust),
            'linear_mps': float(linear),
            'yaw_rate_radps': float(angular),
            'effective_yaw_rate_radps': float(effective_angular),
            'stale': bool(stale),
        }
        self.status_pub.publish(String(data=json.dumps(status)))

    def timeout_cb(self):
        now = self.get_clock().now().nanoseconds / 1e9
        command_stale = now - self.last_cmd_ts > self.command_timeout_s
        if self.last_cmd_ts <= 0.0 or command_stale:
            self.publish_thrust(0.0, 0.0, stale=True)


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
