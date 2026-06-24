#!/usr/bin/env python3
"""
Simulasyon NavSat + IMU verilerini ida_otonom'un bekledigi topic'lere donusturur.
VRX Sydney origin (-33.72, 150.68) -> Turkey origin (40.1181, 26.4081) offset.
IMU yaw -> pusula basligi (0-360 derece, Kuzey=0).
"""

import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, Imu
from std_msgs.msg import Float32


class SimNavConverter(Node):
    def __init__(self):
        super().__init__('sim_nav_converter')

        self.declare_parameter('sim_origin_lat', -33.724223)
        self.declare_parameter('sim_origin_lon', 150.679736)
        self.declare_parameter('spawn_east_m', 0.0)
        self.declare_parameter('spawn_north_m', 0.0)
        self.declare_parameter('target_origin_lat', 40.1181)
        self.declare_parameter('target_origin_lon', 26.4081)
        self.declare_parameter('navsat_topic', '/model/ida_katamaran/navsat')
        self.declare_parameter('imu_topic', '/model/ida_katamaran/imu')

        self.sim_lat0 = self.get_parameter('sim_origin_lat').value
        self.sim_lon0 = self.get_parameter('sim_origin_lon').value
        self.spawn_east_m = self.get_parameter('spawn_east_m').value
        self.spawn_north_m = self.get_parameter('spawn_north_m').value
        self.target_lat0 = self.get_parameter('target_origin_lat').value
        self.target_lon0 = self.get_parameter('target_origin_lon').value
        self.spawn_lat0, self.spawn_lon0 = self.world_xy_to_lat_lon(
            self.spawn_east_m,
            self.spawn_north_m,
        )

        navsat_topic = self.get_parameter('navsat_topic').value
        imu_topic = self.get_parameter('imu_topic').value

        self.create_subscription(NavSatFix, navsat_topic, self.navsat_cb, 10)
        self.create_subscription(Imu, imu_topic, self.imu_cb, 10)

        self.gps_pub = self.create_publisher(
            NavSatFix,
            '/mavros/global_position/global',
            10,
        )
        self.hdg_pub = self.create_publisher(
            Float32,
            '/mavros/global_position/compass_hdg',
            10,
        )

        self.get_logger().info(
            f'SimNavConverter basladi: '
            f'spawn_gps=({self.spawn_lat0}, {self.spawn_lon0}) -> '
            f'target_origin=({self.target_lat0}, {self.target_lon0})'
        )

    def world_xy_to_lat_lon(self, east_m: float, north_m: float):
        lat = self.sim_lat0 + north_m / 111_320.0
        lon_scale = max(
            111_320.0 * math.cos(math.radians(self.sim_lat0)),
            1e-6,
        )
        lon = self.sim_lon0 + east_m / lon_scale
        return lat, lon

    def navsat_cb(self, msg: NavSatFix):
        dlat = msg.latitude - self.spawn_lat0
        dlon = msg.longitude - self.spawn_lon0

        out = NavSatFix()
        out.header = msg.header
        out.latitude = self.target_lat0 + dlat
        out.longitude = self.target_lon0 + dlon
        out.altitude = msg.altitude
        out.status = msg.status
        out.position_covariance = msg.position_covariance
        out.position_covariance_type = msg.position_covariance_type

        self.gps_pub.publish(out)

    def imu_cb(self, msg: Imu):
        q = msg.orientation
        # Quaternion -> yaw (ENU)
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        yaw_deg = math.degrees(yaw)
        # ENU: yaw=0 -> Dogu (X). Pusula: 0 -> Kuzey (Y).
        hdg = (90.0 - yaw_deg) % 360.0

        self.hdg_pub.publish(Float32(data=float(hdg)))


def main(args=None):
    rclpy.init(args=args)
    node = SimNavConverter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
