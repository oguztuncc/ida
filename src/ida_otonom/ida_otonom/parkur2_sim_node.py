import math
from dataclasses import dataclass

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, NavSatFix
from std_msgs.msg import Float32, String

from .common import clamp, to_json


@dataclass(frozen=True)
class SimObject:
    object_id: str
    kind: str
    class_name: str
    east_m: float
    north_m: float
    radius_m: float
    hue_deg: float
    color: str


class Parkur2SimNode(Node):
    def __init__(self) -> None:
        super().__init__("parkur2_sim_node")

        self.declare_parameter("initial_lat", 40.1181000)
        self.declare_parameter("initial_lon", 26.4081000)
        self.declare_parameter("initial_heading_deg", 45.0)
        self.declare_parameter("update_rate_hz", 20.0)
        self.declare_parameter("scan_rate_hz", 10.0)
        self.declare_parameter("detection_rate_hz", 8.0)
        self.declare_parameter("world_rate_hz", 2.0)
        self.declare_parameter("time_scale", 1.0)
        self.declare_parameter("max_speed_mps", 0.40)
        self.declare_parameter("max_yaw_rate_radps", 0.70)
        self.declare_parameter("cmd_vel_topic", "/control/cmd_vel_safe")
        self.declare_parameter("scan_range_max_m", 12.0)
        self.declare_parameter("scan_range_min_m", 0.15)
        self.declare_parameter("scan_ray_count", 361)
        self.declare_parameter("detection_range_max_m", 10.0)
        self.declare_parameter("detection_fov_deg", 110.0)
        self.declare_parameter("world_variant", "parkur2")
        self.declare_parameter("course_width_m", 8.82)
        self.declare_parameter("course_jitter_m", 0.0)
        self.declare_parameter("include_obstacles", True)

        self.origin_lat = float(self.get_parameter("initial_lat").value)
        self.origin_lon = float(self.get_parameter("initial_lon").value)
        self.heading_deg = float(
            self.get_parameter("initial_heading_deg").value
        )
        self.update_rate_hz = float(self.get_parameter("update_rate_hz").value)
        self.dt = 1.0 / max(self.update_rate_hz, 1.0)
        self.time_scale = max(
            0.1,
            float(self.get_parameter("time_scale").value),
        )
        self.max_speed_mps = float(self.get_parameter("max_speed_mps").value)
        self.max_yaw_rate_radps = float(
            self.get_parameter("max_yaw_rate_radps").value
        )
        self.scan_range_max_m = float(
            self.get_parameter("scan_range_max_m").value
        )
        self.scan_range_min_m = float(
            self.get_parameter("scan_range_min_m").value
        )
        self.scan_ray_count = max(
            31,
            int(self.get_parameter("scan_ray_count").value),
        )
        self.detection_range_max_m = float(
            self.get_parameter("detection_range_max_m").value
        )
        self.detection_fov_deg = float(
            self.get_parameter("detection_fov_deg").value
        )
        self.world_variant = str(self.get_parameter("world_variant").value)
        self.course_width_m = max(
            2.0,
            float(self.get_parameter("course_width_m").value),
        )
        self.course_jitter_m = max(
            0.0,
            float(self.get_parameter("course_jitter_m").value),
        )
        self.include_obstacles = bool(
            self.get_parameter("include_obstacles").value
        )

        self.east_m = 0.0
        self.north_m = 0.0
        self.last_cmd = Twist()
        self.objects = self._build_world()

        self.gps_pub = self.create_publisher(
            NavSatFix,
            "/mavros/global_position/global",
            10,
        )
        self.heading_pub = self.create_publisher(
            Float32,
            "/mavros/global_position/compass_hdg",
            10,
        )
        self.scan_pub = self.create_publisher(LaserScan, "/scan", 10)
        self.detection_pub = self.create_publisher(
            String,
            "/perception/buoy_detections",
            10,
        )
        self.world_pub = self.create_publisher(String, "/sim/world", 10)

        cmd_vel_topic = str(self.get_parameter("cmd_vel_topic").value)
        self.create_subscription(Twist, cmd_vel_topic, self.cmd_cb, 10)
        self.create_subscription(
            Float32,
            "/sim/time_scale",
            self.time_scale_cb,
            10,
        )

        self.motion_timer = self.create_timer(self.dt, self.motion_loop)
        scan_rate_hz = float(self.get_parameter("scan_rate_hz").value)
        detection_rate_hz = float(self.get_parameter("detection_rate_hz").value)
        world_rate_hz = float(self.get_parameter("world_rate_hz").value)
        self.scan_timer = self.create_timer(
            1.0 / max(scan_rate_hz, 1.0),
            self.publish_scan,
        )
        self.detection_timer = self.create_timer(
            1.0 / max(detection_rate_hz, 1.0),
            self.publish_detections,
        )
        self.world_timer = self.create_timer(
            1.0 / max(world_rate_hz, 0.2),
            self.publish_world,
        )

        self.get_logger().info(
            f"{self.world_variant} sim world loaded with "
            f"{len(self.objects)} object(s), time_scale={self.time_scale:.1f}x"
        )

    def _build_world(self) -> list[SimObject]:
        if self.world_variant == "parkur1":
            return self._build_parkur1_world()
        return self._build_parkur2_world()

    def _build_parkur2_world(self) -> list[SimObject]:
        route = [
            (0.0, 0.0),
            (12.0, 12.0),
            (25.0, 22.0),
            (40.0, 35.0),
            (55.0, 48.0),
        ]
        stations = [
            (6.0, 6.0),
            (13.0, 12.8),
            (20.0, 18.2),
            (28.0, 24.6),
            (36.0, 31.4),
            (45.0, 39.3),
            (53.0, 46.2),
        ]
        objects = []
        previous = route[0]
        for index, center in enumerate(stations):
            next_point = route[min(index // 2 + 1, len(route) - 1)]
            dx = next_point[0] - previous[0]
            dy = next_point[1] - previous[1]
            length = max(math.hypot(dx, dy), 1e-6)
            left_x = -dy / length
            left_y = dx / length
            offset = self.course_width_m / 2.0
            for side, sign in (("left", 1.0), ("right", -1.0)):
                east = center[0] + left_x * offset * sign
                north = center[1] + left_y * offset * sign
                objects.append(
                    SimObject(
                        object_id=f"course_{side}_{index}",
                        kind="course_boundary",
                        class_name="course_buoy",
                        east_m=east,
                        north_m=north,
                        radius_m=0.35,
                        hue_deg=28.0,
                        color="#ff8b2e",
                    )
                )
            previous = center

        if self.include_obstacles:
            objects.extend(
                [
                    SimObject(
                        "obstacle_0",
                        "obstacle",
                        "obstacle_buoy",
                        9.4,
                        8.4,
                        0.70,
                        62.0,
                        "#ffe15a",
                    ),
                    SimObject(
                        "obstacle_1",
                        "obstacle",
                        "obstacle_buoy",
                        32.0,
                        29.3,
                        0.65,
                        62.0,
                        "#ffe15a",
                    ),
                    SimObject(
                        "obstacle_2",
                        "obstacle",
                        "obstacle_buoy",
                        44.6,
                        37.0,
                        0.70,
                        62.0,
                        "#ffe15a",
                    ),
                ]
            )
        return objects

    def _build_parkur1_world(self) -> list[SimObject]:
        route = [
            (0.0, 0.0),
            (16.0, 15.0),
            (34.0, 34.0),
            (50.0, 56.0),
            (67.0, 76.0),
            (84.0, 94.0),
            (101.0, 111.0),
        ]
        return self._build_course_boundaries(
            route,
            spacing_m=7.0,
            object_prefix="parkur1_course",
        )

    def _build_course_boundaries(
        self,
        route: list[tuple[float, float]],
        spacing_m: float,
        object_prefix: str,
    ) -> list[SimObject]:
        objects = []
        half_width = self.course_width_m / 2.0
        station_index = 0
        for start, end in zip(route, route[1:]):
            dx = end[0] - start[0]
            dy = end[1] - start[1]
            length = math.hypot(dx, dy)
            if length <= 1e-6:
                continue
            forward_x = dx / length
            forward_y = dy / length
            left_x = -forward_y
            left_y = forward_x
            count = max(1, int(round(length / max(spacing_m, 0.5))))
            for index in range(count):
                t = (index + 0.5) / count
                center_east = start[0] + dx * t
                center_north = start[1] + dy * t
                for side, sign in (("left", 1.0), ("right", -1.0)):
                    phase = station_index * 1.37 + (
                        0.0 if sign > 0.0 else 2.1
                    )
                    lateral_jitter = math.sin(phase) * self.course_jitter_m
                    forward_jitter = (
                        math.cos(phase * 0.7) * self.course_jitter_m * 0.5
                    )
                    offset = max(1.0, half_width + lateral_jitter)
                    east = (
                        center_east
                        + left_x * offset * sign
                        + forward_x * forward_jitter
                    )
                    north = (
                        center_north
                        + left_y * offset * sign
                        + forward_y * forward_jitter
                    )
                    objects.append(
                        SimObject(
                            object_id=f"{object_prefix}_{side}_{station_index}",
                            kind="course_boundary",
                            class_name="course_buoy",
                            east_m=east,
                            north_m=north,
                            radius_m=0.35,
                            hue_deg=28.0,
                            color="#ff8b2e",
                        )
                    )
                station_index += 1
        return objects

    def cmd_cb(self, msg: Twist) -> None:
        self.last_cmd = msg

    def time_scale_cb(self, msg: Float32) -> None:
        self.time_scale = clamp(float(msg.data), 0.1, 20.0)
        self.get_logger().info(
            f"Simulation time scale set to {self.time_scale:.1f}x"
        )

    def motion_loop(self) -> None:
        speed = clamp(
            float(self.last_cmd.linear.x),
            -self.max_speed_mps,
            self.max_speed_mps,
        )
        yaw_rate = clamp(
            float(self.last_cmd.angular.z),
            -self.max_yaw_rate_radps,
            self.max_yaw_rate_radps,
        )

        scaled_dt = self.dt * self.time_scale
        self.heading_deg = (
            self.heading_deg + math.degrees(yaw_rate * scaled_dt)
        ) % 360.0
        heading_rad = math.radians(self.heading_deg)
        distance_m = speed * scaled_dt
        self.north_m += distance_m * math.cos(heading_rad)
        self.east_m += distance_m * math.sin(heading_rad)

        lat, lon = self._lat_lon(self.east_m, self.north_m)
        gps = NavSatFix()
        gps.header.stamp = self.get_clock().now().to_msg()
        gps.header.frame_id = "map"
        gps.latitude = lat
        gps.longitude = lon
        gps.altitude = 0.0
        self.gps_pub.publish(gps)
        self.heading_pub.publish(Float32(data=float(self.heading_deg)))

    def _lat_lon(self, east_m: float, north_m: float) -> tuple[float, float]:
        lat = self.origin_lat + north_m / 111_320.0
        lon_scale = max(
            111_320.0 * math.cos(math.radians(self.origin_lat)),
            1e-6,
        )
        lon = self.origin_lon + east_m / lon_scale
        return lat, lon

    def _object_in_boat_frame(self, obj: SimObject) -> tuple[float, float]:
        de = obj.east_m - self.east_m
        dn = obj.north_m - self.north_m
        heading = math.radians(self.heading_deg)
        forward = de * math.sin(heading) + dn * math.cos(heading)
        left = dn * math.sin(heading) - de * math.cos(heading)
        return forward, left

    def _ray_hit_distance(
        self,
        angle_rad: float,
        forward: float,
        left: float,
        radius: float,
    ) -> float | None:
        dx = math.cos(angle_rad)
        dy = math.sin(angle_rad)
        projection = forward * dx + left * dy
        if projection <= 0.0:
            return None
        center_sq = forward * forward + left * left
        closest_sq = center_sq - projection * projection
        radius_sq = radius * radius
        if closest_sq > radius_sq:
            return None
        offset = math.sqrt(max(radius_sq - closest_sq, 0.0))
        distance = projection - offset
        if distance < self.scan_range_min_m:
            distance = projection + offset
        if self.scan_range_min_m <= distance <= self.scan_range_max_m:
            return distance
        return None

    def publish_scan(self) -> None:
        scan = LaserScan()
        scan.header.stamp = self.get_clock().now().to_msg()
        scan.header.frame_id = "base_link"
        scan.angle_min = -math.pi
        scan.angle_max = math.pi
        scan.angle_increment = (
            scan.angle_max - scan.angle_min
        ) / max(self.scan_ray_count - 1, 1)
        scan.time_increment = 0.0
        scan.scan_time = 1.0 / max(
            float(self.get_parameter("scan_rate_hz").value),
            1.0,
        )
        scan.range_min = self.scan_range_min_m
        scan.range_max = self.scan_range_max_m
        scan.ranges = []

        boat_frame_objects = [
            (self._object_in_boat_frame(obj), obj.radius_m) for obj in self.objects
        ]
        for idx in range(self.scan_ray_count):
            angle = scan.angle_min + idx * scan.angle_increment
            best = self.scan_range_max_m
            hit = False
            for (forward, left), radius in boat_frame_objects:
                distance = self._ray_hit_distance(angle, forward, left, radius)
                if distance is not None and distance < best:
                    best = distance
                    hit = True
            scan.ranges.append(best if hit else float("inf"))

        self.scan_pub.publish(scan)

    def publish_detections(self) -> None:
        detections = []
        image_w = 640
        image_h = 480
        half_fov = self.detection_fov_deg / 2.0
        for obj in self.objects:
            forward, left = self._object_in_boat_frame(obj)
            if forward <= 0.2:
                continue
            range_m = math.hypot(forward, left)
            if range_m > self.detection_range_max_m:
                continue
            bearing_deg = math.degrees(math.atan2(left, forward))
            if abs(bearing_deg) > half_fov:
                continue

            center_x = (
                image_w / 2.0
                + (bearing_deg / self.detection_fov_deg) * image_w
            )
            box_h = clamp(120.0 / max(range_m, 0.5), 14.0, 90.0)
            box_w = box_h * 0.65
            detections.append(
                {
                    "id": obj.object_id,
                    "class_name": obj.class_name,
                    "confidence": (
                        0.88 if obj.kind == "course_boundary" else 0.82
                    ),
                    "bbox_xyxy": [
                        center_x - box_w / 2.0,
                        image_h / 2.0 - box_h / 2.0,
                        center_x + box_w / 2.0,
                        image_h / 2.0 + box_h / 2.0,
                    ],
                    "center_px": [center_x, image_h / 2.0],
                    "range_m": range_m,
                    "bearing_deg": bearing_deg,
                    "mean_hsv": [obj.hue_deg, 0.88, 0.92],
                    "image_size": [image_w, image_h],
                }
            )

        self.detection_pub.publish(
            String(
                data=to_json(
                    {
                        "timestamp": self.get_clock().now().nanoseconds / 1e9,
                        "frame_id": "sim_camera",
                        "model_path": f"synthetic_{self.world_variant}_sim",
                        "model_loaded": True,
                        "detections": detections,
                    }
                )
            )
        )

    def publish_world(self) -> None:
        objects = []
        closest_clearance = None
        closest_id = None
        for obj in self.objects:
            lat, lon = self._lat_lon(obj.east_m, obj.north_m)
            clearance = (
                math.hypot(obj.east_m - self.east_m, obj.north_m - self.north_m)
                - obj.radius_m
            )
            if closest_clearance is None or clearance < closest_clearance:
                closest_clearance = clearance
                closest_id = obj.object_id
            objects.append(
                {
                    "id": obj.object_id,
                    "kind": obj.kind,
                    "class_name": obj.class_name,
                    "east_m": obj.east_m,
                    "north_m": obj.north_m,
                    "lat": lat,
                    "lon": lon,
                    "radius_m": obj.radius_m,
                    "color": obj.color,
                }
            )

        self.world_pub.publish(
            String(
                data=to_json(
                    {
                        "timestamp": self.get_clock().now().nanoseconds / 1e9,
                        "world_variant": self.world_variant,
                        "time_scale": self.time_scale,
                        "origin": {
                            "lat": self.origin_lat,
                            "lon": self.origin_lon,
                        },
                        "boat": {
                            "east_m": self.east_m,
                            "north_m": self.north_m,
                            "heading_deg": self.heading_deg,
                            "closest_object_id": closest_id,
                            "closest_clearance_m": closest_clearance,
                            "collision": closest_clearance is not None
                            and closest_clearance <= 0.0,
                        },
                        "objects": objects,
                    }
                )
            )
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = Parkur2SimNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
