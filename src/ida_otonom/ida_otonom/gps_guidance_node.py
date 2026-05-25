import json
import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float32, String, Int32, Bool

from .common import (
    bearing_deg,
    default_mission_file,
    haversine_m,
    resolve_mission_file,
    to_json,
)


class GpsGuidanceNode(Node):
    def __init__(self) -> None:
        super().__init__("gps_guidance_node")

        self.declare_parameter("mission_file", default_mission_file())
        self.declare_parameter("arrival_radius_m", 1.0)
        self.declare_parameter("use_route_lookahead", False)
        self.declare_parameter("route_lookahead_m", 2.5)
        self.declare_parameter("route_lookahead_cross_turns", True)

        self.mission_file = str(self.get_parameter("mission_file").value)
        self.arrival_radius_m = float(
            self.get_parameter("arrival_radius_m").value
        )
        self.use_route_lookahead = bool(
            self.get_parameter("use_route_lookahead").value
        )
        self.route_lookahead_m = max(
            0.1,
            float(self.get_parameter("route_lookahead_m").value),
        )
        self.route_lookahead_cross_turns = bool(
            self.get_parameter("route_lookahead_cross_turns").value
        )

        self.current_lat = None
        self.current_lon = None
        self.current_heading = None
        self.active_waypoint_index = 0
        self.mission_started = False
        self.waypoints = []

        self.target_bearing_pub = self.create_publisher(
            Float32,
            "/guidance/target_bearing_deg",
            10,
        )
        self.target_distance_pub = self.create_publisher(
            Float32,
            "/guidance/target_distance_m",
            10,
        )
        self.status_pub = self.create_publisher(String, "/guidance/status", 10)
        self.advance_pub = self.create_publisher(
            Int32,
            "/guidance/advance_waypoint",
            10,
        )

        self.create_subscription(
            NavSatFix,
            "/mavros/global_position/global",
            self.gps_cb,
            10,
        )
        self.create_subscription(
            Float32,
            "/mavros/global_position/compass_hdg",
            self.heading_cb,
            10,
        )
        self.create_subscription(
            Int32,
            "/mission/active_waypoint",
            self.wp_index_cb,
            10,
        )
        self.create_subscription(
            String,
            "/mission/waypoints",
            self.waypoints_cb,
            10,
        )
        self.create_subscription(Bool, "/mission/started", self.started_cb, 10)
        self.create_subscription(
            Bool,
            "/mission/waypoints_changed",
            self.waypoints_changed_cb,
            10,
        )

        self.timer = self.create_timer(0.1, self.loop)

        self.load_mission()

    def _local_origin(self):
        if not self.waypoints:
            return None
        return float(self.waypoints[0]["lat"]), float(self.waypoints[0]["lon"])

    def _to_local(self, lat: float, lon: float) -> tuple[float, float]:
        origin = self._local_origin()
        if origin is None:
            return 0.0, 0.0
        origin_lat, origin_lon = origin
        north = (lat - origin_lat) * 111_320.0
        lon_scale = max(111_320.0 * math.cos(math.radians(origin_lat)), 1e-6)
        east = (lon - origin_lon) * lon_scale
        return east, north

    def _bearing_to_local_target(
        self,
        target_east: float,
        target_north: float,
    ) -> float:
        cur_east, cur_north = self._to_local(self.current_lat, self.current_lon)
        de = target_east - cur_east
        dn = target_north - cur_north
        if abs(de) < 1e-6 and abs(dn) < 1e-6:
            return float(self.current_heading or 0.0)
        return (math.degrees(math.atan2(de, dn)) + 360.0) % 360.0

    def _route_lookahead_target(self):
        if len(self.waypoints) < 2:
            return None
        points = [
            self._to_local(float(wp["lat"]), float(wp["lon"]))
            for wp in self.waypoints
        ]
        cur = self._to_local(self.current_lat, self.current_lon)
        segment_index = max(
            0,
            min(self.active_waypoint_index - 1, len(points) - 2),
        )
        ax, ay = points[segment_index]
        bx, by = points[segment_index + 1]
        dx = bx - ax
        dy = by - ay
        seg_len = math.hypot(dx, dy)
        if seg_len < 1e-6:
            return points[segment_index + 1]

        projection = ((cur[0] - ax) * dx + (cur[1] - ay) * dy) / (
            seg_len * seg_len
        )
        projection = max(0.0, min(1.0, projection))
        remaining = projection * seg_len + self.route_lookahead_m
        if not self.route_lookahead_cross_turns:
            # Keskin dönüşlerde hedefi bir sonraki segmente taşımak köşeyi
            # kestirir. Bu modda aktif waypoint'e yaklaşmadan geçiş yok.
            remaining = min(remaining, seg_len)

        index = segment_index
        while index < len(points) - 1:
            sx, sy = points[index]
            ex, ey = points[index + 1]
            leg_len = math.hypot(ex - sx, ey - sy)
            if remaining <= leg_len or index == len(points) - 2:
                ratio = 0.0 if leg_len < 1e-6 else remaining / leg_len
                ratio = max(0.0, min(1.0, ratio))
                return sx + (ex - sx) * ratio, sy + (ey - sy) * ratio
            remaining -= leg_len
            index += 1
        return points[-1]

    def load_mission(self) -> None:
        path = resolve_mission_file(self.mission_file)
        if not path.exists():
            self.get_logger().error(f"Mission file not found: {path}")
            return
        try:
            data = json.loads(path.read_text(encoding="utf-8"))
            self.waypoints = data.get("waypoints", [])
            self.get_logger().info(
                f"Loaded {len(self.waypoints)} waypoint(s) from {path}"
            )
        except Exception as exc:
            self.get_logger().error(f"Failed to load mission file: {exc}")

    def gps_cb(self, msg: NavSatFix) -> None:
        self.current_lat = msg.latitude
        self.current_lon = msg.longitude

    def heading_cb(self, msg: Float32) -> None:
        self.current_heading = float(msg.data)

    def wp_index_cb(self, msg: Int32) -> None:
        self.active_waypoint_index = int(msg.data)

    def waypoints_cb(self, msg: String) -> None:
        try:
            data = json.loads(msg.data)
            new_waypoints = data.get("waypoints", [])
            if new_waypoints:
                self.waypoints = new_waypoints
                self.active_waypoint_index = 0  # Waypoint index'ini sıfırla
                self.get_logger().info(
                    f"Updated waypoints from mission_manager: {len(self.waypoints)} waypoint(s)"
                )
        except Exception as exc:
            self.get_logger().warning(f"Ignoring invalid waypoint message: {exc}")

    def started_cb(self, msg: Bool) -> None:
        self.mission_started = bool(msg.data)

    def waypoints_changed_cb(self, msg: Bool) -> None:
        if msg.data:
            # Waypointler değişti, güncellemeyi bekle
            self.get_logger().debug("Waypoints changed signal received")

    def loop(self) -> None:
        if self.current_lat is None or self.current_lon is None:
            return
        if not self.waypoints:
            self.get_logger().warning("No waypoints available")
            return
        if self.active_waypoint_index >= len(self.waypoints):
            self.get_logger().warning(
                f"Waypoint index {self.active_waypoint_index} out of range "
                f"({len(self.waypoints)} waypoints)"
            )
            return

        target = self.waypoints[self.active_waypoint_index]
        tgt_lat = float(target["lat"])
        tgt_lon = float(target["lon"])

        distance_m = haversine_m(
            self.current_lat,
            self.current_lon,
            tgt_lat,
            tgt_lon,
        )
        waypoint_bearing = bearing_deg(
            self.current_lat,
            self.current_lon,
            tgt_lat,
            tgt_lon,
        )
        bearing = waypoint_bearing
        lookahead_target = None
        if self.use_route_lookahead:
            lookahead_target = self._route_lookahead_target()
            if lookahead_target is not None:
                bearing = self._bearing_to_local_target(*lookahead_target)
        if self.active_waypoint_index > 0:
            prev_target = self.waypoints[self.active_waypoint_index - 1]
            leg_bearing = bearing_deg(
                float(prev_target["lat"]),
                float(prev_target["lon"]),
                tgt_lat,
                tgt_lon,
            )
        else:
            leg_bearing = waypoint_bearing

        upcoming_turn_angle = 0.0
        next_waypoint_index = None
        next_bearing = None
        if self.active_waypoint_index + 1 < len(self.waypoints):
            next_waypoint_index = self.active_waypoint_index + 1
            next_target = self.waypoints[next_waypoint_index]
            next_lat = float(next_target["lat"])
            next_lon = float(next_target["lon"])
            next_bearing = bearing_deg(tgt_lat, tgt_lon, next_lat, next_lon)
            upcoming_turn_angle = abs(
                (next_bearing - leg_bearing + 180.0) % 360.0 - 180.0
            )

        self.target_bearing_pub.publish(Float32(data=float(bearing)))
        self.target_distance_pub.publish(Float32(data=float(distance_m)))

        if self.mission_started and distance_m <= self.arrival_radius_m:
            self.advance_pub.publish(Int32(data=self.active_waypoint_index))

        self.status_pub.publish(
            String(
                data=to_json(
                    {
                        "active_waypoint_index": self.active_waypoint_index,
                        "target_lat": tgt_lat,
                        "target_lon": tgt_lon,
                        "target_bearing_deg": bearing,
                        "waypoint_bearing_deg": waypoint_bearing,
                        "leg_bearing_deg": leg_bearing,
                        "target_distance_m": distance_m,
                        "route_lookahead_enabled": self.use_route_lookahead,
                        "route_lookahead_m": self.route_lookahead_m,
                        "route_lookahead_cross_turns": (
                            self.route_lookahead_cross_turns
                        ),
                        "route_lookahead_target": lookahead_target,
                        "next_waypoint_index": next_waypoint_index,
                        "next_bearing_deg": next_bearing,
                        "upcoming_turn_angle_deg": upcoming_turn_angle,
                        "mission_started": self.mission_started,
                    }
                )
            )
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = GpsGuidanceNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
