import json
import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import String

from .common import bearing_deg, from_json, resolve_mission_file, to_json


class GeofenceMonitorNode(Node):
    """Track parkur boundary status and publish a return-to-course bearing."""

    def __init__(self) -> None:
        super().__init__("geofence_monitor_node")

        self.declare_parameter("enabled", True)
        self.declare_parameter("mission_file", "")
        self.declare_parameter("geofence_polygon_json", "")
        self.declare_parameter("generated_half_width_m", 7.0)
        self.declare_parameter("outside_grace_s", 1.0)
        self.declare_parameter("outside_penalty_s", 40.0)
        self.declare_parameter("return_speed_mps", 0.12)
        self.declare_parameter("publish_rate_hz", 2.0)

        self.enabled = bool(self.get_parameter("enabled").value)
        self.mission_file = str(self.get_parameter("mission_file").value)
        self.generated_half_width_m = float(
            self.get_parameter("generated_half_width_m").value
        )
        self.outside_grace_s = float(self.get_parameter("outside_grace_s").value)
        self.outside_penalty_s = float(
            self.get_parameter("outside_penalty_s").value
        )
        self.return_speed_mps = float(self.get_parameter("return_speed_mps").value)

        self.current_lat = None
        self.current_lon = None
        self.origin_lat = None
        self.origin_lon = None
        self.waypoints = []
        self.polygon_ll = []
        self.polygon_xy = []
        self.has_explicit_polygon = False
        self.outside_since_ts = None
        self.outside_event_count = 0
        self.last_status = {}

        self._load_configured_boundary()

        self.status_pub = self.create_publisher(String, "/geofence/status", 10)
        self.create_subscription(
            NavSatFix,
            "/mavros/global_position/global",
            self.gps_cb,
            10,
        )
        self.create_subscription(
            String,
            "/mission/waypoints",
            self.waypoints_cb,
            10,
        )

        period = 1.0 / max(float(self.get_parameter("publish_rate_hz").value), 0.1)
        self.timer = self.create_timer(period, self.loop)

    def _load_configured_boundary(self) -> None:
        polygon_text = str(self.get_parameter("geofence_polygon_json").value)
        if polygon_text.strip():
            try:
                self.polygon_ll = self._parse_polygon(json.loads(polygon_text))
                self.has_explicit_polygon = True
                return
            except Exception as exc:
                self.get_logger().warn(f"Ignoring invalid geofence polygon: {exc}")

        if not self.mission_file:
            return
        path = resolve_mission_file(self.mission_file)
        if not path.exists():
            return
        try:
            data = json.loads(path.read_text(encoding="utf-8"))
            self.waypoints = data.get("waypoints", [])
            for key in ("geofence", "boundary_polygon", "parkur_polygon"):
                if key in data:
                    self.polygon_ll = self._parse_polygon(data[key])
                    self.has_explicit_polygon = True
                    return
        except Exception as exc:
            self.get_logger().warn(f"Geofence mission load failed: {exc}")

    def _parse_polygon(self, items) -> list[tuple[float, float]]:
        polygon = []
        for item in items:
            polygon.append((float(item["lat"]), float(item["lon"])))
        if len(polygon) < 3:
            raise ValueError("geofence polygon must have at least 3 points")
        return polygon

    def _project(self, lat: float, lon: float) -> tuple[float, float]:
        if self.origin_lat is None or self.origin_lon is None:
            self.origin_lat = lat
            self.origin_lon = lon
        north = (lat - self.origin_lat) * 111_320.0
        east = (
            (lon - self.origin_lon)
            * 111_320.0
            * math.cos(math.radians(self.origin_lat))
        )
        return east, north

    def _unproject(self, east: float, north: float) -> tuple[float, float]:
        lat = self.origin_lat + north / 111_320.0
        lon_scale = max(
            111_320.0 * math.cos(math.radians(self.origin_lat)),
            1e-6,
        )
        lon = self.origin_lon + east / lon_scale
        return lat, lon

    def _build_generated_polygon(self) -> None:
        if len(self.waypoints) < 2:
            return
        points = [
            self._project(float(wp["lat"]), float(wp["lon"]))
            for wp in self.waypoints
        ]
        left_side = []
        right_side = []
        half_width = max(self.generated_half_width_m, 1.0)
        for index, (east, north) in enumerate(points):
            if index == 0:
                dx = points[1][0] - east
                dy = points[1][1] - north
            elif index == len(points) - 1:
                dx = east - points[index - 1][0]
                dy = north - points[index - 1][1]
            else:
                dx = points[index + 1][0] - points[index - 1][0]
                dy = points[index + 1][1] - points[index - 1][1]
            length = max(math.hypot(dx, dy), 1e-6)
            left_x = -dy / length
            left_y = dx / length
            left_side.append((east + left_x * half_width, north + left_y * half_width))
            right_side.append((east - left_x * half_width, north - left_y * half_width))
        self.polygon_xy = left_side + list(reversed(right_side))
        self.polygon_ll = [self._unproject(x, y) for x, y in self.polygon_xy]

    def _refresh_polygon_xy(self) -> None:
        if self.polygon_ll:
            self.polygon_xy = [self._project(lat, lon) for lat, lon in self.polygon_ll]
        elif self.waypoints:
            self._build_generated_polygon()

    def gps_cb(self, msg: NavSatFix) -> None:
        self.current_lat = msg.latitude
        self.current_lon = msg.longitude
        if self.origin_lat is None:
            self.origin_lat = self.current_lat
            self.origin_lon = self.current_lon
            self._refresh_polygon_xy()

    def waypoints_cb(self, msg: String) -> None:
        try:
            self.waypoints = from_json(msg.data).get("waypoints", [])
            if not self.has_explicit_polygon:
                self.polygon_ll = []
                self.polygon_xy = []
                self._refresh_polygon_xy()
        except Exception as exc:
            self.get_logger().warn(f"Ignoring invalid waypoint geofence input: {exc}")

    def _point_in_polygon(self, x: float, y: float) -> bool:
        inside = False
        points = self.polygon_xy
        j = len(points) - 1
        for i, (xi, yi) in enumerate(points):
            xj, yj = points[j]
            intersects = ((yi > y) != (yj > y)) and (
                x < (xj - xi) * (y - yi) / max(yj - yi, 1e-9) + xi
            )
            if intersects:
                inside = not inside
            j = i
        return inside

    def _closest_boundary_point(self, x: float, y: float) -> tuple[float, float, float]:
        best = None
        points = self.polygon_xy
        for index, a in enumerate(points):
            b = points[(index + 1) % len(points)]
            ax, ay = a
            bx, by = b
            dx = bx - ax
            dy = by - ay
            length2 = max(dx * dx + dy * dy, 1e-9)
            t = ((x - ax) * dx + (y - ay) * dy) / length2
            t = max(0.0, min(1.0, t))
            px = ax + t * dx
            py = ay + t * dy
            dist = math.hypot(x - px, y - py)
            if best is None or dist < best[2]:
                best = (px, py, dist)
        return best if best is not None else (x, y, 0.0)

    def _polygon_centroid(self) -> tuple[float, float]:
        if not self.polygon_xy:
            return 0.0, 0.0
        x = sum(p[0] for p in self.polygon_xy) / len(self.polygon_xy)
        y = sum(p[1] for p in self.polygon_xy) / len(self.polygon_xy)
        return x, y

    def loop(self) -> None:
        now = self.get_clock().now().nanoseconds / 1e9
        status = {
            "enabled": self.enabled,
            "has_polygon": len(self.polygon_xy) >= 3,
            "outside": False,
            "outside_duration_s": 0.0,
            "outside_event_count": self.outside_event_count,
            "penalty_equivalent_exit_count": self.outside_event_count,
            "return_bearing_deg": None,
            "return_speed_mps": self.return_speed_mps,
            "distance_to_boundary_m": None,
            "reason": "disabled" if not self.enabled else "waiting_for_gps_or_polygon",
        }
        if (
            not self.enabled
            or self.current_lat is None
            or self.current_lon is None
            or len(self.polygon_xy) < 3
        ):
            self.status_pub.publish(String(data=to_json(status)))
            return

        x, y = self._project(self.current_lat, self.current_lon)
        inside = self._point_in_polygon(x, y)
        closest_x, closest_y, distance = self._closest_boundary_point(x, y)
        target_x, target_y = closest_x, closest_y
        if not inside:
            center_x, center_y = self._polygon_centroid()
            target_x = closest_x + (center_x - closest_x) * 0.25
            target_y = closest_y + (center_y - closest_y) * 0.25
            if self.outside_since_ts is None:
                self.outside_since_ts = now
                self.outside_event_count += 1
        else:
            self.outside_since_ts = None

        outside_duration = 0.0
        if self.outside_since_ts is not None:
            outside_duration = max(0.0, now - self.outside_since_ts)
        penalty_count = self.outside_event_count
        if outside_duration > self.outside_penalty_s:
            penalty_count += 1

        target_lat, target_lon = self._unproject(target_x, target_y)
        return_bearing = bearing_deg(
            self.current_lat,
            self.current_lon,
            target_lat,
            target_lon,
        )
        status.update(
            {
                "outside": (not inside) and outside_duration >= self.outside_grace_s,
                "outside_raw": not inside,
                "outside_duration_s": outside_duration,
                "outside_event_count": self.outside_event_count,
                "penalty_equivalent_exit_count": penalty_count,
                "return_bearing_deg": return_bearing,
                "return_target_lat": target_lat,
                "return_target_lon": target_lon,
                "distance_to_boundary_m": distance,
                "reason": "outside_return_to_course" if not inside else "inside",
            }
        )
        self.last_status = status
        self.status_pub.publish(String(data=to_json(status)))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = GeofenceMonitorNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
