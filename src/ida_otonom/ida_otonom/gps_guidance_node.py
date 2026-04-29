import json
from pathlib import Path

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float32, String, Int32, Bool

from .common import haversine_m, bearing_deg, to_json


class GpsGuidanceNode(Node):
    def __init__(self) -> None:
        super().__init__("gps_guidance_node")

        self.declare_parameter("mission_file", "/home/talay/Documents/ida/src/ida_otonom/ida_otonom/missions/mission.json")
        self.declare_parameter("arrival_radius_m", 3.0)

        self.mission_file = self.get_parameter("mission_file").value
        self.arrival_radius_m = float(self.get_parameter("arrival_radius_m").value)

        self.current_lat = None
        self.current_lon = None
        self.current_heading = None
        self.active_waypoint_index = 0
        self.waypoints = []

        self.target_bearing_pub = self.create_publisher(Float32, "/guidance/target_bearing_deg", 10)
        self.target_distance_pub = self.create_publisher(Float32, "/guidance/target_distance_m", 10)
        self.status_pub = self.create_publisher(String, "/guidance/status", 10)
        self.advance_pub = self.create_publisher(Bool, "/guidance/advance_waypoint", 10)

        self.create_subscription(NavSatFix, "/mavros/global_position/global", self.gps_cb, 10)
        self.create_subscription(Float32, "/mavros/global_position/compass_hdg", self.heading_cb, 10)
        self.create_subscription(Int32, "/mission/active_waypoint", self.wp_index_cb, 10)

        self.timer = self.create_timer(0.1, self.loop)

        self.load_mission()

    def load_mission(self) -> None:
        path = Path(self.mission_file)
        if not path.exists():
            self.get_logger().error(f"Mission file not found: {self.mission_file}")
            return
        try:
            data = json.loads(path.read_text(encoding="utf-8"))
            self.waypoints = data.get("waypoints", [])
        except Exception as exc:
            self.get_logger().error(f"Failed to load mission file: {exc}")

    def gps_cb(self, msg: NavSatFix) -> None:
        self.current_lat = msg.latitude
        self.current_lon = msg.longitude

    def heading_cb(self, msg: Float32) -> None:
        self.current_heading = float(msg.data)

    def wp_index_cb(self, msg: Int32) -> None:
        self.active_waypoint_index = int(msg.data)

    def loop(self) -> None:
        if self.current_lat is None or self.current_lon is None:
            return
        if not self.waypoints:
            return
        if self.active_waypoint_index >= len(self.waypoints):
            return

        target = self.waypoints[self.active_waypoint_index]
        tgt_lat = float(target["lat"])
        tgt_lon = float(target["lon"])

        distance_m = haversine_m(self.current_lat, self.current_lon, tgt_lat, tgt_lon)
        bearing = bearing_deg(self.current_lat, self.current_lon, tgt_lat, tgt_lon)

        self.target_bearing_pub.publish(Float32(data=float(bearing)))
        self.target_distance_pub.publish(Float32(data=float(distance_m)))

        if distance_m <= self.arrival_radius_m:
            self.advance_pub.publish(Bool(data=True))
        else:
            self.advance_pub.publish(Bool(data=False))

        self.status_pub.publish(
            String(
                data=to_json(
                    {
                        "active_waypoint_index": self.active_waypoint_index,
                        "target_lat": tgt_lat,
                        "target_lon": tgt_lon,
                        "target_bearing_deg": bearing,
                        "target_distance_m": distance_m,
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