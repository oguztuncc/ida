from pathlib import Path
import json

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int32, Bool

from .common import to_json


class MissionManagerNode(Node):
    def __init__(self) -> None:
        super().__init__("mission_manager_node")

        self.declare_parameter("mission_file", "/home/jetson/missions/mission.json")

        self.mission_file = self.get_parameter("mission_file").value
        self.active_waypoint_index = 0
        self.mission_loaded = False
        self.mission_completed = False
        self.waypoints = []

        self.active_wp_pub = self.create_publisher(Int32, "/mission/active_waypoint", 10)
        self.status_pub = self.create_publisher(String, "/mission/status", 10)
        self.done_pub = self.create_publisher(Bool, "/mission/completed", 10)
        self.waypoints_pub = self.create_publisher(String, "/mission/waypoints", 10)

        self.create_subscription(Bool, "/guidance/advance_waypoint", self.advance_cb, 10)

        self.timer = self.create_timer(1.0, self.loop)

        self.load_mission()

    def load_mission(self) -> None:
        path = Path(self.mission_file)
        if not path.exists():
            self.get_logger().error(f"Mission file not found: {self.mission_file}")
            return

        try:
            data = json.loads(path.read_text(encoding="utf-8"))
            self.waypoints = data.get("waypoints", [])
            if not self.waypoints:
                raise ValueError("No waypoints found in mission file")
            self.mission_loaded = True
            self.mission_completed = False
            self.active_waypoint_index = 0
            self.get_logger().info(f"Loaded {len(self.waypoints)} waypoint(s)")
        except Exception as exc:
            self.get_logger().error(f"Mission load failed: {exc}")

    def advance_cb(self, msg: Bool) -> None:
        if not msg.data or not self.mission_loaded or self.mission_completed:
            return

        self.active_waypoint_index += 1
        if self.active_waypoint_index >= len(self.waypoints):
            self.active_waypoint_index = len(self.waypoints) - 1
            self.mission_completed = True
            self.get_logger().info("Mission completed")

    def loop(self) -> None:
        self.active_wp_pub.publish(Int32(data=self.active_waypoint_index))
        self.done_pub.publish(Bool(data=self.mission_completed))
        self.waypoints_pub.publish(String(data=to_json({"waypoints": self.waypoints})))
        self.status_pub.publish(
            String(
                data=to_json(
                    {
                        "mission_loaded": self.mission_loaded,
                        "mission_completed": self.mission_completed,
                        "active_waypoint_index": self.active_waypoint_index,
                        "waypoint_count": len(self.waypoints),
                    }
                )
            )
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MissionManagerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()