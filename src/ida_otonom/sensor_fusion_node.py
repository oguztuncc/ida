import json
from typing import Dict, Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from .ida_otonom.common import to_json, now_ts


class SensorFusionNode(Node):
    def __init__(self) -> None:
        super().__init__("sensor_fusion_node")

        self.latest_detections: Optional[Dict] = None
        self.latest_lidar: Optional[Dict] = None

        self.create_subscription(
            String,
            "/perception/detections",
            self.det_cb,
            10,
        )
        self.create_subscription(
            String,
            "/perception/lidar_summary",
            self.lidar_cb,
            10,
        )

        self.gate_pub = self.create_publisher(
            String,
            "/fusion/gate_candidate",
            10,
        )
        self.world_pub = self.create_publisher(
            String,
            "/fusion/world_state",
            10,
        )

        self.timer = self.create_timer(0.1, self.loop)

    def det_cb(self, msg: String) -> None:
        self.latest_detections = json.loads(msg.data)

    def lidar_cb(self, msg: String) -> None:
        self.latest_lidar = json.loads(msg.data)

    def loop(self) -> None:
        if self.latest_detections is None or self.latest_lidar is None:
            return

        detections = self.latest_detections.get("detections", [])

        # Burada class_name == "parkur_duba" gibi ortak isim standardı kullan
        buoy_boxes = [
            d for d in detections
            if d["class_name"] in {"parkur", "duba", "parkur_duba"}
        ]
        buoy_boxes = sorted(buoy_boxes, key=lambda x: x["cx"])

        gate_candidate = None
        if len(buoy_boxes) >= 2:
            left = buoy_boxes[0]
            right = buoy_boxes[-1]
            if right["cx"] - left["cx"] > 80:
                gate_candidate = {
                    "timestamp": now_ts(),
                    "left_x": left["cx"],
                    "right_x": right["cx"],
                    "center_x": (left["cx"] + right["cx"]) / 2.0,
                    "confidence": min(left["confidence"], right["confidence"]),
                }

        world_state = {
            "timestamp": now_ts(),
            "collision_imminent": self.latest_lidar["collision_imminent"],
            "best_free_angle_deg": self.latest_lidar["best_free_angle_deg"],
            "front_clearance_m": self.latest_lidar["front_clearance_m"],
            "gate_candidate": gate_candidate,
            "detections_count": len(detections),
        }

        self.world_pub.publish(String(data=to_json(world_state)))
        if gate_candidate is not None:
            self.gate_pub.publish(String(data=to_json(gate_candidate)))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SensorFusionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
