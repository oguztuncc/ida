import math
from typing import Optional, Tuple

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, String

from .common import from_json, now_ts, to_json


def hue_distance_deg(a: float, b: float) -> float:
    return abs((a - b + 180.0) % 360.0 - 180.0)


class CourseMemoryNode(Node):
    def __init__(self) -> None:
        super().__init__("course_memory_node")

        self.declare_parameter("learn_until_waypoint_index", 0)
        self.declare_parameter("min_samples", 8)
        self.declare_parameter("min_detection_confidence", 0.45)
        self.declare_parameter("course_class_names", ["course_buoy", "buoy"])
        self.declare_parameter("max_hue_std_deg", 35.0)
        self.declare_parameter("publish_rate_hz", 2.0)

        self.learn_until_waypoint_index = int(
            self.get_parameter("learn_until_waypoint_index").value
        )
        self.min_samples = int(self.get_parameter("min_samples").value)
        self.min_detection_confidence = float(
            self.get_parameter("min_detection_confidence").value
        )
        self.course_class_names = set(
            str(v) for v in self.get_parameter("course_class_names").value
        )
        self.max_hue_std_deg = float(
            self.get_parameter("max_hue_std_deg").value
        )

        self.active_waypoint_index = 0
        self.samples = []
        self.profile = None

        self.pub = self.create_publisher(
            String,
            "/perception/course_memory",
            10,
        )
        self.create_subscription(
            String,
            "/perception/buoy_detections",
            self.detections_cb,
            10,
        )
        self.create_subscription(
            Int32,
            "/mission/active_waypoint",
            self.wp_cb,
            10,
        )

        publish_rate_hz = float(self.get_parameter("publish_rate_hz").value)
        self.timer = self.create_timer(
            1.0 / max(publish_rate_hz, 0.1),
            self.publish_profile,
        )

    def wp_cb(self, msg: Int32) -> None:
        self.active_waypoint_index = int(msg.data)

    def _is_learning_phase(self) -> bool:
        return self.active_waypoint_index <= self.learn_until_waypoint_index

    def _usable_hsv(self, detection: dict) -> Optional[Tuple[float, float, float]]:
        hsv = detection.get("mean_hsv")
        if not hsv or len(hsv) < 3:
            return None
        hue = float(hsv[0]) % 360.0
        sat = float(hsv[1])
        val = float(hsv[2])
        if sat < 0.12 or val < 0.08:
            return None
        return hue, sat, val

    def detections_cb(self, msg: String) -> None:
        if not self._is_learning_phase():
            return
        try:
            payload = from_json(msg.data)
        except Exception:
            return

        for detection in payload.get("detections", []):
            class_name = str(detection.get("class_name", ""))
            confidence = float(detection.get("confidence", 0.0))
            if (
                self.course_class_names
                and class_name not in self.course_class_names
            ):
                continue
            if confidence < self.min_detection_confidence:
                continue
            hsv = self._usable_hsv(detection)
            if hsv is not None:
                self.samples.append(hsv)

        if len(self.samples) >= self.min_samples:
            self.profile = self._build_profile()

    def _build_profile(self) -> dict:
        hues = [math.radians(h) for h, _, _ in self.samples]
        mean_sin = sum(math.sin(h) for h in hues) / len(hues)
        mean_cos = sum(math.cos(h) for h in hues) / len(hues)
        mean_hue = math.degrees(math.atan2(mean_sin, mean_cos)) % 360.0
        distances = [
            hue_distance_deg(hue, mean_hue) for hue, _, _ in self.samples
        ]
        hue_std = math.sqrt(sum(d * d for d in distances) / len(distances))
        mean_sat = sum(s for _, s, _ in self.samples) / len(self.samples)
        mean_val = sum(v for _, _, v in self.samples) / len(self.samples)
        return {
            "learned": hue_std <= self.max_hue_std_deg,
            "sample_count": len(self.samples),
            "mean_hue_deg": mean_hue,
            "hue_std_deg": hue_std,
            "mean_saturation": mean_sat,
            "mean_value": mean_val,
            "max_hue_std_deg": self.max_hue_std_deg,
        }

    def publish_profile(self) -> None:
        profile = self.profile or {
            "learned": False,
            "sample_count": len(self.samples),
        }
        payload = {
            "timestamp": now_ts(),
            "learning_phase": self._is_learning_phase(),
            "active_waypoint_index": self.active_waypoint_index,
            "profile": profile,
        }
        self.pub.publish(String(data=to_json(payload)))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CourseMemoryNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
