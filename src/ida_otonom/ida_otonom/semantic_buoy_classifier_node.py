import math

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from .common import angular_distance_deg, from_json, now_ts, to_json


class SemanticBuoyClassifierNode(Node):
    def __init__(self) -> None:
        super().__init__("semantic_buoy_classifier_node")

        self.declare_parameter("course_class_names", ["course_buoy", "buoy"])
        self.declare_parameter(
            "obstacle_class_names",
            ["obstacle_buoy", "obstacle", "yellow_buoy"],
        )
        self.declare_parameter("target_class_names", ["target_buoy"])
        self.declare_parameter("course_hue_tolerance_deg", 35.0)
        self.declare_parameter("strong_color_diff_deg", 55.0)
        self.declare_parameter("min_confidence", 0.35)
        self.declare_parameter("require_sensor_validation", False)
        self.declare_parameter(
            "accepted_validation_statuses",
            ["validated", "lidar_only"],
        )

        self.course_class_names = set(
            str(v) for v in self.get_parameter("course_class_names").value
        )
        self.obstacle_class_names = set(
            str(v) for v in self.get_parameter("obstacle_class_names").value
        )
        self.target_class_names = set(
            str(v) for v in self.get_parameter("target_class_names").value
        )
        self.course_hue_tolerance_deg = float(
            self.get_parameter("course_hue_tolerance_deg").value
        )
        self.strong_color_diff_deg = float(
            self.get_parameter("strong_color_diff_deg").value
        )
        self.min_confidence = float(self.get_parameter("min_confidence").value)
        self.require_sensor_validation = bool(
            self.get_parameter("require_sensor_validation").value
        )
        self.accepted_validation_statuses = set(
            str(v) for v in self.get_parameter("accepted_validation_statuses").value
        )

        self.course_profile = None
        self.pub = self.create_publisher(
            String,
            "/perception/semantic_buoys",
            10,
        )
        self.create_subscription(
            String,
            "/perception/course_memory",
            self.memory_cb,
            10,
        )
        self.create_subscription(
            String,
            "/perception/buoy_detections",
            self.detections_cb,
            10,
        )

    def memory_cb(self, msg: String) -> None:
        try:
            payload = from_json(msg.data)
            profile = payload.get("profile", {})
            if profile.get("learned"):
                self.course_profile = profile
        except Exception:
            self.course_profile = None

    def _color_similarity(self, detection: dict):
        if self.course_profile is None:
            return None
        hsv = detection.get("mean_hsv")
        if not hsv or len(hsv) < 3:
            return None
        hue = float(hsv[0]) % 360.0
        mean_hue = float(self.course_profile.get("mean_hue_deg", hue))
        return angular_distance_deg(hue, mean_hue)

    def _position_fields(self, detection: dict) -> dict:
        range_m = detection.get("range_m")
        bearing_deg = detection.get("bearing_deg")
        if range_m is None or bearing_deg is None:
            return {}
        range_m = float(range_m)
        bearing_deg = float(bearing_deg)
        bearing_rad = math.radians(bearing_deg)
        return {
            "forward_m": range_m * math.cos(bearing_rad),
            "left_m": range_m * math.sin(bearing_rad),
        }

    def _classify(self, detection: dict) -> dict:
        class_name = str(detection.get("class_name", ""))
        confidence = float(detection.get("confidence", 0.0))
        hue_diff = self._color_similarity(detection)

        sensor_validation = detection.get("sensor_validation") or {}
        validation_status = str(sensor_validation.get("status", ""))
        if (
            self.require_sensor_validation
            and validation_status not in self.accepted_validation_statuses
        ):
            result = dict(detection)
            result.update(self._position_fields(detection))
            result["semantic"] = "invalid_sensor_fusion"
            result["classification_reason"] = "sensor_validation_failed"
            result["course_hue_diff_deg"] = hue_diff
            return result

        semantic = "unknown"
        reason = "low_confidence" if confidence < self.min_confidence else ""
        if confidence >= self.min_confidence:
            if class_name in self.obstacle_class_names:
                semantic = "obstacle_candidate"
                reason = "yolo_obstacle_class"
            elif class_name in self.target_class_names:
                semantic = "target_candidate"
                reason = "yolo_target_class"
            elif class_name in self.course_class_names:
                semantic = "course_boundary_candidate"
                reason = "yolo_course_class"

            if hue_diff is not None:
                if hue_diff <= self.course_hue_tolerance_deg:
                    if semantic in ("unknown", "course_boundary_candidate"):
                        semantic = "course_boundary_candidate"
                        reason = "course_color_memory_match"
                elif hue_diff >= self.strong_color_diff_deg:
                    if semantic != "target_candidate":
                        semantic = "obstacle_candidate"
                        reason = "different_from_course_color_memory"

        result = dict(detection)
        result.update(self._position_fields(detection))
        result["semantic"] = semantic
        result["classification_reason"] = reason
        result["course_hue_diff_deg"] = hue_diff
        return result

    def detections_cb(self, msg: String) -> None:
        try:
            payload = from_json(msg.data)
        except Exception:
            return
        classified = [
            self._classify(det) for det in payload.get("detections", [])
        ]
        self.pub.publish(
            String(
                data=to_json(
                    {
                        "timestamp": now_ts(),
                        "source_timestamp": payload.get("timestamp"),
                        "course_profile_learned": self.course_profile
                        is not None,
                        "buoys": classified,
                    }
                )
            )
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SemanticBuoyClassifierNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
