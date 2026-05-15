import math
import os
from typing import List, Optional, Tuple

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String

from .common import now_ts, to_json

try:
    from cv_bridge import CvBridge
except Exception:  # pragma: no cover - depends on ROS image stack
    CvBridge = None

try:
    from ultralytics import YOLO
except Exception:  # pragma: no cover - optional until model install
    YOLO = None


class BuoyDetectorNode(Node):
    def __init__(self) -> None:
        super().__init__("buoy_detector_node")

        self.declare_parameter("model_path", "models/buoy_yolo.pt")
        self.declare_parameter("confidence_threshold", 0.45)
        self.declare_parameter("camera_frame_id", "realsense_color")
        self.declare_parameter("depth_sample_radius_px", 4)
        self.declare_parameter("assumed_horizontal_fov_deg", 87.0)
        self.declare_parameter("publish_empty_detections", True)
        self.declare_parameter("enable_yolo", True)
        self.declare_parameter("detection_topic", "/perception/buoy_detections")
        self.declare_parameter("color_image_topic", "/camera/camera/color/image_raw")
        self.declare_parameter(
            "depth_image_topic",
            "/camera/camera/aligned_depth_to_color/image_raw",
        )
        self.declare_parameter(
            "camera_info_topic",
            "/camera/camera/color/camera_info",
        )

        self.model_path = str(self.get_parameter("model_path").value)
        self.confidence_threshold = float(
            self.get_parameter("confidence_threshold").value
        )
        self.camera_frame_id = str(
            self.get_parameter("camera_frame_id").value
        )
        self.depth_sample_radius_px = int(
            self.get_parameter("depth_sample_radius_px").value
        )
        self.assumed_horizontal_fov_deg = float(
            self.get_parameter("assumed_horizontal_fov_deg").value
        )
        self.publish_empty_detections = bool(
            self.get_parameter("publish_empty_detections").value
        )
        self.enable_yolo = bool(self.get_parameter("enable_yolo").value)

        self.bridge = CvBridge() if CvBridge is not None else None
        self.model = self._load_model()
        self.latest_depth = None
        self.fx = None
        self.cx = None

        self.pub = self.create_publisher(
            String,
            str(self.get_parameter("detection_topic").value),
            10,
        )
        self.create_subscription(
            Image,
            str(self.get_parameter("color_image_topic").value),
            self.color_cb,
            10,
        )
        self.create_subscription(
            Image,
            str(self.get_parameter("depth_image_topic").value),
            self.depth_cb,
            10,
        )
        self.create_subscription(
            CameraInfo,
            str(self.get_parameter("camera_info_topic").value),
            self.camera_info_cb,
            10,
        )

    def _load_model(self):
        if not self.enable_yolo:
            self.get_logger().warn("YOLO disabled; publishing empty detections")
            return None
        if YOLO is None:
            self.get_logger().warn(
                "ultralytics is not installed; buoy detector stays in "
                "safe empty-detection mode"
            )
            return None
        if not os.path.exists(os.path.expanduser(self.model_path)):
            self.get_logger().warn(
                f"YOLO model not found at {self.model_path}; "
                "replace this path when the trained model is ready"
            )
            return None
        try:
            model = YOLO(os.path.expanduser(self.model_path))
            self.get_logger().info(f"Loaded YOLO model: {self.model_path}")
            return model
        except Exception as exc:
            self.get_logger().error(f"YOLO model load failed: {exc}")
            return None

    def camera_info_cb(self, msg: CameraInfo) -> None:
        if len(msg.k) >= 6 and msg.k[0] > 0.0:
            self.fx = float(msg.k[0])
            self.cx = float(msg.k[2])

    def depth_cb(self, msg: Image) -> None:
        if self.bridge is None:
            return
        try:
            self.latest_depth = self.bridge.imgmsg_to_cv2(
                msg,
                desired_encoding="passthrough",
            )
        except Exception as exc:
            self.get_logger().warn(f"Depth conversion failed: {exc}")

    def _range_at(self, cx_px: float, cy_px: float) -> Optional[float]:
        if self.latest_depth is None:
            return None
        h, w = self.latest_depth.shape[:2]
        x = int(round(cx_px))
        y = int(round(cy_px))
        radius = max(1, self.depth_sample_radius_px)
        x0 = max(0, x - radius)
        x1 = min(w, x + radius + 1)
        y0 = max(0, y - radius)
        y1 = min(h, y + radius + 1)
        roi = self.latest_depth[y0:y1, x0:x1].astype(np.float32)
        vals = roi[np.isfinite(roi)]
        vals = vals[vals > 0.0]
        if vals.size == 0:
            return None
        median = float(np.median(vals))
        if median > 50.0:
            median /= 1000.0
        return median

    def _bearing_for_pixel(self, cx_px: float, image_width: int) -> float:
        # Positive bearing means port/left to match semantic left_m fields.
        if self.fx is not None and self.cx is not None:
            return math.degrees(math.atan2(self.cx - cx_px, self.fx))
        normalized = (image_width / 2.0 - cx_px) / max(image_width, 1)
        return normalized * self.assumed_horizontal_fov_deg

    def _mean_hsv(
        self,
        frame_bgr,
        bbox: Tuple[int, int, int, int],
    ) -> Optional[Tuple[float, float, float]]:
        x1, y1, x2, y2 = bbox
        h, w = frame_bgr.shape[:2]
        x1 = max(0, min(w - 1, x1))
        x2 = max(0, min(w, x2))
        y1 = max(0, min(h - 1, y1))
        y2 = max(0, min(h, y2))
        if x2 <= x1 or y2 <= y1:
            return None
        roi = frame_bgr[y1:y2, x1:x2]
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        mask = hsv[:, :, 1] > 40
        if not np.any(mask):
            return None
        selected = hsv[mask].astype(np.float32)
        return (
            float(np.mean(selected[:, 0]) * 2.0),
            float(np.mean(selected[:, 1]) / 255.0),
            float(np.mean(selected[:, 2]) / 255.0),
        )

    def _run_yolo(self, frame_bgr) -> List[dict]:
        if self.model is None:
            return []
        results = self.model.predict(
            source=frame_bgr,
            conf=self.confidence_threshold,
            verbose=False,
        )
        detections = []
        image_h, image_w = frame_bgr.shape[:2]
        for result in results:
            names = getattr(result, "names", {})
            boxes = getattr(result, "boxes", None)
            if boxes is None:
                continue
            for idx, box in enumerate(boxes):
                conf = float(box.conf[0])
                if conf < self.confidence_threshold:
                    continue
                cls_id = int(box.cls[0])
                class_name = str(names.get(cls_id, cls_id))
                x1, y1, x2, y2 = [float(v) for v in box.xyxy[0]]
                center_x = (x1 + x2) / 2.0
                center_y = (y1 + y2) / 2.0
                bbox_i = (
                    int(round(x1)),
                    int(round(y1)),
                    int(round(x2)),
                    int(round(y2)),
                )
                hsv = self._mean_hsv(frame_bgr, bbox_i)
                range_m = self._range_at(center_x, center_y)
                bearing = self._bearing_for_pixel(center_x, image_w)
                detections.append(
                    {
                        "id": f"{class_name}_{idx}",
                        "class_name": class_name,
                        "confidence": conf,
                        "bbox_xyxy": [x1, y1, x2, y2],
                        "center_px": [center_x, center_y],
                        "range_m": range_m,
                        "bearing_deg": bearing,
                        "mean_hsv": hsv,
                        "image_size": [image_w, image_h],
                    }
                )
        return detections

    def color_cb(self, msg: Image) -> None:
        if self.bridge is None:
            if self.publish_empty_detections:
                self._publish([])
            return
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as exc:
            self.get_logger().warn(f"Color conversion failed: {exc}")
            return

        detections = self._run_yolo(frame)
        if detections or self.publish_empty_detections:
            self._publish(detections, frame_id=msg.header.frame_id)

    def _publish(self, detections: List[dict], frame_id: str = "") -> None:
        payload = {
            "timestamp": now_ts(),
            "frame_id": frame_id or self.camera_frame_id,
            "model_path": self.model_path,
            "model_loaded": self.model is not None,
            "detections": detections,
        }
        self.pub.publish(String(data=to_json(payload)))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = BuoyDetectorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
