import math
from statistics import median

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String

from .common import from_json, normalize_angle_deg, now_ts, to_json


class SensorCrossValidatorNode(Node):
    def __init__(self) -> None:
        super().__init__("sensor_cross_validator_node")

        self.declare_parameter("input_topic", "/perception/buoy_detections_raw")
        self.declare_parameter("output_topic", "/perception/buoy_detections")
        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("scan_timeout_s", 0.7)
        self.declare_parameter("bearing_window_deg", 5.0)
        self.declare_parameter("range_tolerance_m", 0.75)
        self.declare_parameter("relative_range_tolerance", 0.25)
        self.declare_parameter("allow_lidar_only", True)
        self.declare_parameter("min_valid_range_m", 0.10)

        self.scan_timeout_s = float(self.get_parameter("scan_timeout_s").value)
        self.bearing_window_deg = float(
            self.get_parameter("bearing_window_deg").value
        )
        self.range_tolerance_m = float(
            self.get_parameter("range_tolerance_m").value
        )
        self.relative_range_tolerance = float(
            self.get_parameter("relative_range_tolerance").value
        )
        self.allow_lidar_only = bool(self.get_parameter("allow_lidar_only").value)
        self.min_valid_range_m = float(
            self.get_parameter("min_valid_range_m").value
        )

        self.latest_scan = None
        self.scan_ts = 0.0

        self.pub = self.create_publisher(
            String,
            str(self.get_parameter("output_topic").value),
            10,
        )
        self.create_subscription(
            LaserScan,
            str(self.get_parameter("scan_topic").value),
            self.scan_cb,
            10,
        )
        self.create_subscription(
            String,
            str(self.get_parameter("input_topic").value),
            self.detections_cb,
            10,
        )

    def scan_cb(self, msg: LaserScan) -> None:
        self.latest_scan = msg
        self.scan_ts = now_ts()

    def _scan_fresh(self) -> bool:
        return (
            self.latest_scan is not None
            and now_ts() - self.scan_ts <= self.scan_timeout_s
        )

    def _lidar_range_at_bearing(self, bearing_deg: float) -> float | None:
        scan = self.latest_scan
        if scan is None or not scan.ranges:
            return None

        ranges = []
        angle = scan.angle_min
        for distance in scan.ranges:
            sample_bearing = normalize_angle_deg(math.degrees(angle))
            bearing_error = abs(
                normalize_angle_deg(sample_bearing - bearing_deg)
            )
            if bearing_error <= self.bearing_window_deg:
                if (
                    math.isfinite(distance)
                    and max(scan.range_min, self.min_valid_range_m)
                    <= distance
                    <= scan.range_max
                ):
                    ranges.append(float(distance))
            angle += scan.angle_increment

        if not ranges:
            return None
        return float(median(ranges))

    def _coerce_range(self, value) -> float | None:
        if value is None:
            return None
        try:
            value = float(value)
        except (TypeError, ValueError):
            return None
        if not math.isfinite(value) or value < self.min_valid_range_m:
            return None
        return value

    def _validate_detection(self, detection: dict) -> dict:
        out = dict(detection)
        depth_range = self._coerce_range(detection.get("range_m"))
        bearing = detection.get("bearing_deg")
        try:
            bearing = float(bearing)
        except (TypeError, ValueError):
            bearing = None

        lidar_range = None
        if bearing is not None and self._scan_fresh():
            lidar_range = self._lidar_range_at_bearing(bearing)

        status = "missing_bearing" if bearing is None else "unvalidated_no_scan"
        valid = False
        range_delta = None
        fused_range = depth_range
        tolerance = None

        if bearing is not None and self._scan_fresh():
            if depth_range is not None and lidar_range is not None:
                range_delta = abs(depth_range - lidar_range)
                tolerance = max(
                    self.range_tolerance_m,
                    depth_range * self.relative_range_tolerance,
                )
                valid = range_delta <= tolerance
                status = "validated" if valid else "range_mismatch"
                if valid:
                    fused_range = (depth_range + lidar_range) / 2.0
            elif (
                depth_range is None
                and lidar_range is not None
                and self.allow_lidar_only
            ):
                valid = True
                status = "lidar_only"
                fused_range = lidar_range
            elif depth_range is None:
                status = "missing_depth"
            else:
                status = "missing_lidar"

        confidence = float(out.get("confidence", 0.0) or 0.0)
        if status == "validated" and tolerance and range_delta is not None:
            margin = max(0.0, 1.0 - range_delta / max(tolerance, 1e-6))
            confidence *= 0.75 + 0.25 * margin
        elif status == "lidar_only":
            confidence *= 0.70
        elif status not in ("unvalidated_no_scan", "missing_bearing"):
            confidence *= 0.25

        out["confidence"] = max(0.0, min(confidence, 1.0))
        out["range_m_depth"] = depth_range
        out["range_m_lidar"] = lidar_range
        out["range_m_fused"] = fused_range
        if fused_range is not None:
            out["range_m"] = fused_range
        out["sensor_validation"] = {
            "status": status,
            "valid": valid,
            "depth_range_m": depth_range,
            "lidar_range_m": lidar_range,
            "range_delta_m": range_delta,
            "range_tolerance_m": tolerance,
            "bearing_window_deg": self.bearing_window_deg,
            "source": "realsense_depth_lidar",
        }
        return out

    def detections_cb(self, msg: String) -> None:
        try:
            payload = from_json(msg.data)
        except Exception as exc:
            self.get_logger().warn(f"Ignoring invalid detection payload: {exc}")
            return

        detections = [
            self._validate_detection(det)
            for det in payload.get("detections", [])
        ]
        source_timestamp = payload.get("timestamp")
        payload = dict(payload)
        payload["timestamp"] = now_ts()
        payload["source_timestamp"] = source_timestamp
        payload["sensor_cross_validation"] = {
            "enabled": True,
            "scan_fresh": self._scan_fresh(),
            "scan_age_s": None
            if self.latest_scan is None
            else now_ts() - self.scan_ts,
        }
        payload["detections"] = detections
        self.pub.publish(String(data=to_json(payload)))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SensorCrossValidatorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
