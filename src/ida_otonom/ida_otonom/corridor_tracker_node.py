import math
from typing import List, Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from .common import from_json, now_ts, to_json


class CorridorTrackerNode(Node):
    def __init__(self) -> None:
        super().__init__("corridor_tracker_node")

        self.declare_parameter("lookahead_m", 4.0)
        self.declare_parameter("expected_corridor_width_m", 4.0)
        self.declare_parameter("vehicle_width_m", 1.10)
        self.declare_parameter("side_safety_margin_m", 0.35)
        self.declare_parameter("min_gate_width_m", 1.80)
        self.declare_parameter("max_boundary_bearing_deg", 70.0)
        self.declare_parameter("min_boundary_range_m", 0.4)
        self.declare_parameter("max_boundary_range_m", 8.0)

        self.lookahead_m = float(self.get_parameter("lookahead_m").value)
        self.expected_corridor_width_m = float(
            self.get_parameter("expected_corridor_width_m").value
        )
        self.vehicle_width_m = float(
            self.get_parameter("vehicle_width_m").value
        )
        self.side_safety_margin_m = float(
            self.get_parameter("side_safety_margin_m").value
        )
        self.min_gate_width_m = max(
            float(self.get_parameter("min_gate_width_m").value),
            self.vehicle_width_m + 2.0 * self.side_safety_margin_m,
        )
        self.max_boundary_bearing_deg = float(
            self.get_parameter("max_boundary_bearing_deg").value
        )
        self.min_boundary_range_m = float(
            self.get_parameter("min_boundary_range_m").value
        )
        self.max_boundary_range_m = float(
            self.get_parameter("max_boundary_range_m").value
        )

        self.pub = self.create_publisher(String, "/planner/corridor", 10)
        self.create_subscription(
            String,
            "/perception/semantic_buoys",
            self.semantic_cb,
            10,
        )

    def _usable_boundary(self, buoy: dict) -> bool:
        if buoy.get("semantic") != "course_boundary_candidate":
            return False
        range_m = buoy.get("range_m")
        bearing = buoy.get("bearing_deg")
        if range_m is None or bearing is None:
            return False
        return (
            self.min_boundary_range_m <= float(range_m)
            <= self.max_boundary_range_m
            and abs(float(bearing)) <= self.max_boundary_bearing_deg
        )

    def _mean_side(self, buoys: List[dict]) -> Optional[float]:
        vals = [
            float(b.get("left_m"))
            for b in buoys
            if b.get("left_m") is not None
        ]
        if not vals:
            return None
        return sum(vals) / len(vals)

    def _nearest_gate(self, left: List[dict], right: List[dict]) -> dict:
        best = {
            "valid": False,
            "width_m": None,
            "center_left_m": None,
            "center_forward_m": None,
        }
        best_score = None
        for lb in left:
            for rb in right:
                lx = float(lb.get("forward_m", 0.0))
                rx = float(rb.get("forward_m", 0.0))
                ly = float(lb.get("left_m", 0.0))
                ry = float(rb.get("left_m", 0.0))
                width = ly - ry
                if width <= 0.0:
                    continue
                center_forward = (lx + rx) / 2.0
                center_left = (ly + ry) / 2.0
                distance_match = abs(lx - rx)
                score = distance_match + abs(center_left) * 0.4
                if best_score is None or score < best_score:
                    best_score = score
                    best = {
                        "valid": width >= self.min_gate_width_m,
                        "width_m": width,
                        "center_left_m": center_left,
                        "center_forward_m": center_forward,
                        "left_buoy_id": lb.get("id"),
                        "right_buoy_id": rb.get("id"),
                    }
        return best

    def semantic_cb(self, msg: String) -> None:
        try:
            payload = from_json(msg.data)
        except Exception:
            return

        candidates = [
            b for b in payload.get("buoys", []) if self._usable_boundary(b)
        ]
        left = [b for b in candidates if float(b.get("left_m", 0.0)) > 0.0]
        right = [b for b in candidates if float(b.get("left_m", 0.0)) < 0.0]

        left_mean = self._mean_side(left)
        right_mean = self._mean_side(right)
        confidence = 0.0
        center_left = 0.0
        estimated_width = None

        if left_mean is not None and right_mean is not None:
            estimated_width = left_mean - right_mean
            center_left = (left_mean + right_mean) / 2.0
            confidence = 0.9 if estimated_width >= self.min_gate_width_m else 0.45
        elif left_mean is not None:
            estimated_width = self.expected_corridor_width_m
            center_left = left_mean - self.expected_corridor_width_m / 2.0
            confidence = 0.45
        elif right_mean is not None:
            estimated_width = self.expected_corridor_width_m
            center_left = right_mean + self.expected_corridor_width_m / 2.0
            confidence = 0.45

        center_bearing_deg = math.degrees(
            math.atan2(center_left, max(self.lookahead_m, 0.1))
        )
        gate = self._nearest_gate(left, right)

        if gate["valid"] and gate["center_left_m"] is not None:
            center_bearing_deg = math.degrees(
                math.atan2(
                    float(gate["center_left_m"]),
                    max(float(gate["center_forward_m"] or self.lookahead_m), 0.1),
                )
            )
            confidence = max(confidence, 0.85)

        status = "corridor_visible" if confidence > 0.0 else "no_corridor"
        self.pub.publish(
            String(
                data=to_json(
                    {
                        "timestamp": now_ts(),
                        "status": status,
                        "center_bearing_deg": center_bearing_deg,
                        "center_left_m": center_left,
                        "confidence": confidence,
                        "left_boundary_count": len(left),
                        "right_boundary_count": len(right),
                        "estimated_width_m": estimated_width,
                        "min_gate_width_m": self.min_gate_width_m,
                        "gate": gate,
                    }
                )
            )
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CorridorTrackerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
