import math
from typing import List, Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from .common import clamp, from_json, now_ts, to_json


class CorridorTrackerNode(Node):
    def __init__(self) -> None:
        super().__init__("corridor_tracker_node")

        self.declare_parameter("lookahead_m", 4.0)
        self.declare_parameter("expected_corridor_width_m", 8.82)
        self.declare_parameter("vehicle_width_m", 1.10)
        self.declare_parameter("side_safety_margin_m", 0.35)
        self.declare_parameter("min_gate_width_m", 1.80)
        self.declare_parameter("max_boundary_bearing_deg", 70.0)
        self.declare_parameter("min_boundary_range_m", 0.4)
        self.declare_parameter("max_boundary_range_m", 8.0)
        self.declare_parameter("lookahead_window_m", 3.0)
        self.declare_parameter("center_smoothing_alpha", 0.35)
        self.declare_parameter("max_center_step_m", 1.0)
        self.declare_parameter("obstacle_avoidance_enabled", False)
        self.declare_parameter("obstacle_avoidance_range_m", 8.0)
        self.declare_parameter("obstacle_avoidance_margin_m", 0.80)
        self.declare_parameter("obstacle_max_offset_m", 1.50)

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
        self.lookahead_window_m = max(
            0.2,
            float(self.get_parameter("lookahead_window_m").value),
        )
        self.center_smoothing_alpha = clamp(
            float(self.get_parameter("center_smoothing_alpha").value),
            0.0,
            1.0,
        )
        self.max_center_step_m = max(
            0.0,
            float(self.get_parameter("max_center_step_m").value),
        )
        self.obstacle_avoidance_enabled = bool(
            self.get_parameter("obstacle_avoidance_enabled").value
        )
        self.obstacle_avoidance_range_m = max(
            1.0,
            float(self.get_parameter("obstacle_avoidance_range_m").value),
        )
        self.obstacle_avoidance_margin_m = max(
            0.0,
            float(self.get_parameter("obstacle_avoidance_margin_m").value),
        )
        self.obstacle_max_offset_m = max(
            0.0,
            float(self.get_parameter("obstacle_max_offset_m").value),
        )
        self.last_center_left_m = None

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

    def _usable_obstacle(self, buoy: dict) -> bool:
        if buoy.get("semantic") not in ("obstacle_candidate", "unknown"):
            return False
        forward = buoy.get("forward_m")
        left = buoy.get("left_m")
        if forward is None or left is None:
            return False
        return 0.0 < float(forward) < self.obstacle_avoidance_range_m

    def _obstacle_offset(
        self,
        buoys: List[dict],
        left_boundary: Optional[float],
        right_boundary: Optional[float],
    ) -> float:
        if not self.obstacle_avoidance_enabled:
            return 0.0
        if left_boundary is None or right_boundary is None:
            return 0.0

        # Find the nearest obstacle that lies close to the center
        nearest = None
        for buoy in buoys:
            if not self._usable_obstacle(buoy):
                continue
            forward = float(buoy["forward_m"])
            left = float(buoy["left_m"])
            if abs(left) > self.obstacle_avoidance_margin_m:
                continue
            if nearest is None or forward < nearest["forward_m"]:
                nearest = {"forward_m": forward, "left_m": left}

        if nearest is None:
            return 0.0

        obstacle_left = nearest["left_m"]
        obstacle_forward = nearest["forward_m"]

        left_space = left_boundary - obstacle_left
        right_space = obstacle_left - right_boundary

        if left_space <= 0.0 and right_space <= 0.0:
            return 0.0

        if left_space >= right_space:
            direction = 1.0  # shift left (positive left_m)
        else:
            direction = -1.0  # shift right (negative left_m)

        # Scale offset by proximity: closer obstacle -> larger offset
        proximity = 1.0 - (obstacle_forward / self.obstacle_avoidance_range_m)
        proximity = clamp(proximity, 0.0, 1.0)
        offset = self.obstacle_max_offset_m * proximity * direction

        # Also consider how much room we actually have on the chosen side
        available_space = left_space if direction > 0 else right_space
        offset = clamp(offset, -available_space * 0.5, available_space * 0.5)

        return offset

    def _side_estimate(self, buoys: List[dict]) -> Optional[dict]:
        weighted = []
        for buoy in buoys:
            forward = buoy.get("forward_m")
            left = buoy.get("left_m")
            if forward is None or left is None:
                continue
            forward = float(forward)
            left = float(left)
            if forward <= 0.0:
                continue
            confidence = clamp(float(buoy.get("confidence", 1.0)), 0.05, 1.0)
            distance_weight = 1.0 / (
                1.0
                + abs(forward - self.lookahead_m) / self.lookahead_window_m
            )
            weighted.append((forward, left, confidence * distance_weight))

        if not weighted:
            return None

        weight_sum = sum(w for _, _, w in weighted)
        mean_forward = sum(x * w for x, _, w in weighted) / weight_sum
        mean_left = sum(y * w for _, y, w in weighted) / weight_sum
        forward_span = max(x for x, _, _ in weighted) - min(
            x for x, _, _ in weighted
        )

        estimate_left = mean_left
        if len(weighted) >= 2 and forward_span > 0.5:
            denominator = sum(
                w * (x - mean_forward) * (x - mean_forward)
                for x, _, w in weighted
            )
            if denominator > 1e-6:
                slope = sum(
                    w * (x - mean_forward) * (y - mean_left)
                    for x, y, w in weighted
                ) / denominator
                estimate_left = mean_left + slope * (
                    self.lookahead_m - mean_forward
                )

        lateral_spread = math.sqrt(
            sum(
                w * (y - estimate_left) * (y - estimate_left)
                for _, y, w in weighted
            )
            / weight_sum
        )
        return {
            "left_m": estimate_left,
            "count": len(weighted),
            "mean_forward_m": mean_forward,
            "lateral_spread_m": lateral_spread,
        }

    def _smooth_center(self, raw_center_left_m: float) -> float:
        if self.last_center_left_m is None:
            self.last_center_left_m = raw_center_left_m
            return raw_center_left_m

        smoothed = (
            self.last_center_left_m
            + self.center_smoothing_alpha
            * (raw_center_left_m - self.last_center_left_m)
        )
        if self.max_center_step_m > 0.0:
            step = clamp(
                smoothed - self.last_center_left_m,
                -self.max_center_step_m,
                self.max_center_step_m,
            )
            smoothed = self.last_center_left_m + step
        self.last_center_left_m = smoothed
        return smoothed

    def _two_sided_confidence(self, width_m: float, sample_count: int) -> float:
        if width_m <= 0.0:
            return 0.0
        if width_m < self.min_gate_width_m:
            return 0.45 * width_m / max(self.min_gate_width_m, 0.1)

        width_error = abs(width_m - self.expected_corridor_width_m)
        width_score = clamp(
            1.0 - width_error / max(self.expected_corridor_width_m, 0.1),
            0.35,
            1.0,
        )
        sample_score = clamp(sample_count / 4.0, 0.5, 1.0)
        return 0.55 + 0.35 * min(width_score, sample_score)

    def _single_sided_confidence(self, estimate: dict) -> float:
        sample_score = clamp(float(estimate["count"]) / 3.0, 0.0, 1.0)
        spread_penalty = clamp(
            float(estimate["lateral_spread_m"])
            / max(self.expected_corridor_width_m, 0.1),
            0.0,
            0.35,
        )
        return 0.40 + 0.10 * sample_score - spread_penalty

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

        all_buoys = payload.get("buoys", [])
        candidates = [
            b for b in all_buoys if self._usable_boundary(b)
        ]
        left = [b for b in candidates if float(b.get("left_m", 0.0)) > 0.0]
        right = [b for b in candidates if float(b.get("left_m", 0.0)) < 0.0]

        left_estimate = self._side_estimate(left)
        right_estimate = self._side_estimate(right)
        confidence = 0.0
        center_left = 0.0
        raw_center_left = None
        estimated_width = None
        tracking_method = "none"
        left_boundary = None
        right_boundary = None

        if left_estimate is not None and right_estimate is not None:
            left_boundary = float(left_estimate["left_m"])
            right_boundary = float(right_estimate["left_m"])
            estimated_width = left_boundary - right_boundary
            if estimated_width > 0.0:
                raw_center_left = (left_boundary + right_boundary) / 2.0
                confidence = self._two_sided_confidence(
                    estimated_width,
                    int(left_estimate["count"]) + int(right_estimate["count"]),
                )
                tracking_method = "two_side_weighted_center"
        elif left_estimate is not None:
            estimated_width = self.expected_corridor_width_m
            left_boundary = float(left_estimate["left_m"])
            right_boundary = left_boundary - self.expected_corridor_width_m
            raw_center_left = (
                float(left_estimate["left_m"])
                - self.expected_corridor_width_m / 2.0
            )
            confidence = self._single_sided_confidence(left_estimate)
            tracking_method = "left_side_expected_width"
        elif right_estimate is not None:
            estimated_width = self.expected_corridor_width_m
            right_boundary = float(right_estimate["left_m"])
            left_boundary = right_boundary + self.expected_corridor_width_m
            raw_center_left = (
                float(right_estimate["left_m"])
                + self.expected_corridor_width_m / 2.0
            )
            confidence = self._single_sided_confidence(right_estimate)
            tracking_method = "right_side_expected_width"

        # Apply obstacle-aware lateral offset
        obstacle_offset = 0.0
        if raw_center_left is not None:
            obstacle_offset = self._obstacle_offset(
                all_buoys, left_boundary, right_boundary
            )
            raw_center_left = raw_center_left + obstacle_offset
            center_left = self._smooth_center(raw_center_left)

        center_bearing_deg = math.degrees(
            math.atan2(center_left, max(self.lookahead_m, 0.1))
        )
        gate = self._nearest_gate(left, right)

        if gate["valid"] and gate["center_left_m"] is not None:
            confidence = max(confidence, 0.75)

        status = "corridor_visible" if confidence > 0.0 else "no_corridor"
        self.pub.publish(
            String(
                data=to_json(
                    {
                        "timestamp": now_ts(),
                        "status": status,
                        "center_bearing_deg": center_bearing_deg,
                        "center_left_m": center_left,
                        "raw_center_left_m": raw_center_left,
                        "confidence": confidence,
                        "left_boundary_count": len(left),
                        "right_boundary_count": len(right),
                        "left_boundary_estimate_m": None
                        if left_estimate is None
                        else left_estimate["left_m"],
                        "right_boundary_estimate_m": None
                        if right_estimate is None
                        else right_estimate["left_m"],
                        "estimated_width_m": estimated_width,
                        "min_gate_width_m": self.min_gate_width_m,
                        "tracking_method": tracking_method,
                        "obstacle_offset_m": obstacle_offset,
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
