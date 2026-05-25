import math
from typing import Iterable, List, Tuple

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String

from .common import normalize_angle_deg, now_ts, to_json


class LidarProcessorNode(Node):
    def __init__(self) -> None:
        super().__init__("lidar_processor_node")

        self.declare_parameter("collision_distance_m", 1.2)
        self.declare_parameter("sector_width_deg", 30.0)
        self.declare_parameter("max_avoidance_angle_deg", 65.0)
        self.declare_parameter("min_avoidance_angle_deg", 35.0)
        self.declare_parameter("candidate_step_deg", 5.0)
        self.declare_parameter("candidate_sector_width_deg", 18.0)
        self.declare_parameter("avoidance_release_distance_m", 5.0)
        self.declare_parameter("latch_avoidance_direction", True)
        self.declare_parameter("latch_override_ratio", 1.5)
        self.declare_parameter("front_path_half_width_m", 0.70)
        self.declare_parameter("vehicle_width_m", 0.76)
        self.declare_parameter("vehicle_length_m", 1.11)

        self.collision_distance_m = float(
            self.get_parameter("collision_distance_m").value
        )
        self.sector_width_deg = float(
            self.get_parameter("sector_width_deg").value
        )
        self.max_avoidance_angle_deg = float(
            self.get_parameter("max_avoidance_angle_deg").value
        )
        self.min_avoidance_angle_deg = float(
            self.get_parameter("min_avoidance_angle_deg").value
        )
        self.candidate_step_deg = max(
            1.0,
            float(self.get_parameter("candidate_step_deg").value),
        )
        self.candidate_sector_width_deg = float(
            self.get_parameter("candidate_sector_width_deg").value
        )
        self.avoidance_release_distance_m = float(
            self.get_parameter("avoidance_release_distance_m").value
        )
        self.latch_avoidance_direction = bool(
            self.get_parameter("latch_avoidance_direction").value
        )
        self.latch_override_ratio = max(
            1.0,
            float(self.get_parameter("latch_override_ratio").value),
        )
        self.front_path_half_width_m = max(
            0.1,
            float(self.get_parameter("front_path_half_width_m").value),
        )
        self.vehicle_width_m = max(
            0.1,
            float(self.get_parameter("vehicle_width_m").value),
        )
        self.vehicle_length_m = max(
            0.1,
            float(self.get_parameter("vehicle_length_m").value),
        )
        self.vehicle_half_width_m = self.vehicle_width_m / 2.0
        self.vehicle_half_length_m = self.vehicle_length_m / 2.0
        self.last_best_angle_deg = 0.0
        self.avoidance_sign = 0.0

        self.create_subscription(LaserScan, "/scan", self.scan_cb, 10)
        self.pub = self.create_publisher(
            String,
            "/perception/lidar_summary",
            10,
        )

    def _valid_ranges(self, msg: LaserScan) -> Iterable[Tuple[float, float]]:
        angle = msg.angle_min
        for distance in msg.ranges:
            if (
                math.isfinite(distance)
                and msg.range_min <= distance <= msg.range_max
            ):
                yield normalize_angle_deg(math.degrees(angle)), distance
            angle += msg.angle_increment

    def _sector_min(self, values: List[Tuple[float, float]]) -> float:
        vals = [
            distance for _, distance in values
            if math.isfinite(distance) and distance > 0.05
        ]
        return min(vals) if vals else 999.0

    def _window_min(
        self,
        values: List[Tuple[float, float]],
        center_deg: float,
        width_deg: float,
    ) -> float:
        half = width_deg / 2.0
        window = [
            (a, r) for a, r in values
            if abs(normalize_angle_deg(a - center_deg)) <= half
        ]
        return self._sector_min(window)

    def _front_path_clearance(self, values: List[Tuple[float, float]]) -> float:
        forward_distances = []
        for angle_deg, distance in values:
            angle_rad = math.radians(angle_deg)
            forward = distance * math.cos(angle_rad)
            left = distance * math.sin(angle_rad)
            if forward > 0.05 and abs(left) <= self.front_path_half_width_m:
                forward_distances.append(forward)
        return min(forward_distances) if forward_distances else 999.0

    def _footprint_clearance(self, forward: float, left: float) -> float:
        dx = abs(forward) - self.vehicle_half_length_m
        dy = abs(left) - self.vehicle_half_width_m
        outside = math.hypot(max(dx, 0.0), max(dy, 0.0))
        inside = min(max(dx, dy), 0.0)
        return outside + inside

    def _front_footprint_clearance(self, values: List[Tuple[float, float]]) -> float:
        clearances = []
        for angle_deg, distance in values:
            angle_rad = math.radians(angle_deg)
            forward = distance * math.cos(angle_rad)
            left = distance * math.sin(angle_rad)
            if forward > -self.vehicle_half_length_m:
                clearances.append(self._footprint_clearance(forward, left))
        return min(clearances) if clearances else 999.0

    def _preferred_escape_sign(
        self,
        values: List[Tuple[float, float]],
        front_min: float,
    ) -> float:
        close_front = [
            angle for angle, distance in values
            if abs(angle) <= self.sector_width_deg
            and distance <= front_min + 0.35
        ]
        if close_front:
            mean_angle = sum(close_front) / len(close_front)
            if mean_angle > 3.0:
                return -1.0
            if mean_angle < -3.0:
                return 1.0
        if abs(self.last_best_angle_deg) > 1.0:
            return 1.0 if self.last_best_angle_deg > 0.0 else -1.0
        return -1.0

    def _best_free_angle(
        self,
        values: List[Tuple[float, float]],
        front_min: float,
    ) -> Tuple[float, float]:
        front_blocked = front_min < self.avoidance_release_distance_m
        preferred_sign = self._preferred_escape_sign(values, front_min)
        if front_blocked and self.latch_avoidance_direction:
            if self.avoidance_sign == 0.0:
                self.avoidance_sign = preferred_sign
            preferred_sign = self.avoidance_sign
        elif not front_blocked:
            self.avoidance_sign = 0.0

        half_window = self.candidate_sector_width_deg
        best_angle = 0.0
        best_clearance = self._window_min(values, 0.0, half_window)
        best_score = None

        steps = int(
            round(2.0 * self.max_avoidance_angle_deg / self.candidate_step_deg)
        )

        # Latch aktifse: önce latch yönündeki en iyiyi bul,
        # sonra global en iyiyi bul ve karşılaştır.
        latched_best_score = None

        for step in range(steps + 1):
            angle = -self.max_avoidance_angle_deg + step * self.candidate_step_deg
            if front_blocked and abs(angle) < self.min_avoidance_angle_deg:
                continue
            if front_blocked and angle * preferred_sign <= 0.0:
                continue

            clearance = self._window_min(values, angle, half_window)
            clearance_score = min(clearance, 20.0)
            turn_penalty = abs(angle) * 0.012
            continuity_bonus = 0.25 if angle * self.last_best_angle_deg > 0.0 else 0.0
            escape_bonus = 0.35 if angle * preferred_sign > 0.0 else 0.0
            score = clearance_score - turn_penalty + continuity_bonus + escape_bonus

            if latched_best_score is None or score > latched_best_score:
                latched_best_score = score

            if best_score is None or score > best_score:
                best_score = score
                best_angle = angle
                best_clearance = clearance

        # Latch override kontrolü: eğer karşı yön çok daha iyiyse latch'i kır
        if (
            front_blocked
            and self.latch_avoidance_direction
            and self.avoidance_sign != 0.0
            and best_score is not None
            and latched_best_score is not None
            and best_angle * self.avoidance_sign < 0.0
            and best_score > latched_best_score * self.latch_override_ratio
        ):
            self.avoidance_sign = -self.avoidance_sign
            best_angle = best_angle
            best_clearance = best_clearance

        self.last_best_angle_deg = best_angle
        return best_angle, best_clearance

    def scan_cb(self, msg: LaserScan) -> None:
        n = len(msg.ranges)
        if n < 10:
            return

        half = self.sector_width_deg / 2.0
        values = list(self._valid_ranges(msg))
        front = [(a, r) for a, r in values if abs(a) <= half]
        left = [
            (a, r) for a, r in values
            if half < a <= half + self.sector_width_deg
        ]
        right = [
            (a, r) for a, r in values
            if -half - self.sector_width_deg <= a < -half
        ]

        front_min = self._sector_min(front)
        front_path_min = self._front_path_clearance(values)
        front_footprint_min = self._front_footprint_clearance(values)
        left_min = self._sector_min(left)
        right_min = self._sector_min(right)

        collision = front_footprint_min < self.collision_distance_m
        best_angle, best_clearance = self._best_free_angle(values, front_min)

        payload = {
            "timestamp": now_ts(),
            "collision_imminent": collision,
            "front_clearance_m": front_min,
            "front_sector_clearance_m": front_min,
            "front_path_clearance_m": front_path_min,
            "front_footprint_clearance_m": front_footprint_min,
            "front_path_half_width_m": self.front_path_half_width_m,
            "vehicle_width_m": self.vehicle_width_m,
            "vehicle_length_m": self.vehicle_length_m,
            "left_clearance_m": left_min,
            "right_clearance_m": right_min,
            "best_free_angle_deg": best_angle,
            "best_free_clearance_m": best_clearance,
            "avoidance_sign": self.avoidance_sign,
        }
        self.pub.publish(String(data=to_json(payload)))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = LidarProcessorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
