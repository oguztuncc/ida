import math

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, String

from .common import clamp, from_json, normalize_angle_deg, now_ts, to_json


class Parkur2PlannerNode(Node):
    def __init__(self) -> None:
        super().__init__("parkur2_planner_node")

        self.declare_parameter("max_speed_mps", 0.25)
        self.declare_parameter("approach_speed_mps", 0.15)
        self.declare_parameter("danger_distance_m", 0.60)
        self.declare_parameter("avoid_start_distance_m", 1.20)
        self.declare_parameter("safe_clearance_m", 1.50)
        self.declare_parameter("max_relative_bearing_deg", 55.0)
        self.declare_parameter("corridor_min_confidence", 0.55)
        self.declare_parameter("corridor_keepout_margin_m", 1.10)
        self.declare_parameter("min_corridor_return_angle_deg", 25.0)
        self.declare_parameter("scan_before_bypass", True)
        self.declare_parameter("scan_duration_s", 3.0)
        self.declare_parameter("scan_spin_angle_deg", 65.0)
        self.declare_parameter("scan_cooldown_s", 2.0)
        self.declare_parameter("bypass_hold_s", 4.0)
        self.declare_parameter("obstacle_pass_margin_m", 1.10)
        self.declare_parameter("bypass_lookahead_m", 3.0)
        self.declare_parameter("require_corridor_for_motion", False)
        self.declare_parameter("corridor_search_angle_deg", 35.0)
        self.declare_parameter("sensor_timeout_s", 1.0)

        self.max_speed_mps = float(self.get_parameter("max_speed_mps").value)
        self.approach_speed_mps = float(
            self.get_parameter("approach_speed_mps").value
        )
        self.danger_distance_m = float(
            self.get_parameter("danger_distance_m").value
        )
        self.avoid_start_distance_m = float(
            self.get_parameter("avoid_start_distance_m").value
        )
        self.safe_clearance_m = float(
            self.get_parameter("safe_clearance_m").value
        )
        self.max_relative_bearing_deg = float(
            self.get_parameter("max_relative_bearing_deg").value
        )
        self.corridor_min_confidence = float(
            self.get_parameter("corridor_min_confidence").value
        )
        self.corridor_keepout_margin_m = float(
            self.get_parameter("corridor_keepout_margin_m").value
        )
        self.min_corridor_return_angle_deg = float(
            self.get_parameter("min_corridor_return_angle_deg").value
        )
        self.scan_before_bypass = bool(
            self.get_parameter("scan_before_bypass").value
        )
        self.scan_duration_s = float(
            self.get_parameter("scan_duration_s").value
        )
        self.scan_spin_angle_deg = float(
            self.get_parameter("scan_spin_angle_deg").value
        )
        self.scan_cooldown_s = float(
            self.get_parameter("scan_cooldown_s").value
        )
        self.bypass_hold_s = float(
            self.get_parameter("bypass_hold_s").value
        )
        self.obstacle_pass_margin_m = float(
            self.get_parameter("obstacle_pass_margin_m").value
        )
        self.bypass_lookahead_m = float(
            self.get_parameter("bypass_lookahead_m").value
        )
        self.require_corridor_for_motion = bool(
            self.get_parameter("require_corridor_for_motion").value
        )
        self.corridor_search_angle_deg = float(
            self.get_parameter("corridor_search_angle_deg").value
        )
        self.sensor_timeout_s = float(
            self.get_parameter("sensor_timeout_s").value
        )

        self.heading_deg = None
        self.target_bearing_deg = None
        self.lidar_summary = None
        self.corridor = None
        self.semantic = None
        self.lidar_ts = 0.0
        self.corridor_ts = 0.0
        self.semantic_ts = 0.0
        self.scan_active = False
        self.scan_started_ts = 0.0
        self.scan_direction = 1.0
        self.scan_target_bearing_deg = None
        self.scan_samples = []
        self.scan_obstacle = None
        self.last_scan_completed_ts = -999.0
        self.bypass_body_bearing = None
        self.bypass_until_ts = 0.0

        self.safe_bearing_pub = self.create_publisher(
            Float32,
            "/planner/safe_bearing_deg",
            10,
        )
        self.speed_limit_pub = self.create_publisher(
            Float32,
            "/planner/speed_limit_mps",
            10,
        )
        self.status_pub = self.create_publisher(
            String,
            "/planner/status",
            10,
        )

        self.create_subscription(
            Float32,
            "/mavros/global_position/compass_hdg",
            self.heading_cb,
            10,
        )
        self.create_subscription(
            Float32,
            "/guidance/target_bearing_deg",
            self.target_cb,
            10,
        )
        self.create_subscription(
            String,
            "/perception/lidar_summary",
            self.lidar_cb,
            10,
        )
        self.create_subscription(
            String,
            "/planner/corridor",
            self.corridor_cb,
            10,
        )
        self.create_subscription(
            String,
            "/perception/semantic_buoys",
            self.semantic_cb,
            10,
        )

        self.timer = self.create_timer(0.1, self.loop)

    def heading_cb(self, msg: Float32) -> None:
        self.heading_deg = float(msg.data)

    def target_cb(self, msg: Float32) -> None:
        self.target_bearing_deg = float(msg.data)

    def lidar_cb(self, msg: String) -> None:
        try:
            self.lidar_summary = from_json(msg.data)
            self.lidar_ts = now_ts()
        except Exception:
            self.lidar_summary = None

    def corridor_cb(self, msg: String) -> None:
        try:
            self.corridor = from_json(msg.data)
            self.corridor_ts = now_ts()
        except Exception:
            self.corridor = None

    def semantic_cb(self, msg: String) -> None:
        try:
            self.semantic = from_json(msg.data)
            self.semantic_ts = now_ts()
        except Exception:
            self.semantic = None

    def _absolute_from_relative(self, relative_deg: float) -> float:
        if self.heading_deg is None:
            return self.target_bearing_deg or 0.0
        return (self.heading_deg + relative_deg) % 360.0

    def _nav_relative_from_body_left(self, relative_deg: float) -> float:
        return -relative_deg

    def _semantic_front_obstacle(self) -> bool:
        return self._nearest_front_obstacle() is not None

    def _nearest_front_obstacle(self):
        if self.semantic is None:
            return None
        nearest = None
        for buoy in self.semantic.get("buoys", []):
            if buoy.get("semantic") not in (
                "obstacle_candidate",
                "unknown",
            ):
                continue
            forward = buoy.get("forward_m")
            left = buoy.get("left_m")
            if forward is None or left is None:
                continue
            forward = float(forward)
            left = float(left)
            if not 0.0 < forward < self.avoid_start_distance_m + 1.0:
                continue
            if abs(left) > self.safe_clearance_m:
                continue
            if nearest is None or forward < nearest["forward_m"]:
                nearest = {
                    "forward_m": forward,
                    "left_m": left,
                    "id": buoy.get("id"),
                }
        return nearest

    def _lidar_state(self):
        now = now_ts()
        if self.lidar_summary is None:
            return "missing", None, 0.0
        if now - self.lidar_ts > self.sensor_timeout_s:
            return "timeout", None, 0.0

        front = float(self.lidar_summary.get("front_clearance_m", 999.0))
        best_free = float(self.lidar_summary.get("best_free_angle_deg", 0.0))
        if front < self.danger_distance_m:
            return "danger", front, best_free
        if front < self.avoid_start_distance_m:
            return "avoid", front, best_free
        return "clear", front, best_free

    def _corridor_state(self):
        if self.corridor is None:
            return None, 0.0, "no_corridor", None, None
        if now_ts() - self.corridor_ts > self.sensor_timeout_s:
            return None, 0.0, "corridor_timeout", None, None
        confidence = float(self.corridor.get("confidence", 0.0))
        if confidence < self.corridor_min_confidence:
            return None, confidence, "low_corridor_confidence", None, None
        bearing = float(self.corridor.get("center_bearing_deg", 0.0))
        bearing = clamp(
            bearing,
            -self.max_relative_bearing_deg,
            self.max_relative_bearing_deg,
        )
        center_left = self.corridor.get("center_left_m")
        estimated_width = self.corridor.get("estimated_width_m")
        center_left = None if center_left is None else float(center_left)
        estimated_width = (
            None if estimated_width is None else float(estimated_width)
        )
        return bearing, confidence, "corridor_track", center_left, estimated_width

    def _corridor_limited_body_bearing(
        self,
        body_bearing: float,
        corridor_body_bearing: float | None,
        center_left_m: float | None,
        estimated_width_m: float | None,
    ) -> tuple[float, str | None]:
        if (
            corridor_body_bearing is None
            or center_left_m is None
            or estimated_width_m is None
            or estimated_width_m <= 0.0
        ):
            return body_bearing, None

        half_width = estimated_width_m / 2.0
        left_clearance = center_left_m + half_width
        right_clearance = half_width - center_left_m
        margin = max(self.corridor_keepout_margin_m, self.safe_clearance_m / 2.0)

        if body_bearing < 0.0 and right_clearance < margin:
            return self._corridor_return_angle(corridor_body_bearing, 1.0), (
                "right_boundary_keepout"
            )
        if body_bearing > 0.0 and left_clearance < margin:
            return self._corridor_return_angle(corridor_body_bearing, -1.0), (
                "left_boundary_keepout"
            )
        return body_bearing, None

    def _corridor_return_angle(
        self,
        corridor_body_bearing: float,
        sign: float,
    ) -> float:
        angle = max(
            abs(corridor_body_bearing),
            self.min_corridor_return_angle_deg,
        )
        return clamp(
            sign * angle,
            -self.max_relative_bearing_deg,
            self.max_relative_bearing_deg,
        )

    def _corridor_clearances(
        self,
        center_left_m: float | None,
        estimated_width_m: float | None,
    ) -> tuple[float | None, float | None]:
        if center_left_m is None or estimated_width_m is None:
            return None, None
        half_width = estimated_width_m / 2.0
        return center_left_m + half_width, half_width - center_left_m

    def _scan_direction_for_corridor(
        self,
        best_free: float,
        center_left_m: float | None,
        estimated_width_m: float | None,
    ) -> float:
        left_clearance, right_clearance = self._corridor_clearances(
            center_left_m,
            estimated_width_m,
        )
        if left_clearance is not None and right_clearance is not None:
            if left_clearance > right_clearance + 0.25:
                return 1.0
            if right_clearance > left_clearance + 0.25:
                return -1.0
        if abs(best_free) > 1.0:
            return 1.0 if best_free > 0.0 else -1.0
        return 1.0

    def _record_scan_sample(
        self,
        front_clearance: float | None,
        best_free: float,
        front_obstacle,
        corridor_body_bearing: float | None,
        corridor_center_left: float | None,
        corridor_width: float | None,
    ) -> None:
        if front_obstacle is not None:
            self.scan_obstacle = front_obstacle
        best_clearance = 0.0
        if self.lidar_summary is not None:
            best_clearance = float(
                self.lidar_summary.get("best_free_clearance_m", 0.0)
            )
        self.scan_samples.append(
            {
                "front_clearance_m": front_clearance,
                "best_free_body_deg": best_free,
                "best_free_clearance_m": best_clearance,
                "front_obstacle": front_obstacle,
                "corridor_body_bearing_deg": corridor_body_bearing,
                "corridor_center_left_m": corridor_center_left,
                "corridor_width_m": corridor_width,
            }
        )
        self.scan_samples = self.scan_samples[-80:]

    def _choose_bypass_body_bearing(
        self,
        fallback_best_free: float,
        front_obstacle,
        fallback_corridor_bearing: float | None,
        fallback_center_left: float | None,
        fallback_width: float | None,
    ) -> tuple[float, str]:
        corridor_bypass = self._corridor_obstacle_bypass_bearing(
            front_obstacle or self.scan_obstacle,
            fallback_center_left,
            fallback_width,
        )
        if corridor_bypass is not None:
            return corridor_bypass

        best = None
        for sample in self.scan_samples:
            corridor_bypass = self._corridor_obstacle_bypass_bearing(
                sample.get("front_obstacle"),
                sample.get("corridor_center_left_m"),
                sample.get("corridor_width_m"),
            )
            if corridor_bypass is not None:
                return corridor_bypass

            candidate = float(sample.get("best_free_body_deg", 0.0))
            if abs(candidate) < self.min_corridor_return_angle_deg:
                continue

            limited, limit_reason = self._corridor_limited_body_bearing(
                candidate,
                sample.get("corridor_body_bearing_deg"),
                sample.get("corridor_center_left_m"),
                sample.get("corridor_width_m"),
            )
            clearance = float(sample.get("best_free_clearance_m", 0.0))
            front = sample.get("front_clearance_m")
            front_score = 0.0 if front is None else min(float(front), 8.0) * 0.15
            center_left = sample.get("corridor_center_left_m")
            center_penalty = 0.0 if center_left is None else abs(float(center_left)) * 0.1
            limit_penalty = 0.35 if limit_reason else 0.0
            score = min(clearance, 12.0) + front_score - center_penalty - limit_penalty
            if best is None or score > best[0]:
                best = (score, limited, limit_reason)

        if best is not None:
            reason = best[2] or "scan_bypass_selected"
            return float(best[1]), reason

        limited, limit_reason = self._corridor_limited_body_bearing(
            fallback_best_free,
            fallback_corridor_bearing,
            fallback_center_left,
            fallback_width,
        )
        if abs(limited) >= 1.0:
            return limited, limit_reason or "scan_bypass_fallback"

        direction = self._scan_direction_for_corridor(
            fallback_best_free,
            fallback_center_left,
            fallback_width,
        )
        if fallback_corridor_bearing is None:
            fallback_corridor_bearing = 0.0
        return self._corridor_return_angle(
            fallback_corridor_bearing,
            direction,
        ), "scan_bypass_corridor_return"

    def _corridor_obstacle_bypass_bearing(
        self,
        obstacle,
        center_left_m: float | None,
        estimated_width_m: float | None,
    ) -> tuple[float, str] | None:
        if obstacle is None or center_left_m is None or estimated_width_m is None:
            return None
        if estimated_width_m <= 0.0:
            return None

        half_width = estimated_width_m / 2.0
        boundary_margin = max(
            self.corridor_keepout_margin_m,
            self.safe_clearance_m / 2.0,
        )
        min_left = center_left_m - half_width + boundary_margin
        max_left = center_left_m + half_width - boundary_margin
        if min_left >= max_left:
            return None

        obstacle_left = float(obstacle["left_m"])
        obstacle_forward = float(obstacle["forward_m"])
        obstacle_margin = max(
            self.obstacle_pass_margin_m,
            self.safe_clearance_m / 2.0,
        )
        left_gap = max_left - (obstacle_left + obstacle_margin)
        right_gap = (obstacle_left - obstacle_margin) - min_left

        if left_gap > 0.0 and left_gap >= right_gap:
            target_left = (max_left + obstacle_left + obstacle_margin) / 2.0
            reason = "corridor_bypass_left_of_obstacle"
        elif right_gap > 0.0:
            target_left = (min_left + obstacle_left - obstacle_margin) / 2.0
            reason = "corridor_bypass_right_of_obstacle"
        else:
            target_left = clamp(center_left_m, min_left, max_left)
            reason = "corridor_bypass_center_hold"

        lookahead = clamp(
            obstacle_forward,
            1.5,
            max(self.bypass_lookahead_m, 1.5),
        )
        body_bearing = math.degrees(math.atan2(target_left, lookahead))
        return clamp(
            body_bearing,
            -self.max_relative_bearing_deg,
            self.max_relative_bearing_deg,
        ), reason

    def _publish_plan(
        self,
        mode: str,
        relative_bearing: float,
        speed_limit: float,
        front_clearance: float | None,
        lidar_state: str,
        corridor_confidence: float,
        corridor_center_left: float | None,
        corridor_width: float | None,
        corridor_limit_reason: str | None,
        reason: str,
    ) -> None:
        safe_bearing = self._absolute_from_relative(relative_bearing)
        self.safe_bearing_pub.publish(Float32(data=float(safe_bearing)))
        self.speed_limit_pub.publish(Float32(data=float(speed_limit)))
        self.status_pub.publish(
            String(
                data=to_json(
                    {
                        "timestamp": now_ts(),
                        "mode": mode,
                        "safe_bearing_deg": safe_bearing,
                        "relative_bearing_deg": relative_bearing,
                        "speed_limit_mps": speed_limit,
                        "front_clearance_m": front_clearance,
                        "lidar_state": lidar_state,
                        "corridor_confidence": corridor_confidence,
                        "corridor_center_left_m": corridor_center_left,
                        "corridor_width_m": corridor_width,
                        "corridor_limit_reason": corridor_limit_reason,
                        "scan_active": self.scan_active,
                        "scan_sample_count": len(self.scan_samples),
                        "bypass_body_bearing_deg": self.bypass_body_bearing,
                        "reason": reason,
                    }
                )
            )
        )

    def loop(self) -> None:
        if self.heading_deg is None or self.target_bearing_deg is None:
            return

        lidar_state, front_clearance, best_free = self._lidar_state()
        (
            corridor_body_bearing,
            corridor_confidence,
            corridor_reason,
            corridor_center_left,
            corridor_width,
        ) = self._corridor_state()
        corridor_bearing = None
        if corridor_body_bearing is not None:
            corridor_bearing = self._nav_relative_from_body_left(
                corridor_body_bearing
            )
        front_obstacle = self._nearest_front_obstacle()
        semantic_front_obstacle = front_obstacle is not None
        corridor_limit_reason = None
        now = now_ts()
        near_obstacle = lidar_state in ("avoid", "danger") or semantic_front_obstacle

        # Clear expired bypass so old value doesn't linger in logs
        if self.bypass_body_bearing is not None and self.bypass_until_ts <= now:
            self.bypass_body_bearing = None

        if self.scan_active:
            self._record_scan_sample(
                front_clearance,
                best_free,
                front_obstacle,
                corridor_body_bearing,
                corridor_center_left,
                corridor_width,
            )
            if now - self.scan_started_ts < self.scan_duration_s:
                if self.scan_target_bearing_deg is None:
                    body_bearing = self.scan_direction * self.scan_spin_angle_deg
                    self.scan_target_bearing_deg = (
                        self.heading_deg
                        + self._nav_relative_from_body_left(body_bearing)
                    ) % 360.0
                self._publish_plan(
                    "SCAN_OBSTACLE",
                    normalize_angle_deg(
                        self.scan_target_bearing_deg - self.heading_deg
                    ),
                    0.0,
                    front_clearance,
                    lidar_state,
                    corridor_confidence,
                    corridor_center_left,
                    corridor_width,
                    None,
                    "scan_before_bypass",
                )
                return

            self.scan_active = False
            self.last_scan_completed_ts = now
            if corridor_body_bearing is None and (
                near_obstacle or self.scan_obstacle is not None
            ):
                self.bypass_body_bearing = None
                self.bypass_until_ts = 0.0
                self._publish_plan(
                    "STOP",
                    clamp(
                        self._nav_relative_from_body_left(best_free),
                        -self.max_relative_bearing_deg,
                        self.max_relative_bearing_deg,
                    ),
                    0.0,
                    front_clearance,
                    lidar_state,
                    corridor_confidence,
                    corridor_center_left,
                    corridor_width,
                    None,
                    "corridor_missing_after_scan",
                )
                return

            self.bypass_body_bearing, corridor_limit_reason = (
                self._choose_bypass_body_bearing(
                    best_free,
                    front_obstacle,
                    corridor_body_bearing,
                    corridor_center_left,
                    corridor_width,
                )
            )
            self.bypass_until_ts = now + self.bypass_hold_s

        if (
            self.scan_before_bypass
            and near_obstacle
            and self.bypass_until_ts <= now
            and now - self.last_scan_completed_ts >= self.scan_cooldown_s
        ):
            self.scan_active = True
            self.scan_started_ts = now
            self.scan_samples = []
            self.scan_direction = self._scan_direction_for_corridor(
                best_free,
                corridor_center_left,
                corridor_width,
            )
            body_bearing = self.scan_direction * self.scan_spin_angle_deg
            self.scan_target_bearing_deg = (
                self.heading_deg + self._nav_relative_from_body_left(body_bearing)
            ) % 360.0
            self.scan_obstacle = front_obstacle
            self._record_scan_sample(
                front_clearance,
                best_free,
                front_obstacle,
                corridor_body_bearing,
                corridor_center_left,
                corridor_width,
            )
            self._publish_plan(
                "SCAN_OBSTACLE",
                normalize_angle_deg(
                    self.scan_target_bearing_deg - self.heading_deg
                ),
                0.0,
                front_clearance,
                lidar_state,
                corridor_confidence,
                corridor_center_left,
                corridor_width,
                None,
                "scan_before_bypass",
            )
            return

        if near_obstacle and corridor_body_bearing is None:
            self._publish_plan(
                "STOP",
                clamp(
                    self._nav_relative_from_body_left(best_free),
                    -self.max_relative_bearing_deg,
                    self.max_relative_bearing_deg,
                ),
                0.0,
                front_clearance,
                lidar_state,
                corridor_confidence,
                corridor_center_left,
                corridor_width,
                None,
                "corridor_missing_near_obstacle",
            )
            return

        if self.require_corridor_for_motion and corridor_body_bearing is None:
            self._publish_plan(
                "SEARCH_CORRIDOR",
                self.corridor_search_angle_deg,
                0.0,
                front_clearance,
                lidar_state,
                corridor_confidence,
                corridor_center_left,
                corridor_width,
                None,
                "corridor_required_for_motion",
            )
            return

        mode = "WAYPOINT"
        relative_bearing = normalize_angle_deg(
            self.target_bearing_deg - self.heading_deg
        )
        speed_limit = self.max_speed_mps
        reason = "target_bearing"

        if self.bypass_body_bearing is not None and self.bypass_until_ts > now:
            # Early exit if obstacle has been cleared and front is safe
            if front_obstacle is None and lidar_state not in ("danger", "avoid"):
                self.bypass_until_ts = now
            else:
                mode = "BYPASS"
                body_bearing, corridor_limit_reason = (
                    self._corridor_limited_body_bearing(
                        self.bypass_body_bearing,
                        corridor_body_bearing,
                        corridor_center_left,
                        corridor_width,
                    )
                )
                relative_bearing = clamp(
                    self._nav_relative_from_body_left(body_bearing),
                    -self.max_relative_bearing_deg,
                    self.max_relative_bearing_deg,
                )
                speed_limit = 0.0 if lidar_state == "danger" else self.approach_speed_mps
                reason = corridor_limit_reason or "scan_bypass"
        elif lidar_state in ("missing", "timeout"):
            mode = "STOP"
            relative_bearing = 0.0
            speed_limit = 0.0
            reason = f"lidar_{lidar_state}"
        elif lidar_state == "danger":
            mode = "STOP"
            body_bearing, corridor_limit_reason = (
                self._corridor_limited_body_bearing(
                    best_free,
                    corridor_body_bearing,
                    corridor_center_left,
                    corridor_width,
                )
            )
            relative_bearing = clamp(
                self._nav_relative_from_body_left(body_bearing),
                -self.max_relative_bearing_deg,
                self.max_relative_bearing_deg,
            )
            speed_limit = 0.0
            reason = corridor_limit_reason or "front_danger_turn_in_place"
        elif lidar_state == "avoid" or semantic_front_obstacle:
            mode = "AVOID"
            body_bearing, corridor_limit_reason = (
                self._corridor_limited_body_bearing(
                    best_free,
                    corridor_body_bearing,
                    corridor_center_left,
                    corridor_width,
                )
            )
            relative_bearing = clamp(
                self._nav_relative_from_body_left(body_bearing),
                -self.max_relative_bearing_deg,
                self.max_relative_bearing_deg,
            )
            speed_limit = self.approach_speed_mps
            reason = corridor_limit_reason or (
                "local_obstacle_avoidance"
                if lidar_state == "avoid"
                else "semantic_front_obstacle"
            )
        elif corridor_bearing is not None:
            mode = "CORRIDOR_TRACK"
            relative_bearing = corridor_bearing
            speed_limit = self.max_speed_mps
            reason = corridor_reason

        self._publish_plan(
            mode,
            relative_bearing,
            speed_limit,
            front_clearance,
            lidar_state,
            corridor_confidence,
            corridor_center_left,
            corridor_width,
            corridor_limit_reason,
            reason,
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = Parkur2PlannerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
