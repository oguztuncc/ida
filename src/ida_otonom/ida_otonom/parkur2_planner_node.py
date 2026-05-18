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
        self.declare_parameter("danger_distance_m", 0.70)
        self.declare_parameter("avoid_start_distance_m", 3.20)
        self.declare_parameter("emergency_stop_distance_m", 0.45)
        self.declare_parameter("contact_recovery_enabled", True)
        self.declare_parameter("contact_recovery_distance_m", 0.30)
        self.declare_parameter("recovery_backoff_duration_s", 1.2)
        self.declare_parameter("recovery_backoff_speed_mps", -0.12)
        self.declare_parameter("recovery_turn_bearing_deg", 25.0)
        self.declare_parameter("safe_clearance_m", 1.50)
        self.declare_parameter("max_relative_bearing_deg", 35.0)
        self.declare_parameter("max_pass_bearing_deg", 25.0)
        self.declare_parameter("max_return_bearing_deg", 18.0)
        self.declare_parameter("corridor_min_confidence", 0.45)
        self.declare_parameter("corridor_keepout_margin_m", 1.20)
        self.declare_parameter("obstacle_pass_margin_m", 1.20)
        self.declare_parameter("min_pass_gap_width_m", 1.20)
        self.declare_parameter("path_block_margin_m", 0.0)
        self.declare_parameter("pass_lookahead_m", 6.0)
        self.declare_parameter("return_lookahead_m", 6.0)
        self.declare_parameter("pass_min_duration_s", 1.5)
        self.declare_parameter("pass_release_after_lost_s", 1.0)
        self.declare_parameter("corridor_coast_s", 1.2)
        self.declare_parameter("bearing_rate_limit_degps", 70.0)
        self.declare_parameter("sensor_timeout_s", 1.0)
        self.declare_parameter("waypoint_blend_factor", 0.0)
        self.declare_parameter("expected_corridor_width_m", 8.82)
        self.declare_parameter("final_approach_distance_m", 12.0)
        self.declare_parameter("waypoint_bias_weight", 0.15)

        self.declare_parameter("scan_before_bypass", False)
        self.declare_parameter("scan_duration_s", 0.0)
        self.declare_parameter("scan_spin_angle_deg", 0.0)
        self.declare_parameter("scan_cooldown_s", 0.0)

        # Legacy parameters may still exist in YAML files; declare them so
        # launch overrides remain harmless while this node uses the new state
        # machine below.
        self.declare_parameter("bypass_hold_s", 0.0)
        self.declare_parameter("bypass_release_distance_m", 0.0)
        self.declare_parameter("bypass_lookahead_m", 0.0)
        self.declare_parameter("heading_recovery_threshold_deg", 0.0)
        self.declare_parameter("require_corridor_for_motion", False)
        self.declare_parameter("corridor_search_angle_deg", 0.0)
        self.declare_parameter("min_corridor_return_angle_deg", 0.0)

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
        self.emergency_stop_distance_m = float(
            self.get_parameter("emergency_stop_distance_m").value
        )
        self.contact_recovery_enabled = bool(
            self.get_parameter("contact_recovery_enabled").value
        )
        self.contact_recovery_distance_m = float(
            self.get_parameter("contact_recovery_distance_m").value
        )
        self.recovery_backoff_duration_s = float(
            self.get_parameter("recovery_backoff_duration_s").value
        )
        self.recovery_backoff_speed_mps = -abs(
            float(self.get_parameter("recovery_backoff_speed_mps").value)
        )
        self.recovery_turn_bearing_deg = float(
            self.get_parameter("recovery_turn_bearing_deg").value
        )
        self.safe_clearance_m = float(
            self.get_parameter("safe_clearance_m").value
        )
        self.max_relative_bearing_deg = float(
            self.get_parameter("max_relative_bearing_deg").value
        )
        self.max_pass_bearing_deg = float(
            self.get_parameter("max_pass_bearing_deg").value
        )
        self.max_return_bearing_deg = float(
            self.get_parameter("max_return_bearing_deg").value
        )
        self.corridor_min_confidence = float(
            self.get_parameter("corridor_min_confidence").value
        )
        self.corridor_keepout_margin_m = float(
            self.get_parameter("corridor_keepout_margin_m").value
        )
        self.obstacle_pass_margin_m = float(
            self.get_parameter("obstacle_pass_margin_m").value
        )
        self.min_pass_gap_width_m = float(
            self.get_parameter("min_pass_gap_width_m").value
        )
        configured_path_block_margin = float(
            self.get_parameter("path_block_margin_m").value
        )
        self.path_block_margin_m = (
            configured_path_block_margin
            if configured_path_block_margin > 0.0
            else max(self.safe_clearance_m, self.obstacle_pass_margin_m)
        )
        self.pass_lookahead_m = float(
            self.get_parameter("pass_lookahead_m").value
        )
        self.return_lookahead_m = float(
            self.get_parameter("return_lookahead_m").value
        )
        self.pass_min_duration_s = float(
            self.get_parameter("pass_min_duration_s").value
        )
        self.pass_release_after_lost_s = float(
            self.get_parameter("pass_release_after_lost_s").value
        )
        self.corridor_coast_s = float(
            self.get_parameter("corridor_coast_s").value
        )
        self.bearing_rate_limit_degps = float(
            self.get_parameter("bearing_rate_limit_degps").value
        )
        self.sensor_timeout_s = float(
            self.get_parameter("sensor_timeout_s").value
        )
        self.waypoint_blend_factor = float(
            self.get_parameter("waypoint_blend_factor").value
        )
        self.expected_corridor_width_m = float(
            self.get_parameter("expected_corridor_width_m").value
        )
        self.final_approach_distance_m = float(
            self.get_parameter("final_approach_distance_m").value
        )
        self.require_corridor_for_motion = bool(
            self.get_parameter("require_corridor_for_motion").value
        )
        self.waypoint_bias_weight = clamp(
            float(self.get_parameter("waypoint_bias_weight").value),
            0.0,
            1.0,
        )
        if self.waypoint_bias_weight == 0.0 and self.require_corridor_for_motion:
            self.get_logger().warning(
                "waypoint_bias_weight=0 with require_corridor_for_motion=True can cause drift."
            )
        self.scan_before_bypass = bool(
            self.get_parameter("scan_before_bypass").value
        )
        self.scan_duration_s = max(
            0.0,
            float(self.get_parameter("scan_duration_s").value),
        )
        self.scan_spin_angle_deg = abs(
            float(self.get_parameter("scan_spin_angle_deg").value)
        )
        self.scan_cooldown_s = max(
            0.0,
            float(self.get_parameter("scan_cooldown_s").value),
        )
        self.corridor_search_angle_deg = float(
            self.get_parameter("corridor_search_angle_deg").value
        )

        self.heading_deg = None
        self.target_bearing_deg = None
        self.target_distance_m = None
        self.lidar_summary = None
        self.corridor = None
        self.semantic = None
        self.lidar_ts = 0.0
        self.corridor_ts = 0.0
        self.semantic_ts = 0.0
        self.last_corridor = None
        self.last_corridor_ts = 0.0

        self.mode = "CRUISE"
        self.pass_side = 0.0
        self.pass_started_ts = 0.0
        self.pass_last_obstacle_ts = 0.0
        self.active_obstacle_id = None
        self.target_left_m = 0.0
        self.pass_corridor = None
        self.pass_corridor_ts = 0.0
        self.recovery_until_ts = 0.0
        self.recovery_relative_bearing = 0.0
        self.scan_started_ts = 0.0
        self.last_scan_ts = -999.0
        self.scan_obstacle_id = None
        self.scan_left_score = 0.0
        self.scan_right_score = 0.0
        self.last_relative_bearing = None
        self.last_plan_ts = 0.0

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
            Float32,
            "/guidance/target_distance_m",
            self.target_distance_cb,
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

    def target_distance_cb(self, msg: Float32) -> None:
        self.target_distance_m = max(0.0, float(msg.data))

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

    def _target_relative_bearing(self) -> float:
        return normalize_angle_deg(self.target_bearing_deg - self.heading_deg)

    def _blend_relative_bearing(
        self,
        primary: float,
        secondary: float,
        weight: float,
    ) -> float:
        delta = normalize_angle_deg(secondary - primary)
        return normalize_angle_deg(primary + delta * clamp(weight, 0.0, 1.0))

    def _absolute_from_relative(self, relative_deg: float) -> float:
        return (self.heading_deg + relative_deg) % 360.0

    def _nav_relative_from_body_left(self, body_left_deg: float) -> float:
        return -body_left_deg

    def _body_left_from_nav_relative(self, nav_relative_deg: float) -> float:
        return -nav_relative_deg

    def _lidar_state(self) -> tuple[str, float | None, float]:
        now = now_ts()
        if self.lidar_summary is None:
            return "missing", None, 0.0
        if now - self.lidar_ts > self.sensor_timeout_s:
            return "timeout", None, 0.0

        front = float(self.lidar_summary.get("front_clearance_m", 999.0))
        front_sector = float(
            self.lidar_summary.get("front_sector_clearance_m", front)
        )
        front_path = float(
            self.lidar_summary.get("front_path_clearance_m", front)
        )
        front_footprint = float(
            self.lidar_summary.get("front_footprint_clearance_m", front_path)
        )
        best_free = float(self.lidar_summary.get("best_free_angle_deg", 0.0))
        if front_footprint < self.emergency_stop_distance_m:
            return "emergency", front_footprint, best_free
        if front_footprint < self.danger_distance_m:
            return "danger", front_footprint, best_free
        if front_sector < self.avoid_start_distance_m:
            return "avoid", front_sector, best_free
        return "clear", front_sector, best_free

    def _corridor_state(self):
        now = now_ts()
        current = None
        if self.corridor is not None and now - self.corridor_ts <= self.sensor_timeout_s:
            confidence = float(self.corridor.get("confidence", 0.0))
            if confidence >= self.corridor_min_confidence:
                center_left = self.corridor.get("center_left_m")
                width = self.corridor.get("estimated_width_m")
                bearing = float(self.corridor.get("center_bearing_deg", 0.0))
                if center_left is not None and width is not None:
                    current = {
                        "center_left_m": float(center_left),
                        "width_m": float(width),
                        "body_bearing_deg": clamp(
                            bearing,
                            -self.max_relative_bearing_deg,
                            self.max_relative_bearing_deg,
                        ),
                        "confidence": confidence,
                        "reason": "corridor_track",
                        "tracking_method": self.corridor.get("tracking_method"),
                    }

        if current is not None:
            self.last_corridor = current
            self.last_corridor_ts = now
            return current

        if (
            self.last_corridor is not None
            and now - self.last_corridor_ts <= self.corridor_coast_s
        ):
            if self.require_corridor_for_motion:
                return None
            coast = dict(self.last_corridor)
            coast["confidence"] = min(float(coast["confidence"]), 0.45)
            coast["reason"] = "corridor_coast"
            return coast

        return None

    def _corridor_bounds(self, corridor) -> tuple[float, float] | None:
        if corridor is None:
            return None
        width = float(corridor["width_m"])
        if width <= 0.0:
            return None
        center = float(corridor["center_left_m"])
        half = width / 2.0
        margin = max(self.corridor_keepout_margin_m, self.safe_clearance_m / 2.0)
        min_left = center - half + margin
        max_left = center + half - margin
        if min_left >= max_left:
            return None
        return min_left, max_left

    def _nearest_obstacle(self):
        if self.semantic is None:
            return None
        if now_ts() - self.semantic_ts > self.sensor_timeout_s:
            return None

        nearest = None
        for buoy in self.semantic.get("buoys", []):
            if buoy.get("semantic") not in ("obstacle_candidate", "unknown"):
                continue
            forward = buoy.get("forward_m")
            left = buoy.get("left_m")
            if forward is None or left is None:
                continue
            forward = float(forward)
            left = float(left)
            if forward < -0.8 or forward > self.avoid_start_distance_m + 1.0:
                continue
            if nearest is None or forward < nearest["forward_m"]:
                nearest = {
                    "id": buoy.get("id"),
                    "forward_m": forward,
                    "left_m": left,
                }
        return nearest

    def _obstacle_requires_pass(self, obstacle, corridor) -> bool:
        if obstacle is None:
            return False
        forward = float(obstacle["forward_m"])
        left = float(obstacle["left_m"])
        if forward <= -0.2 or forward > self.avoid_start_distance_m:
            return False

        path_left = 0.0 if corridor is None else float(corridor["center_left_m"])
        if abs(left - path_left) > self.path_block_margin_m:
            return False

        bounds = self._corridor_bounds(corridor)
        if bounds is None:
            return True

        min_left, max_left = bounds
        return min_left <= left <= max_left

    def _choose_pass_side(self, obstacle, corridor, best_free: float) -> float:
        gap_options = self._pass_gap_options(obstacle, corridor)
        best = gap_options.get("best")
        if best is not None:
            return float(best["side"])

        obstacle_left = float(obstacle["left_m"])
        left_width = gap_options.get("left_width_m")
        right_width = gap_options.get("right_width_m")
        if left_width is not None and right_width is not None:
            return 1.0 if left_width >= right_width else -1.0

        # When no corridor is available, always pass on the side OPPOSITE to
        # the obstacle.  Using best_free here is risky because LiDAR may report
        # the obstacle's far side as "free space", steering the boat *towards*
        # the obstacle.
        return -1.0 if obstacle_left > 0.0 else 1.0

    def _pass_gap_options(self, obstacle, corridor) -> dict:
        result = {
            "left_width_m": None,
            "right_width_m": None,
            "left_target_m": None,
            "right_target_m": None,
            "best": None,
        }
        if obstacle is None:
            return result
        bounds = self._corridor_bounds(corridor)
        if bounds is None:
            return result

        min_left, max_left = bounds
        obstacle_left = float(obstacle["left_m"])
        obstacle_clearance = max(
            self.obstacle_pass_margin_m,
            self.safe_clearance_m / 2.0,
        )

        left_start = obstacle_left + obstacle_clearance
        left_end = max_left
        left_width = max(0.0, left_end - left_start)
        right_start = min_left
        right_end = obstacle_left - obstacle_clearance
        right_width = max(0.0, right_end - right_start)

        result["left_width_m"] = left_width
        result["right_width_m"] = right_width
        if left_width > 0.0:
            result["left_target_m"] = (left_start + left_end) / 2.0
        if right_width > 0.0:
            result["right_target_m"] = (right_start + right_end) / 2.0

        feasible = []
        if left_width >= self.min_pass_gap_width_m:
            feasible.append(
                {
                    "side": 1.0,
                    "width_m": left_width,
                    "target_left_m": result["left_target_m"],
                    "label": "left",
                }
            )
        if right_width >= self.min_pass_gap_width_m:
            feasible.append(
                {
                    "side": -1.0,
                    "width_m": right_width,
                    "target_left_m": result["right_target_m"],
                    "label": "right",
                }
            )
        if feasible:
            result["best"] = max(feasible, key=lambda item: item["width_m"])
        else:
            # No fully-safe gap: if a partial gap exists (>0.2 m), favour the
            # larger side so BLOCKED_ALIGN can rotate toward it.
            partial = []
            if left_width > 0.2:
                partial.append(
                    {
                        "side": 1.0,
                        "width_m": left_width,
                        "target_left_m": result["left_target_m"],
                        "label": "left_partial",
                    }
                )
            if right_width > 0.2:
                partial.append(
                    {
                        "side": -1.0,
                        "width_m": right_width,
                        "target_left_m": result["right_target_m"],
                        "label": "right_partial",
                    }
                )
            if partial:
                result["best"] = max(
                    partial, key=lambda item: item["width_m"]
                )
        return result

    def _remember_pass_corridor(self, corridor) -> None:
        if corridor is None:
            return
        if corridor.get("reason") == "pass_corridor_memory":
            return
        self.pass_corridor = dict(corridor)
        self.pass_corridor_ts = now_ts()

    def _pass_corridor_from_memory(self):
        if self.pass_corridor is None:
            return None
        remembered = dict(self.pass_corridor)
        remembered["reason"] = "pass_corridor_memory"
        remembered["confidence"] = min(
            float(remembered.get("confidence", 0.0)),
            0.45,
        )
        return remembered

    def _pass_planning_corridor(self, corridor, obstacle=None):
        if corridor is not None:
            if obstacle is not None and self.pass_corridor is not None:
                live_has_gap = self._has_safe_pass_gap(obstacle, corridor)
                remembered = self._pass_corridor_from_memory()
                if not live_has_gap and self._has_safe_pass_gap(
                    obstacle,
                    remembered,
                ):
                    return remembered
            self._remember_pass_corridor(corridor)
            return corridor
        return self._pass_corridor_from_memory()

    def _has_safe_pass_gap(self, obstacle, corridor) -> bool:
        if obstacle is None:
            return True
        return self._pass_gap_options(obstacle, corridor).get("best") is not None

    def _stop_for_no_safe_pass(
        self,
        front_clearance,
        lidar_state: str,
        corridor,
        obstacle,
    ) -> None:
        # When blocked, stop forward motion but continue rotating toward
        # the target or corridor center so the vehicle can re-align.
        target_relative = self._target_relative_bearing()

        if corridor is not None:
            corridor_relative = self._nav_relative_from_body_left(
                clamp(
                    float(corridor["body_bearing_deg"]),
                    -self.max_return_bearing_deg,
                    self.max_return_bearing_deg,
                )
            )
            relative = self._blend_relative_bearing(
                corridor_relative, target_relative, 0.3
            )
        else:
            relative = target_relative

        relative = clamp(
            relative,
            -self.max_relative_bearing_deg,
            self.max_relative_bearing_deg,
        )

        self._publish_plan(
            "BLOCKED_ALIGN",
            relative,
            0.0,
            front_clearance,
            lidar_state,
            corridor,
            obstacle,
            "blocked_no_safe_pass_align",
        )

    def _scan_available(self, timestamp: float) -> bool:
        if not self.scan_before_bypass or self.scan_duration_s <= 0.0:
            return False
        return timestamp - self.last_scan_ts >= self.scan_cooldown_s

    def _start_scan_before_pass(
        self,
        obstacle,
        corridor,
        best_free: float,
        timestamp: float,
    ) -> None:
        self.mode = "SCAN_BEFORE_PASS"
        self.scan_started_ts = timestamp
        self.scan_obstacle_id = None if obstacle is None else obstacle.get("id")
        self.scan_left_score = 0.0
        self.scan_right_score = 0.0
        self.pass_side = self._choose_pass_side(obstacle, corridor, best_free)
        self._update_scan_scores(obstacle, corridor, best_free)

    def _update_scan_scores(self, obstacle, corridor, best_free: float) -> None:
        left_score = 0.0
        right_score = 0.0
        if abs(best_free) > 1.0:
            if best_free > 0.0:
                left_score += abs(best_free)
            else:
                right_score += abs(best_free)

        if obstacle is not None:
            bounds = self._corridor_bounds(corridor)
            obstacle_left = float(obstacle["left_m"])
            margin = max(self.obstacle_pass_margin_m, self.safe_clearance_m / 2.0)
            if bounds is not None:
                min_left, max_left = bounds
                left_gap = max_left - (obstacle_left + margin)
                right_gap = (obstacle_left - margin) - min_left
                left_score += max(0.0, left_gap) * 20.0
                right_score += max(0.0, right_gap) * 20.0
            else:
                if obstacle_left <= 0.0:
                    left_score += 10.0
                else:
                    right_score += 10.0

        self.scan_left_score = max(self.scan_left_score, left_score)
        self.scan_right_score = max(self.scan_right_score, right_score)

    def _finish_scan_before_pass(
        self,
        obstacle,
        corridor,
        best_free: float,
        timestamp: float,
    ) -> bool:
        self.last_scan_ts = timestamp
        if obstacle is None:
            self.mode = "RETURN_TO_CENTER" if corridor is not None else "CRUISE"
            return False

        if not self._has_safe_pass_gap(obstacle, corridor):
            self.mode = "BLOCKED_ALIGN"
            return False

        if self.scan_left_score > self.scan_right_score + 0.5:
            self.pass_side = 1.0
        elif self.scan_right_score > self.scan_left_score + 0.5:
            self.pass_side = -1.0
        else:
            self.pass_side = self._choose_pass_side(obstacle, corridor, best_free)

        self.mode = "PASS_COMMITTED"
        self.pass_started_ts = timestamp
        self.pass_last_obstacle_ts = timestamp
        self.active_obstacle_id = obstacle["id"]
        self._remember_pass_corridor(corridor)
        self.target_left_m = self._target_left_for_pass(obstacle, corridor)
        return True

    def _scan_relative_bearing(self, timestamp: float) -> float:
        elapsed = max(0.0, timestamp - self.scan_started_ts)
        phase = elapsed / max(self.scan_duration_s, 0.1)
        body_scan = (
            self.scan_spin_angle_deg
            if phase < 0.5
            else -self.scan_spin_angle_deg
        )
        return self._nav_relative_from_body_left(
            clamp(
                body_scan,
                -self.max_relative_bearing_deg,
                self.max_relative_bearing_deg,
            )
        )

    def _target_left_for_pass(self, obstacle, corridor) -> float:
        gap_options = self._pass_gap_options(obstacle, corridor)
        preferred_key = "left_target_m" if self.pass_side >= 0.0 else "right_target_m"
        preferred_target = gap_options.get(preferred_key)
        if preferred_target is not None:
            return float(preferred_target)

        best = gap_options.get("best")
        if best is not None and best.get("target_left_m") is not None:
            self.pass_side = float(best["side"])
            return float(best["target_left_m"])

        bounds = self._corridor_bounds(corridor)
        if bounds is not None:
            return clamp(float(corridor["center_left_m"]), bounds[0], bounds[1])
        return 0.0

    def _body_bearing_for_left(self, target_left_m: float, lookahead_m: float) -> float:
        return math.degrees(math.atan2(target_left_m, max(lookahead_m, 0.5)))

    def _pass_lookahead_for_obstacle(self, obstacle) -> float:
        if obstacle is None:
            return self.pass_lookahead_m
        forward = max(
            float(obstacle.get("forward_m", self.pass_lookahead_m)),
            0.0,
        )
        return clamp(forward, 1.5, self.pass_lookahead_m)

    def _lidar_escape_body_bearing(self, best_free: float) -> float:
        return clamp(
            float(best_free),
            -self.max_pass_bearing_deg,
            self.max_pass_bearing_deg,
        )

    def _clamp_body_bearing_to_corridor(
        self,
        body_bearing_deg: float,
        corridor,
        lookahead_m: float,
    ) -> float:
        bounds = self._corridor_bounds(corridor)
        if bounds is None:
            return body_bearing_deg

        lookahead = max(float(lookahead_m), 0.5)
        target_left = math.tan(math.radians(body_bearing_deg)) * lookahead
        clamped_left = clamp(target_left, bounds[0], bounds[1])
        if clamped_left == target_left:
            return body_bearing_deg
        return self._body_bearing_for_left(clamped_left, lookahead)

    def _rate_limit_relative(self, relative_bearing: float, timestamp: float) -> float:
        if self.last_relative_bearing is None or self.last_plan_ts <= 0.0:
            self.last_relative_bearing = relative_bearing
            self.last_plan_ts = timestamp
            return relative_bearing

        dt = max(timestamp - self.last_plan_ts, 1e-3)
        max_step = max(self.bearing_rate_limit_degps, 1.0) * dt
        delta = normalize_angle_deg(relative_bearing - self.last_relative_bearing)
        limited = self.last_relative_bearing + clamp(delta, -max_step, max_step)
        limited = clamp(
            normalize_angle_deg(limited),
            -self.max_relative_bearing_deg,
            self.max_relative_bearing_deg,
        )
        self.last_relative_bearing = limited
        self.last_plan_ts = timestamp
        return limited

    def _publish_plan(
        self,
        mode: str,
        relative_bearing: float,
        speed_limit: float,
        front_clearance: float | None,
        lidar_state: str,
        corridor,
        obstacle,
        reason: str,
    ) -> None:
        timestamp = now_ts()
        relative_bearing = clamp(
            relative_bearing,
            -self.max_relative_bearing_deg,
            self.max_relative_bearing_deg,
        )
        if mode == "EMERGENCY_STOP":
            relative_bearing = 0.0
            self.last_relative_bearing = 0.0
            self.last_plan_ts = timestamp
        else:
            relative_bearing = self._rate_limit_relative(relative_bearing, timestamp)

        pass_gaps = self._pass_gap_options(obstacle, corridor)
        selected_gap = pass_gaps.get("best")
        safe_bearing = self._absolute_from_relative(relative_bearing)
        self.safe_bearing_pub.publish(Float32(data=float(safe_bearing)))
        self.speed_limit_pub.publish(Float32(data=float(speed_limit)))
        self.status_pub.publish(
            String(
                data=to_json(
                    {
                        "timestamp": timestamp,
                        "mode": mode,
                        "safe_bearing_deg": safe_bearing,
                        "relative_bearing_deg": relative_bearing,
                        "speed_limit_mps": speed_limit,
                        "front_clearance_m": front_clearance,
                        "target_distance_m": self.target_distance_m,
                        "lidar_state": lidar_state,
                        "corridor_confidence": None
                        if corridor is None
                        else corridor["confidence"],
                        "corridor_center_left_m": None
                        if corridor is None
                        else corridor["center_left_m"],
                        "corridor_width_m": None
                        if corridor is None
                        else corridor["width_m"],
                        "corridor_reason": None
                        if corridor is None
                        else corridor["reason"],
                        "corridor_source": None
                        if corridor is None
                        else (
                            "pass_memory"
                            if corridor.get("reason") == "pass_corridor_memory"
                            else "live"
                        ),
                        "pass_corridor_memory_age_s": None
                        if self.pass_corridor is None
                        else max(0.0, timestamp - self.pass_corridor_ts),
                        "obstacle_id": None if obstacle is None else obstacle["id"],
                        "obstacle_forward_m": None
                        if obstacle is None
                        else obstacle["forward_m"],
                        "obstacle_left_m": None
                        if obstacle is None
                        else obstacle["left_m"],
                        "pass_side": self.pass_side,
                        "target_left_m": self.target_left_m,
                        "left_pass_gap_m": pass_gaps.get("left_width_m"),
                        "right_pass_gap_m": pass_gaps.get("right_width_m"),
                        "selected_pass_gap": None
                        if selected_gap is None
                        else selected_gap.get("label"),
                        "selected_pass_gap_width_m": None
                        if selected_gap is None
                        else selected_gap.get("width_m"),
                        "selected_pass_target_left_m": None
                        if selected_gap is None
                        else selected_gap.get("target_left_m"),
                        "min_pass_gap_width_m": self.min_pass_gap_width_m,
                        "scan_elapsed_s": max(0.0, timestamp - self.scan_started_ts)
                        if mode == "SCAN_BEFORE_PASS"
                        else None,
                        "scan_left_score": self.scan_left_score
                        if mode == "SCAN_BEFORE_PASS"
                        else None,
                        "scan_right_score": self.scan_right_score
                        if mode == "SCAN_BEFORE_PASS"
                        else None,
                        "scan_obstacle_id": self.scan_obstacle_id
                        if mode == "SCAN_BEFORE_PASS"
                        else None,
                        "reason": reason,
                    }
                )
            )
        )

    def _reset_pass(self) -> None:
        self.pass_side = 0.0
        self.pass_started_ts = 0.0
        self.pass_last_obstacle_ts = 0.0
        self.active_obstacle_id = None
        self.target_left_m = 0.0
        self.pass_corridor = None
        self.pass_corridor_ts = 0.0

    def _reset_scan(self) -> None:
        self.scan_started_ts = 0.0
        self.scan_obstacle_id = None
        self.scan_left_score = 0.0
        self.scan_right_score = 0.0

    def _start_recovery_backoff(self, best_free: float, timestamp: float) -> None:
        self.mode = "RECOVERY_BACKOFF"
        self.recovery_until_ts = timestamp + max(
            self.recovery_backoff_duration_s,
            0.1,
        )
        body_turn = best_free
        if abs(body_turn) < 1.0:
            if self.pass_side != 0.0:
                body_turn = self.pass_side * self.recovery_turn_bearing_deg
            else:
                body_turn = self.recovery_turn_bearing_deg
        body_turn = clamp(
            body_turn,
            -abs(self.recovery_turn_bearing_deg),
            abs(self.recovery_turn_bearing_deg),
        )
        self.recovery_relative_bearing = self._nav_relative_from_body_left(
            body_turn
        )

    def loop(self) -> None:
        if self.heading_deg is None or self.target_bearing_deg is None:
            return

        lidar_state, front_clearance, best_free = self._lidar_state()
        corridor = self._corridor_state()
        obstacle = self._nearest_obstacle()
        timestamp = now_ts()
        target_relative = self._target_relative_bearing()

        if self.mode == "RECOVERY_BACKOFF":
            if (
                timestamp < self.recovery_until_ts
                or lidar_state == "emergency"
            ):
                self._publish_plan(
                    self.mode,
                    self.recovery_relative_bearing,
                    self.recovery_backoff_speed_mps,
                    front_clearance,
                    lidar_state,
                    corridor,
                    obstacle,
                    "contact_recovery_backoff",
                )
                return
            self.mode = "RETURN_TO_CENTER" if corridor is not None else "CRUISE"
            self.recovery_until_ts = 0.0

        if lidar_state in ("missing", "timeout"):
            self.mode = "EMERGENCY_STOP"
            self._publish_plan(
                self.mode,
                0.0,
                0.0,
                front_clearance,
                lidar_state,
                corridor,
                obstacle,
                f"lidar_{lidar_state}",
            )
            return

        if lidar_state == "emergency":
            if (
                self.contact_recovery_enabled
                and front_clearance is not None
                and front_clearance <= self.contact_recovery_distance_m
            ):
                self._start_recovery_backoff(best_free, timestamp)
                self._publish_plan(
                    self.mode,
                    self.recovery_relative_bearing,
                    self.recovery_backoff_speed_mps,
                    front_clearance,
                    lidar_state,
                    corridor,
                    obstacle,
                    "front_contact_recovery",
                )
                return

            self.mode = "EMERGENCY_STOP"
            self._publish_plan(
                self.mode,
                0.0,
                0.0,
                front_clearance,
                lidar_state,
                corridor,
                obstacle,
                "front_emergency_stop",
            )
            return

        if self.mode == "SCAN_BEFORE_PASS":
            self._update_scan_scores(obstacle, corridor, best_free)
            elapsed = timestamp - self.scan_started_ts
            if elapsed < self.scan_duration_s:
                self._publish_plan(
                    self.mode,
                    self._scan_relative_bearing(timestamp),
                    min(self.approach_speed_mps * 0.5, self.max_speed_mps),
                    front_clearance,
                    lidar_state,
                    corridor,
                    obstacle,
                    "scan_before_pass",
                )
                return

            if self._finish_scan_before_pass(
                obstacle,
                corridor,
                best_free,
                timestamp,
            ):
                body_bearing = self._body_bearing_for_left(
                    self.target_left_m,
                    self.pass_lookahead_m,
                )
                relative = self._nav_relative_from_body_left(
                    clamp(
                        body_bearing,
                        -self.max_pass_bearing_deg,
                        self.max_pass_bearing_deg,
                    )
                )
                self._publish_plan(
                    self.mode,
                    relative,
                    min(self.approach_speed_mps, self.max_speed_mps),
                    front_clearance,
                    lidar_state,
                    corridor,
                    obstacle,
                    "scan_selected_pass_side",
                )
                self._reset_scan()
                return

            self._publish_plan(
                self.mode,
                target_relative,
                0.0,
                front_clearance,
                lidar_state,
                corridor,
                obstacle,
                "scan_aborted_no_obstacle",
            )
            self._reset_scan()
            return

        if self.mode == "PASS_COMMITTED":
            pass_corridor = self._pass_planning_corridor(corridor, obstacle)
            if obstacle is not None:
                self.pass_last_obstacle_ts = timestamp
                if not self._has_safe_pass_gap(obstacle, pass_corridor):
                    self._stop_for_no_safe_pass(
                        front_clearance,
                        lidar_state,
                        pass_corridor,
                        obstacle,
                    )
                    return

            pass_age = timestamp - self.pass_started_ts
            lost_age = timestamp - self.pass_last_obstacle_ts
            obstacle_behind = obstacle is not None and obstacle["forward_m"] < -1.0
            obstacle_lost = (
                obstacle is None
                and lost_age > self.pass_release_after_lost_s
                and (
                    front_clearance is None
                    or front_clearance > self.danger_distance_m
                )
            )
            if (
                pass_age >= self.pass_min_duration_s
                and (obstacle_behind or obstacle_lost)
            ):
                self.mode = "RETURN_TO_CENTER"
            else:
                if obstacle is not None:
                    self.target_left_m = self._target_left_for_pass(
                        obstacle,
                        pass_corridor,
                    )
                body_bearing = self._body_bearing_for_left(
                    self.target_left_m,
                    self._pass_lookahead_for_obstacle(obstacle),
                )
                reason = (
                    "committed_pass_corridor_memory"
                    if pass_corridor is not corridor
                    else "committed_pass"
                )
                speed = min(self.approach_speed_mps, self.max_speed_mps)
                if lidar_state == "danger":
                    speed = min(speed, 0.08)
                    escape_body_bearing = self._lidar_escape_body_bearing(best_free)
                    escape_body_bearing = self._clamp_body_bearing_to_corridor(
                        escape_body_bearing,
                        pass_corridor,
                        self._pass_lookahead_for_obstacle(obstacle),
                    )
                    if abs(escape_body_bearing) > 1.0:
                        body_bearing = escape_body_bearing
                        reason = f"{reason}_lidar_escape"
                relative = self._nav_relative_from_body_left(
                    clamp(
                        body_bearing,
                        -self.max_pass_bearing_deg,
                        self.max_pass_bearing_deg,
                    )
                )
                self._publish_plan(
                    self.mode,
                    relative,
                    speed,
                    front_clearance,
                    lidar_state,
                    pass_corridor,
                    obstacle,
                    reason,
                )
                return

        if self.mode == "RETURN_TO_CENTER":
            return_corridor = (
                self._pass_planning_corridor(corridor, obstacle)
                if obstacle is not None
                else corridor
            )
            if self._obstacle_requires_pass(obstacle, return_corridor):
                if not self._has_safe_pass_gap(obstacle, return_corridor):
                    self._stop_for_no_safe_pass(
                        front_clearance,
                        lidar_state,
                        return_corridor,
                        obstacle,
                    )
                    return
                if self._scan_available(timestamp):
                    self._start_scan_before_pass(
                        obstacle,
                        return_corridor,
                        best_free,
                        timestamp,
                    )
                    self._publish_plan(
                        self.mode,
                        self._scan_relative_bearing(timestamp),
                        min(self.approach_speed_mps * 0.5, self.max_speed_mps),
                        front_clearance,
                        lidar_state,
                        return_corridor,
                        obstacle,
                        "scan_before_pass_return",
                    )
                    return
                self.mode = "PASS_COMMITTED"
                obstacle_id = None if obstacle is None else obstacle.get("id")
                if self.pass_side == 0.0 or self.active_obstacle_id != obstacle_id:
                    self.pass_side = self._choose_pass_side(
                        obstacle, return_corridor, best_free
                    )
                self.pass_started_ts = timestamp
                self.pass_last_obstacle_ts = timestamp
                self.active_obstacle_id = obstacle["id"]
                self._remember_pass_corridor(return_corridor)
                self.target_left_m = self._target_left_for_pass(
                    obstacle,
                    return_corridor,
                )
            else:
                if return_corridor is not None:
                    if lidar_state in ("danger", "avoid"):
                        escape_body_bearing = self._lidar_escape_body_bearing(best_free)
                        escape_body_bearing = self._clamp_body_bearing_to_corridor(
                            escape_body_bearing,
                            return_corridor,
                            self.return_lookahead_m,
                        )
                        relative = self._nav_relative_from_body_left(
                            escape_body_bearing
                        )
                        speed = min(self.approach_speed_mps, self.max_speed_mps)
                        reason = "return_to_corridor_center_lidar_escape"
                        if lidar_state == "danger":
                            speed = min(speed, 0.08)
                        if abs(escape_body_bearing) <= 1.0:
                            reason = "return_to_corridor_center_lidar_slowdown"
                        self._publish_plan(
                            "RETURN_TO_CENTER",
                            relative,
                            speed,
                            front_clearance,
                            lidar_state,
                            return_corridor,
                            obstacle,
                            reason,
                        )
                        return

                    body_bearing = self._body_bearing_for_left(
                        float(return_corridor["center_left_m"]),
                        self.return_lookahead_m,
                    )
                    relative = self._nav_relative_from_body_left(
                        clamp(
                            body_bearing,
                            -self.max_return_bearing_deg,
                            self.max_return_bearing_deg,
                        )
                    )
                    if abs(float(return_corridor["center_left_m"])) < 0.35:
                        self.mode = "CRUISE"
                        if obstacle is None or not self._obstacle_requires_pass(
                            obstacle, return_corridor
                        ):
                            self._reset_pass()
                    self._publish_plan(
                        "RETURN_TO_CENTER",
                        relative,
                        min(self.approach_speed_mps, self.max_speed_mps),
                        front_clearance,
                        lidar_state,
                        return_corridor,
                        obstacle,
                        "return_to_corridor_center",
                    )
                    return

                self.mode = "CRUISE"
                if obstacle is None or not self._obstacle_requires_pass(
                    obstacle, return_corridor
                ):
                    self._reset_pass()

        if self._obstacle_requires_pass(obstacle, corridor):
            if not self._has_safe_pass_gap(obstacle, corridor):
                self._stop_for_no_safe_pass(
                    front_clearance,
                    lidar_state,
                    corridor,
                    obstacle,
                )
                return
            if self._scan_available(timestamp):
                self._start_scan_before_pass(
                    obstacle,
                    corridor,
                    best_free,
                    timestamp,
                )
                self._publish_plan(
                    self.mode,
                    self._scan_relative_bearing(timestamp),
                    min(self.approach_speed_mps * 0.5, self.max_speed_mps),
                    front_clearance,
                    lidar_state,
                    corridor,
                    obstacle,
                    "scan_before_pass",
                )
                return
            self.mode = "PASS_COMMITTED"
            if self.pass_side == 0.0:
                self.pass_side = self._choose_pass_side(
                    obstacle, corridor, best_free
                )
            self.pass_started_ts = timestamp
            self.pass_last_obstacle_ts = timestamp
            self.active_obstacle_id = obstacle["id"]
            self._remember_pass_corridor(corridor)
            self.target_left_m = self._target_left_for_pass(obstacle, corridor)
            body_bearing = self._body_bearing_for_left(
                self.target_left_m,
                self._pass_lookahead_for_obstacle(obstacle),
            )
            relative = self._nav_relative_from_body_left(
                clamp(
                    body_bearing,
                    -self.max_pass_bearing_deg,
                    self.max_pass_bearing_deg,
                )
            )
            self._publish_plan(
                self.mode,
                relative,
                min(self.approach_speed_mps, self.max_speed_mps),
                front_clearance,
                lidar_state,
                corridor,
                obstacle,
                "plan_pass",
            )
            return

        if corridor is None and self.require_corridor_for_motion:
            self.mode = "CORRIDOR_SEARCH"
            # Use last known corridor direction or target direction; keep moving slowly.
            if (
                self.last_corridor is not None
                and now_ts() - self.last_corridor_ts <= self.corridor_coast_s + 2.0
            ):
                relative = float(self.last_corridor.get("body_bearing_deg", 0.0))
            else:
                relative = target_relative
            relative = clamp(
                relative,
                -self.max_relative_bearing_deg,
                self.max_relative_bearing_deg,
            )
            self._publish_plan(
                self.mode,
                relative,
                min(self.approach_speed_mps * 0.5, self.max_speed_mps),
                front_clearance,
                lidar_state,
                corridor,
                obstacle,
                "searching_for_corridor",
            )
            return

        self.mode = "CRUISE"
        self._reset_pass()
        if corridor is not None:
            corridor_relative = self._nav_relative_from_body_left(
                float(corridor["body_bearing_deg"])
            )
            reason = corridor["reason"]
            speed = self.max_speed_mps
            if (
                self.target_distance_m is not None
                and self.target_distance_m <= self.final_approach_distance_m
            ):
                relative = target_relative
                reason = "final_waypoint_approach"
                speed = min(
                    speed,
                    max(
                        self.approach_speed_mps,
                        self.max_speed_mps
                        * self.target_distance_m
                        / max(self.final_approach_distance_m, 0.1),
                    ),
                )
            else:
                relative = self._blend_relative_bearing(
                    corridor_relative,
                    target_relative,
                    self.waypoint_bias_weight,
                )
        else:
            relative = target_relative
            reason = "waypoint_no_corridor"
            speed = min(self.approach_speed_mps, self.max_speed_mps)

        if lidar_state in ("danger", "avoid"):
            speed = min(speed, self.approach_speed_mps)
            if lidar_state == "danger":
                speed = min(speed, 0.08)
            escape_body_bearing = self._lidar_escape_body_bearing(best_free)
            escape_body_bearing = self._clamp_body_bearing_to_corridor(
                escape_body_bearing,
                corridor,
                self.pass_lookahead_m,
            )
            if abs(escape_body_bearing) > 1.0:
                relative = self._nav_relative_from_body_left(escape_body_bearing)
                reason = f"{reason}_lidar_escape"
            else:
                reason = f"{reason}_lidar_slowdown"

        self._publish_plan(
            self.mode,
            relative,
            speed,
            front_clearance,
            lidar_state,
            corridor,
            obstacle,
            reason,
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = Parkur2PlannerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
