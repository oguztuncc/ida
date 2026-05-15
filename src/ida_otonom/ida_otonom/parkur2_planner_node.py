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
        self.declare_parameter("safe_clearance_m", 1.50)
        self.declare_parameter("max_relative_bearing_deg", 35.0)
        self.declare_parameter("max_pass_bearing_deg", 25.0)
        self.declare_parameter("max_return_bearing_deg", 18.0)
        self.declare_parameter("corridor_min_confidence", 0.45)
        self.declare_parameter("corridor_keepout_margin_m", 1.20)
        self.declare_parameter("obstacle_pass_margin_m", 1.20)
        self.declare_parameter("pass_lookahead_m", 6.0)
        self.declare_parameter("return_lookahead_m", 6.0)
        self.declare_parameter("pass_min_duration_s", 1.5)
        self.declare_parameter("pass_release_after_lost_s", 1.0)
        self.declare_parameter("corridor_coast_s", 1.2)
        self.declare_parameter("bearing_rate_limit_degps", 70.0)
        self.declare_parameter("sensor_timeout_s", 1.0)
        self.declare_parameter("final_approach_distance_m", 12.0)
        self.declare_parameter("waypoint_bias_weight", 0.15)

        # Legacy parameters may still exist in YAML files; declare them so
        # launch overrides remain harmless while this node uses the new state
        # machine below.
        self.declare_parameter("scan_before_bypass", False)
        self.declare_parameter("scan_duration_s", 0.0)
        self.declare_parameter("scan_spin_angle_deg", 0.0)
        self.declare_parameter("scan_cooldown_s", 0.0)
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
        self.final_approach_distance_m = float(
            self.get_parameter("final_approach_distance_m").value
        )
        self.waypoint_bias_weight = clamp(
            float(self.get_parameter("waypoint_bias_weight").value),
            0.0,
            1.0,
        )
        self.require_corridor_for_motion = bool(
            self.get_parameter("require_corridor_for_motion").value
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
        best_free = float(self.lidar_summary.get("best_free_angle_deg", 0.0))
        if front < self.emergency_stop_distance_m:
            return "emergency", front, best_free
        if front < self.danger_distance_m:
            return "danger", front, best_free
        if front < self.avoid_start_distance_m:
            return "avoid", front, best_free
        return "clear", front, best_free

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
                    }

        if current is not None:
            self.last_corridor = current
            self.last_corridor_ts = now
            return current

        if (
            self.last_corridor is not None
            and now - self.last_corridor_ts <= self.corridor_coast_s
        ):
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

        bounds = self._corridor_bounds(corridor)
        if bounds is None:
            return abs(left) <= self.safe_clearance_m

        min_left, max_left = bounds
        return min_left <= left <= max_left

    def _choose_pass_side(self, obstacle, corridor, best_free: float) -> float:
        obstacle_left = float(obstacle["left_m"])
        bounds = self._corridor_bounds(corridor)
        if bounds is not None:
            min_left, max_left = bounds
            margin = max(self.obstacle_pass_margin_m, self.safe_clearance_m / 2.0)
            left_gap = max_left - (obstacle_left + margin)
            right_gap = (obstacle_left - margin) - min_left
            if left_gap > 0.0 or right_gap > 0.0:
                return 1.0 if left_gap >= right_gap else -1.0

        if abs(best_free) > 1.0:
            return 1.0 if best_free > 0.0 else -1.0
        return -1.0 if obstacle_left > 0.0 else 1.0

    def _target_left_for_pass(self, obstacle, corridor) -> float:
        obstacle_left = float(obstacle["left_m"])
        side = 1.0 if self.pass_side >= 0.0 else -1.0
        target_left = obstacle_left + side * self.obstacle_pass_margin_m
        bounds = self._corridor_bounds(corridor)
        if bounds is not None:
            target_left = clamp(target_left, bounds[0], bounds[1])
        return target_left

    def _body_bearing_for_left(self, target_left_m: float, lookahead_m: float) -> float:
        return math.degrees(math.atan2(target_left_m, max(lookahead_m, 0.5)))

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

        safe_bearing = self._absolute_from_relative(relative_bearing)
        self.safe_bearing_pub.publish(Float32(data=float(safe_bearing)))
        self.speed_limit_pub.publish(Float32(data=float(max(0.0, speed_limit))))
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
                        "obstacle_id": None if obstacle is None else obstacle["id"],
                        "obstacle_forward_m": None
                        if obstacle is None
                        else obstacle["forward_m"],
                        "obstacle_left_m": None
                        if obstacle is None
                        else obstacle["left_m"],
                        "pass_side": self.pass_side,
                        "target_left_m": self.target_left_m,
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

    def loop(self) -> None:
        if self.heading_deg is None or self.target_bearing_deg is None:
            return

        lidar_state, front_clearance, best_free = self._lidar_state()
        corridor = self._corridor_state()
        obstacle = self._nearest_obstacle()
        timestamp = now_ts()
        target_relative = self._target_relative_bearing()

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

        if self.mode == "PASS_COMMITTED":
            if obstacle is not None:
                self.pass_last_obstacle_ts = timestamp

            pass_age = timestamp - self.pass_started_ts
            lost_age = timestamp - self.pass_last_obstacle_ts
            obstacle_behind = obstacle is not None and obstacle["forward_m"] < -0.4
            obstacle_lost = obstacle is None and lost_age > self.pass_release_after_lost_s
            if pass_age >= self.pass_min_duration_s and (obstacle_behind or obstacle_lost):
                self.mode = "RETURN_TO_CENTER"
            else:
                if obstacle is not None:
                    self.target_left_m = self._target_left_for_pass(obstacle, corridor)
                body_bearing = self._body_bearing_for_left(
                    self.target_left_m,
                    self.pass_lookahead_m,
                )
                relative = self._nav_relative_from_body_left(
                    clamp(body_bearing, -self.max_pass_bearing_deg, self.max_pass_bearing_deg)
                )
                speed = min(self.approach_speed_mps, self.max_speed_mps)
                if lidar_state == "danger":
                    speed = min(speed, 0.08)
                self._publish_plan(
                    self.mode,
                    relative,
                    speed,
                    front_clearance,
                    lidar_state,
                    corridor,
                    obstacle,
                    "committed_pass",
                )
                return

        if self.mode == "RETURN_TO_CENTER":
            if self._obstacle_requires_pass(obstacle, corridor):
                self.mode = "PASS_COMMITTED"
                self.pass_started_ts = timestamp
                self.pass_last_obstacle_ts = timestamp
                self.active_obstacle_id = obstacle["id"]
                self.target_left_m = self._target_left_for_pass(obstacle, corridor)
            else:
                if corridor is not None:
                    body_bearing = self._body_bearing_for_left(
                        float(corridor["center_left_m"]),
                        self.return_lookahead_m,
                    )
                    relative = self._nav_relative_from_body_left(
                        clamp(
                            body_bearing,
                            -self.max_return_bearing_deg,
                            self.max_return_bearing_deg,
                        )
                    )
                    if abs(float(corridor["center_left_m"])) < 0.35:
                        self.mode = "CRUISE"
                        self._reset_pass()
                    self._publish_plan(
                        "RETURN_TO_CENTER",
                        relative,
                        min(self.approach_speed_mps, self.max_speed_mps),
                        front_clearance,
                        lidar_state,
                        corridor,
                        obstacle,
                        "return_to_corridor_center",
                    )
                    return

                self.mode = "CRUISE"
                self._reset_pass()

        if self._obstacle_requires_pass(obstacle, corridor):
            self.mode = "PASS_COMMITTED"
            self.pass_side = self._choose_pass_side(obstacle, corridor, best_free)
            self.pass_started_ts = timestamp
            self.pass_last_obstacle_ts = timestamp
            self.active_obstacle_id = obstacle["id"]
            self.target_left_m = self._target_left_for_pass(obstacle, corridor)
            body_bearing = self._body_bearing_for_left(
                self.target_left_m,
                self.pass_lookahead_m,
            )
            relative = self._nav_relative_from_body_left(
                clamp(body_bearing, -self.max_pass_bearing_deg, self.max_pass_bearing_deg)
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
            relative = clamp(
                self.corridor_search_angle_deg,
                -self.max_relative_bearing_deg,
                self.max_relative_bearing_deg,
            )
            self._publish_plan(
                self.mode,
                relative,
                0.0,
                front_clearance,
                lidar_state,
                corridor,
                obstacle,
                "waiting_for_validated_corridor",
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
            # Without a semantic obstacle inside the corridor, LiDAR only slows
            # the boat down. It must not become the primary steering source.
            speed = min(speed, self.approach_speed_mps)
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
