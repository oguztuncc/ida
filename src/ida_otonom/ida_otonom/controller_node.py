from math import fabs

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float32, String
from geometry_msgs.msg import Twist

from .common import normalize_angle_deg, clamp, from_json, to_json


class ControllerNode(Node):
    def __init__(self) -> None:
        super().__init__("controller_node")

        self.declare_parameter("kp_heading", 0.02)
        self.declare_parameter("max_linear_speed", 0.45)
        self.declare_parameter("max_angular_speed", 0.8)
        self.declare_parameter("use_planner_bearing", False)
        self.declare_parameter("planner_timeout_s", 0.5)
        self.declare_parameter("allow_reverse_planner_speed", False)
        self.declare_parameter("geofence_timeout_s", 1.0)
        self.declare_parameter("turn_in_place_error_deg", 0.0)
        self.declare_parameter("turn_in_place_speed_mps", 0.0)
        self.declare_parameter("preturn_enabled", False)
        self.declare_parameter("preturn_distance_m", 0.0)
        self.declare_parameter("preturn_turn_angle_deg", 60.0)
        self.declare_parameter("preturn_speed_mps", 0.08)
        self.declare_parameter("preturn_max_blend", 0.60)
        self.declare_parameter("preturn_brake_distance_m", 0.0)
        self.declare_parameter("preturn_brake_speed_mps", 0.0)
        self.declare_parameter("waypoint_slowdown_distance_m", 0.0)
        self.declare_parameter("waypoint_min_speed_mps", 0.04)
        self.declare_parameter("waypoint_stop_turn_enabled", False)
        self.declare_parameter("waypoint_stop_turn_distance_m", 0.0)
        self.declare_parameter("waypoint_stop_turn_angle_deg", 35.0)
        self.declare_parameter("waypoint_stop_turn_align_error_deg", 10.0)
        self.declare_parameter("waypoint_stop_turn_depart_s", 1.0)
        self.declare_parameter("waypoint_stop_turn_depart_speed_mps", 0.10)
        self.declare_parameter("waypoint_straight_brake_enabled", False)
        self.declare_parameter("waypoint_straight_brake_distance_m", 2.0)
        self.declare_parameter("waypoint_straight_brake_heading_error_deg", 8.0)

        self.kp_heading = float(self.get_parameter("kp_heading").value)
        self.max_linear_speed = float(
            self.get_parameter("max_linear_speed").value
        )
        self.max_angular_speed = float(
            self.get_parameter("max_angular_speed").value
        )
        self.use_planner_bearing = bool(
            self.get_parameter("use_planner_bearing").value
        )
        self.planner_timeout_s = float(
            self.get_parameter("planner_timeout_s").value
        )
        self.allow_reverse_planner_speed = bool(
            self.get_parameter("allow_reverse_planner_speed").value
        )
        self.geofence_timeout_s = float(
            self.get_parameter("geofence_timeout_s").value
        )
        self.turn_in_place_error_deg = abs(
            float(self.get_parameter("turn_in_place_error_deg").value)
        )
        self.turn_in_place_speed_mps = float(
            self.get_parameter("turn_in_place_speed_mps").value
        )
        self.preturn_enabled = bool(
            self.get_parameter("preturn_enabled").value
        )
        self.preturn_distance_m = max(
            0.0,
            float(self.get_parameter("preturn_distance_m").value),
        )
        self.preturn_turn_angle_deg = abs(
            float(self.get_parameter("preturn_turn_angle_deg").value)
        )
        self.preturn_speed_mps = max(
            0.0,
            float(self.get_parameter("preturn_speed_mps").value),
        )
        self.preturn_max_blend = clamp(
            float(self.get_parameter("preturn_max_blend").value),
            0.0,
            1.0,
        )
        self.preturn_brake_distance_m = max(
            0.0,
            float(self.get_parameter("preturn_brake_distance_m").value),
        )
        self.preturn_brake_speed_mps = float(
            self.get_parameter("preturn_brake_speed_mps").value
        )
        self.waypoint_slowdown_distance_m = max(
            0.0,
            float(self.get_parameter("waypoint_slowdown_distance_m").value),
        )
        self.waypoint_min_speed_mps = max(
            0.0,
            float(self.get_parameter("waypoint_min_speed_mps").value),
        )
        self.waypoint_stop_turn_enabled = bool(
            self.get_parameter("waypoint_stop_turn_enabled").value
        )
        self.waypoint_stop_turn_distance_m = max(
            0.0,
            float(self.get_parameter("waypoint_stop_turn_distance_m").value),
        )
        self.waypoint_stop_turn_angle_deg = abs(
            float(self.get_parameter("waypoint_stop_turn_angle_deg").value)
        )
        self.waypoint_stop_turn_align_error_deg = abs(
            float(
                self.get_parameter(
                    "waypoint_stop_turn_align_error_deg"
                ).value
            )
        )
        self.waypoint_stop_turn_depart_s = max(
            0.0,
            float(self.get_parameter("waypoint_stop_turn_depart_s").value),
        )
        self.waypoint_stop_turn_depart_speed_mps = max(
            0.0,
            float(
                self.get_parameter(
                    "waypoint_stop_turn_depart_speed_mps"
                ).value
            ),
        )
        self.waypoint_straight_brake_enabled = bool(
            self.get_parameter("waypoint_straight_brake_enabled").value
        )
        self.waypoint_straight_brake_distance_m = max(
            0.0,
            float(
                self.get_parameter(
                    "waypoint_straight_brake_distance_m"
                ).value
            ),
        )
        self.waypoint_straight_brake_heading_error_deg = abs(
            float(
                self.get_parameter(
                    "waypoint_straight_brake_heading_error_deg"
                ).value
            )
        )

        self.current_heading = None
        self.target_bearing = None
        self.target_distance = None
        self.geofence_status = None
        self.geofence_ts = 0.0
        self.planner_bearing = None
        self.planner_speed_limit = None
        self.planner_bearing_ts = 0.0
        self.planner_speed_ts = 0.0
        self.planner_status = {}
        self.planner_status_ts = 0.0
        self.vision_heading_bias = 0.0
        self.upcoming_turn_angle = 0.0
        self.next_bearing = None
        self.active_waypoint_index = None
        self.guidance_target_bearing = None
        self.guidance_waypoint_bearing = None
        self.guidance_leg_bearing = None
        self.waypoint_stop_turn_pending = False
        self.mission_started = False
        self.mission_completed = False
        self.waypoint_stop_turn_active = False
        self.waypoint_stop_turn_bearing = None
        self.waypoint_stop_turn_depart_until = 0.0

        self.cmd_pub = self.create_publisher(Twist, "/control/cmd_vel", 10)
        self.setpoint_pub = self.create_publisher(
            String,
            "/control/setpoints",
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
            self.target_bearing_cb,
            10,
        )
        self.create_subscription(
            Float32,
            "/guidance/target_distance_m",
            self.target_distance_cb,
            10,
        )
        self.create_subscription(
            Float32,
            "/planner/safe_bearing_deg",
            self.planner_bearing_cb,
            10,
        )
        self.create_subscription(
            Float32,
            "/planner/speed_limit_mps",
            self.planner_speed_limit_cb,
            10,
        )
        self.create_subscription(
            String,
            "/planner/status",
            self.planner_status_cb,
            10,
        )
        self.create_subscription(
            String,
            "/perception/corridor_hint",
            self.corridor_hint_cb,
            10,
        )
        self.create_subscription(
            String,
            "/guidance/status",
            self.guidance_status_cb,
            10,
        )
        self.create_subscription(
            String,
            "/geofence/status",
            self.geofence_status_cb,
            10,
        )
        self.create_subscription(
            Bool,
            "/mission/completed",
            self.mission_completed_cb,
            10,
        )
        self.create_subscription(
            Bool,
            "/mission/started",
            self.mission_started_cb,
            10,
        )

        self.timer = self.create_timer(0.1, self.loop)

    def heading_cb(self, msg: Float32) -> None:
        self.current_heading = float(msg.data)

    def target_bearing_cb(self, msg: Float32) -> None:
        self.target_bearing = float(msg.data)

    def target_distance_cb(self, msg: Float32) -> None:
        self.target_distance = float(msg.data)

    def planner_bearing_cb(self, msg: Float32) -> None:
        self.planner_bearing = float(msg.data)
        self.planner_bearing_ts = self.get_clock().now().nanoseconds / 1e9

    def planner_speed_limit_cb(self, msg: Float32) -> None:
        speed_limit = float(msg.data)
        if not self.allow_reverse_planner_speed:
            speed_limit = max(0.0, speed_limit)
        self.planner_speed_limit = speed_limit
        self.planner_speed_ts = self.get_clock().now().nanoseconds / 1e9

    def planner_status_cb(self, msg: String) -> None:
        try:
            self.planner_status = from_json(msg.data)
            self.planner_status_ts = self.get_clock().now().nanoseconds / 1e9
        except Exception:
            self.planner_status = {}

    def corridor_hint_cb(self, msg: String) -> None:
        try:
            data = from_json(msg.data)
            self.vision_heading_bias = float(data.get("heading_bias_deg", 0.0))
        except Exception:
            self.vision_heading_bias = 0.0

    def guidance_status_cb(self, msg: String) -> None:
        try:
            data = from_json(msg.data)
            previous_index = self.active_waypoint_index
            active_index = data.get("active_waypoint_index")
            self.active_waypoint_index = (
                None if active_index is None else int(active_index)
            )
            target_bearing = data.get("target_bearing_deg")
            self.guidance_target_bearing = (
                None if target_bearing is None else float(target_bearing)
            )
            waypoint_bearing = data.get("waypoint_bearing_deg")
            self.guidance_waypoint_bearing = (
                None if waypoint_bearing is None else float(waypoint_bearing)
            )
            leg_bearing = data.get("leg_bearing_deg")
            self.guidance_leg_bearing = (
                None if leg_bearing is None else float(leg_bearing)
            )
            self.upcoming_turn_angle = float(
                data.get("upcoming_turn_angle_deg", 0.0) or 0.0
            )
            next_bearing = data.get("next_bearing_deg")
            self.next_bearing = None if next_bearing is None else float(next_bearing)
            if (
                self.waypoint_stop_turn_enabled
                and previous_index is not None
                and self.active_waypoint_index is not None
                and self.active_waypoint_index > previous_index
            ):
                self.waypoint_stop_turn_pending = True
                self.waypoint_stop_turn_active = False
                self.waypoint_stop_turn_depart_until = 0.0
                self.waypoint_stop_turn_bearing = None
            elif (
                previous_index is not None
                and self.active_waypoint_index is not None
                and self.active_waypoint_index < previous_index
            ):
                self.waypoint_stop_turn_pending = False
                self.waypoint_stop_turn_active = False
                self.waypoint_stop_turn_depart_until = 0.0
                self.waypoint_stop_turn_bearing = None
        except Exception:
            self.upcoming_turn_angle = 0.0
            self.next_bearing = None
            self.guidance_waypoint_bearing = None
            self.guidance_leg_bearing = None

    def _waypoint_stop_turn_target(self):
        if self.guidance_leg_bearing is not None:
            return self.guidance_leg_bearing
        if self.guidance_waypoint_bearing is not None:
            return self.guidance_waypoint_bearing
        return self.guidance_target_bearing

    def geofence_status_cb(self, msg: String) -> None:
        try:
            self.geofence_status = from_json(msg.data)
            self.geofence_ts = self.get_clock().now().nanoseconds / 1e9
        except Exception:
            self.geofence_status = None

    def mission_completed_cb(self, msg: Bool) -> None:
        self.mission_completed = bool(msg.data)

    def mission_started_cb(self, msg: Bool) -> None:
        self.mission_started = bool(msg.data)

    def publish_stop(self, reason: str) -> None:
        self.get_logger().info(
            f"STOP: {reason}",
            throttle_duration_sec=2.0,
        )
        cmd = Twist()
        self.cmd_pub.publish(cmd)
        self.setpoint_pub.publish(
            String(
                data=to_json(
                    {
                        "speed_setpoint": 0.0,
                        "yaw_rate_setpoint": 0.0,
                        "heading_error_deg": 0.0,
                        "vision_heading_bias_deg": self.vision_heading_bias,
                        "stop_reason": reason,
                    }
                )
            )
        )

    def _planner_fresh(self) -> bool:
        if self.planner_bearing is None:
            return False
        now = self.get_clock().now().nanoseconds / 1e9
        return now - self.planner_bearing_ts <= self.planner_timeout_s

    def _speed_limit_fresh(self) -> bool:
        if self.planner_speed_limit is None:
            return False
        now = self.get_clock().now().nanoseconds / 1e9
        return now - self.planner_speed_ts <= self.planner_timeout_s

    def _planner_status_fresh(self) -> bool:
        if not self.planner_status:
            return False
        now = self.get_clock().now().nanoseconds / 1e9
        return now - self.planner_status_ts <= self.planner_timeout_s

    def _geofence_override(self):
        if self.geofence_status is None:
            return None
        now = self.get_clock().now().nanoseconds / 1e9
        if now - self.geofence_ts > self.geofence_timeout_s:
            return None
        if not bool(self.geofence_status.get("outside", False)):
            return None
        bearing = self.geofence_status.get("return_bearing_deg")
        if bearing is None:
            return None
        return {
            "bearing_deg": float(bearing),
            "speed_mps": max(
                0.0,
                float(self.geofence_status.get("return_speed_mps", 0.12)),
            ),
            "outside_duration_s": float(
                self.geofence_status.get("outside_duration_s", 0.0)
            ),
            "penalty_equivalent_exit_count": int(
                self.geofence_status.get("penalty_equivalent_exit_count", 0)
            ),
        }

    def loop(self) -> None:
        if self.mission_completed:
            self.waypoint_stop_turn_active = False
            self.waypoint_stop_turn_pending = False
            self.publish_stop("mission_completed")
            return

        if not self.mission_started:
            self.waypoint_stop_turn_active = False
            self.waypoint_stop_turn_pending = False
            self.publish_stop("mission_not_started")
            return

        if self.current_heading is None:
            return

        # Parkur 3: planner bearing varsa target_bearing şart değil
        using_planner = (
            self.use_planner_bearing
            and self._planner_fresh()
            and self.planner_bearing is not None
        )
        geofence = self._geofence_override()
        planner_status_fresh = self._planner_status_fresh()
        planner_mode = None
        planner_reason = None
        planner_lidar_state = None
        if planner_status_fresh:
            planner_mode = str(self.planner_status.get("mode", ""))
            planner_reason = str(self.planner_status.get("reason", ""))
            planner_lidar_state = str(self.planner_status.get("lidar_state", ""))

        if self.target_bearing is None and not using_planner and geofence is None:
            self.publish_stop("no_target_bearing")
            return

        target_source = "guidance"
        if geofence is not None:
            corrected_target = geofence["bearing_deg"]
            target_source = "geofence_return"
        elif using_planner:
            corrected_target = self.planner_bearing
            target_source = "planner"
        else:
            corrected_target = self.target_bearing + self.vision_heading_bias

        now = self.get_clock().now().nanoseconds / 1e9
        waypoint_stop_turn_depart_active = (
            self.waypoint_stop_turn_bearing is not None
            and now <= self.waypoint_stop_turn_depart_until
        )
        if (
            self.waypoint_stop_turn_pending
            and geofence is None
            and self._waypoint_stop_turn_target() is not None
        ):
            self.waypoint_stop_turn_active = True
            self.waypoint_stop_turn_pending = False
            self.waypoint_stop_turn_bearing = self._waypoint_stop_turn_target()

        if self.waypoint_stop_turn_active:
            corrected_target = self.waypoint_stop_turn_bearing
            target_source = "waypoint_stop_turn_align"
        elif waypoint_stop_turn_depart_active:
            corrected_target = self.waypoint_stop_turn_bearing
            target_source = "waypoint_stop_turn_depart"

        preturn_active = False
        preturn_blend = 0.0
        if (
            self.preturn_enabled
            and target_source == "guidance"
            and self.target_distance is not None
            and self.next_bearing is not None
            and self.preturn_distance_m > 0.0
            and self.upcoming_turn_angle >= self.preturn_turn_angle_deg
            and self.target_distance <= self.preturn_distance_m
        ):
            preturn_active = True
            proximity = 1.0 - self.target_distance / self.preturn_distance_m
            preturn_blend = clamp(proximity, 0.0, self.preturn_max_blend)
            corrected_target = corrected_target + normalize_angle_deg(
                self.next_bearing - corrected_target
            ) * preturn_blend

        heading_error = normalize_angle_deg(
            corrected_target - self.current_heading
        )

        if (
            self.waypoint_stop_turn_active
            and fabs(heading_error) <= self.waypoint_stop_turn_align_error_deg
        ):
            self.waypoint_stop_turn_active = False
            self.waypoint_stop_turn_depart_until = (
                now + self.waypoint_stop_turn_depart_s
            )
            target_source = "waypoint_stop_turn_depart"

        # Parkur 3: planner kullanıyorsa hız limitini planner'dan al
        if geofence is not None:
            linear_speed = min(geofence["speed_mps"], self.max_linear_speed)
        elif using_planner and self._speed_limit_fresh():
            if self.allow_reverse_planner_speed:
                linear_speed = clamp(
                    self.planner_speed_limit,
                    -self.max_linear_speed,
                    self.max_linear_speed,
                )
            else:
                linear_speed = min(self.planner_speed_limit, self.max_linear_speed)
        elif using_planner:
            # Planner kullanılıyor ama speed limit eski, default hız kullan
            if fabs(heading_error) < 10.0:
                linear_speed = self.max_linear_speed
            elif fabs(heading_error) < 25.0:
                linear_speed = 0.28
            elif fabs(heading_error) < 45.0:
                linear_speed = 0.15
            else:
                linear_speed = 0.05
        elif fabs(heading_error) < 10.0:
            linear_speed = self.max_linear_speed
        elif fabs(heading_error) < 25.0:
            linear_speed = 0.28
        elif fabs(heading_error) < 45.0:
            linear_speed = 0.15
        else:
            linear_speed = 0.05

        if preturn_active:
            linear_speed = min(linear_speed, self.preturn_speed_mps)
            preturn_brake_active = self.target_distance <= self.preturn_brake_distance_m
            if preturn_brake_active:
                linear_speed = min(linear_speed, self.preturn_brake_speed_mps)
        else:
            preturn_brake_active = False

        waypoint_stop_turn_align_active = (
            target_source == "waypoint_stop_turn_align"
        )
        waypoint_stop_turn_depart_active = (
            target_source == "waypoint_stop_turn_depart"
        )
        if waypoint_stop_turn_align_active:
            linear_speed = 0.0
        elif waypoint_stop_turn_depart_active:
            linear_speed = min(
                linear_speed,
                self.waypoint_stop_turn_depart_speed_mps,
            )

        waypoint_slowdown_active = False
        if (
            target_source == "guidance"
            and self.target_distance is not None
            and self.waypoint_slowdown_distance_m > 0.0
            and self.target_distance <= self.waypoint_slowdown_distance_m
        ):
            waypoint_slowdown_active = True
            slowdown_ratio = self.target_distance / self.waypoint_slowdown_distance_m
            slowdown_limit = max(
                self.waypoint_min_speed_mps,
                self.max_linear_speed * slowdown_ratio,
            )
            linear_speed = min(linear_speed, slowdown_limit)

        planner_blocks_straight_brake = (
            planner_status_fresh and planner_lidar_state in ("danger", "avoid")
        )
        waypoint_straight_brake_active = (
            self.waypoint_straight_brake_enabled
            and self.target_distance is not None
            and self.target_distance <= self.waypoint_straight_brake_distance_m
            and fabs(heading_error)
            <= self.waypoint_straight_brake_heading_error_deg
            and geofence is None
            and not waypoint_stop_turn_align_active
            and not waypoint_stop_turn_depart_active
            and not planner_blocks_straight_brake
        )
        turning_in_place = (
            self.turn_in_place_error_deg > 0.0
            and fabs(heading_error) >= self.turn_in_place_error_deg
            and not preturn_brake_active
            and not waypoint_stop_turn_depart_active
            and not waypoint_straight_brake_active
        )
        if turning_in_place:
            linear_speed = clamp(
                self.turn_in_place_speed_mps,
                -self.max_linear_speed,
                self.max_linear_speed,
            )
        else:
            linear_speed = clamp(
                linear_speed,
                -self.max_linear_speed,
                self.max_linear_speed,
            )

        # Dinamik kazanç: büyük heading error'da daha agresif dönüş
        effective_kp = self.kp_heading
        if fabs(heading_error) > 45.0:
            effective_kp = self.kp_heading * 2.0
        elif fabs(heading_error) > 20.0:
            effective_kp = self.kp_heading * 1.5
        angular_speed = clamp(
            heading_error * effective_kp,
            -self.max_angular_speed,
            self.max_angular_speed,
        )
        if waypoint_straight_brake_active:
            angular_speed = 0.0

        cmd = Twist()
        cmd.linear.x = float(linear_speed)
        cmd.angular.z = float(angular_speed)
        self.cmd_pub.publish(cmd)

        self.get_logger().info(
            f"cmd: linear={linear_speed:.2f} angular={angular_speed:.2f} "
            f"source={target_source} turn_in_place={turning_in_place} "
            f"preturn={preturn_active} "
            f"stop_turn={waypoint_stop_turn_align_active} "
            f"straight_brake={waypoint_straight_brake_active} "
            f"planner_fresh={self._planner_fresh()}",
            throttle_duration_sec=1.0,
        )

        self.setpoint_pub.publish(
            String(
                data=to_json(
                    {
                        "speed_setpoint": linear_speed,
                        "yaw_rate_setpoint": angular_speed,
                        "heading_error_deg": heading_error,
                        "vision_heading_bias_deg": self.vision_heading_bias,
                        "target_source": target_source,
                        "turn_in_place": turning_in_place,
                        "turn_in_place_error_deg": (
                            self.turn_in_place_error_deg
                        ),
                        "preturn_active": preturn_active,
                        "preturn_blend": preturn_blend,
                        "preturn_brake_active": preturn_brake_active,
                        "waypoint_slowdown_active": waypoint_slowdown_active,
                        "waypoint_slowdown_distance_m": (
                            self.waypoint_slowdown_distance_m
                        ),
                        "waypoint_straight_brake_active": (
                            waypoint_straight_brake_active
                        ),
                        "waypoint_straight_brake_distance_m": (
                            self.waypoint_straight_brake_distance_m
                        ),
                        "waypoint_straight_brake_heading_error_deg": (
                            self.waypoint_straight_brake_heading_error_deg
                        ),
                        "waypoint_stop_turn_active": (
                            waypoint_stop_turn_align_active
                        ),
                        "waypoint_stop_turn_pending": (
                            self.waypoint_stop_turn_pending
                        ),
                        "waypoint_stop_turn_depart_active": (
                            waypoint_stop_turn_depart_active
                        ),
                        "waypoint_stop_turn_bearing_deg": (
                            self.waypoint_stop_turn_bearing
                        ),
                        "upcoming_turn_angle_deg": self.upcoming_turn_angle,
                        "next_bearing_deg": self.next_bearing,
                        "active_waypoint_index": self.active_waypoint_index,
                        "guidance_waypoint_bearing_deg": (
                            self.guidance_waypoint_bearing
                        ),
                        "guidance_leg_bearing_deg": self.guidance_leg_bearing,
                        "planner_speed_limit_mps": self.planner_speed_limit,
                        "planner_mode": planner_mode,
                        "planner_reason": planner_reason,
                        "planner_lidar_state": planner_lidar_state,
                        "geofence_outside_duration_s": None
                        if geofence is None
                        else geofence["outside_duration_s"],
                        "geofence_penalty_equivalent_exit_count": None
                        if geofence is None
                        else geofence["penalty_equivalent_exit_count"],
                        "allow_reverse_planner_speed": (
                            self.allow_reverse_planner_speed
                        ),
                    }
                )
            )
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
