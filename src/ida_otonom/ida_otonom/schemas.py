"""
JSON mesajlar icin tip guvenli dataclass parser/validator katmani.

Kritik topic'lerdeki std_msgs/String JSON yapilari bu moduldeki
dataclass'lar uzerinden parse edilir. Eksik alanlar icin guvenli
default degerler kullanilir (backward compatibility).
"""

from __future__ import annotations

import json
from dataclasses import dataclass, field
from typing import Any, Dict, Optional

from std_msgs.msg import String


@dataclass
class LidarSummary:
    """LiDAR islemci ciktisi: /perception/lidar_summary."""

    timestamp: float = 0.0
    collision_imminent: bool = False
    front_clearance_m: float = 999.0
    front_sector_clearance_m: float = 999.0
    front_path_clearance_m: float = 999.0
    front_footprint_clearance_m: float = 999.0
    front_path_half_width_m: float = 0.7
    vehicle_width_m: float = 0.76
    vehicle_length_m: float = 1.11
    left_clearance_m: float = 999.0
    right_clearance_m: float = 999.0
    best_free_angle_deg: float = 0.0
    best_free_clearance_m: float = 999.0
    avoidance_sign: float = 0.0

    @classmethod
    def parse(cls, msg: String) -> Optional[LidarSummary]:
        """std_msgs/String mesajindan LidarSummary uret."""
        try:
            data: Dict[str, Any] = json.loads(msg.data)
        except Exception:
            return None
        return cls(
            timestamp=float(data.get("timestamp", 0.0)),
            collision_imminent=bool(
                data.get("collision_imminent", False)
            ),
            front_clearance_m=float(
                data.get("front_clearance_m", 999.0)
            ),
            front_sector_clearance_m=float(
                data.get("front_sector_clearance_m", 999.0)
            ),
            front_path_clearance_m=float(
                data.get("front_path_clearance_m", 999.0)
            ),
            front_footprint_clearance_m=float(
                data.get("front_footprint_clearance_m", 999.0)
            ),
            front_path_half_width_m=float(
                data.get("front_path_half_width_m", 0.7)
            ),
            vehicle_width_m=float(
                data.get("vehicle_width_m", 0.76)
            ),
            vehicle_length_m=float(
                data.get("vehicle_length_m", 1.11)
            ),
            left_clearance_m=float(
                data.get("left_clearance_m", 999.0)
            ),
            right_clearance_m=float(
                data.get("right_clearance_m", 999.0)
            ),
            best_free_angle_deg=float(
                data.get("best_free_angle_deg", 0.0)
            ),
            best_free_clearance_m=float(
                data.get("best_free_clearance_m", 999.0)
            ),
            avoidance_sign=float(
                data.get("avoidance_sign", 0.0)
            ),
        )

    def to_msg(self) -> String:
        """ROS std_msgs/String mesaji uret."""
        payload = {
            "timestamp": self.timestamp,
            "collision_imminent": self.collision_imminent,
            "front_clearance_m": self.front_clearance_m,
            "front_sector_clearance_m": self.front_sector_clearance_m,
            "front_path_clearance_m": self.front_path_clearance_m,
            "front_footprint_clearance_m": self.front_footprint_clearance_m,
            "front_path_half_width_m": self.front_path_half_width_m,
            "vehicle_width_m": self.vehicle_width_m,
            "vehicle_length_m": self.vehicle_length_m,
            "left_clearance_m": self.left_clearance_m,
            "right_clearance_m": self.right_clearance_m,
            "best_free_angle_deg": self.best_free_angle_deg,
            "best_free_clearance_m": self.best_free_clearance_m,
            "avoidance_sign": self.avoidance_sign,
        }
        return String(data=json.dumps(payload, ensure_ascii=False))


@dataclass
class CorridorEstimate:
    """Koridor takip ciktisi: /planner/corridor."""

    timestamp: float = 0.0
    status: str = "no_corridor"
    center_bearing_deg: float = 0.0
    center_left_m: float = 0.0
    raw_center_left_m: Optional[float] = None
    confidence: float = 0.0
    left_boundary_count: int = 0
    right_boundary_count: int = 0
    left_boundary_estimate_m: Optional[float] = None
    right_boundary_estimate_m: Optional[float] = None
    estimated_width_m: Optional[float] = None
    min_gate_width_m: float = 1.8
    tracking_method: str = "none"
    obstacle_offset_m: float = 0.0
    nearest_obstacle: Optional[Dict[str, Any]] = None
    obstacle_report_only: bool = True
    gate: Dict[str, Any] = field(default_factory=dict)

    @classmethod
    def parse(cls, msg: String) -> Optional[CorridorEstimate]:
        """std_msgs/String mesajindan CorridorEstimate uret."""
        try:
            data: Dict[str, Any] = json.loads(msg.data)
        except Exception:
            return None
        return cls(
            timestamp=float(data.get("timestamp", 0.0)),
            status=str(data.get("status", "no_corridor")),
            center_bearing_deg=float(
                data.get("center_bearing_deg", 0.0)
            ),
            center_left_m=float(data.get("center_left_m", 0.0)),
            raw_center_left_m=data.get("raw_center_left_m"),
            confidence=float(data.get("confidence", 0.0)),
            left_boundary_count=int(
                data.get("left_boundary_count", 0)
            ),
            right_boundary_count=int(
                data.get("right_boundary_count", 0)
            ),
            left_boundary_estimate_m=data.get(
                "left_boundary_estimate_m"
            ),
            right_boundary_estimate_m=data.get(
                "right_boundary_estimate_m"
            ),
            estimated_width_m=data.get("estimated_width_m"),
            min_gate_width_m=float(
                data.get("min_gate_width_m", 1.8)
            ),
            tracking_method=str(
                data.get("tracking_method", "none")
            ),
            obstacle_offset_m=float(
                data.get("obstacle_offset_m", 0.0)
            ),
            nearest_obstacle=data.get("nearest_obstacle"),
            obstacle_report_only=bool(
                data.get("obstacle_report_only", True)
            ),
            gate=data.get("gate", {}),
        )

    def to_msg(self) -> String:
        """ROS std_msgs/String mesaji uret."""
        payload = {
            "timestamp": self.timestamp,
            "status": self.status,
            "center_bearing_deg": self.center_bearing_deg,
            "center_left_m": self.center_left_m,
            "raw_center_left_m": self.raw_center_left_m,
            "confidence": self.confidence,
            "left_boundary_count": self.left_boundary_count,
            "right_boundary_count": self.right_boundary_count,
            "left_boundary_estimate_m": self.left_boundary_estimate_m,
            "right_boundary_estimate_m": self.right_boundary_estimate_m,
            "estimated_width_m": self.estimated_width_m,
            "min_gate_width_m": self.min_gate_width_m,
            "tracking_method": self.tracking_method,
            "obstacle_offset_m": self.obstacle_offset_m,
            "nearest_obstacle": self.nearest_obstacle,
            "obstacle_report_only": self.obstacle_report_only,
            "gate": self.gate,
        }
        return String(data=json.dumps(payload, ensure_ascii=False))


@dataclass
class PlannerStatus:
    """Planner karar ciktisi: /planner/status."""

    timestamp: float = 0.0
    mode: str = "CRUISE"
    safe_bearing_deg: float = 0.0
    relative_bearing_deg: float = 0.0
    speed_limit_mps: float = 999.0
    front_clearance_m: Optional[float] = None
    target_distance_m: Optional[float] = None
    lidar_state: str = "missing"
    corridor_confidence: Optional[float] = None
    corridor_center_left_m: Optional[float] = None
    corridor_width_m: Optional[float] = None
    corridor_reason: Optional[str] = None
    corridor_source: Optional[str] = None
    pass_corridor_memory_age_s: Optional[float] = None
    obstacle_id: Optional[str] = None
    obstacle_forward_m: Optional[float] = None
    obstacle_left_m: Optional[float] = None
    pass_side: float = 0.0
    target_left_m: float = 0.0
    left_pass_gap_m: Optional[float] = None
    right_pass_gap_m: Optional[float] = None
    selected_pass_gap: Optional[str] = None
    selected_pass_gap_width_m: Optional[float] = None
    selected_pass_target_left_m: Optional[float] = None
    min_pass_gap_width_m: float = 1.2
    scan_elapsed_s: Optional[float] = None
    scan_left_score: Optional[float] = None
    scan_right_score: Optional[float] = None
    scan_obstacle_id: Optional[str] = None
    reason: str = "init"

    @classmethod
    def parse(cls, msg: String) -> Optional[PlannerStatus]:
        """std_msgs/String mesajindan PlannerStatus uret."""
        try:
            data: Dict[str, Any] = json.loads(msg.data)
        except Exception:
            return None
        return cls(
            timestamp=float(data.get("timestamp", 0.0)),
            mode=str(data.get("mode", "CRUISE")),
            safe_bearing_deg=float(
                data.get("safe_bearing_deg", 0.0)
            ),
            relative_bearing_deg=float(
                data.get("relative_bearing_deg", 0.0)
            ),
            speed_limit_mps=float(
                data.get("speed_limit_mps", 999.0)
            ),
            front_clearance_m=data.get("front_clearance_m"),
            target_distance_m=data.get("target_distance_m"),
            lidar_state=str(data.get("lidar_state", "missing")),
            corridor_confidence=data.get("corridor_confidence"),
            corridor_center_left_m=data.get(
                "corridor_center_left_m"
            ),
            corridor_width_m=data.get("corridor_width_m"),
            corridor_reason=data.get("corridor_reason"),
            corridor_source=data.get("corridor_source"),
            pass_corridor_memory_age_s=data.get(
                "pass_corridor_memory_age_s"
            ),
            obstacle_id=data.get("obstacle_id"),
            obstacle_forward_m=data.get("obstacle_forward_m"),
            obstacle_left_m=data.get("obstacle_left_m"),
            pass_side=float(data.get("pass_side", 0.0)),
            target_left_m=float(data.get("target_left_m", 0.0)),
            left_pass_gap_m=data.get("left_pass_gap_m"),
            right_pass_gap_m=data.get("right_pass_gap_m"),
            selected_pass_gap=data.get("selected_pass_gap"),
            selected_pass_gap_width_m=data.get(
                "selected_pass_gap_width_m"
            ),
            selected_pass_target_left_m=data.get(
                "selected_pass_target_left_m"
            ),
            min_pass_gap_width_m=float(
                data.get("min_pass_gap_width_m", 1.2)
            ),
            scan_elapsed_s=data.get("scan_elapsed_s"),
            scan_left_score=data.get("scan_left_score"),
            scan_right_score=data.get("scan_right_score"),
            scan_obstacle_id=data.get("scan_obstacle_id"),
            reason=str(data.get("reason", "init")),
        )

    def to_msg(self) -> String:
        """ROS std_msgs/String mesaji uret."""
        payload = {
            "timestamp": self.timestamp,
            "mode": self.mode,
            "safe_bearing_deg": self.safe_bearing_deg,
            "relative_bearing_deg": self.relative_bearing_deg,
            "speed_limit_mps": self.speed_limit_mps,
            "front_clearance_m": self.front_clearance_m,
            "target_distance_m": self.target_distance_m,
            "lidar_state": self.lidar_state,
            "corridor_confidence": self.corridor_confidence,
            "corridor_center_left_m": self.corridor_center_left_m,
            "corridor_width_m": self.corridor_width_m,
            "corridor_reason": self.corridor_reason,
            "corridor_source": self.corridor_source,
            "pass_corridor_memory_age_s": (
                self.pass_corridor_memory_age_s
            ),
            "obstacle_id": self.obstacle_id,
            "obstacle_forward_m": self.obstacle_forward_m,
            "obstacle_left_m": self.obstacle_left_m,
            "pass_side": self.pass_side,
            "target_left_m": self.target_left_m,
            "left_pass_gap_m": self.left_pass_gap_m,
            "right_pass_gap_m": self.right_pass_gap_m,
            "selected_pass_gap": self.selected_pass_gap,
            "selected_pass_gap_width_m": (
                self.selected_pass_gap_width_m
            ),
            "selected_pass_target_left_m": (
                self.selected_pass_target_left_m
            ),
            "min_pass_gap_width_m": self.min_pass_gap_width_m,
            "scan_elapsed_s": self.scan_elapsed_s,
            "scan_left_score": self.scan_left_score,
            "scan_right_score": self.scan_right_score,
            "scan_obstacle_id": self.scan_obstacle_id,
            "reason": self.reason,
        }
        return String(data=json.dumps(payload, ensure_ascii=False))


@dataclass
class MissionStatus:
    """Gorev yoneticisi ciktisi: /mission/status."""

    mission_loaded: bool = False
    mission_started: bool = False
    mission_completed: bool = False
    all_missions_completed: bool = False
    active_waypoint_index: int = 0
    waypoint_count: int = 0
    current_mission_index: int = 0
    total_mission_count: int = 0
    runtime_mission_load_enabled: bool = False
    runtime_mission_load_required: bool = False

    @classmethod
    def parse(cls, msg: String) -> Optional[MissionStatus]:
        try:
            data: Dict[str, Any] = json.loads(msg.data)
        except Exception:
            return None
        return cls(
            mission_loaded=bool(
                data.get("mission_loaded", False)
            ),
            mission_started=bool(
                data.get("mission_started", False)
            ),
            mission_completed=bool(
                data.get("mission_completed", False)
            ),
            all_missions_completed=bool(
                data.get("all_missions_completed", False)
            ),
            active_waypoint_index=int(
                data.get("active_waypoint_index", 0)
            ),
            waypoint_count=int(data.get("waypoint_count", 0)),
            current_mission_index=int(
                data.get("current_mission_index", 0)
            ),
            total_mission_count=int(
                data.get("total_mission_count", 0)
            ),
            runtime_mission_load_enabled=bool(
                data.get("runtime_mission_load_enabled", False)
            ),
            runtime_mission_load_required=bool(
                data.get("runtime_mission_load_required", False)
            ),
        )

    def to_msg(self) -> String:
        payload = {
            "mission_loaded": self.mission_loaded,
            "mission_started": self.mission_started,
            "mission_completed": self.mission_completed,
            "all_missions_completed": self.all_missions_completed,
            "active_waypoint_index": self.active_waypoint_index,
            "waypoint_count": self.waypoint_count,
            "current_mission_index": self.current_mission_index,
            "total_mission_count": self.total_mission_count,
            "runtime_mission_load_enabled": (
                self.runtime_mission_load_enabled
            ),
            "runtime_mission_load_required": (
                self.runtime_mission_load_required
            ),
        }
        return String(data=json.dumps(payload, ensure_ascii=False))


@dataclass
class GuidanceStatus:
    """GPS rehberlik ciktisi: /guidance/status."""

    active_waypoint_index: int = 0
    target_lat: float = 0.0
    target_lon: float = 0.0
    target_bearing_deg: float = 0.0
    waypoint_bearing_deg: float = 0.0
    leg_bearing_deg: float = 0.0
    target_distance_m: float = 0.0
    mission_started: bool = False
    route_lookahead_enabled: bool = False
    route_lookahead_m: float = 0.0
    route_lookahead_cross_turns: bool = False
    route_lookahead_target: Optional[Any] = None
    next_waypoint_index: Optional[int] = None
    next_bearing_deg: Optional[float] = None
    upcoming_turn_angle_deg: float = 0.0

    @classmethod
    def parse(cls, msg: String) -> Optional[GuidanceStatus]:
        try:
            data: Dict[str, Any] = json.loads(msg.data)
        except Exception:
            return None
        return cls(
            active_waypoint_index=int(
                data.get("active_waypoint_index", 0)
            ),
            target_lat=float(data.get("target_lat", 0.0)),
            target_lon=float(data.get("target_lon", 0.0)),
            target_bearing_deg=float(
                data.get("target_bearing_deg", 0.0)
            ),
            waypoint_bearing_deg=float(
                data.get("waypoint_bearing_deg", 0.0)
            ),
            leg_bearing_deg=float(
                data.get("leg_bearing_deg", 0.0)
            ),
            target_distance_m=float(
                data.get("target_distance_m", 0.0)
            ),
            mission_started=bool(
                data.get("mission_started", False)
            ),
            route_lookahead_enabled=bool(
                data.get("route_lookahead_enabled", False)
            ),
            route_lookahead_m=float(
                data.get("route_lookahead_m", 0.0)
            ),
            route_lookahead_cross_turns=bool(
                data.get("route_lookahead_cross_turns", False)
            ),
            route_lookahead_target=data.get(
                "route_lookahead_target"
            ),
            next_waypoint_index=data.get("next_waypoint_index"),
            next_bearing_deg=data.get("next_bearing_deg"),
            upcoming_turn_angle_deg=float(
                data.get("upcoming_turn_angle_deg", 0.0)
            ),
        )

    def to_msg(self) -> String:
        payload = {
            "active_waypoint_index": self.active_waypoint_index,
            "target_lat": self.target_lat,
            "target_lon": self.target_lon,
            "target_bearing_deg": self.target_bearing_deg,
            "waypoint_bearing_deg": self.waypoint_bearing_deg,
            "leg_bearing_deg": self.leg_bearing_deg,
            "target_distance_m": self.target_distance_m,
            "mission_started": self.mission_started,
            "route_lookahead_enabled": (
                self.route_lookahead_enabled
            ),
            "route_lookahead_m": self.route_lookahead_m,
            "route_lookahead_cross_turns": (
                self.route_lookahead_cross_turns
            ),
            "route_lookahead_target": self.route_lookahead_target,
            "next_waypoint_index": self.next_waypoint_index,
            "next_bearing_deg": self.next_bearing_deg,
            "upcoming_turn_angle_deg": self.upcoming_turn_angle_deg,
        }
        return String(data=json.dumps(payload, ensure_ascii=False))


@dataclass
class SafetyStatus:
    """Guvenlik katmani ciktisi: /safety/status."""

    kill_active: bool = False
    physical_kill_active: bool = False
    latch_kill: bool = False
    block_reset_on_physical_kill: bool = False
    command_timed_out: bool = False

    @classmethod
    def parse(cls, msg: String) -> Optional[SafetyStatus]:
        try:
            data: Dict[str, Any] = json.loads(msg.data)
        except Exception:
            return None
        return cls(
            kill_active=bool(data.get("kill_active", False)),
            physical_kill_active=bool(
                data.get("physical_kill_active", False)
            ),
            latch_kill=bool(data.get("latch_kill", False)),
            block_reset_on_physical_kill=bool(
                data.get("block_reset_on_physical_kill", False)
            ),
            command_timed_out=bool(
                data.get("command_timed_out", False)
            ),
        )

    def to_msg(self) -> String:
        payload = {
            "kill_active": self.kill_active,
            "physical_kill_active": self.physical_kill_active,
            "latch_kill": self.latch_kill,
            "block_reset_on_physical_kill": (
                self.block_reset_on_physical_kill
            ),
            "command_timed_out": self.command_timed_out,
        }
        return String(data=json.dumps(payload, ensure_ascii=False))
