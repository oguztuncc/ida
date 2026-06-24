#!/usr/bin/env python3
"""Anlık kontrol kararlarını gösteren terminal dashboard."""

import curses
import json
from typing import Any

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, Float32, Int32, String


class ControlDashboardNode(Node):
    def __init__(self, stdscr: Any) -> None:
        super().__init__("control_dashboard_node")
        self.stdscr = stdscr
        try:
            curses.curs_set(0)
        except Exception:
            pass

        # State
        self.cmd_vel: dict[str, float] = {}
        self.cmd_vel_safe: dict[str, float] = {}
        self.setpoints: dict[str, Any] = {}
        self.guidance: dict[str, Any] = {}
        self.planner: dict[str, Any] = {}
        self.safety: dict[str, Any] = {}
        self.active_wp = -1
        self.mission_started = False
        self.current_heading: float | None = None

        # Subscriptions
        self.create_subscription(Twist, "/control/cmd_vel", self.cmd_vel_cb, 10)
        self.create_subscription(
            Twist, "/control/cmd_vel_safe", self.cmd_vel_safe_cb, 10
        )
        self.create_subscription(String, "/control/setpoints", self.sp_cb, 10)
        self.create_subscription(String, "/guidance/status", self.gd_cb, 10)
        self.create_subscription(String, "/planner/status", self.pl_cb, 10)
        self.create_subscription(String, "/safety/status", self.sf_cb, 10)
        self.create_subscription(Int32, "/mission/active_waypoint", self.wp_cb, 10)
        self.create_subscription(Bool, "/mission/started", self.ms_cb, 10)
        self.create_subscription(
            Float32, "/mavros/global_position/compass_hdg", self.hd_cb, 10
        )

        self.timer = self.create_timer(0.1, self.draw)

    def cmd_vel_cb(self, msg: Twist) -> None:
        self.cmd_vel = {"linear": msg.linear.x, "angular": msg.angular.z}

    def cmd_vel_safe_cb(self, msg: Twist) -> None:
        self.cmd_vel_safe = {"linear": msg.linear.x, "angular": msg.angular.z}

    def sp_cb(self, msg: String) -> None:
        try:
            self.setpoints = json.loads(msg.data)
        except Exception:
            pass

    def gd_cb(self, msg: String) -> None:
        try:
            self.guidance = json.loads(msg.data)
        except Exception:
            pass

    def pl_cb(self, msg: String) -> None:
        try:
            self.planner = json.loads(msg.data)
        except Exception:
            pass

    def sf_cb(self, msg: String) -> None:
        try:
            self.safety = json.loads(msg.data)
        except Exception:
            pass

    def wp_cb(self, msg: Int32) -> None:
        self.active_wp = int(msg.data)

    def ms_cb(self, msg: Bool) -> None:
        self.mission_started = bool(msg.data)

    def hd_cb(self, msg: Float32) -> None:
        self.current_heading = float(msg.data)

    @staticmethod
    def _fmt(val: Any, fmt: str = "{:.2f}", default: str = "-") -> str:
        if val is None:
            return default
        try:
            return fmt.format(float(val))
        except Exception:
            return str(val)

    @staticmethod
    def _flag(val: Any, active: str = "AKTIF", inactive: str = "-") -> str:
        if isinstance(val, bool):
            return active if val else inactive
        if val is None:
            return inactive
        try:
            return active if float(val) > 0 else inactive
        except Exception:
            return active if val else inactive

    def _txt(self, y: int, x: int, text: str, attr: int = 0, clip: int = 200) -> None:
        try:
            self.stdscr.addnstr(y, x, text, clip, attr)
        except Exception:
            pass

    def _hline(self, y: int, x: int, w: int, char: str = "─") -> None:
        for i in range(w):
            try:
                self.stdscr.addch(y, x + i, char)
            except Exception:
                break

    def draw(self) -> None:
        self.stdscr.clear()
        my, mx = self.stdscr.getmaxyx()
        if my < 20 or mx < 70:
            self._txt(0, 0, "Terminal cok kucuk (min 20x70)")
            self.stdscr.refresh()
            return

        w = mx - 2
        # Header
        title = " IDA KONTROL DASHBOARD "
        self._txt(0, (mx - len(title)) // 2, title, curses.A_BOLD)
        self._hline(1, 1, w)

        # Status bar
        status = (
            f" WP: {self.active_wp}  |  "
            f"Heading: {self._fmt(self.current_heading, '{:.1f}°')}  |  "
            f"Mission: {'STARTED' if self.mission_started else 'WAITING'}  "
        )
        self._txt(2, 2, status)
        self._hline(3, 1, w)

        # Commands
        self._txt(4, 2, "KOMUTLAR", curses.A_BOLD)
        cv = self.cmd_vel
        cvs = self.cmd_vel_safe
        self._txt(5, 4, f"cmd_vel:      linear={self._fmt(cv.get('linear'))}  angular={self._fmt(cv.get('angular'))}")
        self._txt(6, 4, f"cmd_vel_safe: linear={self._fmt(cvs.get('linear'))}  angular={self._fmt(cvs.get('angular'))}")
        self._hline(7, 1, w)

        # Controller
        sp = self.setpoints
        source = sp.get("target_source", "-")
        preturn = bool(sp.get("preturn_active", False))
        stop_turn = bool(sp.get("waypoint_stop_turn_active", False))
        straight_brake = bool(sp.get("waypoint_straight_brake_active", False))
        slowdown = bool(sp.get("waypoint_slowdown_active", False))
        turn_place = bool(sp.get("turn_in_place", False))
        geofence = sp.get("geofence_outside_duration_s")
        geofence_active = geofence is not None and float(geofence) > 0

        self._txt(8, 2, "CONTROLLER KARARLARI", curses.A_BOLD)
        row = 9
        self._txt(row, 4, f"Kaynak (source):     {source}", curses.A_REVERSE if source not in ('guidance', '-') else 0)
        row += 1
        self._txt(row, 4, f"Heading Error:       {self._fmt(sp.get('heading_error_deg'), '{:.1f}°')}")
        row += 1
        self._txt(row, 4, f"P-return:            {self._flag(preturn)}  (blend={self._fmt(sp.get('preturn_blend'))})", curses.A_REVERSE if preturn else 0)
        row += 1
        self._txt(row, 4, f"Stop-Turn:           {self._flag(stop_turn)}", curses.A_REVERSE if stop_turn else 0)
        row += 1
        self._txt(row, 4, f"Straight-Brake:      {self._flag(straight_brake)}", curses.A_REVERSE if straight_brake else 0)
        row += 1
        self._txt(row, 4, f"Waypoint Slowdown:   {self._flag(slowdown)}  (dist={self._fmt(sp.get('waypoint_slowdown_distance_m'))})", curses.A_REVERSE if slowdown else 0)
        row += 1
        self._txt(row, 4, f"Turn-in-Place:       {self._flag(turn_place)}", curses.A_REVERSE if turn_place else 0)
        row += 1
        self._txt(row, 4, f"Planner Mode:        {sp.get('planner_mode', '-')} | {sp.get('planner_reason', '-')}")
        row += 1
        self._txt(row, 4, f"Planner Speed Limit: {self._fmt(sp.get('planner_speed_limit_mps'))} m/s")
        row += 1
        self._txt(row, 4, f"Geofence Outside:    {self._fmt(geofence, '{:.1f}s') if geofence_active else '-'}", curses.A_REVERSE if geofence_active else 0)
        row += 1
        self._hline(row, 1, w)
        row += 1

        # Two-column section: Guidance | Planner
        half = w // 2
        self._txt(row, 2, "GUIDANCE", curses.A_BOLD)
        self._txt(row, 2 + half, "PLANNER", curses.A_BOLD)
        row += 1
        gd = self.guidance
        pl = self.planner
        self._txt(row, 4, f"Target Bearing:  {self._fmt(gd.get('target_bearing_deg'), '{:.1f}°')}")
        self._txt(row, 4 + half, f"Mode:         {pl.get('mode', '-')}")
        row += 1
        self._txt(row, 4, f"Waypoint Bearing:{self._fmt(gd.get('waypoint_bearing_deg'), '{:.1f}°')}")
        self._txt(row, 4 + half, f"Reason:       {pl.get('reason', '-')}")
        row += 1
        self._txt(row, 4, f"Leg Bearing:     {self._fmt(gd.get('leg_bearing_deg'), '{:.1f}°')}")
        self._txt(row, 4 + half, f"Lidar State:  {pl.get('lidar_state', '-')}")
        row += 1
        self._txt(row, 4, f"Distance:        {self._fmt(gd.get('target_distance_m'))} m")
        self._txt(row, 4 + half, f"Front Clear:  {self._fmt(pl.get('front_clearance_m'))} m")
        row += 1
        self._txt(row, 4, f"Upcoming Turn:   {self._fmt(gd.get('upcoming_turn_angle_deg'), '{:.1f}°')}")
        self._txt(row, 4 + half, f"Corridor Conf:{self._fmt(pl.get('corridor_confidence'))}")
        row += 1
        self._txt(row, 4, f"Next Bearing:    {self._fmt(gd.get('next_bearing_deg'), '{:.1f}°')}")
        self._txt(row, 4 + half, f"Obstacle:     {pl.get('obstacle_id', '-')}")
        row += 1
        self._hline(row, 1, w)
        row += 1

        # Safety
        self._txt(row, 2, "SAFETY", curses.A_BOLD)
        row += 1
        sf = self.safety
        kill = bool(sf.get("kill_active", False)) if sf else False
        phys = bool(sf.get("physical_kill_active", False)) if sf else False
        self._txt(row, 4, f"Kill Active:   {self._flag(kill, 'EVET', 'HAYIR')}", curses.A_REVERSE if kill else 0)
        self._txt(row, 4 + half, f"Physical Kill: {self._flag(phys, 'EVET', 'HAYIR')}", curses.A_REVERSE if phys else 0)
        row += 1
        tout = bool(sf.get("command_timed_out", False)) if sf else False
        self._txt(row, 4, f"Cmd Timeout:   {self._flag(tout, 'EVET', 'HAYIR')}", curses.A_REVERSE if tout else 0)

        # Footer
        hint = " Ctrl+C ile cikis "
        self._txt(my - 1, mx - len(hint) - 2, hint, curses.A_DIM)

        self.stdscr.refresh()


def main(stdscr=None):
    rclpy.init()
    node = ControlDashboardNode(stdscr)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    curses.wrapper(main)
