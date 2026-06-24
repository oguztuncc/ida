#!/usr/bin/env python3
"""Gazebo motor itki komutlarini ve kontrol kararlarini gosteren dashboard."""

import json
import tkinter as tk
from tkinter import TclError

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String, Int32


class MotorThrustViewer(Node):
    def __init__(self):
        super().__init__('motor_thrust_viewer')

        self.declare_parameter('status_topic', '/sim/motor_thrust')
        self.declare_parameter('window_title', 'IDA Motor Kontrol Dashboard')
        self.declare_parameter('refresh_hz', 20.0)

        self.status_topic = self.get_parameter('status_topic').value
        self.window_title = self.get_parameter('window_title').value
        self.refresh_hz = float(self.get_parameter('refresh_hz').value)
        self.status = {
            'left_thrust_n': 0.0,
            'right_thrust_n': 0.0,
            'left_percent': 0.0,
            'right_percent': 0.0,
            'left_drive_thrust_n': 0.0,
            'right_drive_thrust_n': 0.0,
            'left_drive_percent': 0.0,
            'right_drive_percent': 0.0,
            'max_thrust_n': 1.0,
            'stale': True,
        }
        self.closed = False

        # Controller / Guidance / Planner state
        self.controller: dict = {}
        self.guidance: dict = {}
        self.planner: dict = {}
        self.cmd_vel: dict = {'linear': 0.0, 'angular': 0.0}
        self.active_wp = -1

        self.create_subscription(String, self.status_topic, self.status_cb, 10)
        self.create_subscription(String, '/control/setpoints', self.setpoints_cb, 10)
        self.create_subscription(String, '/guidance/status', self.guidance_cb, 10)
        self.create_subscription(String, '/planner/status', self.planner_cb, 10)
        self.create_subscription(Twist, '/control/cmd_vel', self.cmd_vel_cb, 10)
        self.create_subscription(Int32, '/mission/active_waypoint', self.wp_cb, 10)

        self.root = tk.Tk()
        self.root.title(self.window_title)
        self.root.geometry('960x640')
        self.root.configure(bg='#0f172a')
        self.root.protocol('WM_DELETE_WINDOW', self.close)

        self.canvas = tk.Canvas(
            self.root,
            width=960,
            height=640,
            bg='#0f172a',
            highlightthickness=0,
        )
        self.canvas.pack(fill=tk.BOTH, expand=True)

    def status_cb(self, msg: String) -> None:
        try:
            self.status = json.loads(msg.data)
        except Exception:
            return

    def setpoints_cb(self, msg: String) -> None:
        try:
            self.controller = json.loads(msg.data)
        except Exception:
            pass

    def guidance_cb(self, msg: String) -> None:
        try:
            self.guidance = json.loads(msg.data)
        except Exception:
            pass

    def planner_cb(self, msg: String) -> None:
        try:
            self.planner = json.loads(msg.data)
        except Exception:
            pass

    def cmd_vel_cb(self, msg: Twist) -> None:
        self.cmd_vel = {'linear': msg.linear.x, 'angular': msg.angular.z}

    def wp_cb(self, msg: Int32) -> None:
        self.active_wp = int(msg.data)

    def close(self) -> None:
        self.closed = True
        self.root.quit()

    def run(self) -> None:
        self._tick()
        self.root.mainloop()

    def _tick(self) -> None:
        if self.closed:
            return
        rclpy.spin_once(self, timeout_sec=0.0)
        self.draw()
        delay_ms = int(1000.0 / max(self.refresh_hz, 1.0))
        self.root.after(delay_ms, self._tick)

    # ------------------------------------------------------------------
    # Drawing helpers
    # ------------------------------------------------------------------
    def _txt(self, x, y, text, *, fill='#e2e8f0', font=None, anchor='w'):
        if font is None:
            font = ('Sans', 11)
        return self.canvas.create_text(x, y, text=text, fill=fill, anchor=anchor, font=font)

    def _rect(self, x0, y0, x1, y1, *, fill='#1e293b', outline='#334155', width=1):
        return self.canvas.create_rectangle(x0, y0, x1, y1, fill=fill, outline=outline, width=width)

    def _badge(self, x, y, w, h, label, active, *, active_fill='#22c55e', inactive_fill='#334155'):
        color = active_fill if active else inactive_fill
        self._rect(x, y, x + w, y + h, fill=color, outline='', width=0)
        text_color = '#ffffff' if active else '#94a3b8'
        self._txt(x + w / 2, y + h / 2, label, fill=text_color, font=('Sans', 10, 'bold'), anchor='center')

    def _fmt(self, val, fmt='{:.2f}', default='-'):
        if val is None:
            return default
        try:
            return fmt.format(float(val))
        except Exception:
            return str(val)

    # ------------------------------------------------------------------
    # Main draw
    # ------------------------------------------------------------------
    def draw(self) -> None:
        self.canvas.delete('all')
        W = max(self.canvas.winfo_width(), 1)
        H = max(self.canvas.winfo_height(), 1)

        left_n = self._float_status('left_drive_thrust_n')
        right_n = self._float_status('right_drive_thrust_n')
        left_pct = self._float_status('left_drive_percent')
        right_pct = self._float_status('right_drive_percent')
        max_thrust = max(abs(self._float_status('max_thrust_n')), 1.0)
        stale = bool(self.status.get('stale', True))

        sp = self.controller
        gd = self.guidance
        pl = self.planner
        cv = self.cmd_vel

        source = sp.get('target_source', '-') or '-'
        heading_err = sp.get('heading_error_deg')
        preturn = bool(sp.get('preturn_active', False))
        stop_turn = bool(sp.get('waypoint_stop_turn_active', False))
        straight_brake = bool(sp.get('waypoint_straight_brake_active', False))
        slowdown = bool(sp.get('waypoint_slowdown_active', False))
        turn_place = bool(sp.get('turn_in_place', False))
        geofence_out = sp.get('geofence_outside_duration_s')
        geofence_active = geofence_out is not None and float(geofence_out) > 0

        # ==================== HEADER ====================
        self._txt(24, 26, 'IDA Motor Kontrol Dashboard', fill='#f8fafc', font=('Sans', 18, 'bold'))
        state_text = 'KOMUT YOK' if stale else 'CANLI'
        state_fill = '#f59e0b' if stale else '#22c55e'
        self._txt(W - 24, 28, state_text, fill=state_fill, font=('Sans', 13, 'bold'), anchor='e')

        # Source highlight
        source_colors = {
            'guidance': '#22c55e',
            'planner': '#3b82f6',
            'geofence_return': '#ef4444',
            'waypoint_stop_turn_align': '#f59e0b',
            'waypoint_stop_turn_depart': '#f59e0b',
        }
        src_color = source_colors.get(source, '#94a3b8')
        self._txt(W / 2, 56, f'KAYNAK: {source.upper()}', fill=src_color, font=('Sans', 14, 'bold'), anchor='center')

        # ==================== MOTOR BARS ====================
        bar_y = 130
        bar_w = (W - 120) / 2.0
        self._draw_motor_bar(
            label='Sol Motor', thrust_n=left_n, percent=left_pct,
            max_thrust=max_thrust, x0=40, y=bar_y, width=bar_w,
        )
        self._draw_motor_bar(
            label='Sag Motor', thrust_n=right_n, percent=right_pct,
            max_thrust=max_thrust, x0=W / 2 + 20, y=bar_y, width=bar_w,
        )

        # ==================== CMD VEL INFO (between motors) ====================
        self._txt(W / 2, bar_y - 48,
                  f"cmd_vel  linear={self._fmt(cv.get('linear'))}  angular={self._fmt(cv.get('angular'))}",
                  fill='#cbd5e1', font=('Sans', 11), anchor='center')

        # ==================== DECISION BADGES ====================
        badge_y = 260
        badge_w = 110
        badge_h = 32
        badge_gap = 12
        badges = [
            ('P-return', preturn),
            ('Stop-Turn', stop_turn),
            ('Straight-Brake', straight_brake),
            ('Slowdown', slowdown),
            ('Turn-in-Place', turn_place),
            ('Geofence', geofence_active),
        ]
        total_bw = len(badges) * badge_w + (len(badges) - 1) * badge_gap
        bx0 = (W - total_bw) / 2
        for i, (lbl, active) in enumerate(badges):
            self._badge(bx0 + i * (badge_w + badge_gap), badge_y, badge_w, badge_h, lbl, active)

        # ==================== THREE COLUMNS ====================
        col_w = (W - 80) / 3
        col_x1 = 30
        col_x2 = col_x1 + col_w + 20
        col_x3 = col_x2 + col_w + 20
        col_y = 320
        col_h = 260

        # ---- Column 1: Controller ----
        self._draw_panel(col_x1, col_y, col_w, col_h, 'CONTROLLER')
        rows = [
            ('Heading Error', f"{self._fmt(heading_err, '{:.1f}°')}"),
            ('Planner Mode', f"{sp.get('planner_mode', '-')}"),
            ('Planner Reason', f"{sp.get('planner_reason', '-')}"),
            ('Planner Speed', f"{self._fmt(sp.get('planner_speed_limit_mps'))} m/s"),
            ('Vision Bias', f"{self._fmt(sp.get('vision_heading_bias_deg'), '{:.1f}°')}"),
            ('Active WP', f"{self.active_wp}"),
        ]
        self._draw_rows(col_x1 + 12, col_y + 28, rows, col_w - 24)

        # ---- Column 2: Guidance ----
        self._draw_panel(col_x2, col_y, col_w, col_h, 'GUIDANCE')
        rows = [
            ('Target Bearing', f"{self._fmt(gd.get('target_bearing_deg'), '{:.1f}°')}"),
            ('Waypoint Bearing', f"{self._fmt(gd.get('waypoint_bearing_deg'), '{:.1f}°')}"),
            ('Leg Bearing', f"{self._fmt(gd.get('leg_bearing_deg'), '{:.1f}°')}"),
            ('Distance', f"{self._fmt(gd.get('target_distance_m'))} m"),
            ('Upcoming Turn', f"{self._fmt(gd.get('upcoming_turn_angle_deg'), '{:.1f}°')}"),
            ('Next Bearing', f"{self._fmt(gd.get('next_bearing_deg'), '{:.1f}°')}"),
            ('Route Lookahead', f"{gd.get('route_lookahead_enabled', False)}"),
        ]
        self._draw_rows(col_x2 + 12, col_y + 28, rows, col_w - 24)

        # ---- Column 3: Planner ----
        self._draw_panel(col_x3, col_y, col_w, col_h, 'PLANNER')
        rows = [
            ('Mode', f"{pl.get('mode', '-')}"),
            ('Reason', f"{pl.get('reason', '-')}"),
            ('Lidar State', f"{pl.get('lidar_state', '-')}"),
            ('Front Clearance', f"{self._fmt(pl.get('front_clearance_m'))} m"),
            ('Corridor Conf', f"{self._fmt(pl.get('corridor_confidence'))}"),
            ('Corridor Center', f"{self._fmt(pl.get('corridor_center_left_m'))} m"),
            ('Obstacle ID', f"{pl.get('obstacle_id', '-')}"),
            ('Pass Side', f"{pl.get('pass_side', '-')}"),
        ]
        self._draw_rows(col_x3 + 12, col_y + 28, rows, col_w - 24)

        # ==================== FOOTER ====================
        self._txt(24, H - 18, f'Topic: {self.status_topic}', fill='#64748b', font=('Sans', 9))
        self._txt(W - 24, H - 18, f'Max: {max_thrust:.1f} N', fill='#64748b', font=('Sans', 9), anchor='e')

    def _draw_panel(self, x, y, w, h, title):
        self._rect(x, y, x + w, y + h, fill='#1e293b', outline='#334155', width=2)
        self._txt(x + 12, y + 16, title, fill='#94a3b8', font=('Sans', 11, 'bold'))
        self.canvas.create_line(x + 10, y + 26, x + w - 10, y + 26, fill='#334155', width=1)

    def _draw_rows(self, x, y, rows, max_w):
        line_h = 26
        for i, (label, value) in enumerate(rows):
            self._txt(x, y + i * line_h, f'{label}:', fill='#94a3b8', font=('Sans', 10))
            self._txt(x + max_w, y + i * line_h, str(value), fill='#f1f5f9', font=('Sans', 10, 'bold'), anchor='e')

    def _draw_motor_bar(self, *, label, thrust_n, percent, max_thrust, x0, y, width):
        bar_h = 36
        x1 = x0 + width
        center_x = x0 + width / 2.0
        ratio = max(-1.0, min(1.0, thrust_n / max_thrust))
        fill = '#22c55e' if ratio >= 0.0 else '#ef4444'

        self._txt(x0, y - 56, label, fill='#f8fafc', font=('Sans', 14, 'bold'))
        self._txt(x1, y - 56, f'{thrust_n:+.1f} N  ({percent:+.0f}%)', fill='#e2e8f0', font=('Sans', 12), anchor='e')

        self._rect(x0, y - bar_h / 2, x1, y + bar_h / 2, fill='#0b1220', outline='#334155', width=2)
        self.canvas.create_line(center_x, y - bar_h / 2 - 8, center_x, y + bar_h / 2 + 8, fill='#64748b', width=2)

        if ratio >= 0.0:
            fx0, fx1 = center_x, center_x + ratio * width / 2.0
        else:
            fx0, fx1 = center_x + ratio * width / 2.0, center_x
        self._rect(fx0, y - bar_h / 2, fx1, y + bar_h / 2, fill=fill, outline='', width=0)

        self._txt(x0, y + 44, 'geri', fill='#64748b', font=('Sans', 10))
        self._txt(x1, y + 44, 'ileri', fill='#64748b', font=('Sans', 10), anchor='e')

    def _float_status(self, key: str) -> float:
        try:
            return float(self.status.get(key, 0.0))
        except Exception:
            return 0.0


def main(args=None):
    rclpy.init(args=args)
    try:
        node = MotorThrustViewer()
    except TclError as exc:
        rclpy.shutdown()
        raise RuntimeError(
            'Motor thrust viewer penceresi acilamadi. DISPLAY kontrol edin '
            'veya enable_motor_viewer:=false ile kapatin.'
        ) from exc

    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
