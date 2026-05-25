#!/usr/bin/env python3
"""Gazebo motor itki komutlarını küçük bir Tk penceresinde gösterir."""

import json
import tkinter as tk
from tkinter import TclError

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class MotorThrustViewer(Node):
    def __init__(self):
        super().__init__('motor_thrust_viewer')

        self.declare_parameter('status_topic', '/sim/motor_thrust')
        self.declare_parameter('window_title', 'IDA Motor Thrust')
        self.declare_parameter('refresh_hz', 20.0)

        self.status_topic = self.get_parameter('status_topic').value
        self.window_title = self.get_parameter('window_title').value
        self.refresh_hz = float(self.get_parameter('refresh_hz').value)
        self.status = {
            'left_thrust_n': 0.0,
            'right_thrust_n': 0.0,
            'left_percent': 0.0,
            'right_percent': 0.0,
            'max_thrust_n': 1.0,
            'stale': True,
        }
        self.closed = False

        self.create_subscription(String, self.status_topic, self.status_cb, 10)

        self.root = tk.Tk()
        self.root.title(self.window_title)
        self.root.geometry('520x260')
        self.root.configure(bg='#111827')
        self.root.protocol('WM_DELETE_WINDOW', self.close)

        self.canvas = tk.Canvas(
            self.root,
            width=520,
            height=260,
            bg='#111827',
            highlightthickness=0,
        )
        self.canvas.pack(fill=tk.BOTH, expand=True)

    def status_cb(self, msg: String) -> None:
        try:
            self.status = json.loads(msg.data)
        except Exception:
            return

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

    def draw(self) -> None:
        self.canvas.delete('all')
        width = max(self.canvas.winfo_width(), 1)
        height = max(self.canvas.winfo_height(), 1)

        left_n = self._float_status('left_thrust_n')
        right_n = self._float_status('right_thrust_n')
        left_pct = self._float_status('left_percent')
        right_pct = self._float_status('right_percent')
        max_thrust = max(abs(self._float_status('max_thrust_n')), 1.0)
        stale = bool(self.status.get('stale', True))

        self.canvas.create_text(
            22,
            24,
            text='Motor Itki Komutlari',
            fill='#f9fafb',
            anchor='w',
            font=('Sans', 18, 'bold'),
        )
        state_text = 'KOMUT YOK' if stale else 'CANLI'
        state_fill = '#f59e0b' if stale else '#22c55e'
        self.canvas.create_text(
            width - 24,
            26,
            text=state_text,
            fill=state_fill,
            anchor='e',
            font=('Sans', 13, 'bold'),
        )

        center_y = 136
        self._draw_motor_bar(
            label='Sol',
            thrust_n=left_n,
            percent=left_pct,
            max_thrust=max_thrust,
            x0=34,
            y=center_y,
            width=(width - 92) / 2.0,
        )
        self._draw_motor_bar(
            label='Sag',
            thrust_n=right_n,
            percent=right_pct,
            max_thrust=max_thrust,
            x0=width / 2.0 + 12,
            y=center_y,
            width=(width - 92) / 2.0,
        )

        self.canvas.create_text(
            22,
            height - 22,
            text=f'Topic: {self.status_topic}',
            fill='#93c5fd',
            anchor='w',
            font=('Sans', 10),
        )
        self.canvas.create_text(
            width - 22,
            height - 22,
            text=f'Max: {max_thrust:.1f} N',
            fill='#93c5fd',
            anchor='e',
            font=('Sans', 10),
        )

    def _draw_motor_bar(
        self,
        *,
        label: str,
        thrust_n: float,
        percent: float,
        max_thrust: float,
        x0: float,
        y: float,
        width: float,
    ) -> None:
        bar_h = 34
        x1 = x0 + width
        center_x = x0 + width / 2.0
        ratio = max(-1.0, min(1.0, thrust_n / max_thrust))
        fill = '#22c55e' if ratio >= 0.0 else '#ef4444'

        self.canvas.create_text(
            x0,
            y - 58,
            text=label,
            fill='#f9fafb',
            anchor='w',
            font=('Sans', 15, 'bold'),
        )
        self.canvas.create_text(
            x1,
            y - 58,
            text=f'{thrust_n:+.1f} N  ({percent:+.0f}%)',
            fill='#e5e7eb',
            anchor='e',
            font=('Sans', 12),
        )
        self.canvas.create_rectangle(
            x0,
            y - bar_h / 2,
            x1,
            y + bar_h / 2,
            fill='#1f2937',
            outline='#374151',
            width=2,
        )
        self.canvas.create_line(
            center_x,
            y - bar_h / 2 - 8,
            center_x,
            y + bar_h / 2 + 8,
            fill='#9ca3af',
            width=2,
        )
        if ratio >= 0.0:
            fill_x0 = center_x
            fill_x1 = center_x + ratio * width / 2.0
        else:
            fill_x0 = center_x + ratio * width / 2.0
            fill_x1 = center_x
        self.canvas.create_rectangle(
            fill_x0,
            y - bar_h / 2,
            fill_x1,
            y + bar_h / 2,
            fill=fill,
            outline='',
        )
        self.canvas.create_text(
            x0,
            y + 48,
            text='geri',
            fill='#9ca3af',
            anchor='w',
            font=('Sans', 10),
        )
        self.canvas.create_text(
            x1,
            y + 48,
            text='ileri',
            fill='#9ca3af',
            anchor='e',
            font=('Sans', 10),
        )

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
