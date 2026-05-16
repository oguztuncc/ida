import json
import time
from typing import Any

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from .common import to_json

try:
    import serial
except Exception:  # pragma: no cover - depends on Jetson runtime packages
    serial = None


class PowerMonitorNode(Node):
    """Publish battery/contactor status reported by the power Arduino."""

    def __init__(self) -> None:
        super().__init__("power_monitor_node")

        self.declare_parameter("enabled", False)
        self.declare_parameter("port", "/dev/ttyUSB1")
        self.declare_parameter("baud_rate", 115200)
        self.declare_parameter("read_timeout_s", 0.02)
        self.declare_parameter("stale_timeout_s", 2.0)

        self.enabled = bool(self.get_parameter("enabled").value)
        self.port = str(self.get_parameter("port").value)
        self.baud_rate = int(self.get_parameter("baud_rate").value)
        self.read_timeout_s = float(self.get_parameter("read_timeout_s").value)
        self.stale_timeout_s = float(self.get_parameter("stale_timeout_s").value)

        self.connected = False
        self.last_rx_ts = 0.0
        self.last_line = ""
        self.values: dict[str, Any] = {}
        self.serial_port = self._open_serial()

        self.status_pub = self.create_publisher(String, "/power/status", 10)
        self.timer = self.create_timer(0.1, self.loop)

    def _open_serial(self):
        if not self.enabled:
            self.get_logger().info("Power monitor disabled")
            return None
        if serial is None:
            self.get_logger().error("python3-serial is not available")
            return None
        try:
            port = serial.Serial(
                self.port,
                self.baud_rate,
                timeout=self.read_timeout_s,
            )
            self.connected = True
            self.get_logger().info(f"Power monitor serial opened: {self.port}")
            return port
        except Exception as exc:
            self.get_logger().error(f"Power monitor serial open failed: {exc}")
            return None

    def _parse_line(self, line: str) -> dict[str, Any]:
        try:
            payload = json.loads(line)
            if isinstance(payload, dict):
                return payload
        except Exception:
            pass

        values: dict[str, Any] = {}
        normalized = line.replace(",", " ")
        for part in normalized.split():
            if "=" not in part:
                continue
            key, raw_value = part.split("=", 1)
            key = key.strip().lower()
            raw_value = raw_value.strip()
            if raw_value.lower() in {"true", "false"}:
                values[key] = raw_value.lower() == "true"
                continue
            try:
                values[key] = float(raw_value)
            except ValueError:
                values[key] = raw_value
        return values

    def _read_lines(self) -> None:
        if self.serial_port is None:
            return

        while True:
            try:
                raw = self.serial_port.readline()
            except Exception as exc:
                self.get_logger().error(f"Power monitor serial read failed: {exc}")
                self.connected = False
                self.serial_port = None
                return

            if not raw:
                return

            line = raw.decode("utf-8", errors="ignore").strip()
            if not line:
                continue
            self.last_line = line
            self.last_rx_ts = time.monotonic()
            parsed = self._parse_line(line)
            if parsed:
                self.values.update(parsed)

    def loop(self) -> None:
        self._read_lines()
        stale = (
            self.last_rx_ts <= 0.0
            or time.monotonic() - self.last_rx_ts > self.stale_timeout_s
        )
        payload = {
            "enabled": self.enabled,
            "connected": self.connected,
            "stale": stale,
            "port": self.port,
            "last_line": self.last_line,
            **self.values,
        }
        self.status_pub.publish(String(data=to_json(payload)))

    def destroy_node(self):
        try:
            if self.serial_port is not None:
                self.serial_port.close()
        finally:
            super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PowerMonitorNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
