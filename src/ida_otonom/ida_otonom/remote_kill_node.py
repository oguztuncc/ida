import json
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String

from .common import to_json

try:
    import serial
except Exception:  # pragma: no cover - depends on Jetson runtime packages
    serial = None


class RemoteKillNode(Node):
    """Bridge a LoRa/Arduino kill line into the ROS safety topic."""

    def __init__(self) -> None:
        super().__init__("remote_kill_node")

        self.declare_parameter("enabled", False)
        self.declare_parameter("port", "/dev/ttyUSB0")
        self.declare_parameter("baud_rate", 115200)
        self.declare_parameter("read_timeout_s", 0.02)
        self.declare_parameter("heartbeat_timeout_s", 1.0)
        self.declare_parameter("failsafe_on_timeout", True)
        self.declare_parameter("active_tokens", ["KILL_ACTIVE", "KILL", "ACTIVE", "1"])
        self.declare_parameter("clear_tokens", ["KILL_CLEAR", "CLEAR", "SAFE", "0"])
        self.declare_parameter("heartbeat_tokens", ["HEARTBEAT", "HB", "OK"])

        self.enabled = bool(self.get_parameter("enabled").value)
        self.port = str(self.get_parameter("port").value)
        self.baud_rate = int(self.get_parameter("baud_rate").value)
        self.read_timeout_s = float(self.get_parameter("read_timeout_s").value)
        self.heartbeat_timeout_s = float(
            self.get_parameter("heartbeat_timeout_s").value
        )
        self.failsafe_on_timeout = bool(
            self.get_parameter("failsafe_on_timeout").value
        )
        self.active_tokens = self._tokens("active_tokens")
        self.clear_tokens = self._tokens("clear_tokens")
        self.heartbeat_tokens = self._tokens("heartbeat_tokens")

        self.kill_active = False
        self.connected = False
        self.last_rx_ts = 0.0
        self.last_line = ""
        self.serial_port = self._open_serial()

        self.kill_pub = self.create_publisher(Bool, "/safety/kill", 10)
        self.physical_kill_pub = self.create_publisher(
            Bool,
            "/safety/physical_kill",
            10,
        )
        self.status_pub = self.create_publisher(
            String,
            "/safety/remote_kill_status",
            10,
        )

        self.timer = self.create_timer(0.05, self.loop)

    def _tokens(self, parameter_name: str) -> set[str]:
        value = self.get_parameter(parameter_name).value
        return {str(item).strip().upper() for item in value if str(item).strip()}

    def _open_serial(self):
        if not self.enabled:
            self.get_logger().info("Remote kill bridge disabled")
            return None
        if serial is None:
            self.get_logger().error("python3-serial is not available")
            self.kill_active = self.failsafe_on_timeout
            return None
        try:
            port = serial.Serial(
                self.port,
                self.baud_rate,
                timeout=self.read_timeout_s,
            )
            self.connected = True
            self.last_rx_ts = time.monotonic()
            self.get_logger().info(f"Remote kill serial opened: {self.port}")
            return port
        except Exception as exc:
            self.get_logger().error(f"Remote kill serial open failed: {exc}")
            self.kill_active = self.failsafe_on_timeout
            return None

    def _handle_json(self, line: str) -> bool:
        try:
            payload = json.loads(line)
        except Exception:
            return False

        if "kill_active" in payload:
            self.kill_active = bool(payload["kill_active"])
        elif "kill" in payload:
            self.kill_active = bool(payload["kill"])

        return True

    def _handle_token(self, line: str) -> None:
        token = line.strip().upper()
        if token in self.active_tokens:
            self.kill_active = True
        elif token in self.clear_tokens:
            self.kill_active = False

    def _read_lines(self) -> None:
        if self.serial_port is None:
            return

        while True:
            try:
                raw = self.serial_port.readline()
            except Exception as exc:
                self.get_logger().error(f"Remote kill serial read failed: {exc}")
                self.connected = False
                self.serial_port = None
                if self.failsafe_on_timeout:
                    self.kill_active = True
                return

            if not raw:
                return

            line = raw.decode("utf-8", errors="ignore").strip()
            if not line:
                continue
            self.last_line = line
            self.last_rx_ts = time.monotonic()
            if not self._handle_json(line):
                self._handle_token(line)

    def _apply_timeout_failsafe(self) -> bool:
        if not self.enabled or not self.failsafe_on_timeout:
            return False
        if self.last_rx_ts <= 0.0:
            return True
        return time.monotonic() - self.last_rx_ts > self.heartbeat_timeout_s

    def loop(self) -> None:
        self._read_lines()
        timeout = self._apply_timeout_failsafe()
        if timeout:
            self.kill_active = True

        msg = Bool(data=self.kill_active)
        self.kill_pub.publish(msg)
        self.physical_kill_pub.publish(msg)
        self.status_pub.publish(
            String(
                data=to_json(
                    {
                        "enabled": self.enabled,
                        "connected": self.connected,
                        "port": self.port,
                        "kill_active": self.kill_active,
                        "timeout_failsafe": timeout,
                        "last_line": self.last_line,
                    }
                )
            )
        )

    def destroy_node(self):
        try:
            if self.serial_port is not None:
                self.serial_port.close()
        finally:
            super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = RemoteKillNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
