import time
from typing import Any, Dict, Tuple

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Bool, Int32, String

from .common import from_json, to_json
from .schemas import GuidanceStatus, MissionStatus, SafetyStatus

try:
    from pymavlink import mavutil
except Exception:
    mavutil = None


class YkiBridgeNode(Node):
    def __init__(self) -> None:
        super().__init__("yki_bridge_node")

        self.declare_parameter("enable_command_rx", True)
        self.declare_parameter("mavlink_connection_url", "udpout:127.0.0.1:14550")
        self.declare_parameter("mavlink_baud_rate", 57600)
        self.declare_parameter("mavlink_source_system", 42)
        self.declare_parameter("mavlink_source_component", 191)
        self.declare_parameter("mavlink_send_statustext", True)
        self.declare_parameter("enable_mission_rx", True)
        self.declare_parameter("max_mavlink_mission_items", 100)

        self.enable_command_rx = bool(
            self.get_parameter("enable_command_rx").value
        )
        self.mavlink_connection_url = str(
            self.get_parameter("mavlink_connection_url").value
        )
        self.mavlink_baud_rate = int(
            self.get_parameter("mavlink_baud_rate").value
        )
        self.mavlink_source_system = int(
            self.get_parameter("mavlink_source_system").value
        )
        self.mavlink_source_component = int(
            self.get_parameter("mavlink_source_component").value
        )
        self.mavlink_send_statustext = bool(
            self.get_parameter("mavlink_send_statustext").value
        )
        self.enable_mission_rx = bool(
            self.get_parameter("enable_mission_rx").value
        )
        self.max_mavlink_mission_items = int(
            self.get_parameter("max_mavlink_mission_items").value
        )

        self.boot_time = time.monotonic()
        self.mavlink = None
        self.last_statustext = ""
        self.mission_upload = None

        self._setup_mavlink_transport()

        self.lat = None
        self.lon = None
        self.active_waypoint = 0
        self.mission_started = False
        self.mission_completed = False
        self.mission_status = {}
        self.guidance_status = {}
        self.geofence_status = {}
        self.safety_status = {}
        self.remote_kill_status = {}
        self.power_status = {}
        self.waypoints = []
        self.target_color = None

        self.create_subscription(
            NavSatFix,
            "/mavros/global_position/global",
            self.gps_cb,
            10,
        )
        self.create_subscription(
            Int32,
            "/mission/active_waypoint",
            self.active_wp_cb,
            10,
        )
        self.create_subscription(Bool, "/mission/completed", self.done_cb, 10)
        self.create_subscription(Bool, "/mission/started", self.started_cb, 10)
        self.create_subscription(
            String,
            "/mission/status",
            self.mission_status_cb,
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
            String,
            "/safety/status",
            self.safety_status_cb,
            10,
        )
        self.create_subscription(
            String,
            "/safety/remote_kill_status",
            self.remote_kill_status_cb,
            10,
        )
        self.create_subscription(
            String,
            "/power/status",
            self.power_status_cb,
            10,
        )
        self.create_subscription(
            String,
            "/mission/waypoints",
            self.waypoints_cb,
            10,
        )

        self.kill_pub = self.create_publisher(Bool, "/safety/kill", 10)
        self.mission_load_pub = self.create_publisher(
            String,
            "/mission/load",
            10,
        )

        self.timer = self.create_timer(0.5, self.loop)
        if self.enable_command_rx:
            self.command_timer = self.create_timer(0.1, self.command_loop)

    def _setup_mavlink_transport(self) -> None:
        if mavutil is None:
            self.get_logger().error(
                "pymavlink is unavailable; YKI MAVLink bridge cannot start"
            )
            return

        try:
            self.mavlink = mavutil.mavlink_connection(
                self.mavlink_connection_url,
                baud=self.mavlink_baud_rate,
                source_system=self.mavlink_source_system,
                source_component=self.mavlink_source_component,
                autoreconnect=True,
            )
        except TypeError:
            self.mavlink = mavutil.mavlink_connection(
                self.mavlink_connection_url,
                baud=self.mavlink_baud_rate,
                source_system=self.mavlink_source_system,
                source_component=self.mavlink_source_component,
            )
        self.get_logger().info(
            f"YKI bridge using MAVLink transport: {self.mavlink_connection_url}"
        )

    def gps_cb(self, msg: NavSatFix) -> None:
        self.lat = msg.latitude
        self.lon = msg.longitude

    def active_wp_cb(self, msg: Int32) -> None:
        self.active_waypoint = int(msg.data)

    def done_cb(self, msg: Bool) -> None:
        self.mission_completed = bool(msg.data)

    def started_cb(self, msg: Bool) -> None:
        self.mission_started = bool(msg.data)

    def mission_status_cb(self, msg: String) -> None:
        parsed = MissionStatus.parse(msg)
        self.mission_status = parsed.__dict__ if parsed is not None else {}

    def guidance_status_cb(self, msg: String) -> None:
        parsed = GuidanceStatus.parse(msg)
        self.guidance_status = parsed.__dict__ if parsed is not None else {}

    def geofence_status_cb(self, msg: String) -> None:
        self.geofence_status = from_json(msg.data)

    def safety_status_cb(self, msg: String) -> None:
        parsed = SafetyStatus.parse(msg)
        self.safety_status = parsed.__dict__ if parsed is not None else {}

    def remote_kill_status_cb(self, msg: String) -> None:
        self.remote_kill_status = from_json(msg.data)

    def power_status_cb(self, msg: String) -> None:
        self.power_status = from_json(msg.data)

    def waypoints_cb(self, msg: String) -> None:
        self.waypoints = from_json(msg.data).get("waypoints", [])

    def command_loop(self) -> None:
        self._mavlink_command_loop()

    def _mavlink_command_loop(self) -> None:
        if self.mavlink is None:
            return

        while True:
            msg = self.mavlink.recv_match(blocking=False)
            if msg is None:
                return

            msg_type = msg.get_type()
            if msg_type == "BAD_DATA":
                continue
            if msg_type == "COMMAND_LONG":
                self._handle_mavlink_command_long(msg)
            elif msg_type == "STATUSTEXT":
                self._handle_mavlink_text_command(msg)
            elif self.enable_mission_rx and msg_type == "MISSION_COUNT":
                self._handle_mavlink_mission_count(msg)
            elif self.enable_mission_rx and msg_type in (
                "MISSION_ITEM_INT",
                "MISSION_ITEM",
            ):
                self._handle_mavlink_mission_item(msg)
            elif msg_type == "MISSION_REQUEST_LIST":
                self._handle_mavlink_mission_request_list(msg)
            elif msg_type in ("MISSION_REQUEST_INT", "MISSION_REQUEST"):
                self._handle_mavlink_mission_request(msg)

    def _handle_mavlink_mission_count(self, msg) -> None:
        mav = mavutil.mavlink
        target_system, target_component = self._message_source(msg)
        mission_type = self._message_mission_type(msg)
        mission_type_mission = getattr(mav, "MAV_MISSION_TYPE_MISSION", 0)

        if mission_type != mission_type_mission:
            self._send_mission_ack(
                target_system,
                target_component,
                getattr(mav, "MAV_MISSION_UNSUPPORTED", 3),
                mission_type,
            )
            return

        if self.mission_started:
            self._send_mission_ack(
                target_system,
                target_component,
                getattr(mav, "MAV_MISSION_DENIED", 14),
                mission_type,
            )
            self.get_logger().warning(
                "MAVLink mission upload rejected after mission start"
            )
            return

        count = int(msg.count)
        if count <= 0 or count > self.max_mavlink_mission_items:
            self._send_mission_ack(
                target_system,
                target_component,
                getattr(mav, "MAV_MISSION_INVALID_SEQUENCE", 13),
                mission_type,
            )
            self.get_logger().warning(
                f"MAVLink mission upload rejected: invalid count {count}"
            )
            return

        self.mission_upload = {
            "target_system": target_system,
            "target_component": target_component,
            "mission_type": mission_type,
            "count": count,
            "items": [None] * count,
            "next_seq": 0,
            "started_at": time.monotonic(),
        }
        self._send_mission_request(target_system, target_component, 0, mission_type)
        self.get_logger().info(
            f"MAVLink mission upload started: {count} item(s)"
        )

    def _handle_mavlink_mission_item(self, msg) -> None:
        mav = mavutil.mavlink
        if self.mission_upload is None:
            target_system, target_component = self._message_source(msg)
            self._send_mission_ack(
                target_system,
                target_component,
                getattr(mav, "MAV_MISSION_ERROR", 1),
                self._message_mission_type(msg),
            )
            return

        seq = int(msg.seq)
        count = int(self.mission_upload["count"])
        expected_seq = int(self.mission_upload["next_seq"])
        target_system = int(self.mission_upload["target_system"])
        target_component = int(self.mission_upload["target_component"])
        mission_type = int(self.mission_upload["mission_type"])

        if seq != expected_seq or seq < 0 or seq >= count:
            self._send_mission_ack(
                target_system,
                target_component,
                getattr(mav, "MAV_MISSION_INVALID_SEQUENCE", 13),
                mission_type,
            )
            self.mission_upload = None
            self.get_logger().warning(
                f"MAVLink mission upload rejected: expected seq "
                f"{expected_seq}, got {seq}"
            )
            return

        try:
            waypoint = self._waypoint_from_mavlink_mission_item(msg)
        except ValueError as exc:
            self._send_mission_ack(
                target_system,
                target_component,
                getattr(mav, "MAV_MISSION_INVALID", 5),
                mission_type,
            )
            self.mission_upload = None
            self.get_logger().warning(f"MAVLink mission item rejected: {exc}")
            return

        self.mission_upload["items"][seq] = waypoint
        next_seq = seq + 1
        self.mission_upload["next_seq"] = next_seq

        if next_seq < count:
            self._send_mission_request(
                target_system,
                target_component,
                next_seq,
                mission_type,
            )
            return

        waypoints = [
            item for item in self.mission_upload["items"] if item is not None
        ]
        self.mission_upload = None
        payload = {
            "mission_name": "mavlink_yki_mission",
            "source": "mavlink_yki",
            "waypoints": waypoints,
        }
        self.mission_load_pub.publish(String(data=to_json(payload)))
        self._send_mission_ack(
            target_system,
            target_component,
            getattr(mav, "MAV_MISSION_ACCEPTED", 0),
            mission_type,
        )
        self.get_logger().info(
            f"MAVLink mission upload accepted: {len(waypoints)} waypoint(s)"
        )

    def _handle_mavlink_mission_request_list(self, msg) -> None:
        target_system, target_component = self._message_source(msg)
        mission_type = self._message_mission_type(msg)
        self._send_mission_count(
            target_system,
            target_component,
            len(self.waypoints),
            mission_type,
        )

    def _handle_mavlink_mission_request(self, msg) -> None:
        target_system, target_component = self._message_source(msg)
        mission_type = self._message_mission_type(msg)
        seq = int(msg.seq)
        if seq < 0 or seq >= len(self.waypoints):
            self._send_mission_ack(
                target_system,
                target_component,
                getattr(mavutil.mavlink, "MAV_MISSION_INVALID_SEQUENCE", 13),
                mission_type,
            )
            return
        self._send_mission_item(target_system, target_component, seq, mission_type)

    def _waypoint_from_mavlink_mission_item(self, msg) -> Dict[str, Any]:
        mav = mavutil.mavlink
        nav_waypoint = getattr(mav, "MAV_CMD_NAV_WAYPOINT", 16)
        if int(msg.command) != nav_waypoint:
            raise ValueError(f"unsupported command {int(msg.command)}")

        frame = int(msg.frame)
        supported_frames = {
            getattr(mav, "MAV_FRAME_GLOBAL", 0),
            getattr(mav, "MAV_FRAME_GLOBAL_RELATIVE_ALT", 3),
            getattr(mav, "MAV_FRAME_GLOBAL_INT", 5),
            getattr(mav, "MAV_FRAME_GLOBAL_RELATIVE_ALT_INT", 6),
        }
        if frame not in supported_frames:
            raise ValueError(f"unsupported frame {frame}")

        msg_type = msg.get_type()
        if msg_type == "MISSION_ITEM_INT":
            lat = float(msg.x) / 1e7
            lon = float(msg.y) / 1e7
        else:
            lat = float(msg.x)
            lon = float(msg.y)

        if not -90.0 <= lat <= 90.0 or not -180.0 <= lon <= 180.0:
            raise ValueError("lat/lon out of range")

        return {
            "lat": lat,
            "lon": lon,
            "seq": int(msg.seq),
            "source": "mavlink",
        }

    def _message_source(self, msg) -> Tuple[int, int]:
        target_system = int(msg.get_srcSystem() or 255)
        target_component = int(msg.get_srcComponent() or 0)
        return target_system, target_component

    def _message_mission_type(self, msg) -> int:
        return int(
            getattr(
                msg,
                "mission_type",
                getattr(mavutil.mavlink, "MAV_MISSION_TYPE_MISSION", 0),
            )
        )

    def _send_mission_request(
        self,
        target_system: int,
        target_component: int,
        seq: int,
        mission_type: int,
    ) -> None:
        try:
            self.mavlink.mav.mission_request_int_send(
                target_system,
                target_component,
                seq,
                mission_type,
            )
        except TypeError:
            self.mavlink.mav.mission_request_int_send(
                target_system,
                target_component,
                seq,
            )

    def _send_mission_ack(
        self,
        target_system: int,
        target_component: int,
        result: int,
        mission_type: int,
    ) -> None:
        try:
            self.mavlink.mav.mission_ack_send(
                target_system,
                target_component,
                result,
                mission_type,
            )
        except TypeError:
            self.mavlink.mav.mission_ack_send(
                target_system,
                target_component,
                result,
            )

    def _send_mission_count(
        self,
        target_system: int,
        target_component: int,
        count: int,
        mission_type: int,
    ) -> None:
        try:
            self.mavlink.mav.mission_count_send(
                target_system,
                target_component,
                count,
                mission_type,
            )
        except TypeError:
            self.mavlink.mav.mission_count_send(
                target_system,
                target_component,
                count,
            )

    def _send_mission_item(
        self,
        target_system: int,
        target_component: int,
        seq: int,
        mission_type: int,
    ) -> None:
        mav = mavutil.mavlink
        waypoint = self.waypoints[seq]
        lat = int(float(waypoint["lat"]) * 1e7)
        lon = int(float(waypoint["lon"]) * 1e7)
        try:
            self.mavlink.mav.mission_item_int_send(
                target_system,
                target_component,
                seq,
                getattr(mav, "MAV_FRAME_GLOBAL_RELATIVE_ALT_INT", 6),
                getattr(mav, "MAV_CMD_NAV_WAYPOINT", 16),
                int(seq == self.active_waypoint),
                1,
                0.0,
                0.0,
                0.0,
                0.0,
                lat,
                lon,
                0.0,
                mission_type,
            )
        except TypeError:
            self.mavlink.mav.mission_item_int_send(
                target_system,
                target_component,
                seq,
                getattr(mav, "MAV_FRAME_GLOBAL_RELATIVE_ALT_INT", 6),
                getattr(mav, "MAV_CMD_NAV_WAYPOINT", 16),
                int(seq == self.active_waypoint),
                1,
                0.0,
                0.0,
                0.0,
                0.0,
                lat,
                lon,
                0.0,
            )

    def _handle_mavlink_command_long(self, msg) -> None:
        command = int(msg.command)
        accepted = True
        mav = mavutil.mavlink

        flight_termination_cmd = getattr(
            mav,
            "MAV_CMD_DO_FLIGHTTERMINATION",
            185,
        )
        denied_result = getattr(
            mav,
            "MAV_RESULT_DENIED",
            mav.MAV_RESULT_UNSUPPORTED,
        )

        if command == flight_termination_cmd:
            active = float(msg.param1) >= 0.5
            if active:
                self._publish_yki_kill()
            else:
                accepted = False
        else:
            accepted = False

        result = mav.MAV_RESULT_ACCEPTED if accepted else denied_result
        try:
            self.mavlink.mav.command_ack_send(command, result)
        except Exception as exc:
            self.get_logger().warning(f"MAVLink command ack failed: {exc}")

    def _handle_mavlink_text_command(self, msg) -> None:
        text = msg.text
        if isinstance(text, bytes):
            text = text.decode("utf-8", errors="ignore")
        text = str(text).strip().strip("\x00")
        if not text:
            return

        if text.startswith("{"):
            try:
                self.handle_command(from_json(text))
                return
            except Exception:
                pass

        upper = text.upper()
        if not upper.startswith("IDA "):
            return

        parts = text.split()
        command = parts[1].lower() if len(parts) > 1 else ""
        if command == "kill":
            active = len(parts) < 3 or parts[2].lower() in (
                "1",
                "true",
                "active",
                "on",
            )
            self.handle_command({"command": "kill", "active": active})

    def handle_command(self, command: Dict[str, Any]) -> None:
        command_name = str(command.get("command", "")).lower()

        if command_name == "kill":
            active = self._is_active_kill_value(command.get("active", True))
            if active:
                self._publish_yki_kill()
            else:
                self.get_logger().warning(
                    "YKI kill clear ignored; YKI may only activate kill"
                )
            return

        self.get_logger().warning(
            f"YKI command ignored; only kill activation is allowed: {command_name}"
        )

    def _publish_yki_kill(self) -> None:
        self.kill_pub.publish(Bool(data=True))
        self.get_logger().warning("YKI kill command accepted")

    def _is_active_kill_value(self, value: Any) -> bool:
        if isinstance(value, str):
            return value.strip().lower() not in ("0", "false", "clear", "off")
        return bool(value)

    def _time_boot_ms(self) -> int:
        return int((time.monotonic() - self.boot_time) * 1000.0) & 0xFFFFFFFF

    def _send_named_int(self, name: str, value: int) -> None:
        self.mavlink.mav.named_value_int_send(
            self._time_boot_ms(),
            name.encode("ascii", errors="ignore")[:10],
            int(value),
        )

    def _send_named_float(self, name: str, value: float) -> None:
        self.mavlink.mav.named_value_float_send(
            self._time_boot_ms(),
            name.encode("ascii", errors="ignore")[:10],
            float(value),
        )

    def _send_statustext(self, text: str) -> None:
        if not self.mavlink_send_statustext or text == self.last_statustext:
            return
        self.last_statustext = text
        self.mavlink.mav.statustext_send(
            mavutil.mavlink.MAV_SEVERITY_INFO,
            text.encode("utf-8", errors="ignore")[:50],
        )

    def _send_mavlink_payload(self) -> None:
        if self.mavlink is None:
            return

        mav = mavutil.mavlink
        state = mav.MAV_STATE_ACTIVE if self.mission_started else mav.MAV_STATE_STANDBY
        self.mavlink.mav.heartbeat_send(
            mav.MAV_TYPE_SURFACE_BOAT,
            mav.MAV_AUTOPILOT_INVALID,
            0,
            0,
            state,
        )

        if self.lat is not None and self.lon is not None:
            self.mavlink.mav.global_position_int_send(
                self._time_boot_ms(),
                int(float(self.lat) * 1e7),
                int(float(self.lon) * 1e7),
                0,
                0,
                0,
                0,
                0,
                65535,
            )

        self.mavlink.mav.mission_current_send(int(self.active_waypoint))
        self._send_named_int("IDA_START", int(self.mission_started))
        self._send_named_int("IDA_DONE", int(self.mission_completed))
        self._send_named_int(
            "IDA_KILL",
            int(bool(self.safety_status.get("kill_active", False))),
        )
        self._send_named_int(
            "IDA_OUT",
            int(bool(self.geofence_status.get("outside", False))),
        )
        self._send_named_int(
            "IDA_EXIT",
            int(self.geofence_status.get("penalty_equivalent_exit_count", 0)),
        )
        self._send_named_float(
            "IDA_OUT_S",
            float(self.geofence_status.get("outside_duration_s", 0.0)),
        )

        text = (
            f"IDA wp={self.active_waypoint} "
            f"start={int(self.mission_started)} "
            f"kill={int(bool(self.safety_status.get('kill_active', False)))} "
            f"out={int(bool(self.geofence_status.get('outside', False)))}"
        )
        self._send_statustext(text)

    def loop(self) -> None:
        self._send_mavlink_payload()

    def destroy_node(self):
        try:
            if self.mavlink is not None:
                self.mavlink.close()
        finally:
            super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = YkiBridgeNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
