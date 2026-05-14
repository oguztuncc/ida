import json
import math
import tkinter as tk
from tkinter import TclError

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, NavSatFix
from std_msgs.msg import Float32, Int32, String


class SimVisualizerNode(Node):
    def __init__(self) -> None:
        super().__init__("sim_visualizer_node")

        self.declare_parameter("window_width", 1000)
        self.declare_parameter("window_height", 720)
        self.declare_parameter("refresh_hz", 20.0)
        self.declare_parameter("trail_length", 2500)
        self.declare_parameter("window_title", "IDA Simulation")
        self.declare_parameter("draw_lidar_rays", True)

        self.width = int(self.get_parameter("window_width").value)
        self.height = int(self.get_parameter("window_height").value)
        self.refresh_hz = float(self.get_parameter("refresh_hz").value)
        self.trail_length = int(self.get_parameter("trail_length").value)
        self.window_title = str(self.get_parameter("window_title").value)
        self.draw_lidar_rays = bool(
            self.get_parameter("draw_lidar_rays").value
        )

        self.current_lat = None
        self.current_lon = None
        self.heading_deg = 0.0
        self.active_waypoint_index = 0
        self.waypoints = []
        self.trail = []
        self.mission_status = {}
        self.guidance_status = {}
        self.setpoints = {}
        self.sim_world = {}
        self.planner_status = {}
        self.lidar_summary = {}
        self.latest_scan = None
        self.origin_lat = None
        self.origin_lon = None
        self.closed = False

        self.create_subscription(
            NavSatFix,
            "/mavros/global_position/global",
            self.gps_cb,
            10,
        )
        self.create_subscription(
            Float32,
            "/mavros/global_position/compass_hdg",
            self.heading_cb,
            10,
        )
        self.create_subscription(
            Int32,
            "/mission/active_waypoint",
            self.active_waypoint_cb,
            10,
        )
        self.create_subscription(
            String,
            "/mission/waypoints",
            self.waypoints_cb,
            10,
        )
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
            "/control/setpoints",
            self.setpoints_cb,
            10,
        )
        self.create_subscription(
            String,
            "/sim/world",
            self.sim_world_cb,
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
            "/perception/lidar_summary",
            self.lidar_summary_cb,
            10,
        )
        self.create_subscription(LaserScan, "/scan", self.scan_cb, 10)

        self.root = tk.Tk()
        self.root.title(self.window_title)
        self.canvas = tk.Canvas(
            self.root,
            width=self.width,
            height=self.height,
            bg="#0b2a3d",
            highlightthickness=0,
        )
        self.canvas.pack(fill=tk.BOTH, expand=True)
        self.root.protocol("WM_DELETE_WINDOW", self.close)

    def gps_cb(self, msg: NavSatFix) -> None:
        self.current_lat = float(msg.latitude)
        self.current_lon = float(msg.longitude)
        if self.origin_lat is None or self.origin_lon is None:
            self.origin_lat = self.current_lat
            self.origin_lon = self.current_lon

        self.trail.append((self.current_lat, self.current_lon))
        if len(self.trail) > self.trail_length:
            self.trail = self.trail[-self.trail_length:]

    def heading_cb(self, msg: Float32) -> None:
        self.heading_deg = float(msg.data) % 360.0

    def active_waypoint_cb(self, msg: Int32) -> None:
        self.active_waypoint_index = int(msg.data)

    def waypoints_cb(self, msg: String) -> None:
        try:
            data = json.loads(msg.data)
            waypoints = data.get("waypoints", [])
            self.waypoints = [
                (float(wp["lat"]), float(wp["lon"])) for wp in waypoints
            ]
            if self.waypoints:
                self.origin_lat, self.origin_lon = self.waypoints[0]
        except Exception as exc:
            self.get_logger().warn(f"Ignoring invalid waypoint message: {exc}")

    def mission_status_cb(self, msg: String) -> None:
        self.mission_status = self._parse_json(msg.data)

    def guidance_status_cb(self, msg: String) -> None:
        self.guidance_status = self._parse_json(msg.data)

    def setpoints_cb(self, msg: String) -> None:
        self.setpoints = self._parse_json(msg.data)

    def sim_world_cb(self, msg: String) -> None:
        self.sim_world = self._parse_json(msg.data)
        origin = self.sim_world.get("origin", {})
        if origin.get("lat") is not None and origin.get("lon") is not None:
            self.origin_lat = float(origin["lat"])
            self.origin_lon = float(origin["lon"])

    def planner_status_cb(self, msg: String) -> None:
        self.planner_status = self._parse_json(msg.data)

    def lidar_summary_cb(self, msg: String) -> None:
        self.lidar_summary = self._parse_json(msg.data)

    def scan_cb(self, msg: LaserScan) -> None:
        self.latest_scan = msg

    def _parse_json(self, text: str) -> dict:
        try:
            return json.loads(text)
        except Exception:
            return {}

    def _project(self, lat: float, lon: float) -> tuple[float, float]:
        if self.origin_lat is None or self.origin_lon is None:
            return 0.0, 0.0

        lat_scale = 111_320.0
        lon_scale = lat_scale * math.cos(math.radians(self.origin_lat))
        east_m = (lon - self.origin_lon) * lon_scale
        north_m = (lat - self.origin_lat) * lat_scale
        return east_m, north_m

    def _scan_world_points(self, stride: int = 8) -> list[tuple[float, float]]:
        if (
            self.latest_scan is None
            or self.current_lat is None
            or self.current_lon is None
        ):
            return []

        boat_east, boat_north = self._project(self.current_lat, self.current_lon)
        heading = math.radians(self.heading_deg)
        points = []
        angle = self.latest_scan.angle_min
        for index, distance in enumerate(self.latest_scan.ranges):
            if index % max(stride, 1) != 0:
                angle += self.latest_scan.angle_increment
                continue
            if (
                math.isfinite(distance)
                and self.latest_scan.range_min
                <= distance
                <= self.latest_scan.range_max
            ):
                forward = distance * math.cos(angle)
                left = distance * math.sin(angle)
                east = boat_east + forward * math.sin(heading) - left * math.cos(heading)
                north = boat_north + forward * math.cos(heading) + left * math.sin(heading)
                points.append((east, north))
            angle += self.latest_scan.angle_increment
        return points

    def _screen_transform(self):
        points = []
        points.extend(self._project(lat, lon) for lat, lon in self.waypoints)
        points.extend(self._project(lat, lon) for lat, lon in self.trail)
        for obj in self.sim_world.get("objects", []):
            if obj.get("lat") is not None and obj.get("lon") is not None:
                points.append(self._project(float(obj["lat"]), float(obj["lon"])))
        points.extend(self._scan_world_points(stride=12))
        if self.current_lat is not None and self.current_lon is not None:
            points.append(self._project(self.current_lat, self.current_lon))

        canvas_w = max(self.canvas.winfo_width(), 1)
        canvas_h = max(self.canvas.winfo_height(), 1)
        status_h = 155
        map_h = max(canvas_h - status_h, 1)
        margin = 55

        if not points:
            points = [(0.0, 0.0)]

        min_x = min(p[0] for p in points)
        max_x = max(p[0] for p in points)
        min_y = min(p[1] for p in points)
        max_y = max(p[1] for p in points)

        span_x = max(max_x - min_x, 20.0)
        span_y = max(max_y - min_y, 20.0)
        mid_x = (min_x + max_x) / 2.0
        mid_y = (min_y + max_y) / 2.0
        min_x = mid_x - span_x / 2.0
        max_y = mid_y + span_y / 2.0

        scale = min(
            (canvas_w - 2 * margin) / span_x,
            (map_h - 2 * margin) / span_y,
        )

        def to_screen(east_m: float, north_m: float) -> tuple[float, float]:
            return (
                margin + (east_m - min_x) * scale,
                margin + (max_y - north_m) * scale,
            )

        return to_screen, status_h

    def draw(self) -> None:
        self.canvas.delete("all")
        canvas_w = max(self.canvas.winfo_width(), 1)
        canvas_h = max(self.canvas.winfo_height(), 1)
        to_screen, status_h = self._screen_transform()
        map_h = canvas_h - status_h

        self.canvas.create_rectangle(
            0,
            0,
            canvas_w,
            map_h,
            fill="#0b2a3d",
            outline="",
        )
        self._draw_grid(canvas_w, map_h)
        self._draw_scan(to_screen)
        self._draw_sim_world(to_screen)
        self._draw_route(to_screen)
        self._draw_trail(to_screen)
        self._draw_target_line(to_screen)
        self._draw_boat(to_screen)
        self._draw_status(canvas_w, canvas_h, status_h)

    def _draw_grid(self, canvas_w: int, map_h: int) -> None:
        for x in range(0, canvas_w, 80):
            self.canvas.create_line(x, 0, x, map_h, fill="#12364c")
        for y in range(0, map_h, 80):
            self.canvas.create_line(0, y, canvas_w, y, fill="#12364c")
        self.canvas.create_text(
            18,
            18,
            text="N",
            fill="#d7f3ff",
            anchor="w",
            font=("Sans", 18, "bold"),
        )
        self.canvas.create_line(22, 50, 22, 25, fill="#d7f3ff", width=3)
        self.canvas.create_polygon(
            22,
            17,
            15,
            30,
            29,
            30,
            fill="#d7f3ff",
            outline="",
        )

    def _draw_scan(self, to_screen) -> None:
        if self.current_lat is None or self.current_lon is None:
            return
        scan_points = self._scan_world_points(stride=4)
        if not scan_points:
            return

        boat_x, boat_y = to_screen(*self._project(self.current_lat, self.current_lon))
        for east, north in scan_points:
            x, y = to_screen(east, north)
            if self.draw_lidar_rays:
                self.canvas.create_line(boat_x, boat_y, x, y, fill="#14566b")
            self.canvas.create_oval(
                x - 2,
                y - 2,
                x + 2,
                y + 2,
                fill="#2cf0ff",
                outline="",
            )

    def _draw_sim_world(self, to_screen) -> None:
        for obj in self.sim_world.get("objects", []):
            if obj.get("lat") is None or obj.get("lon") is None:
                continue
            east, north = self._project(float(obj["lat"]), float(obj["lon"]))
            x, y = to_screen(east, north)
            radius_m = float(obj.get("radius_m", 0.4))
            rx, _ = to_screen(east + radius_m, north)
            radius_px = max(5.0, abs(rx - x))
            kind = str(obj.get("kind", ""))
            fill = str(obj.get("color") or "#d7f3ff")
            outline = "#ffffff" if kind == "obstacle" else "#7ec8ff"
            width = 3 if kind == "obstacle" else 2
            self.canvas.create_oval(
                x - radius_px,
                y - radius_px,
                x + radius_px,
                y + radius_px,
                fill=fill,
                outline=outline,
                width=width,
            )
            if kind == "obstacle":
                self.canvas.create_text(
                    x,
                    y - radius_px - 10,
                    text="OBS",
                    fill="#ffe15a",
                    anchor="s",
                    font=("Sans", 9, "bold"),
                )

    def _draw_route(self, to_screen) -> None:
        if len(self.waypoints) >= 2:
            route = []
            for lat, lon in self.waypoints:
                route.extend(to_screen(*self._project(lat, lon)))
            self.canvas.create_line(
                *route,
                fill="#7ec8ff",
                width=3,
                dash=(8, 6),
            )

        completed = bool(self.mission_status.get("mission_completed", False))
        for index, (lat, lon) in enumerate(self.waypoints):
            x, y = to_screen(*self._project(lat, lon))
            if completed or index < self.active_waypoint_index:
                fill = "#48d17a"
            elif index == self.active_waypoint_index:
                fill = "#ffcf4a"
            else:
                fill = "#d7f3ff"

            self.canvas.create_oval(
                x - 9,
                y - 9,
                x + 9,
                y + 9,
                fill=fill,
                outline="#ffffff",
                width=2,
            )
            self.canvas.create_text(
                x + 14,
                y - 14,
                text=f"WP{index}",
                fill="#ffffff",
                anchor="w",
                font=("Sans", 10, "bold"),
            )

    def _draw_trail(self, to_screen) -> None:
        if len(self.trail) < 2:
            return
        points = []
        for lat, lon in self.trail:
            points.extend(to_screen(*self._project(lat, lon)))
        self.canvas.create_line(*points, fill="#ff8b6b", width=3)

    def _draw_target_line(self, to_screen) -> None:
        if self.current_lat is None or self.current_lon is None:
            return
        if self.active_waypoint_index >= len(self.waypoints):
            return

        x0, y0 = to_screen(*self._project(self.current_lat, self.current_lon))
        lat, lon = self.waypoints[self.active_waypoint_index]
        x1, y1 = to_screen(*self._project(lat, lon))
        self.canvas.create_line(x0, y0, x1, y1, fill="#ffcf4a", width=2)

    def _draw_boat(self, to_screen) -> None:
        if self.current_lat is None or self.current_lon is None:
            self.canvas.create_text(
                self.width / 2,
                self.height / 2,
                text="Waiting for simulated GPS...",
                fill="#ffffff",
                font=("Sans", 18, "bold"),
            )
            return

        x, y = to_screen(*self._project(self.current_lat, self.current_lon))
        heading = math.radians(self.heading_deg)
        forward_x = math.sin(heading)
        forward_y = -math.cos(heading)
        right_x = math.cos(heading)
        right_y = math.sin(heading)
        size = 18

        nose = (x + forward_x * size, y + forward_y * size)
        left = (
            x - forward_x * size * 0.75 - right_x * size * 0.55,
            y - forward_y * size * 0.75 - right_y * size * 0.55,
        )
        right = (
            x - forward_x * size * 0.75 + right_x * size * 0.55,
            y - forward_y * size * 0.75 + right_y * size * 0.55,
        )

        self.canvas.create_polygon(
            nose,
            right,
            left,
            fill="#ffffff",
            outline="#ff8b6b",
            width=3,
        )
        self.canvas.create_oval(
            x - 4,
            y - 4,
            x + 4,
            y + 4,
            fill="#ff8b6b",
            outline="",
        )

    def _draw_status(self, canvas_w: int, canvas_h: int, status_h: int) -> None:
        top = canvas_h - status_h
        self.canvas.create_rectangle(
            0,
            top,
            canvas_w,
            canvas_h,
            fill="#071b28",
            outline="#19445c",
        )

        mission_started = self.mission_status.get("mission_started", False)
        mission_completed = self.mission_status.get("mission_completed", False)
        target_distance = self.guidance_status.get("target_distance_m")
        target_bearing = self.guidance_status.get("target_bearing_deg")
        speed = self.setpoints.get("speed_setpoint", 0.0)
        yaw_rate = self.setpoints.get("yaw_rate_setpoint", 0.0)
        stop_reason = self.setpoints.get("stop_reason")
        planner_mode = self.planner_status.get("mode", "--")
        planner_reason = self.planner_status.get("reason", "--")
        lidar_state = self.planner_status.get("lidar_state", "--")
        front_clearance = self.planner_status.get(
            "front_clearance_m",
            self.lidar_summary.get("front_clearance_m"),
        )
        sim_boat = self.sim_world.get("boat", {})
        closest_clearance = sim_boat.get("closest_clearance_m")
        collision = bool(sim_boat.get("collision", False))

        left_lines = [
            f"Mission: started={mission_started} completed={mission_completed}",
            f"Active waypoint: {self.active_waypoint_index}/{max(len(self.waypoints) - 1, 0)}",
            self._format_gps_line(),
            f"LiDAR: {lidar_state} front={self._fmt(front_clearance)} m",
            f"Closest object: {self._fmt(closest_clearance)} m collision={collision}",
        ]
        right_lines = [
            f"Heading: {self.heading_deg:.1f} deg",
            f"Target: {self._fmt(target_distance)} m @ {self._fmt(target_bearing)} deg",
            f"Command: speed={speed:.2f} m/s yaw={yaw_rate:.2f} rad/s",
            f"Planner: {planner_mode} ({planner_reason})",
        ]
        if stop_reason:
            right_lines.append(f"Stop reason: {stop_reason}")

        for row, text in enumerate(left_lines):
            self.canvas.create_text(
                18,
                top + 24 + row * 28,
                text=text,
                fill="#d7f3ff",
                anchor="w",
                font=("Sans", 12),
            )
        for row, text in enumerate(right_lines):
            self.canvas.create_text(
                canvas_w / 2,
                top + 24 + row * 28,
                text=text,
                fill="#d7f3ff",
                anchor="w",
                font=("Sans", 12),
            )

        self.canvas.create_text(
            canvas_w - 18,
            canvas_h - 18,
            text="Autonomous view - close window to stop visualizer",
            fill="#7ec8ff",
            anchor="e",
            font=("Sans", 10),
        )

    def _format_gps_line(self) -> str:
        if self.current_lat is None or self.current_lon is None:
            return "GPS: waiting"
        return f"GPS: lat={self.current_lat:.7f} lon={self.current_lon:.7f}"

    def _fmt(self, value) -> str:
        if value is None:
            return "--"
        try:
            return f"{float(value):.1f}"
        except Exception:
            return "--"

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


def main(args=None) -> None:
    rclpy.init(args=args)
    try:
        node = SimVisualizerNode()
    except TclError as exc:
        rclpy.shutdown()
        raise RuntimeError(
            "Unable to open Tk visualizer window. Check DISPLAY or run with "
            "enable_visualizer:=false."
        ) from exc

    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
