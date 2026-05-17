import json
import math
import os
import tkinter as tk
from tkinter import filedialog, messagebox, ttk

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from .common import package_share_path


class ParkurEditorNode(Node):
    def __init__(self) -> None:
        super().__init__("parkur_editor_node")

        self.declare_parameter("window_width", 1200)
        self.declare_parameter("window_height", 800)
        self.declare_parameter("initial_zoom", 8.0)
        self.declare_parameter("grid_spacing_m", 5.0)
        self.declare_parameter("default_export_dir", "")

        self.width = int(self.get_parameter("window_width").value)
        self.height = int(self.get_parameter("window_height").value)
        self.zoom = float(self.get_parameter("initial_zoom").value)
        self.grid_spacing_m = float(self.get_parameter("grid_spacing_m").value)
        self.default_export_dir = str(
            self.get_parameter("default_export_dir").value
        )

        self.origin_x = 80
        self.origin_y = self.height - 80
        self.pan_east = 0.0
        self.pan_north = 0.0
        self.dragging = False
        self.drag_item = None
        self.drag_start = (0, 0)

        self.mode = "SELECT"
        self.route_points: list[tuple[float, float]] = []
        self.waypoints: list[tuple[float, float]] = []
        self.boundaries: list[dict] = []
        self.obstacles: list[dict] = []
        self.selected_item = None

        self.world_pub = self.create_publisher(String, "/sim/editor_world", 10)

        self.root = tk.Tk()
        self.root.title("IDA Parkur Editor")
        self._build_ui()
        self._bind_events()
        self._draw()

    def _build_ui(self) -> None:
        self.toolbar = tk.Frame(self.root, bg="#1a2f3d", height=44)
        self.toolbar.pack(fill=tk.X, side=tk.TOP)

        btn_cfg = {
            "bg": "#2a4f6d",
            "fg": "#ffffff",
            "activebackground": "#3a6f9d",
            "activeforeground": "#ffffff",
            "font": ("Sans", 10, "bold"),
            "relief": tk.FLAT,
            "padx": 10,
            "pady": 4,
        }

        tk.Button(
            self.toolbar, text="Select", command=lambda: self._set_mode("SELECT"), **btn_cfg
        ).pack(side=tk.LEFT, padx=4, pady=4)
        tk.Button(
            self.toolbar, text="Route Pt", command=lambda: self._set_mode("ROUTE"), **btn_cfg
        ).pack(side=tk.LEFT, padx=4, pady=4)
        tk.Button(
            self.toolbar, text="Waypoint", command=lambda: self._set_mode("WAYPOINT"), **btn_cfg
        ).pack(side=tk.LEFT, padx=4, pady=4)
        tk.Button(
            self.toolbar, text="Boundary", command=lambda: self._set_mode("BOUNDARY"), **btn_cfg
        ).pack(side=tk.LEFT, padx=4, pady=4)
        tk.Button(
            self.toolbar, text="Obstacle", command=lambda: self._set_mode("OBSTACLE"), **btn_cfg
        ).pack(side=tk.LEFT, padx=4, pady=4)

        tk.Frame(self.toolbar, bg="#1a2f3d", width=20).pack(side=tk.LEFT)

        tk.Button(
            self.toolbar,
            text="Load P1 Zikzak",
            command=self._load_parkur1_zikzak,
            **btn_cfg,
        ).pack(side=tk.LEFT, padx=4, pady=4)
        tk.Button(
            self.toolbar,
            text="Load P1 Corridor",
            command=self._load_parkur1_corridor,
            **btn_cfg,
        ).pack(side=tk.LEFT, padx=4, pady=4)
        tk.Button(
            self.toolbar, text="Load Parkur2", command=self._load_parkur2, **btn_cfg
        ).pack(side=tk.LEFT, padx=4, pady=4)
        tk.Button(
            self.toolbar, text="Import JSON", command=self._import_json, **btn_cfg
        ).pack(side=tk.LEFT, padx=4, pady=4)
        tk.Button(
            self.toolbar, text="Export JSON", command=self._export_json, **btn_cfg
        ).pack(side=tk.LEFT, padx=4, pady=4)
        tk.Button(
            self.toolbar, text="Clear", command=self._clear_all, **btn_cfg
        ).pack(side=tk.LEFT, padx=4, pady=4)

        self.status_label = tk.Label(
            self.toolbar,
            text="Mode: SELECT",
            bg="#1a2f3d",
            fg="#7ec8ff",
            font=("Sans", 10, "bold"),
        )
        self.status_label.pack(side=tk.RIGHT, padx=10)

        self.canvas = tk.Canvas(
            self.root,
            width=self.width,
            height=self.height,
            bg="#0b2a3d",
            highlightthickness=0,
        )
        self.canvas.pack(fill=tk.BOTH, expand=True)

        self.info_label = tk.Label(
            self.root,
            text="Left=add/move | Right=delete | Wheel=zoom | Drag=pan",
            bg="#071b28",
            fg="#7ec8ff",
            font=("Sans", 10),
            anchor="w",
            padx=10,
            pady=4,
        )
        self.info_label.pack(fill=tk.X, side=tk.BOTTOM)

    def _bind_events(self) -> None:
        self.canvas.bind("<ButtonPress-1>", self._on_left_press)
        self.canvas.bind("<B1-Motion>", self._on_left_drag)
        self.canvas.bind("<ButtonRelease-1>", self._on_left_release)
        self.canvas.bind("<ButtonPress-3>", self._on_right_click)
        self.canvas.bind("<MouseWheel>", self._on_mousewheel)
        self.canvas.bind("<Button-4>", self._on_mousewheel)
        self.canvas.bind("<Button-5>", self._on_mousewheel)
        self.root.bind("<Key-plus>", lambda _e: self._change_zoom(1.15))
        self.root.bind("<Key-equal>", lambda _e: self._change_zoom(1.15))
        self.root.bind("<Key-minus>", lambda _e: self._change_zoom(1 / 1.15))
        self.root.bind("<Key-0>", lambda _e: self._reset_view())
        self.root.bind("<Delete>", lambda _e: self._delete_selected())
        self.root.bind("<BackSpace>", lambda _e: self._delete_selected())
        self.canvas.focus_set()

    def _set_mode(self, mode: str) -> None:
        self.mode = mode
        self.selected_item = None
        self.status_label.config(text=f"Mode: {mode}")
        self._draw()

    def _worlds_dir(self) -> str:
        return os.path.join(str(package_share_path()), "worlds")

    def _parkurs_dir(self) -> str:
        return os.path.join(str(package_share_path()), "parkurlar")

    def _parkur_path(self, filename: str, fallback_world: str) -> str:
        parkur_path = os.path.join(self._parkurs_dir(), filename)
        if os.path.isfile(parkur_path):
            return parkur_path
        return os.path.join(self._worlds_dir(), fallback_world)

    def _load_parkur1_zikzak(self) -> None:
        self._load_json_file(
            self._parkur_path("parkur1_zikzak.json", "parkur1_world.json")
        )

    def _load_parkur1_corridor(self) -> None:
        self._load_json_file(
            self._parkur_path("parkur1_corridor.json", "parkur1_world.json")
        )

    def _load_parkur2(self) -> None:
        self._load_json_file(self._parkur_path("parkur2.json", "parkur2_world.json"))

    def _import_json(self) -> None:
        path = filedialog.askopenfilename(
            defaultextension=".json",
            filetypes=[("JSON files", "*.json"), ("All files", "*.*")],
            initialdir=self.default_export_dir or os.getcwd(),
        )
        if path:
            self._load_json_file(path)

    def _build_station_pairs(
        self, route: list[tuple[float, float]], stations: list[tuple[float, float]], course_width_m: float
    ) -> list[dict]:
        boundaries = []
        previous = route[0]
        for index, center in enumerate(stations):
            next_point = route[min(index // 2 + 1, len(route) - 1)]
            dx = next_point[0] - previous[0]
            dy = next_point[1] - previous[1]
            length = max(math.hypot(dx, dy), 1e-6)
            left_x = -dy / length
            left_y = dx / length
            offset = course_width_m / 2.0
            for side, sign in (("left", 1.0), ("right", -1.0)):
                east = center[0] + left_x * offset * sign
                north = center[1] + left_y * offset * sign
                boundaries.append(
                    {
                        "id": f"course_{side}_{index}",
                        "kind": "course_boundary",
                        "class_name": "course_buoy",
                        "east_m": round(east, 2),
                        "north_m": round(north, 2),
                        "radius_m": 0.35,
                        "hue_deg": 28.0,
                        "color": "#ff8b2e",
                    }
                )
            previous = center
        return boundaries

    def _build_course_boundaries(
        self,
        route: list[tuple[float, float]],
        spacing_m: float,
        object_prefix: str,
        course_width_m: float,
        course_jitter_m: float,
    ) -> list[dict]:
        boundaries = []
        half_width = course_width_m / 2.0
        station_index = 0
        for start, end in zip(route, route[1:]):
            dx = end[0] - start[0]
            dy = end[1] - start[1]
            length = math.hypot(dx, dy)
            if length <= 1e-6:
                continue
            forward_x = dx / length
            forward_y = dy / length
            left_x = -forward_y
            left_y = forward_x
            count = max(1, int(round(length / max(spacing_m, 0.5))))
            for index in range(count):
                t = (index + 0.5) / count
                center_east = start[0] + dx * t
                center_north = start[1] + dy * t
                for side, sign in (("left", 1.0), ("right", -1.0)):
                    phase = station_index * 1.37 + (0.0 if sign > 0.0 else 2.1)
                    lateral_jitter = math.sin(phase) * course_jitter_m
                    forward_jitter = math.cos(phase * 0.7) * course_jitter_m * 0.5
                    offset = max(1.0, half_width + lateral_jitter)
                    east = center_east + left_x * offset * sign + forward_x * forward_jitter
                    north = center_north + left_y * offset * sign + forward_y * forward_jitter
                    boundaries.append(
                        {
                            "id": f"{object_prefix}_{side}_{station_index}",
                            "kind": "course_boundary",
                            "class_name": "course_buoy",
                            "east_m": round(east, 2),
                            "north_m": round(north, 2),
                            "radius_m": 0.35,
                            "hue_deg": 28.0,
                            "color": "#ff8b2e",
                        }
                    )
                station_index += 1
        return boundaries

    def _is_gps_waypoint_list(self, points) -> bool:
        return (
            bool(points)
            and isinstance(points[0], dict)
            and "lat" in points[0]
            and "lon" in points[0]
        )

    def _gps_waypoints_to_local(self, points) -> list[tuple[float, float]]:
        origin_lat = float(points[0]["lat"])
        origin_lon = float(points[0]["lon"])
        lat_scale = 111_320.0
        lon_scale = lat_scale * math.cos(math.radians(origin_lat))
        local_points = []
        for point in points:
            east_m = (float(point["lon"]) - origin_lon) * lon_scale
            north_m = (float(point["lat"]) - origin_lat) * lat_scale
            local_points.append((round(east_m, 2), round(north_m, 2)))
        return local_points

    def _parse_point_list(self, points) -> list[tuple[float, float]]:
        if self._is_gps_waypoint_list(points):
            return self._gps_waypoints_to_local(points)
        return [(float(p[0]), float(p[1])) for p in points]

    def _load_json_file(self, path: str) -> None:
        try:
            with open(path, "r", encoding="utf-8") as f:
                data = json.load(f)
        except Exception as exc:
            messagebox.showerror("Load Error", str(exc))
            return

        raw_route = data.get("route", [])
        raw_waypoints = data.get("waypoints", [])
        self.route_points = self._parse_point_list(raw_route)
        self.waypoints = self._parse_point_list(raw_waypoints)
        if not self.route_points and self._is_gps_waypoint_list(raw_waypoints):
            self.route_points = list(self.waypoints)
        self.boundaries = data.get("boundaries", [])
        self.obstacles = data.get("obstacles", [])

        # Backward compatibility: old format with build_method / stations
        if "build_method" in data and not self.boundaries:
            build_method = data.get("build_method", "")
            course_width = float(data.get("course_width_m", 8.82))
            route = list(self.route_points)
            if build_method == "station_pairs":
                stations = [tuple(p) for p in data.get("stations", [])]
                self.boundaries = self._build_station_pairs(route, stations, course_width)
                # Migrate stations to waypoints if no explicit waypoints
                if not self.waypoints:
                    self.waypoints = stations
            elif build_method == "course_boundaries":
                spacing = float(data.get("spacing_m", 7.0))
                prefix = data.get("object_prefix", "course")
                jitter = float(data.get("course_jitter_m", 0.0))
                self.boundaries = self._build_course_boundaries(
                    route, spacing, prefix, course_width, jitter
                )
                if not self.waypoints:
                    self.waypoints = route

        # Also migrate old standalone obstacles list if present under old keys
        if not self.obstacles and "obstacles" not in data:
            pass  # keep empty

        self._auto_fit()
        self._draw()
        self.get_logger().info(f"Loaded parkur from {path}")

    def _export_json(self) -> None:
        data = {
            "variant": "custom",
            "route": [[round(e, 2), round(n, 2)] for e, n in self.route_points],
            "waypoints": [[round(e, 2), round(n, 2)] for e, n in self.waypoints],
            "boundaries": self.boundaries,
        }
        if self.obstacles:
            data["obstacles"] = self.obstacles

        path = filedialog.asksaveasfilename(
            defaultextension=".json",
            filetypes=[("JSON files", "*.json"), ("All files", "*.*")],
            initialdir=self.default_export_dir or self._parkurs_dir(),
            initialfile="custom_parkur.json",
        )
        if path:
            try:
                with open(path, "w", encoding="utf-8") as f:
                    json.dump(data, f, indent=2, ensure_ascii=False)
                messagebox.showinfo("Saved", f"Parkur saved to:\n{path}")
                self.get_logger().info(f"Exported parkur to {path}")
            except Exception as exc:
                messagebox.showerror("Save Error", str(exc))

    def _clear_all(self) -> None:
        self.route_points.clear()
        self.waypoints.clear()
        self.boundaries.clear()
        self.obstacles.clear()
        self.selected_item = None
        self._draw()

    def _auto_fit(self) -> None:
        all_points = list(self.route_points) + list(self.waypoints)
        all_points += [(b["east_m"], b["north_m"]) for b in self.boundaries]
        all_points += [(o["east_m"], o["north_m"]) for o in self.obstacles]
        if not all_points:
            self.pan_east = 0.0
            self.pan_north = 0.0
            self.zoom = 8.0
            return
        min_e = min(p[0] for p in all_points)
        max_e = max(p[0] for p in all_points)
        min_n = min(p[1] for p in all_points)
        max_n = max(p[1] for p in all_points)
        self.pan_east = (min_e + max_e) / 2.0
        self.pan_north = (min_n + max_n) / 2.0
        span_e = max(max_e - min_e, 10.0)
        span_n = max(max_n - min_n, 10.0)
        self.zoom = min(
            (self.width - 160) / span_e,
            (self.height - 160) / span_n,
        )
        self.zoom = max(2.0, min(self.zoom, 50.0))

    def _to_screen(self, east_m: float, north_m: float) -> tuple[float, float]:
        x = self.origin_x + (east_m - self.pan_east) * self.zoom
        y = self.origin_y - (north_m - self.pan_north) * self.zoom
        return x, y

    def _to_world(self, x: float, y: float) -> tuple[float, float]:
        east_m = self.pan_east + (x - self.origin_x) / self.zoom
        north_m = self.pan_north + (self.origin_y - y) / self.zoom
        return east_m, north_m

    def _nearest_item(
        self, east_m: float, north_m: float
    ) -> tuple[str, int] | None:
        best = None
        best_dist = 1.5
        for idx, (e, n) in enumerate(self.route_points):
            d = math.hypot(e - east_m, n - north_m)
            if d < best_dist:
                best_dist = d
                best = ("route", idx)
        for idx, (e, n) in enumerate(self.waypoints):
            d = math.hypot(e - east_m, n - north_m)
            if d < best_dist:
                best_dist = d
                best = ("waypoint", idx)
        for idx, b in enumerate(self.boundaries):
            d = math.hypot(b["east_m"] - east_m, b["north_m"] - north_m)
            if d < best_dist:
                best_dist = d
                best = ("boundary", idx)
        for idx, o in enumerate(self.obstacles):
            d = math.hypot(o["east_m"] - east_m, o["north_m"] - north_m)
            if d < best_dist:
                best_dist = d
                best = ("obstacle", idx)
        return best

    def _on_left_press(self, event) -> None:
        self.canvas.focus_set()
        east_m, north_m = self._to_world(event.x, event.y)

        if self.mode == "SELECT":
            item = self._nearest_item(east_m, north_m)
            if item:
                self.dragging = True
                self.drag_item = item
                self.drag_start = (east_m, north_m)
                self.selected_item = item
            else:
                self.dragging = True
                self.drag_item = ("canvas", -1)
                self.drag_start = (east_m, north_m)
            self._draw()
            return

        if self.mode == "ROUTE":
            self.route_points.append((round(east_m, 2), round(north_m, 2)))
        elif self.mode == "WAYPOINT":
            self.waypoints.append((round(east_m, 2), round(north_m, 2)))
        elif self.mode == "BOUNDARY":
            b_id = f"boundary_{len(self.boundaries)}"
            self.boundaries.append(
                {
                    "id": b_id,
                    "kind": "course_boundary",
                    "class_name": "course_buoy",
                    "east_m": round(east_m, 2),
                    "north_m": round(north_m, 2),
                    "radius_m": 0.35,
                    "hue_deg": 28.0,
                    "color": "#ff8b2e",
                }
            )
        elif self.mode == "OBSTACLE":
            obs_id = f"obstacle_{len(self.obstacles)}"
            self.obstacles.append(
                {
                    "id": obs_id,
                    "kind": "obstacle",
                    "class_name": "obstacle_buoy",
                    "east_m": round(east_m, 2),
                    "north_m": round(north_m, 2),
                    "radius_m": 0.70,
                    "hue_deg": 62.0,
                    "color": "#ffe15a",
                }
            )
        self._draw()

    def _on_left_drag(self, event) -> None:
        if not self.dragging or not self.drag_item:
            return
        east_m, north_m = self._to_world(event.x, event.y)
        kind, idx = self.drag_item

        if kind == "canvas":
            de = self.drag_start[0] - east_m
            dn = self.drag_start[1] - north_m
            self.pan_east += de
            self.pan_north += dn
            self._draw()
            return

        if kind == "route" and 0 <= idx < len(self.route_points):
            self.route_points[idx] = (round(east_m, 2), round(north_m, 2))
        elif kind == "waypoint" and 0 <= idx < len(self.waypoints):
            self.waypoints[idx] = (round(east_m, 2), round(north_m, 2))
        elif kind == "boundary" and 0 <= idx < len(self.boundaries):
            self.boundaries[idx]["east_m"] = round(east_m, 2)
            self.boundaries[idx]["north_m"] = round(north_m, 2)
        elif kind == "obstacle" and 0 <= idx < len(self.obstacles):
            self.obstacles[idx]["east_m"] = round(east_m, 2)
            self.obstacles[idx]["north_m"] = round(north_m, 2)
        self._draw()

    def _on_left_release(self, _event) -> None:
        self.dragging = False
        self.drag_item = None

    def _on_right_click(self, event) -> None:
        east_m, north_m = self._to_world(event.x, event.y)
        item = self._nearest_item(east_m, north_m)
        if not item:
            return
        kind, idx = item
        if kind == "route" and 0 <= idx < len(self.route_points):
            del self.route_points[idx]
        elif kind == "waypoint" and 0 <= idx < len(self.waypoints):
            del self.waypoints[idx]
        elif kind == "boundary" and 0 <= idx < len(self.boundaries):
            del self.boundaries[idx]
        elif kind == "obstacle" and 0 <= idx < len(self.obstacles):
            del self.obstacles[idx]
        self.selected_item = None
        self._draw()

    def _on_mousewheel(self, event) -> None:
        if getattr(event, "num", None) == 4 or getattr(event, "delta", 0) > 0:
            self._change_zoom(1.15)
        else:
            self._change_zoom(1 / 1.15)

    def _change_zoom(self, factor: float) -> None:
        self.zoom = max(1.0, min(100.0, self.zoom * factor))
        self._draw()

    def _reset_view(self) -> None:
        self.zoom = 8.0
        self.pan_east = 0.0
        self.pan_north = 0.0
        self._draw()

    def _delete_selected(self) -> None:
        if self.selected_item is None:
            return
        kind, idx = self.selected_item
        if kind == "route" and 0 <= idx < len(self.route_points):
            del self.route_points[idx]
        elif kind == "waypoint" and 0 <= idx < len(self.waypoints):
            del self.waypoints[idx]
        elif kind == "boundary" and 0 <= idx < len(self.boundaries):
            del self.boundaries[idx]
        elif kind == "obstacle" and 0 <= idx < len(self.obstacles):
            del self.obstacles[idx]
        self.selected_item = None
        self._draw()

    def _draw(self) -> None:
        self.canvas.delete("all")
        self._draw_grid()
        self._draw_route()
        self._draw_waypoints()
        self._draw_boundaries()
        self._draw_obstacles()
        self._draw_ui_overlay()

    def _draw_grid(self) -> None:
        spacing = self.grid_spacing_m * self.zoom
        if spacing < 10:
            spacing *= 2
        elif spacing > 80:
            spacing /= 2

        offset_x = ((self.pan_east * self.zoom) % spacing)
        offset_y = ((self.pan_north * self.zoom) % spacing)

        x = self.origin_x - offset_x
        while x < self.width:
            self.canvas.create_line(x, 0, x, self.height, fill="#12364c", width=1)
            x += spacing

        y = self.origin_y + offset_y
        while y > 0:
            self.canvas.create_line(0, y, self.width, y, fill="#12364c", width=1)
            y -= spacing

        self.canvas.create_line(
            self.origin_x, 0, self.origin_x, self.height, fill="#19445c", width=2
        )
        self.canvas.create_line(
            0, self.origin_y, self.width, self.origin_y, fill="#19445c", width=2
        )

        self.canvas.create_text(
            self.origin_x + 8, 18, text="N", fill="#d7f3ff",
            anchor="w", font=("Sans", 14, "bold")
        )
        self.canvas.create_line(
            self.origin_x, 22, self.origin_x, 45, fill="#d7f3ff", width=2
        )
        self.canvas.create_polygon(
            self.origin_x, 15, self.origin_x - 6, 30,
            self.origin_x + 6, 30, fill="#d7f3ff", outline=""
        )

    def _draw_route(self) -> None:
        if len(self.route_points) >= 2:
            pts = []
            for e, n in self.route_points:
                x, y = self._to_screen(e, n)
                pts.extend([x, y])
            self.canvas.create_line(
                pts, fill="#7ec8ff", width=2, dash=(8, 6)
            )

        for idx, (e, n) in enumerate(self.route_points):
            x, y = self._to_screen(e, n)
            r = 7
            fill = "#ffcf4a" if self.selected_item == ("route", idx) else "#4da6ff"
            self.canvas.create_oval(
                x - r, y - r, x + r, y + r,
                fill=fill, outline="#ffffff", width=2
            )
            self.canvas.create_text(
                x + 12, y - 12, text=f"R{idx}",
                fill="#ffffff", anchor="w", font=("Sans", 9, "bold")
            )

    def _draw_waypoints(self) -> None:
        for idx, (e, n) in enumerate(self.waypoints):
            x, y = self._to_screen(e, n)
            r = 6
            fill = "#48d17a" if self.selected_item == ("waypoint", idx) else "#2ecc71"
            self.canvas.create_rectangle(
                x - r, y - r, x + r, y + r,
                fill=fill, outline="#ffffff", width=2
            )
            self.canvas.create_text(
                x + 12, y - 12, text=f"W{idx}",
                fill="#48d17a", anchor="w", font=("Sans", 9, "bold")
            )

    def _draw_boundaries(self) -> None:
        for idx, b in enumerate(self.boundaries):
            x, y = self._to_screen(b["east_m"], b["north_m"])
            r = max(6.0, b.get("radius_m", 0.35) * self.zoom)
            fill = b.get("color", "#ff8b2e")
            outline = "#ffffff" if self.selected_item == ("boundary", idx) else "#ff8b2e"
            width = 3 if self.selected_item == ("boundary", idx) else 2
            self.canvas.create_oval(
                x - r, y - r, x + r, y + r,
                fill=fill, outline=outline, width=width
            )
            self.canvas.create_text(
                x, y - r - 8, text=b.get("id", f"b{idx}"),
                fill="#ff8b2e", anchor="s", font=("Sans", 8, "bold")
            )

    def _draw_obstacles(self) -> None:
        for idx, o in enumerate(self.obstacles):
            x, y = self._to_screen(o["east_m"], o["north_m"])
            r = max(6.0, o.get("radius_m", 0.7) * self.zoom)
            fill = o.get("color", "#ffe15a")
            outline = "#ffffff" if self.selected_item == ("obstacle", idx) else "#ffe15a"
            width = 3 if self.selected_item == ("obstacle", idx) else 2
            self.canvas.create_oval(
                x - r, y - r, x + r, y + r,
                fill=fill, outline=outline, width=width
            )
            self.canvas.create_text(
                x, y - r - 8, text=o.get("id", f"obs{idx}"),
                fill="#ffe15a", anchor="s", font=("Sans", 8, "bold")
            )

    def _draw_ui_overlay(self) -> None:
        info = [
            f"Zoom: {self.zoom:.1f}x",
            f"Route pts: {len(self.route_points)}",
            f"Waypoints: {len(self.waypoints)}",
            f"Boundaries: {len(self.boundaries)}",
            f"Obstacles: {len(self.obstacles)}",
        ]
        for row, text in enumerate(info):
            self.canvas.create_text(
                self.width - 12,
                18 + row * 22,
                text=text,
                fill="#7ec8ff",
                anchor="e",
                font=("Sans", 10),
            )

    def run(self) -> None:
        self._tick()
        self.root.mainloop()

    def _tick(self) -> None:
        rclpy.spin_once(self, timeout_sec=0.0)
        self.root.after(50, self._tick)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ParkurEditorNode()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
