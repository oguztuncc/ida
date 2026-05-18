import json
import math
import os
import tkinter as tk
from pathlib import Path
from tkinter import filedialog, messagebox, ttk

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from .common import package_share_path


BUOY_RADIUS_M = 0.15


class ParkurEditorNode(Node):
    def __init__(self) -> None:
        super().__init__("parkur_editor_node")

        self.declare_parameter("window_width", 1200)
        self.declare_parameter("window_height", 800)
        self.declare_parameter("initial_zoom", 8.0)
        self.declare_parameter("grid_spacing_m", 5.0)
        self.declare_parameter("default_export_dir", "")
        self.declare_parameter("origin_lat", 40.1181000)
        self.declare_parameter("origin_lon", 26.4081000)

        self.width = int(self.get_parameter("window_width").value)
        self.height = int(self.get_parameter("window_height").value)
        self.zoom = float(self.get_parameter("initial_zoom").value)
        self.grid_spacing_m = float(self.get_parameter("grid_spacing_m").value)
        self.default_export_dir = str(
            self.get_parameter("default_export_dir").value
        )
        self.origin_lat = float(self.get_parameter("origin_lat").value)
        self.origin_lon = float(self.get_parameter("origin_lon").value)

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
        self.measure_points: list[tuple[float, float]] = []
        self.spawn: dict | None = None
        self.selected_item = None

        self.world_pub = self.create_publisher(String, "/sim/editor_world", 10)

        self.root = tk.Tk()
        self.root.title("IDA Parkur Editor")
        self.auto_pair_var = tk.BooleanVar(value=True)
        self.pair_distance_var = tk.StringVar(value="8.12")
        self.spawn_heading_var = tk.StringVar(value="")
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
        tk.Button(
            self.toolbar, text="Measure", command=lambda: self._set_mode("MEASURE"), **btn_cfg
        ).pack(side=tk.LEFT, padx=4, pady=4)
        tk.Button(
            self.toolbar, text="Spawn", command=lambda: self._set_mode("SPAWN"), **btn_cfg
        ).pack(side=tk.LEFT, padx=4, pady=4)

        tk.Checkbutton(
            self.toolbar,
            text="Auto Pair",
            variable=self.auto_pair_var,
            bg="#1a2f3d",
            fg="#ffffff",
            selectcolor="#2a4f6d",
            activebackground="#1a2f3d",
            activeforeground="#ffffff",
            font=("Sans", 10, "bold"),
        ).pack(side=tk.LEFT, padx=(10, 2), pady=4)
        tk.Label(
            self.toolbar,
            text="Pair m",
            bg="#1a2f3d",
            fg="#7ec8ff",
            font=("Sans", 9, "bold"),
        ).pack(side=tk.LEFT, padx=(4, 2))
        tk.Entry(
            self.toolbar,
            width=5,
            textvariable=self.pair_distance_var,
            font=("Sans", 10),
        ).pack(side=tk.LEFT, padx=(0, 8), pady=4)
        tk.Label(
            self.toolbar,
            text="Spawn hdg",
            bg="#1a2f3d",
            fg="#7ec8ff",
            font=("Sans", 9, "bold"),
        ).pack(side=tk.LEFT, padx=(2, 2))
        tk.Entry(
            self.toolbar,
            width=5,
            textvariable=self.spawn_heading_var,
            font=("Sans", 10),
        ).pack(side=tk.LEFT, padx=(0, 8), pady=4)

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

    def _set_status(self, text: str) -> None:
        self.status_label.config(text=text)

    def _parse_positive_float(self, text: str, fallback: float) -> float:
        try:
            value = float(text)
        except (TypeError, ValueError):
            return fallback
        return value if value > 0.0 else fallback

    def _pair_distance_m(self) -> float:
        return self._parse_positive_float(self.pair_distance_var.get(), 8.12)

    def _spawn_heading_deg(self, update_status: bool = True) -> float | None:
        text = self.spawn_heading_var.get().strip()
        if not text:
            return None
        try:
            return float(text) % 360.0
        except ValueError:
            if update_status:
                self._set_status("Invalid spawn heading")
            return None

    def _package_data_dir(self) -> str:
        local_package_dir = Path(__file__).resolve().parent
        if (local_package_dir / "missions").is_dir():
            return str(local_package_dir)

        share_path = package_share_path()
        for parent in share_path.parents:
            if parent.name == "install":
                source_package_dir = (
                    parent.parent / "src" / "ida_otonom" / "ida_otonom"
                )
                if (source_package_dir / "missions").is_dir():
                    return str(source_package_dir)

        return str(share_path)

    def _missions_dir(self) -> str:
        return os.path.join(self._package_data_dir(), "missions")

    def _mission_path(self, filename: str) -> str:
        return os.path.join(self._missions_dir(), filename)

    def _load_parkur1_zikzak(self) -> None:
        self._load_json_file(self._mission_path("parkur1_zikzak.json"))

    def _load_parkur1_corridor(self) -> None:
        self._load_json_file(self._mission_path("parkur1_corridor.json"))

    def _load_parkur2(self) -> None:
        self._load_json_file(self._mission_path("parkur2_sim.json"))

    def _import_json(self) -> None:
        path = filedialog.askopenfilename(
            defaultextension=".json",
            filetypes=[("JSON files", "*.json"), ("All files", "*.*")],
            initialdir=self.default_export_dir or self._missions_dir(),
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
                        "radius_m": BUOY_RADIUS_M,
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
                            "radius_m": BUOY_RADIUS_M,
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

    def _gps_waypoints_to_local(
        self,
        points,
        origin=None,
    ) -> list[tuple[float, float]]:
        origin = origin or {}
        origin_lat = float(origin.get("lat", points[0]["lat"]))
        origin_lon = float(origin.get("lon", points[0]["lon"]))
        lat_scale = 111_320.0
        lon_scale = lat_scale * math.cos(math.radians(origin_lat))
        local_points = []
        for point in points:
            east_m = (float(point["lon"]) - origin_lon) * lon_scale
            north_m = (float(point["lat"]) - origin_lat) * lat_scale
            local_points.append((round(east_m, 2), round(north_m, 2)))
        return local_points

    def _parse_point_list(self, points, origin=None) -> list[tuple[float, float]]:
        if self._is_gps_waypoint_list(points):
            return self._gps_waypoints_to_local(points, origin)
        return [(float(p[0]), float(p[1])) for p in points]

    def _parse_spawn(self, spawn, origin=None) -> dict | None:
        if not isinstance(spawn, dict):
            return None
        if "east_m" in spawn and "north_m" in spawn:
            east_m = float(spawn["east_m"])
            north_m = float(spawn["north_m"])
        elif "lat" in spawn and "lon" in spawn:
            origin = origin or {}
            origin_lat = float(origin.get("lat", self.origin_lat))
            origin_lon = float(origin.get("lon", self.origin_lon))
            lat_scale = 111_320.0
            lon_scale = lat_scale * math.cos(math.radians(origin_lat))
            east_m = (float(spawn["lon"]) - origin_lon) * lon_scale
            north_m = (float(spawn["lat"]) - origin_lat) * lat_scale
        else:
            return None

        parsed = {
            "east_m": round(east_m, 2),
            "north_m": round(north_m, 2),
        }
        if spawn.get("heading_deg") is not None:
            parsed["heading_deg"] = float(spawn["heading_deg"]) % 360.0
            self.spawn_heading_var.set(f"{parsed['heading_deg']:.1f}")
        return parsed

    def _next_boundary_id(self) -> str:
        used_ids = {str(b.get("id", "")) for b in self.boundaries}
        index = 0
        while f"boundary_{index}" in used_ids:
            index += 1
        return f"boundary_{index}"

    def _make_boundary(self, east_m: float, north_m: float) -> dict:
        return {
            "id": self._next_boundary_id(),
            "kind": "course_boundary",
            "class_name": "course_buoy",
            "east_m": round(east_m, 2),
            "north_m": round(north_m, 2),
            "radius_m": BUOY_RADIUS_M,
            "hue_deg": 28.0,
            "color": "#ff8b2e",
        }

    def _paired_boundary_point(
        self, east_m: float, north_m: float, radius_m: float
    ) -> tuple[float, float]:
        center_distance_m = self._pair_distance_m() + radius_m * 2.0
        return east_m, north_m + center_distance_m

    def _normalize_buoy_radii(self, buoys: list[dict]) -> list[dict]:
        normalized = []
        for buoy in buoys:
            item = dict(buoy)
            item["radius_m"] = BUOY_RADIUS_M
            normalized.append(item)
        return normalized

    def _local_waypoints_to_gps(self) -> list[dict]:
        lat_scale = 111_320.0
        lon_scale = lat_scale * math.cos(math.radians(self.origin_lat))
        return [
            {
                "lat": round(self.origin_lat + north / lat_scale, 7),
                "lon": round(self.origin_lon + east / lon_scale, 7),
            }
            for east, north in self.waypoints
        ]

    def _load_json_file(self, path: str) -> None:
        try:
            with open(path, "r", encoding="utf-8") as f:
                data = json.load(f)
        except Exception as exc:
            messagebox.showerror("Load Error", str(exc))
            return

        origin = data.get("origin") or {}
        if origin.get("lat") is not None and origin.get("lon") is not None:
            self.origin_lat = float(origin["lat"])
            self.origin_lon = float(origin["lon"])
        raw_route = data.get("route", [])
        raw_waypoints = data.get("local_waypoints", data.get("waypoints", []))
        self.route_points = self._parse_point_list(raw_route, origin)
        self.waypoints = self._parse_point_list(raw_waypoints, origin)
        if not self.route_points and self._is_gps_waypoint_list(raw_waypoints):
            self.route_points = list(self.waypoints)
        self.boundaries = self._normalize_buoy_radii(data.get("boundaries", []))
        self.obstacles = self._normalize_buoy_radii(data.get("obstacles", []))
        self.spawn_heading_var.set("")
        self.spawn = self._parse_spawn(data.get("spawn"), origin)
        self.measure_points.clear()

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
            "origin": {
                "lat": self.origin_lat,
                "lon": self.origin_lon,
            },
            "waypoints": self._local_waypoints_to_gps(),
            "route": [[round(e, 2), round(n, 2)] for e, n in self.route_points],
            "local_waypoints": [
                [round(e, 2), round(n, 2)] for e, n in self.waypoints
            ],
            "boundaries": self.boundaries,
        }
        if self.spawn is not None:
            spawn = {
                "east_m": round(float(self.spawn["east_m"]), 2),
                "north_m": round(float(self.spawn["north_m"]), 2),
            }
            heading_deg = self._spawn_heading_deg()
            if heading_deg is not None:
                spawn["heading_deg"] = round(heading_deg, 1)
            data["spawn"] = spawn
        if self.obstacles:
            data["obstacles"] = self.obstacles

        path = filedialog.asksaveasfilename(
            defaultextension=".json",
            filetypes=[("JSON files", "*.json"), ("All files", "*.*")],
            initialdir=self.default_export_dir or self._missions_dir(),
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
        self.measure_points.clear()
        self.spawn = None
        self.selected_item = None
        self._draw()

    def _auto_fit(self) -> None:
        all_points = list(self.route_points) + list(self.waypoints)
        all_points += [(b["east_m"], b["north_m"]) for b in self.boundaries]
        all_points += [(o["east_m"], o["north_m"]) for o in self.obstacles]
        if self.spawn is not None:
            all_points.append((self.spawn["east_m"], self.spawn["north_m"]))
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
        if self.spawn is not None:
            d = math.hypot(self.spawn["east_m"] - east_m, self.spawn["north_m"] - north_m)
            if d < best_dist:
                best = ("spawn", 0)
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
            boundary = self._make_boundary(east_m, north_m)
            self.boundaries.append(boundary)
            if self.auto_pair_var.get():
                radius_m = float(boundary.get("radius_m", BUOY_RADIUS_M))
                paired = self._paired_boundary_point(east_m, north_m, radius_m)
                self.boundaries.append(self._make_boundary(*paired))
                center_distance_m = self._pair_distance_m() + radius_m * 2.0
                self._set_status(
                    f"Auto Pair: {self._pair_distance_m():.2f} m surface, "
                    f"{center_distance_m:.2f} m center"
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
                    "radius_m": BUOY_RADIUS_M,
                    "hue_deg": 62.0,
                    "color": "#ffe15a",
                }
            )
        elif self.mode == "MEASURE":
            point = (round(east_m, 2), round(north_m, 2))
            if len(self.measure_points) >= 2:
                self.measure_points = [point]
            else:
                self.measure_points.append(point)
            if len(self.measure_points) == 2:
                start, end = self.measure_points
                distance_m = math.hypot(end[0] - start[0], end[1] - start[1])
                self._set_status(f"Measure: {distance_m:.2f} m")
        elif self.mode == "SPAWN":
            self.spawn = {
                "east_m": round(east_m, 2),
                "north_m": round(north_m, 2),
            }
            heading_deg = self._spawn_heading_deg()
            if heading_deg is not None:
                self.spawn["heading_deg"] = heading_deg
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
        elif kind == "spawn" and self.spawn is not None:
            self.spawn["east_m"] = round(east_m, 2)
            self.spawn["north_m"] = round(north_m, 2)
        self._draw()

    def _on_left_release(self, _event) -> None:
        self.dragging = False
        self.drag_item = None

    def _on_right_click(self, event) -> None:
        if self.mode == "MEASURE":
            self.measure_points.clear()
            self._draw()
            return
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
        elif kind == "spawn":
            self.spawn = None
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
        elif kind == "spawn":
            self.spawn = None
        self.selected_item = None
        self._draw()

    def _draw(self) -> None:
        self.canvas.delete("all")
        self._draw_grid()
        self._draw_route()
        self._draw_waypoints()
        self._draw_boundaries()
        self._draw_obstacles()
        self._draw_measurement()
        self._draw_spawn()
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
            r = max(6.0, b.get("radius_m", BUOY_RADIUS_M) * self.zoom)
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
            r = max(6.0, o.get("radius_m", BUOY_RADIUS_M) * self.zoom)
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

    def _draw_measurement(self) -> None:
        if not self.measure_points:
            return
        x0, y0 = self._to_screen(*self.measure_points[0])
        self.canvas.create_oval(
            x0 - 5, y0 - 5, x0 + 5, y0 + 5,
            fill="#ffffff", outline="#2cf0ff", width=2
        )
        if len(self.measure_points) < 2:
            return
        x1, y1 = self._to_screen(*self.measure_points[1])
        start, end = self.measure_points
        distance_m = math.hypot(end[0] - start[0], end[1] - start[1])
        self.canvas.create_line(
            x0, y0, x1, y1, fill="#2cf0ff", width=2, dash=(5, 4)
        )
        self.canvas.create_oval(
            x1 - 5, y1 - 5, x1 + 5, y1 + 5,
            fill="#ffffff", outline="#2cf0ff", width=2
        )
        self.canvas.create_text(
            (x0 + x1) / 2.0,
            (y0 + y1) / 2.0 - 10,
            text=f"{distance_m:.2f} m",
            fill="#2cf0ff",
            anchor="s",
            font=("Sans", 10, "bold"),
        )

    def _draw_spawn(self) -> None:
        if self.spawn is None:
            return
        x, y = self._to_screen(self.spawn["east_m"], self.spawn["north_m"])
        r = 10
        fill = "#ffcf4a" if self.selected_item == ("spawn", 0) else "#ff4ad8"
        self.canvas.create_polygon(
            x, y - r,
            x - r, y + r,
            x + r, y + r,
            fill=fill,
            outline="#ffffff",
            width=2,
        )
        heading_deg = self._spawn_heading_deg(update_status=False)
        if heading_deg is not None:
            heading = math.radians(heading_deg)
            x1 = x + math.sin(heading) * 28.0
            y1 = y - math.cos(heading) * 28.0
            self.canvas.create_line(x, y, x1, y1, fill="#ffffff", width=2)
        self.canvas.create_text(
            x + 14, y - 14, text="SPAWN",
            fill="#ff4ad8", anchor="w", font=("Sans", 9, "bold")
        )

    def _draw_ui_overlay(self) -> None:
        info = [
            f"Zoom: {self.zoom:.1f}x",
            f"Route pts: {len(self.route_points)}",
            f"Waypoints: {len(self.waypoints)}",
            f"Boundaries: {len(self.boundaries)}",
            f"Obstacles: {len(self.obstacles)}",
            f"Spawn: {'set' if self.spawn else 'none'}",
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
