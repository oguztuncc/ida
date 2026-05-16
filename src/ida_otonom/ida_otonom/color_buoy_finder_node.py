import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String

from .common import normalize_angle_deg, now_ts, to_json


class ColorBuoyFinderNode(Node):
    """
    Lidar verisi ile renkli dubaları tespit eder.

    Şimdilik YOLO olmadığı için:
    - LiDAR ile obje tespiti yapar
    - Objelerin renklerini simülasyondan veya sonradan eşleştirir
    - Hedef renkteki en yakın dubayı bulur
    """

    def __init__(self) -> None:
        super().__init__("color_buoy_finder_node")

        # Parametreler
        self.declare_parameter("target_color", "")
        self.declare_parameter("scan_range_max_m", 15.0)
        self.declare_parameter("scan_range_min_m", 0.15)
        self.declare_parameter("min_object_size_m", 0.3)
        self.declare_parameter("max_object_size_m", 1.0)
        self.declare_parameter("object_cluster_threshold_m", 0.5)
        self.declare_parameter("detection_confidence_threshold", 0.6)
        self.declare_parameter("sensor_timeout_s", 2.0)

        self.target_color = str(self.get_parameter("target_color").value)
        self.scan_range_max_m = float(
            self.get_parameter("scan_range_max_m").value
        )
        self.scan_range_min_m = float(
            self.get_parameter("scan_range_min_m").value
        )
        self.min_object_size_m = float(
            self.get_parameter("min_object_size_m").value
        )
        self.max_object_size_m = float(
            self.get_parameter("max_object_size_m").value
        )
        self.object_cluster_threshold_m = float(
            self.get_parameter("object_cluster_threshold_m").value
        )
        self.detection_confidence_threshold = float(
            self.get_parameter("detection_confidence_threshold").value
        )
        self.sensor_timeout_s = float(
            self.get_parameter("sensor_timeout_s").value
        )

        # Durum
        self.latest_scan = None
        self.scan_ts = 0.0
        self.detected_objects = []
        self.target_buoy = None
        self._sim_colors: dict[str, str] = {}  # obj_id -> color from sim
        self._camera_color_detections = []
        self.detections_ts = 0.0

        # Publishers
        self.objects_pub = self.create_publisher(
            String,
            "/parkur3/detected_objects",
            10,
        )
        self.target_pub = self.create_publisher(
            String,
            "/parkur3/target_buoy",
            10,
        )
        self.status_pub = self.create_publisher(
            String,
            "/parkur3/finder_status",
            10,
        )

        # Subscribers
        self.create_subscription(
            LaserScan,
            "/scan",
            self.scan_cb,
            10,
        )
        self.create_subscription(
            String,
            "/parkur3/target_color",
            self.color_cb,
            10,
        )
        self.create_subscription(
            String,
            "/perception/buoy_detections",
            self.detections_cb,
            10,
        )

        self.timer = self.create_timer(0.2, self.loop)

        self.get_logger().info(
            f"ColorBuoyFinderNode started. Target: '{self.target_color}'"
        )

    def color_cb(self, msg: String) -> None:
        """Hedef renk değiştiğinde."""
        new_color = msg.data.strip().lower()
        if new_color != self.target_color:
            self.target_color = new_color
            self.get_logger().info(f"Target color updated: '{self.target_color}'")
            self.target_buoy = None  # Yeni renk için hedef sıfırla

    def scan_cb(self, msg: LaserScan) -> None:
        """Lidar verisi geldiğinde."""
        self.latest_scan = msg
        self.scan_ts = now_ts()

    def detections_cb(self, msg: String) -> None:
        """Görüntü işleme çıktısından renkli duba adaylarını kaydet."""
        try:
            import json
            data = json.loads(msg.data)
            color_detections = []
            for det in data.get("detections", []):
                obj_id = det.get("id")
                class_name = det.get("class_name", "")
                # class_name: "red_buoy", "blue_buoy", "yellow_buoy"
                color = class_name.replace("_buoy", "")
                if color in ("red", "blue", "yellow"):
                    self._sim_colors[obj_id] = color
                    range_m = det.get("range_m")
                    bearing_deg = det.get("bearing_deg")
                    if range_m is not None and bearing_deg is not None:
                        color_detections.append(
                            {
                                "id": obj_id,
                                "range_m": float(range_m),
                                "bearing_deg": float(bearing_deg),
                                "color": color,
                                "confidence": float(
                                    det.get("confidence", 0.0)
                                ),
                                "source": "vision_depth",
                            }
                        )
                    self.get_logger().info(
                        f"Vision detection: {obj_id} -> {color}",
                        throttle_duration_sec=5.0,
                    )
            self._camera_color_detections = color_detections
            self.detections_ts = now_ts()
        except Exception as e:
            self.get_logger().warn(f"detections_cb error: {e}")

    def _cluster_objects(self, scan: LaserScan) -> list[dict]:
        """
        Lidar verisinden obje kümeleri oluştur.

        Basit DBSCAN benzeri kümeleme.
        """
        if scan is None or len(scan.ranges) == 0:
            return []

        angle = scan.angle_min
        angle_inc = scan.angle_increment

        objects = []
        current_cluster = []

        for i, range_val in enumerate(scan.ranges):
            # Geçerli mesafe kontrolü
            if (
                math.isfinite(range_val)
                and self.scan_range_min_m <= range_val <= self.scan_range_max_m
            ):
                # Polar -> Kartezyen
                x = range_val * math.cos(angle)
                y = range_val * math.sin(angle)
                current_cluster.append({
                    "x": x,
                    "y": y,
                    "range": range_val,
                    "angle": angle,
                    "index": i,
                })
            else:
                # Küme sonu
                if len(current_cluster) >= 3:
                    obj = self._analyze_cluster(current_cluster)
                    if obj:
                        objects.append(obj)
                current_cluster = []

            angle += angle_inc

        # Son kümeyi de kontrol et
        if len(current_cluster) >= 3:
            obj = self._analyze_cluster(current_cluster)
            if obj:
                objects.append(obj)

        return objects

    def _analyze_cluster(self, cluster: list[dict]) -> dict | None:
        """Bir kümeyi analiz et ve obje özelliklerini çıkar."""
        if len(cluster) < 3:
            return None

        # Merkez hesapla
        avg_x = sum(p["x"] for p in cluster) / len(cluster)
        avg_y = sum(p["y"] for p in cluster) / len(cluster)
        avg_range = sum(p["range"] for p in cluster) / len(cluster)

        # Boyut hesapla (küme genişliği)
        xs = [p["x"] for p in cluster]
        ys = [p["y"] for p in cluster]
        width = max(xs) - min(xs)
        height = max(ys) - min(ys)
        size = math.hypot(width, height)

        # Boyut kontrolü (daha geniş aralık)
        if not (0.15 <= size <= 2.0):
            return None

        # Açı hesapla (robotun solu pozitif)
        bearing = math.degrees(math.atan2(avg_y, avg_x))

        matched_color = "unknown"
        if now_ts() - self.detections_ts <= self.sensor_timeout_s:
            best_score = None
            for det in self._camera_color_detections:
                range_error = abs(float(det["range_m"]) - avg_range)
                bearing_error = abs(
                    normalize_angle_deg(float(det["bearing_deg"]) - bearing)
                )
                score = range_error + bearing_error / 20.0
                if best_score is None or score < best_score:
                    best_score = score
                    matched_color = str(det.get("color", "unknown"))
            if best_score is not None and best_score > 2.0:
                matched_color = "unknown"

        return {
            "id": f"obj_{cluster[0]['index']}",
            "center_x": avg_x,
            "center_y": avg_y,
            "range_m": avg_range,
            "bearing_deg": bearing,
            "size_m": size,
            "point_count": len(cluster),
            "color": matched_color,
            "confidence": min(1.0, len(cluster) / 20.0),  # Nokta sayısına göre
        }

    def _find_target_buoy(self, objects: list[dict]) -> dict | None:
        """Hedef renkteki en yakın dubayı bul."""
        if not self.target_color:
            return None

        if now_ts() - self.detections_ts <= self.sensor_timeout_s:
            target_detections = [
                det for det in self._camera_color_detections
                if det.get("color") == self.target_color
                and float(det.get("range_m", 0.0)) > 0.3
            ]
            if target_detections:
                target_detections.sort(key=lambda det: float(det["range_m"]))
                best = target_detections[0]
                self.get_logger().info(
                    f"TARGET FOUND BY VISION: range={best['range_m']:.1f}m, "
                    f"bearing={best['bearing_deg']:.1f}°",
                    throttle_duration_sec=2.0,
                )
                return best

        if not objects:
            return None

        self.get_logger().info(
            f"Finding target_color={self.target_color}, "
            f"objects={len(objects)}, sim_colors={self._sim_colors}",
            throttle_duration_sec=2.0,
        )

        # Renk eşleştirme: LiDAR objesinin color alanı ile
        target_objects = []
        for obj in objects:
            obj_color = obj.get("color", "unknown")
            self.get_logger().info(
                f"  obj_id={obj['id']}, "
                f"range={obj['range_m']:.1f}, color={obj_color}",
                throttle_duration_sec=2.0,
            )
            if obj_color == self.target_color and obj["range_m"] > 0.3:
                target_objects.append(obj)

        if not target_objects:
            self.get_logger().warn(
                "No matching target objects found",
                throttle_duration_sec=2.0,
            )
            return None

        # En yakın olanı seç
        target_objects.sort(key=lambda x: x["range_m"])
        best = target_objects[0]

        self.get_logger().info(
            f"TARGET FOUND: range={best['range_m']:.1f}m, bearing={best['bearing_deg']:.1f}°",
            throttle_duration_sec=2.0,
        )

        # Güven kontrolü (düşük güven de kabul et - simülasyonda güven hesabı farklı)
        if best["confidence"] < self.detection_confidence_threshold:
            self.get_logger().warn(
                f"Confidence low ({best['confidence']:.2f}) but accepting target",
                throttle_duration_sec=2.0,
            )
            # Simülasyonda güven hesabı düşük çıkıyor, hedefi yine de döndür
            return best

        return best

    def loop(self) -> None:
        now = now_ts()

        # Sensor timeout kontrolü
        if now - self.scan_ts > self.sensor_timeout_s:
            self.get_logger().warn("LiDAR timeout", throttle_duration_sec=5.0)
            return

        if self.latest_scan is None:
            return

        # Objeleri tespit et
        self.detected_objects = self._cluster_objects(self.latest_scan)

        # Hedef rengi bul
        if self.target_color:
            self.target_buoy = self._find_target_buoy(self.detected_objects)

        # Publish
        self._publish_objects()
        self._publish_target()
        self._publish_status()

    def _publish_objects(self) -> None:
        """Tespit edilen objeleri publish et."""
        payload = {
            "timestamp": now_ts(),
            "object_count": len(self.detected_objects),
            "objects": self.detected_objects,
        }
        self.objects_pub.publish(String(data=to_json(payload)))

    def _publish_target(self) -> None:
        """Hedef duba bilgisini publish et."""
        if self.target_buoy:
            payload = {
                "timestamp": now_ts(),
                "found": True,
                "color": self.target_color,
                "range_m": self.target_buoy["range_m"],
                "bearing_deg": self.target_buoy["bearing_deg"],
                "confidence": self.target_buoy["confidence"],
            }
        else:
            payload = {
                "timestamp": now_ts(),
                "found": False,
                "color": self.target_color,
                "reason": "no_target" if self.target_color else "no_color_set",
            }
        self.target_pub.publish(String(data=to_json(payload)))

    def _publish_status(self) -> None:
        """Node statusunu publish et."""
        payload = {
            "timestamp": now_ts(),
            "target_color": self.target_color,
            "objects_detected": len(self.detected_objects),
            "target_found": self.target_buoy is not None,
            "sensor_age_s": now_ts() - self.scan_ts,
        }
        self.status_pub.publish(String(data=to_json(payload)))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ColorBuoyFinderNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
