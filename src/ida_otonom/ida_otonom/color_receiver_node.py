import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class ColorReceiverNode(Node):
    """
    IHA'dan gelen hedef renk bilgisini alır ve sistemde publish eder.

    IHA ile MAVLink/Telemetry haberleşmesi bu node'da yapılacak.
    Şimdilik manuel test için subscriber ve publisher yapısı hazır.
    """

    def __init__(self) -> None:
        super().__init__("color_receiver_node")

        # Parametreler
        self.declare_parameter("default_color", "")  # Boş = henüz renk alınmadı
        self.declare_parameter("valid_colors", ["red", "blue", "yellow"])
        self.declare_parameter("color_timeout_s", 30.0)  # Renk alınmadan timeout
        self.declare_parameter("auto_start_search", True)  # Renk alınca otomatik ara

        self.valid_colors = list(self.get_parameter("valid_colors").value)
        self.color_timeout_s = float(self.get_parameter("color_timeout_s").value)
        self.auto_start_search = bool(self.get_parameter("auto_start_search").value)

        self.target_color = ""
        self.color_received = False
        self.color_received_ts = 0.0
        default_color = str(self.get_parameter("default_color").value)
        if default_color:
            validated = self._validate_color(default_color)
            if validated is not None:
                self.target_color = validated
                self.color_received = True
                self.color_received_ts = self.get_clock().now().nanoseconds / 1e9

        # Publishers
        self.color_pub = self.create_publisher(
            String,
            "/parkur3/target_color",
            10,
        )
        self.status_pub = self.create_publisher(
            String,
            "/parkur3/color_status",
            10,
        )

        # Subscribers
        # IHA'dan renk gelecek (MAVLink wrapper'dan)
        self.create_subscription(
            String,
            "/iha/target_color",
            self.iha_color_cb,
            10,
        )
        self.create_subscription(
            String,
            "/mission/target_color",
            self.iha_color_cb,
            10,
        )
        # Manuel test için
        self.create_subscription(
            String,
            "/parkur3/set_color",
            self.manual_color_cb,
            10,
        )

        self.timer = self.create_timer(1.0, self.loop)

        self.get_logger().info(
            f"ColorReceiverNode started. Valid colors: {self.valid_colors}. "
            f"Target color: '{self.target_color or 'waiting'}'."
        )

    def _validate_color(self, color: str) -> str | None:
        """Renk doğrulama."""
        color = color.lower().strip()
        if color in self.valid_colors:
            return color
        # Alternatif isimler
        color_map = {
            "kirmizi": "red", "kırmızı": "red", "red": "red",
            "mavi": "blue", "blue": "blue",
            "sari": "yellow", "sarı": "yellow", "yellow": "yellow",
        }
        return color_map.get(color)

    def _set_color(self, color: str) -> None:
        """Hedef rengi ayarla."""
        validated = self._validate_color(color)
        if validated is None:
            self.get_logger().warn(
                f"Invalid color '{color}'. Valid: {self.valid_colors}"
            )
            return

        self.target_color = validated
        self.color_received = True
        self.color_received_ts = self.get_clock().now().nanoseconds / 1e9
        self.get_logger().info(
            f"TARGET COLOR SET: {self.target_color.upper()}"
        )

    def iha_color_cb(self, msg: String) -> None:
        """Iha'dan gelen renk mesajı."""
        if not msg.data:
            return
        self._set_color(msg.data)

    def manual_color_cb(self, msg: String) -> None:
        """Manuel test için renk mesajı."""
        if not msg.data:
            return
        self._set_color(msg.data)

    def loop(self) -> None:
        now = self.get_clock().now().nanoseconds / 1e9

        # Renk publish et
        self.color_pub.publish(String(data=self.target_color))

        # Status publish et
        import json
        status = {
            "color_received": self.color_received,
            "target_color": self.target_color,
            "valid_colors": self.valid_colors,
            "elapsed_since_color_s": (
                now - self.color_received_ts if self.color_received else -1.0
            ),
            "timed_out": (
                self.color_received
                and (now - self.color_received_ts) > self.color_timeout_s
            ),
        }
        self.status_pub.publish(String(data=json.dumps(status)))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ColorReceiverNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
