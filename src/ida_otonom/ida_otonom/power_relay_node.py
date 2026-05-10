import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String

from .common import to_json

try:
    import Jetson.GPIO as GPIO
except Exception:
    GPIO = None


class PowerRelayNode(Node):
    def __init__(self) -> None:
        super().__init__("power_relay_node")

        self.declare_parameter("enabled", False)
        self.declare_parameter("gpio_pin", -1)
        self.declare_parameter("active_low", False)
        self.declare_parameter("initial_motor_power_enabled", False)

        self.enabled = bool(self.get_parameter("enabled").value)
        self.gpio_pin = int(self.get_parameter("gpio_pin").value)
        self.active_low = bool(self.get_parameter("active_low").value)
        self.motor_power_enabled = bool(
            self.get_parameter("initial_motor_power_enabled").value
        )

        if self.enabled and (GPIO is None or self.gpio_pin < 0):
            self.get_logger().error(
                "Power relay GPIO is not available or gpio_pin is invalid; "
                "leaving relay control disabled"
            )
            self.enabled = False

        if self.enabled:
            GPIO.setmode(GPIO.BOARD)
            GPIO.setup(
                self.gpio_pin,
                GPIO.OUT,
                initial=self.output_level(False),
            )
            self.write_motor_power(self.motor_power_enabled)
        else:
            self.motor_power_enabled = False

        self.create_subscription(Bool, "/safety/kill", self.kill_cb, 10)
        self.create_subscription(
            Bool,
            "/safety/motor_power_enable",
            self.motor_power_cb,
            10,
        )
        self.status_pub = self.create_publisher(
            String,
            "/safety/power_relay_status",
            10,
        )
        self.timer = self.create_timer(0.5, self.loop)

    def output_level(self, motor_power_enabled: bool) -> bool:
        level = motor_power_enabled
        return not level if self.active_low else level

    def write_motor_power(self, motor_power_enabled: bool) -> None:
        self.motor_power_enabled = motor_power_enabled
        if self.enabled:
            GPIO.output(self.gpio_pin, self.output_level(motor_power_enabled))

    def kill_cb(self, msg: Bool) -> None:
        if msg.data:
            self.write_motor_power(False)

    def motor_power_cb(self, msg: Bool) -> None:
        if msg.data:
            self.get_logger().warn(
                "Motor power enable requested. Verify physical kill chain "
                "before enabling this topic in real tests."
            )
        self.write_motor_power(bool(msg.data))

    def loop(self) -> None:
        self.status_pub.publish(
            String(
                data=to_json(
                    {
                        "enabled": self.enabled,
                        "gpio_pin": self.gpio_pin,
                        "active_low": self.active_low,
                        "motor_power_enabled": self.motor_power_enabled,
                    }
                )
            )
        )

    def destroy_node(self):
        try:
            if self.enabled:
                self.write_motor_power(False)
                GPIO.cleanup(self.gpio_pin)
        finally:
            super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PowerRelayNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
