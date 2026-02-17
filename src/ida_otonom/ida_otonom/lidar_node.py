import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool

class LidarNode(Node):
    def __init__(self):
        super().__init__('lidar_node')
        self.subscription = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10)
        self.publisher_ = self.create_publisher(Bool, '/acil_durum', 10)

    def scan_callback(self, msg):
        # Lidar verisi 0-360 derece arası bir listedir.
        # Ön taraf genelde 0. indekstir (veya 360'a yakın son indeksler).
        # Önümüzdeki 20 derecelik açıya bakıyoruz:
        on_taraf = msg.ranges[0:10] + msg.ranges[-10:]
        
        engel_var = False
        for mesafe in on_taraf:
            # 0.0 hatalı ölçüm olabilir, 1.0 metreden yakın engel varsa DUR!
            if 0.1 < mesafe < 1.0: 
                engel_var = True
                break
        
        msg_out = Bool()
        msg_out.data = engel_var
        self.publisher_.publish(msg_out)
        
        if engel_var:
            self.get_logger().warn('⚠️ DİKKAT: Önde Engel Var!')

def main(args=None):
    rclpy.init(args=args)
    node = LidarNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()