import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float32
import math

class GPSNode(Node):
    def __init__(self):
        super().__init__('gps_node')
        
        # --- GİRDİLER (Mavros'tan gelen) ---
        # Cube Orange'ın ölçtüğü anlık konum (Enlem/Boylam)
        self.create_subscription(NavSatFix, '/mavros/global_position/global', self.gps_callback, 10)
        
        # Cube Orange'ın ölçtüğü anlık kafa yönü (Pusula)
        self.create_subscription(Float32, '/mavros/global_position/compass_hdg', self.heading_callback, 10)
        
        # --- ÇIKTILAR (Beyin'e giden) ---
        # "Hedef şu açıda (derece)" diyen mesaj
        self.bearing_pub = self.create_publisher(Float32, '/hedef_bearing', 10)
        # "Sen şu an bu açıya bakıyorsun" diyen mesaj
        self.heading_pub = self.create_publisher(Float32, '/pusula_heading', 10)
        
        # --- HEDEF LİSTESİ (Yarışma Günü USB'den Yüklenecek) ---
        # Örnek: İzmir Körfezi veya Okulun bahçesi
        # Buradaki sayıları kendi deneme yapacağın yerin koordinatlarıyla değiştir!
        self.hedefler = [
            {'lat': 40.231234, 'lon': 28.123123}, # GN1
            {'lat': 40.231300, 'lon': 28.123200}, # GN2
            {'lat': 40.231400, 'lon': 28.123300}  # GN3
        ]
        self.aktif_hedef_index = 0
        
        self.mevcut_lat = 0.0
        self.mevcut_lon = 0.0

    def heading_callback(self, msg):
        # Pusula verisini olduğu gibi Beyin düğümüne aktar
        self.heading_pub.publish(msg)

    def gps_callback(self, msg):
        self.mevcut_lat = msg.latitude
        self.mevcut_lon = msg.longitude
        
        # Her yeni konum geldiğinde rotayı yeniden hesapla
        self.rota_hesapla()

    def rota_hesapla(self):
        if self.aktif_hedef_index >= len(self.hedefler):
            self.get_logger().info("🏁 TÜM GÖREVLER TAMAMLANDI!")
            return

        hedef = self.hedefler[self.aktif_hedef_index]
        
        # --- 1. MESAFE HESABI (Metre) ---
        R = 6371000 # Dünya yarıçapı
        dLat = math.radians(hedef['lat'] - self.mevcut_lat)
        dLon = math.radians(hedef['lon'] - self.mevcut_lon)
        lat1 = math.radians(self.mevcut_lat)
        lat2 = math.radians(hedef['lat'])
        
        a = math.sin(dLat/2)**2 + math.cos(lat1)*math.cos(lat2)*math.sin(dLon/2)**2
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))
        mesafe = R * c
        
        # --- 2. AÇI HESABI (Bearing - Derece) ---
        y = math.sin(dLon) * math.cos(lat2)
        x = math.cos(lat1) * math.sin(lat2) - math.sin(lat1) * math.cos(lat2) * math.cos(dLon)
        bearing = (math.degrees(math.atan2(y, x)) + 360) % 360
        
        # --- 3. BEYNE GÖNDER ---
        msg = Float32()
        msg.data = float(bearing)
        self.bearing_pub.publish(msg)
        
        # Bilgi Mesajı
        self.get_logger().info(f"Hedef: GN{self.aktif_hedef_index+1} | Mesafe: {mesafe:.1f}m | Açı: {bearing:.0f}")

        # --- 4. HEDEF DEĞİŞTİRME ---
        # Hedefe 3 metreden fazla yaklaştıysak sıradakine geç
        if mesafe < 3.0:
            self.get_logger().info(f"✅ GN{self.aktif_hedef_index+1} VARILDI! Sıradakine geçiliyor.")
            self.aktif_hedef_index += 1

def main(args=None):
    rclpy.init(args=args)
    node = GPSNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()