import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool, Float32
from geometry_msgs.msg import Twist
import json
import time
import math

class BeyinNode(Node):
    def __init__(self):
        super().__init__('beyin_node')
        
        # --- 1. ABONELİKLER (GİRDİLER) ---
        # Göz: YOLO'dan gelen duba verileri
        self.create_subscription(String, '/yolo_verileri', self.yolo_callback, 10)
        # Refleks: Lidar'dan gelen engel uyarısı
        self.create_subscription(Bool, '/acil_durum', self.lidar_callback, 10)
        # Pusula: Robotun burnunun baktığı yön (0-360 derece) - MAVROS'tan gelecek
        self.create_subscription(Float32, '/pusula_heading', self.pusula_callback, 10)
        # Rota: Gitmemiz gereken GN noktasının açısı - GPS Düğümünden gelecek
        self.create_subscription(Float32, '/hedef_bearing', self.hedef_bearing_callback, 10)
        
        # --- 2. YAYINCILAR (ÇIKTI) ---
        # Motorlara giden emir (Hız ve Dönüş)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # --- 3. AYARLAR ---
        self.acil_durum = False
        self.son_yolo_verisi = []
        self.son_veri_zamani = time.time()
        
        # Navigasyon Değişkenleri
        self.mevcut_heading = 0.0 # Pusula (Kuzey: 0)
        self.hedef_bearing = 0.0  # Hedefin yönü
        
        # Görüntü İşleme Ayarları
        self.GORUNTU_MERKEZI = 320 # 640px genişlik için
        self.KANAL_GENISLIGI = 450 # Sanal şerit genişliği (Piksel) - Testte ayarla!
        
        # PID Ayarları (Hassasiyet)
        self.KP_KAMERA = 0.004   # Kamera hatasına tepki hızı
        self.KP_PUSULA = 0.02    # GPS hatasına tepki hızı
        self.MAX_HIZ = 0.4       # m/s
        
        # YOLO ID Tanımları (Eğitimine göre kontrol et!)
        self.ID_PARKUR = 0  # Turuncu Duba
        self.ID_ENGEL  = 1  # Sarı Engel
        self.ID_HEDEF  = 2  # Kırmızı/Yeşil Hedef

        # Karar Döngüsü (10 Hz - Saniyede 10 kere düşünür)
        self.timer = self.create_timer(0.1, self.kontrol_dongusu)
        self.get_logger().info("🧠 HİBRİT BEYİN (Kamera + GPS + Lidar) Hazır!")

    # --- CALLBACK FONKSİYONLARI ---
    def lidar_callback(self, msg):
        self.acil_durum = msg.data

    def yolo_callback(self, msg):
        try:
            self.son_yolo_verisi = json.loads(msg.data)
            self.son_veri_zamani = time.time()
        except:
            pass
            
    def pusula_callback(self, msg):
        self.mevcut_heading = msg.data

    def hedef_bearing_callback(self, msg):
        self.hedef_bearing = msg.data

    # --- YARDIMCI MATEMATİK ---
    def aci_farki_hesapla(self, hedef, mevcut):
        """En kısa dönüş yönünü bulur (-180 sol, +180 sağ gibi)."""
        fark = (hedef - mevcut + 180) % 360 - 180
        return fark * -1 # ROS koordinatına çevir (Sağ dönüş negatif Z olsun)

    # --- ANA MANTIK ---
    def kontrol_dongusu(self):
        cmd = Twist()

        # 1. ACİL DURUM (LİDAR ÖNCELİKLİDİR)
        if self.acil_durum:
            self.get_logger().warn("🛑 ENGEL ÇOK YAKIN! Geri kaçılıyor...")
            cmd.linear.x = -0.2
            cmd.angular.z = 0.0
            self.cmd_vel_pub.publish(cmd)
            return

        # 2. GÖRSEL HEDEF ANALİZİ
        # Kamera bize bir hedef noktası (piksel) veriyor mu?
        kamera_hedef_x = self.kamera_analizi()
        
        donus_hizi = 0.0
        duz_hiz = 0.0

        # DURUM A: KAMERA GÖRÜYOR (ŞERİT TAKİBİ)
        if kamera_hedef_x is not None:
            # Kameraya göre hata (Piksel farkı)
            hata_piksel = kamera_hedef_x - self.GORUNTU_MERKEZI
            
            # Ana sürüş kameraya emanet
            donus_hizi = -1 * (hata_piksel * self.KP_KAMERA)
            duz_hiz = self.MAX_HIZ
            
            # VİRAJ DESTEĞİ (GPS FÜZYONU):
            # Eğer GPS "Çok sert dönmen lazım" diyorsa, kameraya yardım et.
            # Bu, robotun virajı göremeyip düz gitmesini engeller.
            gps_farki = self.aci_farki_hesapla(self.hedef_bearing, self.mevcut_heading)
            
            if abs(gps_farki) > 40: # 40 dereceden keskin viraj var
                self.get_logger().info(f"🔄 VİRAJ DESTEĞİ: GPS {gps_farki:.1f} derece istiyor.")
                # %60 Kamera + %40 GPS karışımı yap
                gps_katkisi = gps_farki * self.KP_PUSULA
                donus_hizi = (donus_hizi * 0.6) + (gps_katkisi * 0.4)

            self.get_logger().info(f"📷 MOD: KAMERA | Hata: {hata_piksel:.1f}")

        # DURUM B: KAMERA KÖR (DUBA YOK / VİRAJ) -> GPS MODU
        else:
            self.get_logger().info("🌍 MOD: GPS (Kör Uçuş) | Rotaya dönülüyor...")
            
            # Sadece GPS açısına bakarak dön
            gps_farki = self.aci_farki_hesapla(self.hedef_bearing, self.mevcut_heading)
            
            donus_hizi = gps_farki * self.KP_PUSULA
            
            # Eğer açı farkı çok büyükse (örn: arkamızda kaldıysa) yavaşla, olduğu yerde dön
            if abs(gps_farki) > 20:
                duz_hiz = 0.1 # Çok yavaşla
            else:
                duz_hiz = 0.3 # Rota tuttu, devam et

        # --- SON LİMİTLEME VE GÖNDERME ---
        # Dönüş hızını sınırla (Motorlar çıldırmasın)
        donus_hizi = max(min(donus_hizi, 0.8), -0.8)
        
        cmd.linear.x = float(duz_hiz)
        cmd.angular.z = float(donus_hizi)
        self.cmd_vel_pub.publish(cmd)

    def kamera_analizi(self):
        """
        Şartnameye uygun şerit ortalama ve engelden kaçış mantığı.
        """
        # 1. Engel Kontrolü (SARI - ID:1)
        for duba in self.son_yolo_verisi:
            if duba['id'] == self.ID_ENGEL:
                # Engel sağdaysa sola, soldaysa sağa kaç
                return self.GORUNTU_MERKEZI - 200 if duba['x'] > 320 else self.GORUNTU_MERKEZI + 200

        # 2. Hedef Kontrolü (KIRMIZI/YEŞİL - ID:2) -> Sadece Parkur 3'te açılmalı
        # (Şimdilik devre dışı bırakıyorum, Parkur 1 ve 2 için şerit takibi önemli)
        
        # 3. Parkur Takibi (TURUNCU - ID:0)
        sol_duvar = [d['x'] for d in self.son_yolo_verisi if d['id'] == self.ID_PARKUR and d['x'] < 320]
        sag_duvar = [d['x'] for d in self.son_yolo_verisi if d['id'] == self.ID_PARKUR and d['x'] >= 320]

        en_yakin_sol = max(sol_duvar) if sol_duvar else None
        en_yakin_sag = min(sag_duvar) if sag_duvar else None
        
        # --- SANAL ŞERİT (GHOST WALL) MANTIĞI ---
        if en_yakin_sol and en_yakin_sag:
            # İki taraf da dolu: Ortala
            return (en_yakin_sol + en_yakin_sag) / 2
        elif en_yakin_sol:
            # Sadece SOL var: Sağ tarafa hayali duvar koy
            hayalet_sag = en_yakin_sol + self.KANAL_GENISLIGI
            return (en_yakin_sol + hayalet_sag) / 2
        elif en_yakin_sag:
            # Sadece SAĞ var: Sol tarafa hayali duvar koy
            hayalet_sol = en_yakin_sag - self.KANAL_GENISLIGI
            return (hayalet_sol + en_yakin_sag) / 2
        else:
            # Hiçbir şey yok -> None döndür ki GPS devreye girsin
            return None

def main(args=None):
    rclpy.init(args=args)
    node = BeyinNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()