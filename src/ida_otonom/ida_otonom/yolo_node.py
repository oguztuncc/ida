import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from ultralytics import YOLO
import cv2
import json
import time
import os
from datetime import datetime

class YoloNode(Node):
    def __init__(self):
        super().__init__('yolo_node')
        
        # Yayıncı
        self.publisher_ = self.create_publisher(String, '/yolo_verileri', 10)
        
        # Model Yolu (Dosya yoksa hata vermemesi için kontrol)
        model_path = "/home/jetson/best.pt" # Burayı kendi yolunla güncelle
        if not os.path.exists(model_path):
            self.get_logger().error(f"Model bulunamadı: {model_path}")
        self.model = YOLO(model_path) 
        
        # Kamera Başlatma
        self.cap = cv2.VideoCapture(0)
        
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        
        # --- ŞARTNAME GEREĞİ VİDEO KAYDI ---
        # Dosya ismi: Tarih_Saat.mp4 (Örn: yolo_kayit_20260920_1430.mp4)
        zaman_damgasi = datetime.now().strftime("%Y%m%d_%H%M%S")
        dosya_adi = f"yolo_kayit_{zaman_damgasi}.mp4"
        
        # Video ayarları (Genişlik, Yükseklik, FPS)
        # Kameradan gerçek değerleri alıyoruz
        w = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        h = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        fps = 10 # Timer ile uyumlu olması için düşük tuttuk
        
        # Video Writer (MP4 formatı için 'mp4v' codec)
        self.video_writer = cv2.VideoWriter(dosya_adi, cv2.VideoWriter_fourcc(*'mp4v'), fps, (w, h))
        self.get_logger().info(f"Video kaydı başlatıldı: {dosya_adi}")

        # Timer (10 Hz)
        self.timer = self.create_timer(0.1, self.timer_callback)

    def timer_callback(self):
        ret, frame = self.cap.read()
        if not ret:
            return

        height, width, _ = frame.shape
        merkez_x=int(width/2)
        results=self.model(frame,verbose=False)
        annotated_frame=results[0].plot()

        cv2.line(annotated_frame, (merkez_x, 0), (merkez_x, height), (255, 0, 0), 2)
        

        algilanan_nesneler = []

        # Tespitleri Listele
        for box in results[0].boxes:
            x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
            duba_merkezi_x = float((x1 + x2) / 2)
            sinif_id = int(box.cls[0])
            guven = float(box.conf[0])

            nesne_verisi = {
                "id": sinif_id,
                "x": duba_merkezi_x,
                "guven": guven
            }
            algilanan_nesneler.append(nesne_verisi)

            # Ekstra görsellik (Merkez noktası)
            cv2.circle(annotated_frame, (int(duba_merkezi_x), int((y1+y2)/2)), 5, (0, 255, 255), -1)

        # JSON Yayınla
        msg = String()
        msg.data = json.dumps(algilanan_nesneler)
        self.publisher_.publish(msg)
        
        # --- VİDEOYU KAYDET ---
        self.video_writer.write(annotated_frame)
        
        # Ekranda Göster (Sadece monitör varsa çalışır, yoksa pas geçer)
        try:
            cv2.imshow("YOLO Goru", annotated_frame)
            cv2.waitKey(1)
        except Exception:
            pass # Monitör yoksa hata verme, devam et

    def destroy_node(self):
        # Kapatırken videoyu serbest bırak ve kaydet
        self.cap.release()
        self.video_writer.release()
        cv2.destroyAllWindows()
        self.get_logger().info("Video kaydı tamamlandı ve kaydedildi.")
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = YoloNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()