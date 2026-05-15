# TEKNOFEST İDA Otonomi Yazılımı - Detaylı Kod Analizi Raporu

**Tarih:** 2026-05-12  
**Sürüm:** 1.0  
**Analiz Eden:** AI Assistant  
**Proje:** TEKNOFEST İnsansız Deniz Aracı (İDA) Otonomi Yazılımı

---

## Yönetici Özeti

Bu rapor, TEKNOFEST İDA projesi için geliştirilen ROS 2 Humble tabanlı otonomi yazılımının detaylı kod analizini sunmaktadır. Proje, üç parkurdan oluşan bir yarışma senaryosu için tasarlanmıştır: Parkur-1 (GPS waypoint takibi), Parkur-2 (engelden kaçınma ve duba takibi) ve Parkur-3 (İHA hedef renk takibi).

### Temel Bulgular

**Güçlü Yönler:**
- Güvenlik-öncelikli tasarım (varsayılan olarak devre dışı motor kontrolü, latch kill mekanizması)
- Modüler mimari (Parkur-1/2/3 ayrımı)
- Kapsamlı dokümantasyon (README ve PROJE_KOD_RAPORU)
- Merkezi config yönetimi
- Graceful degradation (eksik modüllerde sistem çökmez)

**Kritik Riskler:**
- YKI UDP komutlarında authentication yok (herhangi biri araç kontrolünü ele geçirebilir)
- Depth-Color senkronizasyonu yok (yanlış mesafe ölçümü, çarpışma riski)
- GPS ve Heading verileri eşzamansız (yanlış navigasyon)
- Test coverage sıfır (regresyon riski)
- Tek contributor, tek branch (kod review yok)

**Acil Eylem Gerektiren Konular:**
1. YKI güvenliği (authentication/token ekleme)
2. Sensör senkronizasyonu (depth-color, GPS-heading)
3. Test altyapısı (unit ve integration test)
4. Kod review süreci (en az 1 reviewer)
5. Custom ROS 2 message tanımlamaları (JSON String yerine)

---

## 1. Git History ve Proje Durumu

| Metrik | Değer |
|--------|-------|
| Toplam Commit | 6 |
| Contributor | 1 (oguztuncc) |
| Aktif Branch | main (tek branch) |
| Son Commit | 2026-05-11 |
| Proje Başlangıcı | 2026-02-17 |

**Commit Geçmişi:**
- `v1` (2026-02-17): İlk versiyon
- `first commit` (2026-04-29): Temel yapı
- `Implement Parkur-1 simulation stack` (2026-04-30): Parkur-1 simülasyonu
- `Add TEKNOFEST IDA autonomy stack` (2026-05-10): Ana stack
- `Add project code architecture report` (2026-05-10): Kod raporu
- `Tüm proje dosyaları güncellendi` (2026-05-11): Parkur-2 node'ları eklendi

**Değerlendirme:** Tek contributor, tek branch, çok az commit. Bu, kod review sürecinin olmadığını, pair programming yapılmadığını ve versiyon kontrol disiplininin zayıf olduğunu gösterir.

---

## 2. Mimari Genel Bakış

### Node'lar ve İlişkileri

```
PARKUR-1 (GPS Waypoint Takibi):
┌─────────────────┐     ┌─────────────────┐     ┌─────────────────┐
│ mission_manager │────▶│  gps_guidance   │────▶│   controller    │
│     _node       │     │     _node       │     │     _node       │
└─────────────────┘     └─────────────────┘     └────────┬────────┘
         ▲                                               │
         │                                               ▼
         │                                        ┌─────────────┐
         │                                        │ safety_node │
         │                                        └──────┬──────┘
         │                                               │
         │                                        ┌──────┴──────┐
         │                                        │mavros_bridge│
         │                                        │    _node    │
         │                                        └──────┬──────┘
         │                                               ▼
         │                                          [Pixhawk]
         │
         └──────────────────────────────────────────────┘
                          /mission/started, /mission/completed

PARKUR-2 (Engelden Kaçınma + Duba Takibi):
┌─────────────────┐     ┌─────────────────┐     ┌─────────────────┐
│ buoy_detector   │────▶│course_memory    │────▶│semantic_buoy    │
│     _node       │     │    _node        │     │ classifier_node │
└─────────────────┘     └─────────────────┘     └────────┬────────┘
         ▲                                               │
         │                                               ▼
┌─────────────────┐                            ┌─────────────────┐
│ lidar_processor │                            │ corridor_tracker│
│     _node       │                            │     _node       │
└────────┬────────┘                            └────────┬────────┘
         │                                               │
         └──────────────────────┬────────────────────────┘
                                ▼
                         ┌─────────────────┐
                         │ parkur2_planner │
                         │     _node       │
                         └────────┬────────┘
                                  ▼
                           ┌─────────────┐
                           │ controller  │
                           │   _node     │
                           │(planner mode)│
                           └─────────────┘

GÜVENLİK ve ALTYAPI:
┌─────────────┐  ┌─────────────┐  ┌─────────────┐  ┌─────────────┐
│  rc_kill    │  │ power_relay │  │   logger    │  │    YKI      │
│   _node     │  │   _node     │  │   _node     │  │  _bridge    │
└──────┬──────┘  └─────────────┘  └─────────────┘  └─────────────┘
       │
       ▼
┌─────────────┐
│ safety_node │◄── /safety/kill
└─────────────┘

SİMÜLASYON:
┌─────────────┐
│  sim_gps    │
│   _node     │
└─────────────┘
```

---

## 3. Her Node'un Detaylı Analizi

### 3.1 `mission_manager_node.py` (134 satır)

**Görev:** JSON mission dosyasını okur, waypoint yönetimi ve görev durumunu kontrol eder.

**Girdi Topicleri:**
- `/guidance/advance_waypoint` (Bool) - Bir sonraki waypoint'e geçiş sinyali
- `/mission/start` (Bool) - Görev başlatma komutu

**Çıktı Topicleri:**
- `/mission/active_waypoint` (Int32)
- `/mission/status` (String - JSON)
- `/mission/started` (Bool)
- `/mission/completed` (Bool)
- `/mission/waypoints` (String - JSON)

**Riskler:**
- **CRITICAL:** `advance_cb` callback'i `Bool` mesajı aldığında `active_waypoint_index` artırır. Eğer mesaj spam edilirse (örneğin bir bug veya hatalı publisher), waypoint'ler hızla atlanabilir. Debouncing yok.
- `load_mission()` hem `__init__`'te hem de potansiyel olarak dışarıdan çağrılabilir - thread-safe değil.
- `waypoints` listesi doğrudan publish ediliyor, deep copy yok.

**İyileştirme Önerileri:**
- `advance_cb`'e debounce mekanizması ekle (son advance'den sonra minimum süre bekle)
- Mission dosyası şema validasyonu ekle
- Waypoint koordinatları için bounds check ekle (geçersiz lat/lon değerlerini tespit et)

---

### 3.2 `gps_guidance_node.py` (171 satır)

**Görev:** Mevcut GPS konumundan aktif waypoint'e bearing ve mesafe hesaplar.

**Girdi Topicleri:**
- `/mavros/global_position/global` (NavSatFix)
- `/mavros/global_position/compass_hdg` (Float32)
- `/mission/active_waypoint` (Int32)
- `/mission/waypoints` (String)
- `/mission/started` (Bool)

**Çıktı Topicleri:**
- `/guidance/target_bearing_deg` (Float32)
- `/guidance/target_distance_m` (Float32)
- `/guidance/advance_waypoint` (Bool)
- `/guidance/status` (String)

**Riskler:**
- **HIGH:** `gps_cb` ve `heading_cb` ayrı callback'ler. GPS ve heading verisi aynı anda güncellenmiyor - `loop()` çalıştığında eski heading + yeni GPS kombinasyonu kullanılabilir. Race condition potansiyeli var.
- `arrival_radius_m` parametresi sadece init'te okunuyor, runtime'da değiştirilemez.
- GPS verisi yoksa (`None`) sessizce return ediyor - dışarıya "GPS kayıp" durumu publish edilmiyor.

**İyileştirme Önerileri:**
- GPS ve heading verilerini senkronize et (timestamp karşılaştırması veya timeout)
- GPS kayıp durumunu ayrı bir topic'te publish et
- `arrival_radius_m`'yi dinamik parametre yap

---

### 3.3 `controller_node.py` (229 satır)

**Görev:** Heading hatasına göre hız ve dönüş komutu üretir.

**Girdi Topicleri:**
- `/mavros/global_position/compass_hdg` (Float32)
- `/guidance/target_bearing_deg` (Float32)
- `/guidance/target_distance_m` (Float32)
- `/planner/safe_bearing_deg` (Float32)
- `/planner/speed_limit_mps` (Float32)
- `/perception/corridor_hint` (String)
- `/mission/completed` (Bool)
- `/mission/started` (Bool)

**Çıktı Topicleri:**
- `/control/cmd_vel` (Twist)
- `/control/setpoints` (String - JSON)

**Riskler:**
- **CRITICAL:** `planner_bearing_ts` ve `planner_speed_ts` `float` olarak tutuluyor. `get_clock().now().nanoseconds / 1e9` ile güncelleniyor. Bu değişkenlere farklı callback'lerden erişim var - ROS 2 single-threaded executor kullanıyorsa sorun olmaz ama multi-threaded'a geçilirse race condition oluşur.
- **HIGH:** `vision_heading_bias` JSON parse hatasında 0.0'a resetleniyor - bu geçici bir hata durumunda ani bir yön değişikliğine neden olabilir.
- **MEDIUM:** Hız profili sabit kodlanmış (0.45, 0.28, 0.15, 0.05). Parametrik değil.
- `target_distance` subscribe ediliyor ama hiç kullanılmıyor.

**İyileştirme Önerileri:**
- Thread-safe veri yapıları kullan (ROS 2 callback grupları veya Locks)
- Hız profilini parametrik yap
- `target_distance`'ı kullan veya kaldır
- `vision_heading_bias`'ı low-pass filter ile yumuşat

---

### 3.4 `safety_node.py` (77 satır)

**Görev:** Kill durumunda motor komutlarını keser.

**Girdi Topicleri:**
- `/safety/kill` (Bool)
- `/safety/kill_reset` (Bool)
- `/control/cmd_vel` (Twist)

**Çıktı Topicleri:**
- `/control/cmd_vel_safe` (Twist)
- `/safety/status` (String)

**Riskler:**
- **MEDIUM:** `last_cmd` Twist mesajı her geldiğinde güncelleniyor. Eğer kill aktifken yeni bir komut gelirse, kill resetlendiğinde eski komut aniden uygulanabilir - bu ani bir hızlanmaya neden olabilir.
- **LOW:** `latch_kill=True` varsayılan - bu güvenli bir davranış.

**İyileştirme Önerileri:**
- Kill aktifken gelen komutları queue'ya alma, sadece sıfır komut kabul et
- Kill geçişlerinde komut geçmişini temizle

---

### 3.5 `rc_kill_node.py` (98 satır)

**Görev:** RC kanalından kill sinyali üretir.

**Girdi Topicleri:**
- `/mavros/rc/in` (RCIn)

**Çıktı Topicleri:**
- `/safety/kill` (Bool)
- `/safety/rc_kill_status` (String)

**Riskler:**
- **HIGH:** `time.monotonic()` kullanılıyor ama bu sistem saatine bağlı. ROS 2 clock ile senkronize değil.
- **MEDIUM:** `failsafe_on_missing=True` varsayılan - RC sinyali kaybolursa kill aktif. Bu güvenli ama yanlış bir anten konumlandırması nedeniyle sürekli kill olabilir.
- `RCIn` import hatası durumunda node çalışmaya devam ediyor ama hiçbir şey yapmıyor - sessizce fail oluyor.

**İyileştirme Önerileri:**
- ROS 2 clock kullan
- RC sinyal kalitesini de publish et (SNR, packet loss)
- RCIn import hatasında node'u tamamen kapat veya açıkça uyarı ver

---

### 3.6 `mavros_bridge_node.py` (192 satır)

**Görev:** Güvenli komutları Pixhawk/MAVROS'a iletir.

**Girdi Topicleri:**
- `/control/cmd_vel_safe` (Twist)
- `/safety/kill` (Bool)
- `/mission/completed` (Bool)

**Çıktı Topicleri:**
- `/mavros/setpoint_velocity/cmd_vel_unstamped` (Twist)
- `/mavros/manual_control/send` (ManualControl)
- `/mavros_bridge/status` (String)

**Riskler:**
- **CRITICAL:** `enabled` varsayılan `False` - bu güvenli bir tasarım kararı. Ancak runtime'da `enabled:=true` yapıldığında hiçbir ek doğrulama yok.
- **HIGH:** `command_timeout_s` mekanizması var ama `last_cmd_time` sadece `cmd_cb`'de güncelleniyor. Eğer publisher çökerse timeout çalışır ama bu arada son komut tekrarlanabilir.
- **MEDIUM:** `publish_manual_control` fonksiyonunda axis mapping TODO olarak bırakılmış - bu gerçek donanımda tehlikeli olabilir.

**İyileştirme Önerileri:**
- `enabled` değişikliğinde ek doğrulama mekanizması ekle (örneğin bir "arm" prosedürü)
- Axis mapping'i tamamla ve test et
- Komut timeout'unda gradual stop (ani duruş yerine yavaş yavaş durma)

---

### 3.7 `yki_bridge_node.py` (209 satır)

**Görev:** YKI ile UDP haberleşme köprüsü.

**Girdi Topicleri:**
- `/mavros/global_position/global` (NavSatFix)
- `/mission/active_waypoint` (Int32)
- `/mission/completed` (Bool)
- `/mission/started` (Bool)
- `/mission/status` (String)
- `/guidance/status` (String)
- `/mission/waypoints` (String)

**Çıktı Topicleri:**
- `/mission/start` (Bool)
- `/safety/kill` (Bool)
- `/safety/kill_reset` (Bool)
- `/mission/target_color` (String)

**Riskler:**
- **CRITICAL:** UDP socket blocking olmayan modda ama `command_loop`'ta `while True` ile döngü var. Her timer callback'inde tüm buffer'daki mesajları işliyor - bu, çok sayıda mesaj gelirse diğer callback'leri bloklayabilir.
- **HIGH:** `handle_command`'da `kill` komutu her zaman kabul ediliyor ama `start_mission` sadece `mission_started=False` ise. Ancak `kill` komutunda herhangi bir authentication yok.
- **MEDIUM:** UDP payload JSON - injection riski var (doğrulama yetersiz).
- `sock.sendto` hatası durumunda exception yakalanmıyor.

**İyileştirme Önerileri:**
- UDP komutlarına authentication/token ekle
- `sendto` hatalarını yakala ve logla
- Buffer limiti koy (max N mesaj işle)
- JSON schema validasyonu ekle

---

### 3.8 `logger_node.py` (170 satır)

**Görev:** Telemetri verisini CSV olarak kaydeder.

**Girdi Topicleri:**
- `/mavros/global_position/global` (NavSatFix)
- `/mavros/global_position/compass_hdg` (Float32)
- `/mavros/imu/data` (Imu)
- `/mavros/local_position/velocity_body` (TwistStamped)
- `/guidance/target_bearing_deg` (Float32)
- `/guidance/target_distance_m` (Float32)
- `/control/setpoints` (String)

**Çıktı:** CSV dosyası (disk)

**Riskler:**
- **HIGH:** `file.flush()` her `loop()` çağrısında çalışıyor (1 Hz). Bu I/O-bound operasyon disk dolarsa veya yavaşsa bloklayabilir.
- **MEDIUM:** CSV dosyası açıkken node çökerse veri kaybı olabilir.
- **LOW:** `roll` ve `pitch` hesaplaması matematiksel olarak doğru ama `gimbal lock` durumunda stabil olmayabilir.

**İyileştirme Önerileri:**
- Buffer kullan ve periyodik flush et (örneğin her 10 satırda bir)
- CSV rotation ekle (dosya boyutu limiti)
- Quaternion'dan Euler'a dönüşüm için daha robust bir kütüphane kullan

---

### 3.9 `sim_gps_node.py` (93 satır)

**Görev:** Parkur-1 simülasyonu için sahte GPS ve heading üretir.

**Girdi Topicleri:**
- `/control/cmd_vel` (Twist)

**Çıktı Topicleri:**
- `/mavros/global_position/global` (NavSatFix)
- `/mavros/global_position/compass_hdg` (Float32)

**Riskler:**
- **LOW:** Basit kinematik model - gerçek deniz dinamiğini (dalga, akıntı, rüzgar) modellemiyor.
- **LOW:** `lon_scale`'de `max(..., 1e-6)` ile bölme hatasından kaçınılıyor ama kutup bölgelerinde hala sorunlu.

**İyileştirme Önerileri:**
- Gürültü modeli ekle (GPS drift, heading jitter)
- Deniz dinamiği modeli ekle (opsiyonel)

---

### 3.10 `lidar_processor_node.py` (95 satır)

**Görev:** Lidar scan verisinden ön/sol/sağ engel özeti üretir.

**Girdi Topicleri:**
- `/scan` (LaserScan)

**Çıktı Topicleri:**
- `/perception/lidar_summary` (String - JSON)

**Riskler:**
- **MEDIUM:** `best_free_angle` hesaplaması çok basit - sadece 3 sektör (front, left, right). Dar alanlarda veya karmaşık ortamlarda yetersiz kalabilir.
- **LOW:** `999.0` magic number olarak kullanılıyor.

**İyileştirme Önerileri:**
- Daha fazla sektör veya dinamik bölgeleme ekle
- `best_free_angle`'ı smoothing ile yumuşat (ani değişimleri önle)

---

### 3.11 `local_costmap_node.py` (141 satır)

**Görev:** Lidar verisinden lokal occupancy grid üretir.

**Girdi Topicleri:**
- `/scan` (LaserScan)

**Çıktı Topicleri:**
- `/local_costmap` (OccupancyGrid)

**Riskler:**
- **HIGH:** Her `loop()` çağrısında `OccupancyGrid` mesajı sıfırdan oluşturuluyor. `width * height` kadar bellek ayrılıyor (48x48 = 2304 hücre - şu ankı parametrelerle küçük ama büyütülürse problem olur).
- **MEDIUM:** CSV dosyası her döngüde `flush` ediliyor.
- **LOW:** `build_occupied_cells()` her çağrıda `sorted(occupied)` yapıyor - bu gereksiz.

**İyileştirme Önerileri:**
- OccupancyGrid'i yeniden kullan (sadece data kısmını güncelle)
- CSV flush sıklığını azalt
- `sorted`'ı kaldır (gereksiz hesaplama)

---

### 3.12 `buoy_detector_node.py` (265 satır)

**Görev:** Kamera görüntüsünden YOLO ile duba tespiti yapar.

**Girdi Topicleri:**
- `/camera/camera/color/image_raw` (Image)
- `/camera/camera/aligned_depth_to_color/image_raw` (Image)
- `/camera/camera/color/camera_info` (CameraInfo)

**Çıktı Topicleri:**
- `/perception/buoy_detections` (String - JSON)

**Riskler:**
- **CRITICAL:** `latest_depth` her `depth_cb`'de güncelleniyor ama `color_cb`'de kullanılıyor. Depth ve color frame'lerinin senkronizasyonu yok - eski depth + yeni color kombinasyonu olabilir.
- **HIGH:** `YOLO` ve `CvBridge` import hataları sessizce yakalanıyor. Node çalışmaya devam ediyor ama hiçbir detection üretmiyor.
- **MEDIUM:** `depth_sample_radius_px` ROI hesaplamasında `astype(np.float32)` her çağrıda yapılıyor.
- **MEDIUM:** YOLO `predict` her frame'de çağrılıyor - bu CPU/GPU yoğun bir operasyon.

**İyileştirme Önerileri:**
- Depth ve color frame'lerini senkronize et (message_filters veya timestamp matching)
- YOLO inference'ı ayrı bir thread/process'te çalıştır
- ROI hesaplamalarını optimize et
- Model quantization/TensorRT kullan

---

### 3.13 `course_memory_node.py` (150 satır)

**Görev:** İlk waypoint'lerde course buoy renk profilini öğrenir.

**Girdi Topicleri:**
- `/perception/buoy_detections` (String)
- `/mission/active_waypoint` (Int32)

**Çıktı Topicleri:**
- `/perception/course_memory` (String - JSON)

**Riskler:**
- **MEDIUM:** `samples` listesi sınırsız büyüyebilir. Bellek leak potansiyeli var.
- **LOW:** Hue hesaplaması dairesel istatistik kullanıyor (doğru) ama `max_hue_std_deg` threshold'u çok geniş (35 derece).

**İyileştirme Önerileri:**
- `samples` listesine maximum size limiti koy
- Learning phase bittikten sonra `samples`'ı temizle

---

### 3.14 `semantic_buoy_classifier_node.py` (155 satır)

**Görev:** Duba tespitlerini semantik olarak sınıflandırır (course, obstacle, target).

**Girdi Topicleri:**
- `/perception/course_memory` (String)
- `/perception/buoy_detections` (String)

**Çıktı Topicleri:**
- `/perception/semantic_buoys` (String - JSON)

**Riskler:**
- **MEDIUM:** `course_profile` `None` olabilir - bu durumda sınıflandırma "unknown" olarak kalır. Ancak bu, hiçbir course boundary olmadan ilerlemeye çalışmak anlamına gelir.
- **LOW:** `memory_cb`'de exception durumunda `course_profile = None` yapılıyor - bu öğrenilmiş profilin kaybolmasına neden olabilir.

**İyileştirme Önerileri:**
- `course_profile`'ı kalıcı olarak sakla (disk veya ROS parameter)
- Sınıflandırma confidence'ını publish et

---

### 3.15 `corridor_tracker_node.py` (183 satır)

**Görev:** Semantic dubalardan corridor center ve gate bilgisi çıkarır.

**Girdi Topicleri:**
- `/perception/semantic_buoys` (String)

**Çıktı Topicleri:**
- `/planner/corridor` (String - JSON)

**Riskler:**
- **HIGH:** `_nearest_gate` fonksiyonu O(N*M) karmaşıklığında. Çok sayıda duba varsa performans problemi.
- **MEDIUM:** `center_bearing_deg` hesaplamasında `math.atan2` kullanılıyor ama sonuç dereceye çevriliyor - bu doğru. Ancak `lookahead_m` sabit ve parametrik değil.
- **LOW:** `left` ve `right` ayrımı sadece `left_m`'nin işaretine göre yapılıyor. Bu, aracın yönüne göre değil, kameraya göre yapılıyor.

**İyileştirme Önerileri:**
- Gate detection'ı optimize et (spatial indexing veya R-tree)
- `lookahead_m`'yi dinamik yap (hıza bağlı)
- Camera frame'den vehicle frame'e dönüşüm ekle

---

### 3.16 `parkur2_planner_node.py` (256 satır)

**Görev:** Lidar, corridor ve semantic verilerini birleştirip güvenli bearing ve hız limiti üretir.

**Girdi Topicleri:**
- `/mavros/global_position/compass_hdg` (Float32)
- `/guidance/target_bearing_deg` (Float32)
- `/perception/lidar_summary` (String)
- `/planner/corridor` (String)
- `/perception/semantic_buoys` (String)

**Çıktı Topicleri:**
- `/planner/safe_bearing_deg` (Float32)
- `/planner/speed_limit_mps` (Float32)
- `/planner/status` (String)

**Riskler:**
- **CRITICAL:** `now_ts()` fonksiyonu `time.time()` kullanıyor - bu sistem saatine bağlı. ROS 2 clock ile senkronize değil. Simülasyonda veya clock skew durumunda yanlış timeout kararları verilebilir.
- **HIGH:** `lidar_state == "avoid" and mode == "CORRIDOR_TRACK"` durumunda `speed_limit` düşürülüyor ama `mode` değişmiyor. Bu, planner'ın durum makinesinde tutarsızlığa yol açabilir.
- **HIGH:** `semantic_front_obstacle` her buoy için döngü yapıyor - bu O(N) ama her timer callback'inde çalışıyor.
- **MEDIUM:** `sensor_timeout_s` sadece lidar ve corridor için kontrol ediliyor, semantic için de kontrol edilmeli.

**İyileştirme Önerileri:**
- ROS 2 clock kullan
- State machine'i açıkça tanımla (enum kullan)
- Semantic timeout kontrolü ekle
- Planner kararlarını low-pass filter ile yumuşat

---

### 3.17 `perception_node.py` (142 satır)

**Görev:** Basit kamera görüntü işleme (Hough Lines) ve video kaydı.

**Girdi:** Kamera (OpenCV VideoCapture)

**Çıktı Topicleri:**
- `/perception/corridor_hint` (String - JSON)

**Riskler:**
- **HIGH:** `cv2.VideoCapture` her `loop()`'ta `read()` çağrılıyor. Kamera bağlantısı koparsa sessizce fail olur.
- **HIGH:** `cv2.VideoWriter` her frame için disk I/O yapıyor. Bu, yavaş bir disk veya yüksek çözünürlükte bloklayabilir.
- **MEDIUM:** Hough Lines parametreleri sabit kodlanmış - farklı aydınlatma koşullarında çalışmayabilir.

**İyileştirme Önerileri:**
- Kamera bağlantı kopma durumunu tespit et ve yeniden bağlan
- Video writer'ı ayrı thread'te çalıştır
- Hough Lines parametrelerini adaptif yap

---

### 3.18 `power_relay_node.py` (112 satır)

**Görev:** Jetson GPIO üzerinden motor güç rölesini kontrol eder.

**Girdi Topicleri:**
- `/safety/kill` (Bool)
- `/safety/motor_power_enable` (Bool)

**Çıktı Topicleri:**
- `/safety/power_relay_status` (String)

**Riskler:**
- **CRITICAL:** `GPIO.cleanup(self.gpio_pin)` `destroy_node`'da çağrılıyor. Eğer node anormal şekilde sonlanırsa (SIGKILL), GPIO cleanup yapılmayabilir ve röle açık kalabilir.
- **HIGH:** `Jetson.GPIO` import hatası durumunda `enabled = False` yapılıyor - bu güvenli bir davranış.

**İyileştirme Önerileri:**
- Signal handler ekle (SIGTERM, SIGINT)
- Watchdog mekanizması ekle (heartbeat timeout'unda röleyi kapat)

---

### 3.19 `sensor_fusion_node.py` (95 satır) - ROOT'DA

**Görev:** Kamera ve lidar verilerini birleştirir.

**Not:** Bu dosya `src/ida_otonom/sensor_fusion_node.py` olarak root'ta yer alıyor - paket yapısına uymuyor. `setup.py`'de entry point olarak tanımlanmamış.

**Girdi Topicleri:**
- `/perception/detections` (String)
- `/perception/lidar_summary` (String)

**Çıktı Topicleri:**
- `/fusion/gate_candidate` (String)
- `/fusion/world_state` (String)

**Riskler:**
- **CRITICAL:** Bu node launch dosyalarında çağrılmıyor ve `setup.py`'de tanımlı değil. Kullanılmayan ölü kod.
- **HIGH:** `json.loads` hatası durumunda exception yakalanmıyor.
- `detections` formatı `buoy_detector_node`'un çıktısından farklı (`cx` alanı bekleniyor ama `buoy_detector_node` `center_px` kullanıyor).

**İyileştirme Önerileri:**
- Ya tamamen kaldır ya da düzgün entegre et
- JSON parse hatalarını yakala
- Topic formatlarını senkronize et

---

## 4. Kod Kalitesi Analizi

### 4.1 Hata Yönetimi (Try-Except)

| Node | Try-Except Sayısı | Kalite |
|------|-------------------|--------|
| common.py | 2 | Orta - Genel exception yakalama |
| mission_manager_node.py | 1 | Zayıf - `Exception` genel |
| gps_guidance_node.py | 2 | Zayıf - `Exception` genel |
| controller_node.py | 1 | Zayıf - `Exception` genel |
| buoy_detector_node.py | 5 | İyi - Spesifik hatalar |
| parkur2_planner_node.py | 3 | Zayıf - `Exception` genel |
| corridor_tracker_node.py | 1 | Zayıf - `Exception` genel |
| course_memory_node.py | 1 | Zayıf - `Exception` genel |
| semantic_buoy_classifier_node.py | 1 | Zayıf - `Exception` genel |
| local_costmap_node.py | 2 | İyi - `finally` kullanımı |
| logger_node.py | 2 | İyi - `finally` kullanımı |
| perception_node.py | 2 | İyi - `finally` kullanımı |
| power_relay_node.py | 2 | İyi - `finally` kullanımı |
| mavros_bridge_node.py | 1 | Orta |
| rc_kill_node.py | 1 | Orta |
| yki_bridge_node.py | 1 | Orta |

**Genel Değerlendirme:** Çoğu node'da `except Exception:` genel yakalama kullanılıyor. Bu, spesifik hataları maskeleyebilir ve debugging'i zorlaştırabilir.

### 4.2 Tip Annotasyonları

| Node | Tip Annotasyonu | Kalite |
|------|----------------|--------|
| common.py | Kısmi | Fonksiyon return type'ları var |
| mission_manager_node.py | Yok | Sadece `__init__` -> None |
| gps_guidance_node.py | Yok | Sadece `__init__` -> None |
| controller_node.py | Kısmi | Callback parametreleri |
| buoy_detector_node.py | İyi | `List`, `Optional`, `Tuple` kullanımı |
| parkur2_planner_node.py | Kısmi | Callback parametreleri |
| corridor_tracker_node.py | İyi | `List`, `Optional` kullanımı |
| course_memory_node.py | İyi | `Optional`, `Tuple` kullanımı |
| semantic_buoy_classifier_node.py | Kısmi | Callback parametreleri |
| lidar_processor_node.py | İyi | `Iterable`, `List`, `Tuple` kullanımı |
| local_costmap_node.py | Kısmi | Callback parametreleri |
| logger_node.py | Kısmi | Callback parametreleri |
| perception_node.py | Kısmi | Callback parametreleri |
| power_relay_node.py | Kısmi | Callback parametreleri |
| mavros_bridge_node.py | Kısmi | Callback parametreleri |
| rc_kill_node.py | Kısmi | Callback parametreleri |
| yki_bridge_node.py | İyi | `Dict`, `Any` kullanımı |

### 4.3 Kod Tekrarı

**Tespit Edilen Tekrarlar:**
1. **JSON parse/deserialize:** Her node'da `json.loads`/`json.dumps` tekrarlanıyor. `common.py`'de `from_json`/`to_json` var ama tutarlı kullanılmıyor.
2. **Timestamp alma:** `now_ts()` `common.py`'de var ama bazı node'lar `time.time()` doğrudan kullanıyor.
3. **Parameter declaration pattern:** Her node'da benzer `declare_parameter`/`get_parameter` kalıbı tekrarlanıyor.
4. **Destroy pattern:** `try/finally` ile `super().destroy_node()` çağrısı birçok node'da tekrarlanıyor.
5. **Mission file loading:** `mission_manager_node` ve `gps_guidance_node` aynı dosyayı ayrı ayrı yüklüyor.

### 4.4 Güvenlik Açıkları

| Öncelik | Açık | Etki |
|---------|------|------|
| **CRITICAL** | YKI UDP komutlarında authentication yok | Herhangi biri kill/start komutu gönderebilir |
| **CRITICAL** | `mavros_bridge_node` `enabled:=true` ile açılabilir | Yanlışlıkla motor komutu gönderilebilir |
| **HIGH** | `sensor_fusion_node.py` root'ta ve kullanılmıyor | Confusion, potansiyel güvenlik açığı |
| **HIGH** | `perception_node.py` video dosyası path'i oluşturulurken `datetime.now()` kullanılıyor | Race condition potansiyeli (aynı saniye başlatılırsa çakışma) |
| **MEDIUM** | CSV log dosyaları herhangi bir dizine yazılabilir | Path traversal riski (konfigürasyon üzerinden) |
| **MEDIUM** | `yki_bridge_node` UDP broadcast alıyor | Man-in-the-middle riski |

### 4.5 Race Condition Potansiyeli

| Konum | Risk | Şiddet |
|-------|------|--------|
| `controller_node.py` - `planner_bearing_ts` | Multi-threaded executor'da race | HIGH |
| `gps_guidance_node.py` - GPS + heading | Eşzamansız güncelleme | HIGH |
| `buoy_detector_node.py` - `latest_depth` | Color ve depth callback farklı thread'ler | CRITICAL |
| `safety_node.py` - `last_cmd` | Kill sırasında yeni komut | MEDIUM |
| `parkur2_planner_node.py` - Tüm sensor state'leri | Farklı callback'lerden güncelleme | HIGH |

### 4.6 Memory Leak Potansiyeli

| Konum | Risk | Şiddet |
|-------|------|--------|
| `course_memory_node.py` - `samples` listesi | Sınırsız büyüme | MEDIUM |
| `local_costmap_node.py` - `OccupancyGrid` | Her döngüde yeni nesne | LOW |
| `logger_node.py` - CSV dosyası | Sürekli açık kalma | LOW |
| `perception_node.py` - Video writer | Bellek birikimi | MEDIUM |

---

## 5. Mimari Analizi

### 5.1 Node'lar Arası Bağımlılıklar

```
Mission Manager
├── gps_guidance_node (waypoints, active_waypoint)
├── course_memory_node (active_waypoint)
└── controller_node (started, completed)

GPS Guidance
├── mission_manager_node (waypoints, active_waypoint, started)
├── mavros (GPS, heading)
└── controller_node (advance_waypoint)

Controller
├── mavros (heading)
├── gps_guidance (bearing, distance)
├── parkur2_planner (safe_bearing, speed_limit) [opsiyonel]
├── perception (corridor_hint) [opsiyonel]
└── mission_manager (started, completed)

Parkur2 Planner
├── mavros (heading)
├── gps_guidance (target_bearing)
├── lidar_processor (lidar_summary)
├── corridor_tracker (corridor)
└── semantic_buoy_classifier (semantic_buoys)

Safety
├── controller (cmd_vel)
├── rc_kill (kill)
├── yki_bridge (kill)
└── mavros_bridge (kill, completed)
```

### 5.2 Topic Yapısı

**String JSON Mesaj Kullanımı:** Çok fazla node `std_msgs/String` içinde JSON kullanıyor. Bu:
- Tip güvenliğini azaltır
- ROS 2 message introspection'ı bozar
- `ros2 topic echo` çıktısını okumayı zorlaştırır
- Custom message tanımlamak yerine "kolay" yol seçilmiş

**Öneri:** Custom ROS 2 message tanımlamaları oluşturulmalı.

### 5.3 Single Point of Failure

| SPOF | Etki | Yedekleme |
|------|------|-----------|
| `safety_node` | Tüm motor komutları kesilir | Yok |
| `mission_manager_node` | Görev durumu kaybolur | Yok |
| `mavros_bridge_node` | Pixhawk haberleşmesi kesilir | Yok |
| `yki_bridge_node` | YKI haberleşmesi kesilir | Telemetry kaybı (kritik değil) |

**Değerlendirme:** `safety_node` SPOF olmak zorunda (güvenlik katmanı), ancak `mission_manager` ve `mavros_bridge` için yedekleme stratejisi düşünülebilir.

### 5.4 Test Coverage

| Test | Durum |
|------|-------|
| PEP257 (docstring) | Var - ament_pep257 |
| Flake8 (linting) | Var - ament_flake8 |
| Copyright | Skip edilmiş |
| Unit test | Yok |
| Integration test | Yok |
| Simülasyon test | Parkur-1 için var |

**Değerlendirme:** Sadece linting testleri var. Gerçek unit test, integration test veya simülasyon test yok. Bu, TEKNOFEST gibi kritik bir yarışma için yetersiz.

---

## 6. Performans Analizi

### 6.1 Callback Süreleri

| Node | Timer Hz | İşlem Yükü | Risk |
|------|----------|-----------|------|
| `controller_node` | 10 Hz | Basit matematik | Düşük |
| `gps_guidance_node` | 10 Hz | Haversine | Düşük |
| `parkur2_planner_node` | 10 Hz | Çoklu sensor fusion | Orta |
| `lidar_processor_node` | Event-driven | Scan işleme | Düşük |
| `local_costmap_node` | 1 Hz | Grid oluşturma | Orta |
| `logger_node` | 1 Hz | CSV yazma | Orta |
| `perception_node` | 10 Hz | OpenCV + VideoWriter | **Yüksek** |
| `buoy_detector_node` | Event-driven | YOLO inference | **Çok Yüksek** |
| `yki_bridge_node` | 2 Hz | UDP send | Düşük |
| `safety_node` | 20 Hz | Basit koşul | Düşük |

### 6.2 Veri Akışı Darboğazları

1. **YOLO Inference:** `buoy_detector_node` her frame'de YOLO çalıştırıyor. Bu, Jetson Orin Nano'da bile 30 FPS'de zorlayıcı olabilir.
2. **Video Recording:** `perception_node` her frame'i disk'e yazıyor. SSD yerine SD kart kullanılıyorsa ciddi darboğaz.
3. **CSV Logging:** `logger_node` ve `local_costmap_node` her döngüde flush ediyor.
4. **JSON Serialization:** Her node'da JSON parse/serialize işlemleri CPU kullanıyor.

### 6.3 Gereksiz Hesaplamalar

1. `local_costmap_node.py` - Her döngüde `sorted(occupied)` yapılıyor
2. `controller_node.py` - `target_distance` subscribe ediliyor ama kullanılmıyor
3. `parkur2_planner_node.py` - Her timer'da tüm semantic buoys üzerinden döngü
4. `corridor_tracker_node.py` - `_nearest_gate` her callback'te çalışıyor

---

## 7. Genel Repo Seviyesi Değerlendirme

### 7.1 En Kritik Riskler (Acil Eylem Gerekli)

| # | Risk | Etki | Çözüm |
|---|------|------|-------|
| 1 | **YKI UDP authentication yok** | Herhangi biri araç kontrolünü ele geçirebilir | Token/HMAC authentication ekle |
| 2 | **Depth-Color senkronizasyonu yok** | Yanlış mesafe ölçümü, çarpışma | message_filters veya timestamp matching |
| 3 | **GPS+Heading eşzamansız** | Yanlış navigasyon, waypoint ıskalama | Timeout veya senkronizasyon |
| 4 | **Tek contributor, tek branch** | Kod review yok, hata riski yüksek | En azından 1 kişi daha review yapmalı |
| 5 | **Test coverage sıfır** | Regresyon riski, güvenilirlik düşük | Unit ve integration test yaz |

### 7.2 En Acil Geliştirme İhtiyaçları

1. **Custom ROS 2 Message Tanımlamaları:** JSON String yerine tip güvenli mesajlar
2. **Unit Test Suite:** En azından core fonksiyonlar için test
3. **Integration Test:** Parkur-1 simülasyon pipeline'ı için otomatik test
4. **ROS 2 Clock Senkronizasyonu:** Tüm node'larda `time.time()` yerine ROS clock
5. **YOLO Inference Optimizasyonu:** TensorRT veya ayrı thread
6. **Video Recording Optimizasyonu:** Ayrı thread veya buffer

### 7.3 Teknik Borçlar

| Borç | Maliyet | Öncelik |
|------|---------|---------|
| JSON String mesajlar | Yüksek - Tip güvenliği yok, debugging zor | Yüksek |
| `sensor_fusion_node.py` root'ta | Orta - Confusion, maintenance zor | Orta |
| `common.py` fonksiyonları tutarsız kullanım | Düşük - Kod tekrarı | Düşük |
| Magic numbers (0.03, 999.0, vb.) | Orta - Maintainability | Orta |
| Eksik docstring'ler | Düşük - PEP257 zaten var | Düşük |

### 7.4 Güvenlik Açıkları Özeti

1. **Network:** YKI UDP portları açık, authentication yok
2. **Physical:** `power_relay_node` SIGKILL durumunda cleanup yapmayabilir
3. **Software:** `mavros_bridge_node` yanlışlıkla enable edilebilir
4. **Data:** Log dosyaları herhangi bir dizine yazılabilir

---

## 8. Özet ve Öneriler

### Pozitif Bulgular
- Güvenlik-first tasarım (varsayılan disable, latch kill, timeout'lar)
- Modüler mimari (Parkur-1/2/3 ayrımı)
- İyi dokümantasyon (README ve PROJE_KOD_RAPORU)
- Config parametreleri merkezi
- Graceful degradation (YOLO yoksa boş detection)

### Negatif Bulgular
- Test coverage yok
- Tek contributor
- JSON String mesajlar
- Eşzamansız sensor verileri
- YKI güvenliği zayıf
- Performans optimizasyonu yok

### Acil Yapılacaklar Listesi

**Hafta 1-2:**
1. YKI UDP authentication ekle
2. Depth-Color senkronizasyonunu düzelt
3. `sensor_fusion_node.py`'yi düzelt veya kaldır
4. ROS 2 clock kullanımına geç

**Hafta 3-4:**
5. Custom ROS 2 message tanımlamaları oluştur
6. Unit test suite başlat
7. Parkur-1 simülasyon integration test yaz

**Hafta 5-6:**
8. YOLO inference'ı optimize et
9. Video recording'i ayrı thread'e al
10. Performans profiling yap

**Sürekli:**
11. Kod review süreci başlat (en az 1 reviewer)
12. Branching stratejisi oluştur (feature branch'ler)
13. CI/CD pipeline kur (GitHub Actions)

---

Bu analiz, TEKNOFEST İDA projesinin yazılım stack'inin genel olarak sağlam bir temele sahip olduğunu ancak güvenlik, test ve performans açısından önemli iyileştirmelere ihtiyaç duyduğunu göstermektedir. Özellikle yarışma tarihine yaklaşıldıkça, test coverage ve güvenlik önlemlerinin artırılması kritik öneme sahiptir.
