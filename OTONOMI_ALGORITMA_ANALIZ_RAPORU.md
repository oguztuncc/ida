# IDA Otonom Algoritma Derinlemesine Analiz Raporu

> **Hazırlayan:** Kimi Code CLI (Otonom Sistem Analizi)
> **Tarih:** 2026-05-18
> **Kapsam:** `src/ida_otonom/` kaynak kodları, konfigürasyonlar, launch dosyaları ve sensör-füzyon pipeline'ı
> **Odak:** LiDAR & Kamera kullanımı, yanlış yönlendirme kök nedenleri, kritik riskler, geliştirme önerileri

---

## 1. Özet

Bu rapor, IDA otonom aracının algı-planlama-kontrol pipeline'ını kod düzeyinde incelemektedir. Sistem, **ROS 2** üzerinde çalışan, **LiDAR** (2D lazer tarama) ve **RGB-D kamera** (RealSense + YOLO) tabanlı, görev odaklı bir otonom mimariye sahiptir. Üç ana görev modu vardır: GPS waypoint takibi, koridor/engelden kaçınma (Parkur 2) ve renkli duba dokunma (Parkur 3).

**Temel Bulgular:**
- Algoritma **doğru yönlendirme yerine yanlış yönlendirmeye meyillidir**; bunun en önemli sebepleri **derinlik sensörü dönüşüm hatası**, **tutarsız konvansiyonlar**, **LiDAR kaçınma yönü latch mekanizmasının aşırı agresifliği** ve **koridor takibinde tek taraflı tahmin hatalarıdır**.
- Sensör füzyonu **JSON string mesajlar** üzerinden yapılmakta olup tip güvenliği yoktur ve gecikmelere açıktır.
- Konfigürasyon dosyaları (`ida_real.yaml` vs `ida_sim.yaml`) arasında **kritik davranış farkları** vardır; simülasyonda çalışan kod gerçek donanımda farklı davranabilir.

---

## 2. Sistem Mimarisi ve Veri Akışı

### 2.1 Ana Node'lar ve Akış

```
┌─────────────────┐     ┌──────────────────┐     ┌─────────────────┐
│   LiDAR (SLL)   │────▶│ lidar_processor  │────▶│/perception/     │
│   /scan         │     │_node.py          │     │lidar_summary    │
└─────────────────┘     └──────────────────┘     └────────┬────────┘
                                                         │
┌─────────────────┐     ┌──────────────────┐     ┌───────▼─────────┐
│ RealSense RGB   │────▶│ buoy_detector_   │────▶│ sensor_cross_   │
│ /camera/...     │     │ node.py (YOLO)   │     │ validator_node  │
│ Depth /aligned_ │────▶│                  │     │                 │
│ depth_to_color  │     └──────────────────┘     └───────┬─────────┘
└─────────────────┘                                      │
                                                         ▼
┌─────────────────┐     ┌──────────────────┐     ┌─────────────────┐
│ GPS (MAVROS)    │────▶│ gps_guidance_    │────▶│ semantic_buoy_  │
│ /mavros/global  │     │ node.py          │     │ classifier_node │
│ _position/global│     └──────────────────┘     └───────┬─────────┘
└─────────────────┘                                      │
                                                         ▼
                                              ┌──────────────────────┐
                                              │ corridor_tracker_    │
                                              │ node.py              │
                                              └──────────┬───────────┘
                                                         │
┌────────────────────────────────────────────────────────┼────────────────────┐
│                                                        ▼                    │
│  ┌──────────────────┐     ┌──────────────────┐     ┌─────────────────────┐  │
│  │ parkur2_planner_ │     │ parkur3_planner_ │     │ controller_node     │  │
│  │ node.py          │     │ node.py          │     │ (P kontrolcü)       │  │
│  │ (Engel/Corridor) │     │ (Renkli Duba)    │     │                     │  │
│  └────────┬─────────┘     └────────┬─────────┘     └──────────┬──────────┘  │
│           │                        │                        │             │
│           └────────────────────────┴────────────────────────┘             │
│                                                        │                    │
│                                                        ▼                    │
│                                           ┌─────────────────────┐           │
│                                           │ mavros_bridge_node  │──▶ Pixhawk│
│                                           │ (manual_control)    │           │
│                                           └─────────────────────┘           │
└─────────────────────────────────────────────────────────────────────────────┘
```

### 2.2 Mesajlaşma Yapısı

Sistem, ROS2'nin zengin tip sistemini (örn. `sensor_msgs/LaserScan`, `geometry_msgs/Twist`) yerine büyük ölçüde **JSON string mesajlar** (`std_msgs/String`) kullanmaktadır. Bu tasarım seçimi aşağıdaki sorunları beraberinde getirir:

- **Tip güvensizliği:** Bir alan adı yazım hatası veya tip uyuşmazlığı derleme zamanında yakalanamaz; çalışma zamanında sessizce `None` veya `KeyError` olur.
- **Performans:** JSON serileştirme/deserileştirme her mesajda CPU yükü oluşturur.
- **Araç zinciri uyumsuzluğu:** `ros2 topic echo`, `rqt_graph`, rosbag analiz araçları ile JSON içerikleri doğrudan görselleştirilemez.

---

## 3. LiDAR Kullanımı ve Riskler

### 3.1 LiDAR İşleme (`lidar_processor_node.py`)

LiDAR verisi `/scan` topic'inden alınır ve aşağıdaki metrikler hesaplanır:

| Metrik | Açıklama | Kullanım Yeri |
|--------|----------|---------------|
| `front_clearance_m` | Ön sektördeki (±15°) min mesafe | Genel engel algılama |
| `front_path_clearance_m` | Ön yol şeridindeki (yarı genişlik 0.70m) min ileri mesafe | Yol boşluğu |
| `front_footprint_clearance_m` | Araç footprint'ine göre (0.76×1.11m) min mesafe | Çarpışma tespiti |
| `best_free_angle_deg` | Engelden kaçınmak için önerilen açı | Kaçınma yönü |
| `left/right_clearance_m` | Yan sektörlerdeki min mesafeler | Yan boşluk |

**Algı:** `_best_free_angle` fonksiyonu, ön taraf engelli ise aday açıları tarar. Skorlama şu şekildedir:
```python
score = clearance_score - turn_penalty + continuity_bonus + escape_bonus
```

### 3.2 Kritik LiDAR Riskleri

#### R1: Kaçınma Yönü "Latch" Aşırı Agresifliği
`latch_avoidance_direction: true` olduğunda, bir kez kaçınma yönü (sol/sağ) seçildiğinde o yönde kalınır. Engel geçilene kadar değişmez.

- **Risk:** Sağda daha geniş bir geçit varken sistem sola kaçınmaya devam edebilir. Dar parkurlarda bu, aracı duvara veya yan şamandıraya iter.
- **Kod:** Satır 161-166, `avoidance_sign` değişkeni latch'lenir.

#### R2: `_preferred_escape_sign` Deterministik Olmayan Davranış
Eğer ön tarafta engel yoksa (veya açı farkı <3°), fonksiyon `last_best_angle_deg`'e bakar. İlk çalıştırmada bu 0.0'dır ve **default olarak `-1.0` (sola)** döner.

```python
if abs(self.last_best_angle_deg) > 1.0:
    return 1.0 if self.last_best_angle_deg > 0.0 else -1.0
return -1.0   # <-- Her zaman sola default
```

- **Risk:** Simetrik bir durumda sistem deterministik olmayan bir tarafa yönelir. Rüzgar/akıntı durumunda sola kaçınma aracı parkur dışına çıkarabilir.

#### R3: `best_free_angle` Hesabında "Arkadaki Boşluk" Tuzağı
Aday pencere tarama, engelin arkasındaki boş alanı da "free" olarak değerlendirebilir. Özellikle 2D LiDAR, bir şamandıranın arkasındaki açık suyu "açık yol" olarak işaretleyebilir. Bu durumda `best_free_angle`, aracı **doğrudan engelin içinden geçmeye** yönlendirebilir.

#### R4: `front_path_half_width_m` ile Araç Genişliği Uyumsuzluğu
Real konfigürasyonda `front_path_half_width_m: 0.70m`, yani toplam yol genişliği 1.40m. Araç genişliği 0.76m. Boşluk payı sadece 0.32m. Su dalgaları, rüzgar sürüklenmesi ve kontrol gecikmesi ile bu pay kolayca tükenir.

---

## 4. Kamera Kullanımı ve Riskler

### 4.1 YOLO Algı (`buoy_detector_node.py`)

RealSense RGB görüntüsü üzerinde YOLOv8 çalıştırılır. Derinlik (`aligned_depth_to_color`) kullanılarak her tespit için mesafe hesaplanır.

**Pozisyon Çıkarım Akışı:**
1. `camera_info_cb`: Kamera matrisi `fx`, `cx` alınır.
2. `depth_cb`: Derinlik görüntüsü `latest_depth` olarak saklanır.
3. `_range_at()`: Tespit merkezindeki derinlik değerinin medyanı alınır.
4. `_bearing_for_pixel()`: Piksel merkezinden yatay açı (`bearing_deg`) hesaplanır.

### 4.2 Kritik Kamera Riskleri

#### R5: Derinlik Dönüşüm Hatası (Çok Kritik!)
```python
# buoy_detector_node.py, Satır 163
median = float(np.median(vals))
if median > 50.0:
    median /= 1000.0
return median
```

Bu kod, derinlik değerinin **mm cinsinden mi yoksa metre cinsinden mi** geldiğini anlamak için basitçe `> 50.0` kontrolü yapıyor.

- **Gerçek RealSense:** Genellikle `uint16` mm formatında gönderir. 50 mm = 0.05 m.
- **Hata Senaryosu:** Bir obje 0.05 m (5 cm) yakınındaysa, `median = 50`. `50 > 50.0` **False** olduğu için dönüştürülmez. Sistem bu objeyi **50 metre** uzakta sanar!
- **Sonuç:** Yakındaki engeller (örn. su sıçratma, bir şamandıranın çok yaklaşması) uzağta olarak algılanır. Bu, çarpışma önleme sistemini tamamen devre dışı bırakabilir.

**Düzeltme Önerisi:** `msg.encoding` alanını kontrol edin (`"16UC1"` mm, `"32FC1"` metre) veya her zaman `cv_bridge` ile doğrudan metrik dönüşüm yapın.

#### R6: Kamera-LiDAR Zaman Uyumsuzluğu
`buoy_detector_node`, derinlik görüntüsünü en son gelen mesajdan alır. Ancak RGB ve derinlik mesajları **zaman damgası** karşılaştırılmadan kullanılır. Kamera 30 FPS, derinlik 15 FPS ise veya birinde frame drop olursa, renk koordinatları yanlış derinlik değerine eşleşir.

#### R7: Bearing İşaret Konvansiyonu Karışıklığı
```python
# buoy_detector_node.py, Satır 170
return math.degrees(math.atan2(self.cx - cx_px, self.fx))
```
Pozitif bearing = "port/sol" olarak tanımlanmış. Ancak:
- `gps_guidance_node`: Pozitif bearing = saat yönünde (standart navigasyon).
- `lidar_processor_node`: Pozitif açı = counter-clockwise (standart matematik).
- `parkur2_planner`: Pozitif `left_m` = sol.

Farklı node'lar arasında dönüşüm fonksiyonları (`_nav_relative_from_body_left`) var ancak bu karmaşıklık hata yapmaya açıktır.

---

## 5. Sensör Füzyon Analizi

### 5.1 Çapraz Doğrulama (`sensor_cross_validator_node.py`)

YOLO tespitlerinin mesafe bilgisi, LiDAR ile doğrulanır:

| Parametre | Simülasyon | Gerçek Donanım | Risk |
|-----------|-----------|----------------|------|
| `allow_lidar_only` | `true` | `false` | **Gerçekte depth bozuksa tüm tespitler reddedilir** |
| `range_tolerance_m` | 0.85 | 0.85 | Geniş tolerans, yanlış eşleşmeye izin verir |
| `relative_range_tolerance` | 0.30 | 0.30 | %30 mesafe toleransı çok yüksek |
| `bearing_window_deg` | 6.0 | 6.0 | Dar pencere, montaj offset hatalarına hassas |

#### R8: `allow_lidar_only: false` = Tek Hata Noktası
Gerçek konfigürasyonda (`ida_real.yaml`), eğer RealSense derinlik sensörü güneş parlaması, su sıçratma veya parlama nedeniyle bozuk değer verirse:
- `depth_range = None`
- `allow_lidar_only = false`
- Tespit `status = "missing_depth"` olur
- `confidence *= 0.25`
- `semantic_buoy_classifier` bu tespiti geçersiz sayabilir

**Sonuç:** Görsel algı tamamen çöker ve sistem kör kalır. Sadece LiDAR'a bağımlı hale gelir.

### 5.2 Semantik Sınıflandırma (`semantic_buoy_classifier_node.py`)

YOLO sınıf isimlerine göre (`course_buoy`, `obstacle_buoy`, `target_buoy`) semantik etiket atar. Renk belleği (`course_memory_node`) öğrenilen parkur rengiyle karşılaştırma yapar.

#### R9: Renk Toleransı ve Döngüsel Hue Sorunu
```python
course_hue_tolerance_deg: 35.0
```
HSV hue uzayı **döngüseldir** (0° = 360° = kırmızı). Eğer parkur şamandıraları kırmızı-turuncu tonlarındaysa ve aydınlatma değişirse, tolerans sınırında kalan renkler yanlış sınıflandırılabilir. `angular_distance_deg` fonksiyonu düzgün çalışsa da `course_memory_node`'daki `hue_distance_deg` ile farklı implementasyonlar kullanılması risklidir.

---

## 6. Kontrol ve Planlama Analizi

### 6.1 Kontrolcü (`controller_node.py`)

Basit bir **P kontrolcü** kullanılır:

```python
angular_speed = clamp(heading_error * kp_heading, -max_angular, max_angular)
```

- `kp_heading: 0.02` (real), `0.017` (sim)
- `max_angular_speed: 0.6` rad/s (real), `0.42` rad/s (sim)

**R10: Zayıf P Kazancı ve D İntegral Eksikliği**
45° heading error → `0.02 * 45 = 0.9` rad/s. Max açısal hız sınırlandığı için (0.6 rad/s) kontrolcü doyuma uğrar. Ancak:
- Sürekli dış etken (rüzgar/akıntı) varsa **I (integral) terimi olmadan** kalıcı hata (steady-state error) oluşur.
- **D (türev) terimi olmadan** overshoot yüksektir. Dar manevralarda şamandıraları aşıp geçme riski artar.

**R11: `vision_heading_bias` Devre Dışı**
`perception_node` (USB kamera + HoughLines tabanlı koridor tespiti) `enable_perception: false` olarak default'ta devre dışıdır. Bu durumda `vision_heading_bias` hep 0 olur. Kameranın koridor merkezi bilgisi kontrolcüye asla ulaşmaz.

### 6.2 Parkur 2 Planner (`parkur2_planner_node.py`)

Çok karmaşık bir state machine'dir: `CRUISE → PASS_COMMITTED → RETURN_TO_CENTER → SCAN_BEFORE_PASS → BLOCKED_ALIGN → RECOVERY_BACKOFF → EMERGENCY_STOP → CORRIDOR_SEARCH`.

#### R12: Waypoint Bias Ağırlığı = 0 (Simülasyon)
```yaml
# ida_sim.yaml
waypoint_bias_weight: 0.0
require_corridor_for_motion: true
```

Simülasyonda GPS waypoint yönü **tamamen devre dışıdır**. Sistem sadece koridoru takip eder. Eğer koridor takibi başarısız olursa (şamandıraları göremezse), araç `CORRIDOR_SEARCH` moduna geçer ve rastgele dolaşır.

#### R13: `path_block_margin_m` Çok Geniş
Simülasyonda `path_block_margin_m: 1.90` m. Bu, engelin koridor merkezinden 1.9 metre dışındaysa "yol kapalı" sayılmayacağı anlamına gelir. Dar parkurlarda bu, sistemin engelleri görmezden gelmesine neden olabilir.

#### R14: `BLOCKED_ALIGN` Deadlock
```python
relative = self._blend_relative_bearing(corridor_relative, target_relative, 0.3)
```
Engel kapattığında ve güvenli geçit yoksa sistem durur ve `corridor_relative` ile `target_relative`'yi karıştırarak dönmeye çalışır. Eğer bu iki vektör zıt yöndeyse (örn. koridor sağa, waypoint sola), blend sonucu ~0° kalabilir. Araç **hiç dönmez ve takılı kalır**.

#### R15: `SCAN_BEFORE_PASS` Yanlış Skorlama
Scan aşamasında sol/sağ skorlar LiDAR `best_free` değerine ve koridor genişliğine göre hesaplanır. Ancak `best_free`, engelin arkasındaki boşluğu gösterebilir (R3). Bu durumda scan, **engelin olduğu tarafı** "daha iyi" olarak seçebilir.

### 6.3 Parkur 3 Planner (`parkur3_planner_node.py`)

Renkli duba dokunma görevi.

#### R16: SEARCH Modunda Hedef Yokken Sabit 30° Dönüş
```python
else:
    relative = 30.0  # Sabit sağa dönüş
```
Hedef renkli duba bulunamazsa, araç sürekli sağa 30° dönerek ilerler. Bu deterministik arama deseni, hedefin solunda başlarsa aracı hedeften uzaklaştırır. Spiral veya sistematik arama yoktur.

#### R17: `touch_distance_m: 0.5` Güvenilmez
Simülasyonda ve gerçekte dokunma mesafesi olarak 0.5m kullanılır. Ancak derinlik sensörü hataları (R5) nedeniyle gerçek mesafe 0.3m olabilirken sistem 0.5m sanıp erken durabilir veya 0.7m olabilirken geç durabilir.

---

## 7. Yanlış Yönlendirme Kök Nedenleri (Özet Tablo)

| # | Sorun | Etki | Şiddet | Kaynak Dosya |
|---|-------|------|--------|--------------|
| 1 | **Derinlik dönüşüm hatası** (≤50mm) | Yakın objeler uzağta algılanır, çarpışma | 🔴 Kritik | `buoy_detector_node.py:163` |
| 2 | **LiDAR latch kaçınma** | Dar alanda yanlış tarafa kaçınma | 🔴 Kritik | `lidar_processor_node.py:161` |
| 3 | **Tek taraflı koridor tahmini** | Yanlış merkez hesabı, duvara sürüklenme | 🟠 Yüksek | `corridor_tracker_node.py:371-390` |
| 4 | **Waypoint bias = 0** | GPS hedefi görmezden gelme, kaybolma | 🟠 Yüksek | `ida_sim.yaml:176` |
| 5 | **Allow lidar only = false** | Depth bozulunca görsel algı çöker | 🟠 Yüksek | `ida_real.yaml:182` |
| 6 | **P kontrolcü zayıf kazanç** | Rüzgar/akıntıya karşı sapma, overshoot | 🟡 Orta | `controller_node.py:283` |
| 7 | **BLOCKED_ALIGN deadlock** | Engel karşısında takılma, ilerleyememe | 🟡 Orta | `parkur2_planner_node.py:584-624` |
| 8 | **JSON mesajlaşma** | Tip hataları, sessiz çalışma zamanı hataları | 🟡 Orta | Tüm algı node'ları |
| 9 | **Bearing konvansiyon karmaşası** | Farklı node'larda işaret hatası | 🟡 Orta | `common.py`, planner'lar |
| 10 | **Geofence dönüş hızı çok düşük** | Sınır dışına çıkınca yavaş geri dönüş | 🟡 Orta | `ida_real.yaml:20` |
| 11 | **Perception node devre dışı** | Kamera koridor merkezi bilgisi kullanılamaz | 🟢 Düşük | `ida_real.launch.py:88` |
| 12 | **Parkur 3 sabit 30° arama** | Hedefin yanlış tarafında dolaşma | 🟢 Düşük | `parkur3_planner_node.py:402` |

---

## 8. Acil Düzeltmeler (MVP)

Aşağıdaki değişiklikler **en yüksek etkiye sahiptir** ve göreceli olarak kolaydır:

### A1. Derinlik Dönüşümünü Düzelt
`buoy_detector_node.py` satır 163'ü değiştirin:
```python
# HATALI
if median > 50.0:
    median /= 1000.0

# DOĞRU: Encoding'e göre karar ver
if msg.encoding == "16UC1":  # uint16 mm
    median /= 1000.0
elif msg.encoding == "32FC1":  # float metre
    pass  # zaten metre
else:
    # Güvenlik için: eğer değer 100'den büyükse muhtemelen mm'dir
    if median > 100.0:
        median /= 1000.0
```

### A2. `allow_lidar_only: true` Yapın
`ida_real.yaml` içinde:
```yaml
sensor_cross_validator_node:
  ros__parameters:
    allow_lidar_only: true   # false -> true
```

Bu, depth sensörü arızalandığında veya bozulduğunda sistemin tamamen kör olmasını engeller.

### A3. LiDAR Latch'i Koşullu Yapın
`lidar_processor_node.py` içinde latch mekanizmasını, kaçınma açısı değiştirilebilirse gevşetin:
```python
# Mevcut: avoidance_sign sıfırlanmaz
# Öneri: Eğer alternatif tarafın clearance'ı X kat daha iyiyse yön değiştir
```

Veya en azından simülasyon/testlerde `latch_avoidance_direction: false` deneyin.

### A4. Waypoint Bias Ağırlığını Artırın
`ida_sim.yaml` ve `ida_real.yaml`:
```yaml
waypoint_bias_weight: 0.4   # 0.0/0.15 yerine
```
Bu, koridor kaybolduğunda GPS'in aracı güvenli şekilde ilerletmeye devam etmesini sağlar.

### A5. Kontrolcüye Feed-Forward Ekleyin
`controller_node.py`'de P kontrolcüye basit bir feed-forward ekleyin:
```python
# Mevcut
angular_speed = clamp(heading_error * kp_heading, -max, max)

# Öneri: Büyük hatalarda hızlı tepki
if abs(heading_error) > 20.0:
    kp_heading = 0.035  # geçici olarak kazancı artır
```

---

## 9. Orta Vadeli Geliştirme Önerileri

### 9.1 Sensör Füzyonunu Yeniden Tasarlayın
- **JSON yerine özel ROS2 mesajları** tanımlayın (`BuoyDetection.msg`, `LidarSummary.msg`, `CorridorState.msg`).
- **TF2 çerçeve dönüşümleri** kullanarak tüm sensör verilerini `base_link` çerçevesine getirin. Şu anda her node kendi koordinat dönüşümünü yapıyor.

### 9.2 LiDAR İşlemeyi Geliştirin
- **Dynamic Window Approach (DWA)** veya **VFF (Virtual Force Field)** implemente edin. Mevcut sektör tarama çok basittir.
- **2.5D engel tespiti:** LiDAR'ı biraz eğerek su yüzeyi ile şamandıra arasındaki farkı algılayın (şu anda sadece 2D tarama var).
- **Ray casting ile footprint kontrolü:** `_front_footprint_clearance` düz 2D bounding box kullanıyor. Dönüş anında gerçek footprint (dönmüş dikdörtgen) hesaplanmalı.

### 9.3 Görsel Algıyı Sağlamlaştırın
- **Depth-RGB senkronizasyonu:** `message_filters.ApproximateTimeSynchronizer` kullanarak zaman damgası eşleştirmesi yapın.
- **Multi-scale derinlik örnekleme:** Sadece merkez piksel yerine tespit kutusunun alt orta kısmını örnekleyin (şamandıranın su üzerindeki bölümü daha stabil mesafe verir).
- **YOLO sonrası filtreleme:** Mesafe ve boyut tutarsızlıklarını filtreleyen bir Kalman filtresi ekleyin.

### 9.4 Kontrolcüyü Geliştirin
- **PID kontrolcü** implemente edin (I ve D terimleri ekleyin).
- **Model Predictive Control (MPC)** veya en azından **Dubins yolu** planlayıcı düşünün.
- **Hız profili:** Basamaklı hız yerine, mesafe ve yörünge eğriliğine göre sürekli hız profili oluşturun.

### 9.5 Planlayıcıyı Basitleştirin ve Test Edin
- `parkur2_planner`'ın state machine'i çok karmaşık. Daha az durumlu, **behavior tree** tabanlı bir yapıya geçiş düşünülebilir.
- Her bir state geçişi için **birim testler** yazın. Şu anda test klasöründe sadece `test_copyright.py`, `test_flake8.py` ve `test_pep257.py` var; fonksiyonel test yok.

### 9.6 Simülasyon-Gerçek Uyumu
- `ida_sim.yaml` ve `ida_real.yaml` arasındaki farkları minimize edin. Farklı olan her parametre için açıklama yazın (örn. `# sim: daha yüksek çünkü simülasyonda sürtünme yok`).
- **Hardware-in-the-loop (HITL)** testleri gerçekleştirin; simülasyonda çalışan kodun aynı parametrelerle gerçekte çalışmayabileceğini unutmayın.

---

## 10. Ek: Parametre Karşılaştırma Tablosu

| Parametre | Simülasyon | Gerçek | Risk Değerlendirmesi |
|-----------|-----------|--------|----------------------|
| `kp_heading` | 0.017 | 0.02 | Her ikisi de düşük; I terimi yok |
| `max_linear_speed` | 0.34 | 0.35 | Yüksek; dar parkurda yavaşlatılmalı |
| `collision_distance_m` | 1.00 | 1.20 | Sim'de daha agresif |
| `front_path_half_width_m` | 1.90 | 0.70 | Gerçekte çok dar; araba sığmaz |
| `allow_lidar_only` | **true** | **false** | Gerçekte tek hata noktası |
| `require_corridor_for_motion` | true | true | Koridor kaybolunca hareket durur |
| `waypoint_bias_weight` | **0.0** | **0.15** | Sim'de GPS devre dışı |
| `corridor_coast_s` | 0.0 | 1.2 | Sim'de koridor kaybı anında panik |
| `obstacle_pass_margin_m` | 1.90 | 1.10 | Sim'de çok geniş, dar alanda kaçınma yapamaz |
| `path_block_margin_m` | 1.90 | auto | Sim'de engel görmezden gelinir |
| `scan_before_bypass` | false | true | Gerçekte ekstra duraksama |

---

## 11. Sonuç

Sistem, **düşük hızlı ve basit ortamlarda** çalışabilecek şekilde tasarlanmış ancak birkaç **kritik yazılım hatası** ve **mimari zayıflık** nedeniyle yanlış yönlendirme ve çarpışma riski taşımaktadır.

**En acil eylemler:**
1. Derinlik dönüşüm hatasını düzeltin (A1).
2. Gerçek donanımda `allow_lidar_only: true` yapın (A2).
3. Simülasyonda `waypoint_bias_weight > 0` yapın (A4).
4. LiDAR kaçınma latch'ini test edin ve gerekirse gevşetin (A3).

Bu düzeltmeler uygulandıktan sonra, **sistematik log analizi** ve **HITL testleri** ile iteratif iyileştirme önerilir.

---

*Rapor, `src/ida_otonom/` altındaki Python kaynak kodları ve YAML konfigürasyonlarının satır satır incelenmesiyle hazırlanmıştır.*
