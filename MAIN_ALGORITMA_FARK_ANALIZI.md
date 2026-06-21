# Main Branch'e Göre Algoritma Fark Analizi

> Kapsam: `arif/vrx-simulasyon` branch'i ile `main` branch'i karşılaştırması
> Odak: Simülasyon/Gazebo ekleri hariç, otonomi algoritmasını ve gerçek araç davranışını etkileyen farklar
> Karşılaştırma tabanı: `main...HEAD`

---

## 1. Kısa Özet

Mevcut branch, simülasyon altyapısı dışında otonomi algoritmasını da belirgin şekilde değiştirmiştir. Main branch'teki yapı daha çok GPS waypoint takibi, temel P kontrolcü ve daha basit Parkur-2 engel kaçınma akışına dayanırken; mevcut branch'te daha hassas waypoint geçişi, daha stateful controller davranışı, koridor içinde geçilebilir boşluk hesabı, LiDAR footprint güvenliği, kamera derinlik dönüşümü düzeltmesi ve daha zengin telemetri/schema katmanı eklenmiştir.

En önemli algoritmik farklar:

1. Waypoint varış yarıçapı gerçek ve sim konfigürasyonlarda küçültülmüş.
2. Controller artık yalnızca basit P kontrolcü değil; dinamik kazanç, turn-in-place ve waypoint yaklaşım modları içeriyor.
3. Parkur-2 planner, engelden kaçınmayı geçit/boşluk hesabı ve corridor memory ile yapacak şekilde büyümüş.
4. LiDAR güvenliği raw ön sektör mesafesinden araç footprint/path clearance tabanlı metriklere kaymış.
5. RealSense depth dönüşümünde encoding bazlı daha doğru metre/mm ayrımı yapılmış.
6. Gerçek araç config'inde LiDAR-only doğrulamaya izin verilmiş ve koridor zorunluluğu gevşetilmiş.

---

## 2. Genel Diff Özeti

Toplam değişiklik boyutu:

```text
75 files changed, 10885 insertions(+), 459 deletions(-)
```

Bu farkın büyük bölümü şu alanlardan geliyor:

| Alan | Etki |
|------|------|
| `src/ida_gazebo/` | Gazebo/VRX simülasyon paketi |
| `launch/*sim*` | Simülasyon launch akışları |
| `missions/*.json` | Yeni parkur/görev dosyaları |
| `*.md` raporları | Dokümantasyon ve handoff notları |
| `ida_otonom/*.py` | Simülasyon dışı algoritmik değişikliklerin ana kısmı |

Simülasyon/Gazebo dışındaki ana algoritma dosyaları:

| Dosya | Değişiklik tipi |
|-------|-----------------|
| `gps_guidance_node.py` | Waypoint hedefleme ve route lookahead metadata |
| `controller_node.py` | Kontrol algoritması ve waypoint davranışları |
| `parkur2_planner_node.py` | Engel geçişi, koridor ve LiDAR kaçınma mantığı |
| `lidar_processor_node.py` | Araç footprint/path tabanlı LiDAR metrikleri |
| `buoy_detector_node.py` | Depth encoding düzeltmesi |
| `corridor_tracker_node.py` | Araç profili ve typed corridor output |
| `schemas.py` | JSON mesajları için dataclass schema katmanı |
| `vehicle_params.py` | Araç profili ve güvenlik zarfı altyapısı |
| `ida_real.yaml` | Gerçek araç davranışını değiştiren parametreler |

---

## 3. Waypoint ve Guidance Farkları

Dosya: `src/ida_otonom/ida_otonom/gps_guidance_node.py`

Main branch'te guidance daha basitti: aktif waypoint'e bearing ve distance hesaplanıp yayınlanıyordu. Mevcut branch'te bunun üzerine route ve dönüş metadata'sı eklenmiş.

Önemli farklar:

| Başlık | Main | Mevcut branch |
|--------|------|---------------|
| Varsayılan varış yarıçapı | `3.0 m` | `1.0 m` |
| Lookahead hedefleme | Yok | `use_route_lookahead`, `route_lookahead_m`, `route_lookahead_cross_turns` |
| Guidance status | Sınırlı JSON | `GuidanceStatus` schema ile zengin JSON |
| Dönüş metadata'sı | Yok | `next_bearing_deg`, `upcoming_turn_angle_deg`, `leg_bearing_deg` |

Davranış etkisi:

1. Araç waypoint'i main'e göre daha yakın mesafeden tamamlanmış sayıyor.
2. Keskin dönüşlerde controller'ın önceden bilgi alabileceği metadata eklenmiş.
3. Route lookahead açılırsa hedef doğrudan waypoint değil, rota üstündeki ileri bir nokta olabiliyor.

Gerçek araç config etkisi:

```yaml
gps_guidance_node:
  ros__parameters:
    arrival_radius_m: 1.0
```

Bu, gerçek araçta da aktif davranış değişikliğidir.

---

## 4. Controller Farkları

Dosya: `src/ida_otonom/ida_otonom/controller_node.py`

Main branch'teki controller temel olarak heading error'a göre P kontrol ve kademeli hız seçimi yapıyordu. Mevcut branch'te controller daha stateful hale gelmiş.

Eklenen davranışlar:

| Yeni özellik | Açıklama |
|--------------|----------|
| Dinamik P kazancı | Büyük heading error'da `kp_heading` 1.5x veya 2x çarpılıyor |
| Turn-in-place | Heading error belirli eşiği aşarsa lineer hız düşürülüp/kapalı tutulup dönme önceleniyor |
| Preturn | Waypoint'e yaklaşırken bir sonraki segment bearing'ine blend edebilme |
| Waypoint slowdown | Waypoint'e yaklaşırken lineer hız mesafeye göre azaltılıyor |
| Waypoint stop-turn | Waypoint geçişinden sonra yeni bacağa hizalanıp kısa süre düşük hızla ayrılma altyapısı |
| Planner status tüketimi | `/planner/status` okunup setpoint telemetrisine taşınıyor |

Aktif davranış açısından önemli notlar:

1. Dinamik P kazancı parametreye bağlı kapatılmıyor; mevcut branch'te genel controller davranışını etkiliyor.
2. `turn_in_place`, `preturn`, `waypoint_stop_turn` gibi özellikler default olarak çoğunlukla kapalı, config ile açılabilir.
3. Controller artık `/guidance/status` içindeki `next_bearing`, `leg_bearing`, `upcoming_turn_angle` alanlarına bağımlı davranışlar içeriyor.

Main'e göre temel fark:

```text
Main: heading_error * kp_heading
Mevcut: heading_error * effective_kp + waypoint/turn/planner state etkileri
```

---

## 5. Parkur-2 Planner Farkları

Dosya: `src/ida_otonom/ida_otonom/parkur2_planner_node.py`

Bu dosya simülasyon dışındaki en büyük algoritmik değişikliği içeriyor.

Main branch'e göre temel değişim:

```text
Main: Engel tespit edildi -> taraf seç -> obstacle_left +/- margin ile geç
Mevcut: Engel tespit edildi -> koridor sınırları içinde geçilebilir boşlukları hesapla -> güvenli gap seç -> corridor memory ve state machine ile geç
```

Eklenen/yeni önem kazanan parametreler:

| Parametre | Amaç |
|-----------|------|
| `min_pass_gap_width_m` | Güvenli geçiş için minimum boşluk genişliği |
| `equal_gap_epsilon_m` | Sol/sağ gap benzerse karar toleransı |
| `path_block_margin_m` | Engel gerçekten yolu kapatıyor mu kontrolü |
| `waypoint_blend_factor` | Koridor takip ile GPS hedefini blend etme |
| `expected_corridor_width_m` | Koridor genişliği varsayımı |

Yeni planner davranışları:

1. Engel koridor merkezinden yeterince uzaktaysa yol kapatmıyor sayılabiliyor.
2. Geçiş tarafı artık yalnızca LiDAR `best_free_angle` ile değil, sol/sağ geçit genişliği ile seçiliyor.
3. `PASS_COMMITTED` sırasında aynı obstacle ID korunuyor; farklı engele zıplama azaltılıyor.
4. Koridor geçici kaybolursa `pass_corridor_memory` ile son koridor bilgisi kullanılabiliyor.
5. Güvenli geçit yoksa tam durmak yerine `BLOCKED_ALIGN` modunda yavaş hizalanma deneniyor.
6. `RETURN_TO_CENTER` modunda koridor merkezine dönüş GPS hedefiyle blend ediliyor.
7. `danger/avoid` LiDAR durumunda `best_free_angle` tekrar steering kaynağı olabiliyor.

Önemli davranış farkı:

Main'de LiDAR tehlikesi bazı durumlarda yalnızca yavaşlatma etkisindeydi. Mevcut branch'te LiDAR `best_free_angle` daha aktif şekilde planner bearing'ini etkileyebiliyor.

---

## 6. LiDAR İşleme Farkları

Dosya: `src/ida_otonom/ida_otonom/lidar_processor_node.py`

Main branch'te LiDAR özeti daha çok ön/sol/sağ sektör min mesafelerine dayanıyordu. Mevcut branch'te araç geometrisi dikkate alınmaya başlanmış.

Yeni metrikler:

| Alan | Açıklama |
|------|----------|
| `front_sector_clearance_m` | Eski ön sektör min mesafesine karşılık gelir |
| `front_path_clearance_m` | Araç ön yol şeridi içindeki ileri clearance |
| `front_footprint_clearance_m` | Araç footprint geometrisine göre clearance |
| `front_path_half_width_m` | Ön yol yarı genişliği |
| `vehicle_width_m` | Araç genişliği |
| `vehicle_length_m` | Araç uzunluğu |

Davranış değişiklikleri:

1. `collision_imminent`, artık raw `front_min` yerine `front_footprint_clearance_m` ile hesaplanıyor.
2. Çok yakın/self-detection olabilecek range'ler `min_valid_range_m` ile filtreleniyor.
3. Planner `_lidar_state()` içinde emergency/danger kararında `front_footprint_clearance_m` kullanıyor.
4. Avoid kararı için `front_sector_clearance_m` kullanılmaya devam ediyor.

Dikkat edilmesi gereken nokta:

`latch_override_ratio` parametresi eklenmiş fakat mevcut `_best_free_angle()` akışında adaylar zaten latch yönüyle filtrelendiği için karşı yön global olarak yeniden taranmıyor. Bu yüzden latch override beklendiği kadar etkili olmayabilir.

---

## 7. Kamera/Depth Algısı Farkı

Dosya: `src/ida_otonom/ida_otonom/buoy_detector_node.py`

Main branch'te depth birimi şu heuristikle ayrılıyordu:

```python
if median > 50.0:
    median /= 1000.0
```

Mevcut branch'te `msg.encoding` saklanıyor ve dönüşüm encoding'e göre yapılıyor:

```text
16UC1 / mono16 -> mm kabul edilip metreye çevriliyor
32FC1          -> metre kabul ediliyor
bilinmeyen     -> median > 100 ise mm varsayılıyor
```

Bu önemli bir gerçek donanım düzeltmesidir. RealSense tarafında `16UC1` depth genelde milimetre olduğu için main branch'teki heuristik bazı yakın mesafe senaryolarında hatalı sonuç verebilirdi.

---

## 8. Gerçek Araç Config Farkları

Dosya: `src/ida_otonom/config/ida_real.yaml`

Simülasyon dışı gerçek davranışı etkileyen başlıca config değişiklikleri:

| Parametre | Main | Mevcut branch | Etki |
|-----------|------|---------------|------|
| `gps_guidance_node.arrival_radius_m` | `3.0` | `1.0` | Waypoint daha hassas tamamlanır |
| `sensor_cross_validator.allow_lidar_only` | `false` | `true` | Depth yoksa LiDAR-only detection kabul edilebilir |
| `parkur2_planner.obstacle_pass_margin_m` | `1.10` | `0.60` | Engel geçiş marjı küçülür |
| `parkur2_planner.path_block_margin_m` | Yok | `1.20` | Sadece yolu kapatan engeller bypass tetikler |
| `parkur2_planner.require_corridor_for_motion` | `true` | `false` | Koridor yokken hareket daha serbest olur |
| `parkur2_planner.waypoint_bias_weight` | `0.15` | `0.4` | GPS hedefi koridor takibinde daha baskın olur |

Bu parametreler simülasyon dışı gerçek sistem davranışını doğrudan değiştirir.

Özellikle kritik olanlar:

1. `arrival_radius_m: 1.0` gerçek suda GPS hata payı, akıntı ve kontrol gecikmesi varsa waypoint geçişini zorlaştırabilir.
2. `allow_lidar_only: true` kamera depth hatalarında sistemi daha toleranslı yapar; ancak yanlış LiDAR eşleşmelerini kabul etme riskini artırır.
3. `obstacle_pass_margin_m: 0.60` geçişleri daha agresif hale getirir.
4. `require_corridor_for_motion: false` koridor kaybında tamamen durmak yerine GPS/planner ile devam etme eğilimi yaratır.

---

## 9. Schema ve Telemetri Farkları

Yeni dosya: `src/ida_otonom/ida_otonom/schemas.py`

Eklenen schema/dataclass yapıları:

| Sınıf | Topic |
|-------|-------|
| `LidarSummary` | `/perception/lidar_summary` |
| `CorridorEstimate` | `/planner/corridor` |
| `PlannerStatus` | `/planner/status` |
| `MissionStatus` | `/mission/status` |
| `GuidanceStatus` | `/guidance/status` |
| `SafetyStatus` | `/safety/status` |

Etkisi:

1. JSON mesaj alanları daha düzenli hale getirilmiş.
2. Logger ve YKI bridge bazı status mesajlarını schema parse ile okumaya başlamış.
3. Bu değişiklik doğrudan kontrol algoritması değil, veri güvenilirliği ve telemetri okunabilirliği iyileştirmesidir.

---

## 10. Araç Profili ve Güvenlik Zarfı

Yeni dosyalar:

```text
src/ida_otonom/ida_otonom/vehicle_params.py
src/ida_otonom/config/vehicle_profiles/ida_katamaran.yaml
```

Amaç:

1. Araç boyutlarını tek yerden yönetmek.
2. Güvenlik mesafelerini tek profilden türetmek.
3. Simülasyon ve gerçek araç arasında parametre tutarsızlığını azaltmak.

Profil içeriği:

```yaml
vehicle:
  length_m: 1.11
  beam_m: 0.76
  thruster_track_m: 0.60

safety:
  side_clearance_m: 0.35
  front_clearance_m: 0.85
  emergency_stop_m: 0.45
  danger_m: 0.60
  avoid_start_m: 5.0
  min_pass_gap_m: 1.50
  collision_distance_m: 1.20
```

Mevcut kullanım:

1. `corridor_tracker_node.py` araç profilini okuyup `vehicle_width_m` ve `side_safety_margin_m` defaultlarını bundan türetiyor.
2. Launch dosyalarında `vehicle_profile` argümanı `lidar_processor_node` ve `parkur2_planner_node` için de geçiriliyor.
3. Ancak `lidar_processor_node` ve `parkur2_planner_node` şu an bu parametreyi doğrudan kullanmıyor; bu nedenle profil entegrasyonu kısmi.

---

## 11. Diğer Değişiklikler

### Mission Manager

Dosya: `mission_manager_node.py`

Algoritmik fark sınırlı. Status yayınlama elle JSON yerine `MissionStatus` schema ile yapılıyor. Uyarı logları `warn` yerine `warning` olmuş.

### Safety Node

Dosya: `safety_node.py`

Algoritmik fark sınırlı. `/safety/status` artık `SafetyStatus` schema ile yayınlanıyor.

### Logger / YKI Bridge

Dosyalar: `logger_node.py`, `yki_bridge_node.py`

Planner status alanları daha detaylı loglanıyor ve bazı status mesajları schema ile parse ediliyor. Bu daha çok gözlem/telemetri farkıdır.

### Build/Lint

Dosyalar: `setup.cfg`, `.flake8`, `test_flake8.py`, `sllidar_ros2/CMakeLists.txt`

Algoritmik değil. `control_dashboard_node.py` flake8 dışında bırakılmış. `sllidar_ros2` CMake target linkleme biçimi değişmiş.

---

## 12. Sonuç

Simülasyon eklemek haricinde mevcut branch'in main'den temel farkı, otonomi stack'ini daha agresif ve daha zengin state bilgisi kullanan bir yapıya taşımasıdır.

En kritik algoritmik değişiklikler şunlardır:

1. Waypoint tamamlama `3m` yerine `1m` hassasiyete çekilmiş.
2. Controller büyük heading error'da daha agresif dönüyor ve waypoint yaklaşım/dönüş state'leri destekliyor.
3. Parkur-2 planner engel geçişini artık gap-based ve corridor-memory destekli yapıyor.
4. LiDAR güvenliği araç footprint/path clearance tabanlı hale gelmiş.
5. Kamera depth dönüşüm hatası encoding bazlı çözülmüş.
6. Gerçek araç config'inde LiDAR-only detection ve koridor olmadan hareket daha serbest hale getirilmiş.

Bu değişikliklerin ana etkisi:

```text
Daha gerçekçi/simülasyonla uyumlu davranış + daha fazla manevra kabiliyeti
ama bazı gerçek saha koşullarında daha agresif ve daha fazla tuning isteyen kontrol/planner davranışı.
```
