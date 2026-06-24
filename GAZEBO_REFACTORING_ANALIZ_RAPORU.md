# Gazebo Simülasyon Refactoring Analiz Raporu

Bu analiz read-only yapılmıştır; kaynak kodda değişiklik yapılmamıştır. Odak alanları DRY ihlalleri, dağınık state yönetimi, tight coupling ve özellikle İDA boyutları ile kaçınma parametrelerinin tek yerden yönetilmesidir.

## 1. Araç Boyutları ve Kaçınma Parametreleri Dağınık

### Sorun Tanımı

İDA boyutları, itki mesafesi, güvenli geçiş açıklığı ve kaçınma eşikleri tek merkezden yönetilmiyor.

Referanslar:

- `src/ida_gazebo/models/ida_katamaran/model.sdf:9`: kütle
- `src/ida_gazebo/models/ida_katamaran/model.sdf:25`, `37`, `47`, `59`: gövde boyutları
- `src/ida_gazebo/models/ida_katamaran/model.sdf:343`, `359`: hull length/radius
- `src/ida_gazebo/scripts/cmd_vel_to_thrust.py:26-30`: `wheelbase_m`, `max_thrust`, `yaw_sign`
- `src/ida_gazebo/launch/vrx_parkur1_sim.launch.py:198-201`: launch üzerinden thrust override
- `src/ida_otonom/config/parkur1_sim.yaml:22-25`: sim araç ve parkur ölçüleri
- `src/ida_otonom/config/parkur1_sim.yaml:99-101`: lidar footprint ölçüleri
- `src/ida_otonom/config/parkur1_sim.yaml:151-181`: corridor/planner açıklıkları
- `src/ida_otonom/config/ida_real.yaml:184-227`: gerçek araç corridor/planner güvenlik parametreleri

### Teknik Risk

Aynı araç için `vehicle_width_m` bazı sim configlerinde `0.76`, real config içinde `1.10` olarak kullanılıyor. `min_pass_gap_width_m` bazı sim configlerinde araç genişliğinden bile küçük kalabiliyor. Bu durum simde güvenli görünen kaçınma kararlarının gerçek araçta çarpışma riski taşımasına yol açar.

### Çözüm Önerisi

Tek bir `VehicleProfile` ve `SafetyEnvelope` kaynağı oluşturulmalı:

```yaml
vehicle:
  length_m: 1.11
  beam_m: 0.76
  thruster_track_m: 0.60
  lidar_x_m: 0.0
  lidar_z_m: 0.45

safety:
  side_clearance_m: 0.35
  front_clearance_m: 0.85
  emergency_stop_m: 0.45
  danger_m: 0.65
  avoid_start_m: 5.0
  min_pass_gap_m: 1.50
```

Bu profil launch tarafından `lidar_processor_node`, `corridor_tracker_node`, `parkur2_planner_node`, `cmd_vel_to_thrust` ve SDF üretimine aktarılmalı. `front_path_half_width_m`, `safe_clearance_m`, `min_pass_gap_width_m` gibi değerler elle ayrı ayrı girilmek yerine bu profilden türetilmeli.

## 2. Lidar ve Planner Güvenlik Eşikleri Birbirinden Bağımsız

### Sorun Tanımı

LiDAR collision eşiği ve planner emergency/danger eşiği farklı yerlerde tanımlı.

Referanslar:

- `src/ida_otonom/ida_otonom/lidar_processor_node.py:16-27`
- `src/ida_otonom/ida_otonom/parkur2_planner_node.py:14-44`
- `src/ida_otonom/config/parkur1_sim.yaml:95-108`
- `src/ida_otonom/config/parkur1_sim.yaml:162-201`
- `src/ida_otonom/config/ida_sim.yaml:70-82`
- `src/ida_otonom/config/ida_sim.yaml:139-176`
- `src/ida_otonom/config/ida_real.yaml:123-127`
- `src/ida_otonom/config/ida_real.yaml:201-227`

### Teknik Risk

LiDAR node "çarpışma yakın" diyebilirken planner bunu "kaçınma" veya "normal" görebilir. Özellikle gerçek araçta `lidar_processor_node.collision_distance_m=1.2`, planner tarafında `emergency_stop_distance_m=0.45` gibi farklı ölçekler var.

### Çözüm Önerisi

`SafetyEnvelope` adında ortak bir veri yapısı oluşturulmalı. LiDAR sadece ölçüm üretmeli; "emergency/danger/avoid" sınıflandırması tek yerde, tercihen planner veya ortak safety modülünde yapılmalı.

Startup sırasında şu validasyonlar çalışmalı:

```text
min_pass_gap_m >= vehicle.beam_m + 2 * side_clearance_m
front_path_half_width_m >= vehicle.beam_m / 2
danger_m >= emergency_stop_m
avoid_start_m >= danger_m
```

## 3. Parkur2 Planner Çok Büyük ve Dağınık State Machine

### Sorun Tanımı

`src/ida_otonom/ida_otonom/parkur2_planner_node.py` içinde parametre okuma, sensör cache'i, state machine, geçiş kararları ve publish işlemleri aynı sınıfta.

Referanslar:

- `src/ida_otonom/ida_otonom/parkur2_planner_node.py:14-60`: yoğun parametre deklarasyonu
- `src/ida_otonom/ida_otonom/parkur2_planner_node.py:186-214`: planner state alanları
- `src/ida_otonom/ida_otonom/parkur2_planner_node.py:281-300`: sensör callback cache'leri
- `src/ida_otonom/ida_otonom/parkur2_planner_node.py:667`, `692`, `751`, `970`, `1019`, `1171`, `1252`, `1397`, `1459`: dağınık mode geçişleri
- `src/ida_otonom/ida_otonom/parkur2_planner_node.py:990-1516`: ana karar döngüsü

### Teknik Risk

Yeni bir kaçınma davranışı eklemek veya mesafeleri elle ayarlamak zorlaşıyor. Bir state geçişinin hangi değişkenleri sıfırladığı net değil. `PASS_COMMITTED`, `RETURN_TO_CENTER`, `SCAN_BEFORE_PASS`, `BLOCKED_ALIGN` gibi modlar aynı `loop()` içinde birbirine karışıyor.

### Çözüm Önerisi

Planner üç parçaya ayrılmalı:

```text
PlannerInputs  -> sensör/guidance snapshot
PlannerState   -> mode, active_obstacle_id, pass_side, timers
PlannerCore    -> pure step(inputs, state, params) -> PlanCommand, new_state
```

`mode` string yerine `Enum` kullanılmalı. Tüm geçişler `transition_to(mode, reason, now)` fonksiyonundan geçmeli. Böylece algoritmaya elle müdahale etmek için sadece `PlannerParams` ve state geçiş tablosu düzenlenir.

## 4. Mission State Birden Fazla Topic'e Bölünmüş

### Sorun Tanımı

`mission_manager_node` aynı state'i ayrı ayrı `/mission/active_waypoint`, `/mission/started`, `/mission/completed`, `/mission/waypoints`, `/mission/status` olarak yayınlıyor.

Referanslar:

- `src/ida_otonom/ida_otonom/mission_manager_node.py:79-121`
- `src/ida_otonom/ida_otonom/mission_manager_node.py:439-477`
- `src/ida_otonom/ida_otonom/gps_guidance_node.py:197-203`

`gps_guidance_node`, `waypoints_cb` içinde `active_waypoint_index` değerini tekrar `0` yapıyor.

### Teknik Risk

ROS topic teslim sırası değişirse guidance kısa süre yanlış waypoint'e dönebilir. Mission load, active waypoint ve started bilgisi tek atomik snapshot olmadığı için transient tutarsızlık oluşabilir.

### Çözüm Önerisi

Tek canonical mission state kullanılmalı:

```json
{
  "mission_id": "...",
  "revision": 12,
  "started": true,
  "completed": false,
  "active_waypoint_index": 3,
  "waypoints": []
}
```

Guidance ve controller bu tek state'i tüketmeli. Eski topic'ler gerekiyorsa sadece backward-compatible mirror olarak kalmalı.

## 5. JSON String Mesajlarda Şema Yok

### Sorun Tanımı

Birçok kritik veri `std_msgs/String` içinde serbest JSON olarak taşınıyor.

Referanslar:

- `src/ida_otonom/ida_otonom/parkur2_planner_node.py:281-300`
- `src/ida_otonom/ida_otonom/corridor_tracker_node.py:338-344`
- `src/ida_otonom/ida_otonom/controller_node.py:271-347`
- `src/ida_gazebo/scripts/sim_buoy_detector.py:132-142`

### Teknik Risk

Alan adı değişirse sistem çoğu yerde sessizce fallback yapıyor. Bu, kaçınma kararını "sensör yok" veya "engel yok" gibi hatalı yorumlatabilir.

### Çözüm Önerisi

Kısa vadede `schemas.py` içinde dataclass tabanlı parser'lar oluşturulmalı:

```python
from dataclasses import dataclass

@dataclass
class LidarSummary:
    front_clearance_m: float
    front_footprint_clearance_m: float
    best_free_angle_deg: float
    timestamp: float
```

Orta vadede kritik topic'ler custom ROS message'a taşınmalı: `LidarSummary.msg`, `SemanticBuoyArray.msg`, `CorridorEstimate.msg`, `PlannerStatus.msg`.

## 6. Koordinat Dönüşümü ve Mission Okuma Tekrar Ediyor

### Sorun Tanımı

`common.resolve_mission_file()` zaten var; Gazebo script'leri aynı işi tekrar implemente etmiş.

Referanslar:

- `src/ida_otonom/ida_otonom/common.py:84`
- `src/ida_gazebo/scripts/spawn_waypoint_markers.py:34-44`
- `src/ida_gazebo/scripts/spawn_course_objects.py:44-54`
- `src/ida_gazebo/scripts/sim_buoy_detector.py:75-85`
- `src/ida_gazebo/scripts/mission_eval_node.py:81-91`

Lat/lon dönüşümü de `111_320.0` sabitiyle birçok yerde tekrar edilmiş.

### Teknik Risk

Origin veya mission formatı değişince Gazebo marker, detector, evaluator ve guidance farklı davranabilir.

### Çözüm Önerisi

`ida_common` veya `ida_gazebo/sim_common.py` altında şu ortak yapılar çıkarılmalı:

```python
GeoProjector(origin_lat, origin_lon)
MissionSpec.load(path)
CourseObject
Waypoint
```

Gazebo ve otonomi paketleri aynı parser ve projector'ı kullanmalı.

## 7. Gazebo Spawn Mantığı Tekrarlı ve Sessiz Hata Üretiyor

### Sorun Tanımı

Waypoint ve course object spawn script'leri benzer SDF string üretimi ve `ros2 run ros_gz_sim create` çağrısı yapıyor.

Referanslar:

- `src/ida_gazebo/scripts/spawn_waypoint_markers.py:50-106`
- `src/ida_gazebo/scripts/spawn_course_objects.py:69-122`

İki yerde de `subprocess.run(cmd, check=False)` kullanılıyor.

### Teknik Risk

Spawn başarısız olsa bile sistem bunu kritik hata saymıyor. Simülasyon "engel yokmuş" gibi çalışabilir.

### Çözüm Önerisi

Ortak `GazeboEntitySpawner` helper'ı yazılmalı. SDF string'leri template dosyasına alınmalı. `returncode != 0` durumunda hata loglanmalı ve opsiyonel olarak launch fail edilmeli.

## 8. Control Logic Gazebo Transport'a Sıkı Bağlı

### Sorun Tanımı

`cmd_vel_to_thrust.py` doğrudan `gz.transport13` ve `gz.msgs10` import ediyor. Aynı dosyada hem diferansiyel thrust hesabı hem Gazebo publish hem status JSON var.

Referanslar:

- `src/ida_gazebo/scripts/cmd_vel_to_thrust.py:16-17`
- `src/ida_gazebo/scripts/cmd_vel_to_thrust.py:83-186`

### Teknik Risk

Thrust mixer davranışını test etmek Gazebo'ya bağlı hale geliyor. Gerçek araçtaki MAVROS bridge ile Gazebo thrust adapter aynı actuator contract'ı paylaşmıyor.

### Çözüm Önerisi

Saf hesap kısmı ayrılmalı:

```python
DifferentialThrustMixer(cmd_vel, vehicle_profile) -> left_n, right_n
GazeboThrustAdapter(left_n, right_n) -> gz transport publish
```

Aynı mixer unit test edilebilir; Gazebo ve gerçek donanım sadece adapter olur.

## 9. Corridor Tracker ve Planner Arasında Sorumluluk Çakışması Var

### Sorun Tanımı

`corridor_tracker_node` içinde obstacle avoidance parametreleri ve offset hesabı var. Aynı zamanda planner kendi obstacle pass kararlarını veriyor.

Referanslar:

- `src/ida_otonom/ida_otonom/corridor_tracker_node.py:26-29`
- `src/ida_otonom/ida_otonom/corridor_tracker_node.py:115-164`
- `src/ida_otonom/ida_otonom/parkur2_planner_node.py:443-520`

### Teknik Risk

`obstacle_avoidance_enabled` açılırsa corridor center zaten engelden kaçmış olabilir; planner da ayrıca kaçınır. Bu çift müdahale salınım veya koridordan çıkma üretebilir.

### Çözüm Önerisi

Corridor tracker sadece geometri üretmeli: sol sınır, sağ sınır, merkez, confidence. Obstacle avoidance tek sahip olarak planner'da kalmalı. Corridor'daki obstacle offset ya kaldırılmalı ya da sadece debug/report-only kalmalı.

## 10. Controller İçinde Hız Eğrisi Hard-coded

### Sorun Tanımı

`controller_node.py` içinde heading error'a göre hız değerleri hard-coded.

Referanslar:

- `src/ida_otonom/ida_otonom/controller_node.py:526-543`

Örnek değerler: `10°`, `25°`, `45°`, `0.28`, `0.15`, `0.05`.

### Teknik Risk

Planner ayrı hız limiti üretiyor, controller ayrı hız kırpıyor. Elle tuning yapmak için hem planner hem controller kodunu/config'ini bilmek gerekiyor.

### Çözüm Önerisi

Config tabanlı hız eğrisi kullanılmalı:

```yaml
controller:
  heading_speed_curve:
    - {max_error_deg: 10, speed_mps: 0.34}
    - {max_error_deg: 25, speed_mps: 0.28}
    - {max_error_deg: 45, speed_mps: 0.15}
    - {max_error_deg: 180, speed_mps: 0.05}
```

Böylece algoritmaya elle müdahale config üzerinden yapılır.

## 11. Launch Dosyası Fazla Merkezi ve Kırılgan

### Sorun Tanımı

`vrx_parkur1_sim.launch.py` içinde world name, bridge topic'leri, spawn koordinatları, Gazebo node'ları ve tüm otonomi stack'i tek dosyada hard-coded.

Referanslar:

- `src/ida_gazebo/launch/vrx_parkur1_sim.launch.py:44-55`
- `src/ida_gazebo/launch/vrx_parkur1_sim.launch.py:101-136`
- `src/ida_gazebo/launch/vrx_parkur1_sim.launch.py:142-355`

### Teknik Risk

Model adı, world adı veya topic path değişirse birçok string birlikte güncellenmek zorunda. Parkur profili değiştirmek launch dosyasını elle düzenlemeye dönüşüyor.

### Çözüm Önerisi

Launch iki seviyeye ayrılmalı:

```text
simulation_profile.yaml
  world_name
  model_name
  spawn_pose
  bridge_topics
  vehicle_profile
  mission_file

launch_helpers.py
  make_gazebo_stack(profile)
  make_otonomy_stack(profile)
```

Bu yapı parkur ve araç profillerini elle değiştirmeyi çok kolaylaştırır.

## 12. Paketleme Sorunu: Olmayan Dizinler Install Ediliyor

### Sorun Tanımı

`ida_gazebo/CMakeLists.txt` içinde `config`, `worlds`, `urdf`, `meshes` dizinleri install listesinde; repo altında şu an yalnızca `launch`, `models`, `scripts` var.

Referans:

- `src/ida_gazebo/CMakeLists.txt:33-35`

### Teknik Risk

Temiz ortamda build/install aşaması kırılabilir veya paket yapısı yanıltıcı olur.

### Çözüm Önerisi

Ya bu dizinler oluşturulmalı ya da install listesi mevcut dizinlerle sınırlandırılmalı.

## Önerilen Refactor Sırası

1. `VehicleProfile`, `SafetyEnvelope`, `GeoProjector`, `MissionSpec` ortak modüllerini çıkar.
2. `parkur1_sim.yaml`, `ida_sim.yaml`, `ida_real.yaml` içindeki araç ve güvenlik parametrelerini tek profile bağla.
3. `parkur2_planner_node` için `PlannerInputs`, `PlannerState`, `PlannerParams`, `PlannerCore` ayrımını yap.
4. JSON mesajlar için dataclass parser/validator katmanı ekle.
5. Gazebo tarafında `DifferentialThrustMixer`, `GazeboThrustAdapter`, `GazeboEntitySpawner` ayrımına git.
6. En son SDF'yi profile bağlı template/xacro üretimine taşı.

## En Kritik İlk Adım

Araç boyutu ve kaçınma mesafelerini tek `vehicle/safety profile` dosyasına almak. Bu yapılmadan planner tuning'i her seferinde farklı dosyalarda manuel takip gerektirir.

