# IDA VRX Simülasyon Handoff

> Son güncelleme: 2026-05-25  
> Branch: `arif/vrx-simulasyon`  
> Son commit: `9e842fe feat: VRX Parkur-1 simülasyonunu kalibre et`  
> Ortam: ROS 2 Humble + Gazebo Sim Harmonic + VRX 2.4.0

---

## 1. Genel Hedef

TEKNOFEST İDA otonomi yazılımını Gazebo Sim Harmonic + VRX üzerinde çalıştırmak.

İlk pratik hedef Parkur-1U2:
- IDA katamaran modelini Sydney Regatta dünyasında stabil spawn etmek.
- GPS/IMU/LiDAR/kamera bridge zincirini ROS 2 topic'lerine bağlamak.
- `/control/cmd_vel_safe` komutunu iki sabit iticiye diferansiyel thrust olarak göndermek.
- Mission waypoint, corridor planner ve duba/engel algı zincirini simülasyonda test edilebilir hale getirmek.
- Parkur-1U2 başarıyla çalıştıktan sonra Parkur-2 ve daha sonra ArduPilot SITL'e geçmek.

---

## 2. Bu Oturumda Ne Kadar İlerledik?

### Tamamlandı

- `ida_gazebo` paketi üzerinden tek launch akışı çalışır hale getirildi.
- IDA Gazebo modeli spawn oluyor, su üstünde batmadan kalıyor.
- Sola sürekli dönme ve motor çalışınca gövdenin dönüp su altına girmesi problemleri çözüldü.
- Gazebo NavSat/IMU verisi MAVROS benzeri GPS/heading topic'lerine çevriliyor.
- Spawn world XY noktası `parkur1U2.json` mission origin'e hizalandı.
- `/control/cmd_vel_safe` komutu Gazebo Transport `cmd_thrust` topic'lerine sol/sağ thrust olarak gönderiliyor.
- Parkur waypoint markerları Gazebo'ya spawn edilebiliyor.
- `parkur1U2.json` içindeki boundary ve obstacle dubaları Gazebo'ya collision'lı objeler olarak spawn edilebiliyor.
- Mission JSON tabanlı sentetik duba algı node'u eklendi.
- Corridor/planner stack VRX launch'a koşullu olarak bağlandı.
- Mission evaluator node'u eklendi.
- `gps_guidance_node` içine route lookahead ve yaklaşan dönüş metadata'sı eklendi.
- `controller_node` içine waypoint slowdown, turn-in-place ve stop-turn-go modu eklendi.
- Parkur-1 sim parametreleri yeni kontrol mantığına göre ayarlandı.
- Lint/syntax/build doğrulamaları geçti.

### Kısmen Tamamlandı

- Parkur-1U2 temel simülasyon akışı kabul edilebilir seviyede.
- Duba/koridor stack launch ediliyor ve komut zinciri çalışıyor.
- Ancak tam görev evaluator PASS sonucu henüz son ayarlarla alınmadı.
- Manevra davranışı görsel olarak daha kabul edilebilir hale geldi, fakat Parkur-1U2/Parkur-2 için hâlâ tuning gerekiyor.

### Henüz Yapılmadı

- Parkur-1U2 için headless evaluator ile kesin PASS metriği alınmadı.
- `include_obstacles:=true` ile engel kaçınma tam doğrulanmadı.
- Parkur-2 görevinde aynı stack doğrulanmadı.
- Gerçek kamera/YOLO/LiDAR cluster tabanlı perception'a geçilmedi; şu an sim duba detector ground-truth yardımcı katman.
- ArduPilot SITL entegrasyonuna başlanmadı.

---

## 3. Güncel Çalıştırma Komutu

Kullanıcının son doğruladığı çalışma komutu:

```bash
source install/setup.bash
ros2 launch ida_gazebo vrx_parkur1_sim.launch.py \
  gui:=true \
  auto_start:=true \
  enable_course_objects:=true \
  enable_sim_buoy_detector:=true \
  enable_corridor_planner:=true \
  include_obstacles:=false
```

Son durum: bu komutta IDA spawn olduğunda artık batmıyor.

Headless smoke/evaluator için başlangıç komutu:

```bash
source install/setup.bash
ros2 launch ida_gazebo vrx_parkur1_sim.launch.py \
  gui:=false \
  auto_start:=true \
  enable_waypoint_markers:=false \
  enable_course_objects:=true \
  enable_sim_buoy_detector:=true \
  enable_corridor_planner:=true \
  include_obstacles:=false \
  enable_evaluator:=true
```

---

## 4. Güncel Mimari

### Gazebo Tarafı

- Ana launch: `src/ida_gazebo/launch/vrx_parkur1_sim.launch.py`
- Model: `src/ida_gazebo/models/ida_katamaran/model.sdf`
- Komut çevirici: `src/ida_gazebo/scripts/cmd_vel_to_thrust.py`
- GPS/heading çevirici: `src/ida_gazebo/scripts/sim_nav_converter.py`
- Waypoint marker: `src/ida_gazebo/scripts/spawn_waypoint_markers.py`
- Duba/engel spawner: `src/ida_gazebo/scripts/spawn_course_objects.py`
- Sentetik duba detector: `src/ida_gazebo/scripts/sim_buoy_detector.py`
- Görev evaluator: `src/ida_gazebo/scripts/mission_eval_node.py`

### Otonomi Tarafı

- Mission: `mission_manager_node`
- Guidance: `gps_guidance_node`
- LiDAR özet: `lidar_processor_node`
- Duba doğrulama/sınıflandırma: `sensor_cross_validator_node`, `course_memory_node`, `semantic_buoy_classifier_node`
- Koridor merkezi: `corridor_tracker_node`
- Planner: `parkur2_planner_node`
- Kontrol: `controller_node`
- Safety: `safety_node`

---

## 5. Kritik Parametreler

### Spawn / Mission Origin

- World: Sydney Regatta
- Spawn x/y: `-528.0`, `193.0`
- Spawn z: `0.0`
- Spawn yaw: `0.646 rad`
- `sim_nav_converter.py`, spawn noktasını `40.1181, 26.4081` mission origin'e hizalar.
- Parkur-1U2 ilk segment heading yaklaşık `53°`.

### Fizik Modeli

- Araç uzunluğu: yaklaşık `1.11 m`
- Araç dış genişliği: yaklaşık `0.78 m`
- İtici merkezleri: `y=±0.30 m`, merkezler arası `0.60 m`
- Hedef toplam sim kütlesi: yaklaşık `35 kg`
- `base_link` mass: `32.6 kg`
- Ek housing/prop linkleriyle toplam: yaklaşık `35 kg`
- COM/inertial pose: `0.028 0.008 0.029`
- Inertia: `ixx=0.72`, `iyy=1.73`, `izz=2.16`

### Surface / Hydrodynamics

- Surface plugin: VRX `libSurface.so`
- `hull_length=1.11`
- `hull_radius=0.115`
- `fluid_density=1025.9`
- Surface noktaları: `x=±0.55`, `y=±0.30`, `z=0.0`
- Hydrodynamics plugin: VRX `libSimpleHydrodynamics.so`
- Önemli katsayılar:
  - `xU=18.56`, `xUU=18.56`
  - `yV=36.45`, `yVV=10.93`
  - `zW=88.23`
  - `kP=18.0`, `kPP=36.0`
  - `mQ=36.0`, `mQQ=35.0`
  - `nR=8.0`, `nRR=4.0`

### Thruster

- Varsayım: Degz Ultras efektif `6 kgf`, yaklaşık `58.9 N` motor başına.
- `cmd_vel_to_thrust.py` launch override:
  - `wheelbase_m=0.60`
  - `max_thrust=58.9`
  - `thrust_rate_limit_nps=60.0`
  - `yaw_sign=-1.0`
- SDF thruster:
  - `thrust_coefficient=0.0008`
  - `fluid_density=1025.9`
  - `propeller_diameter=0.10`
  - `velocity_control=false`
  - `p_gain=0.0`

### Parkur-1U2 Kontrol

- `arrival_radius_m=1.1`
- `use_route_lookahead=true`
- `route_lookahead_m=2.0`
- `route_lookahead_cross_turns=false`
- `max_linear_speed=0.20`
- `max_angular_speed=0.75`
- `turn_in_place_error_deg=45.0`
- `turn_in_place_speed_mps=0.0`
- `waypoint_slowdown_distance_m=4.0`
- `waypoint_min_speed_mps=0.03`
- `waypoint_stop_turn_enabled=true`
- `waypoint_stop_turn_distance_m=1.1`
- `waypoint_stop_turn_angle_deg=35.0`
- `waypoint_stop_turn_align_error_deg=10.0`
- `waypoint_stop_turn_depart_s=1.2`
- `waypoint_stop_turn_depart_speed_mps=0.10`

---

## 6. Doğrulamalar

Geçen komutlar:

```bash
python3 -m py_compile \
  src/ida_otonom/ida_otonom/controller_node.py \
  src/ida_gazebo/scripts/cmd_vel_to_thrust.py
```

```bash
python3 -c "import xml.etree.ElementTree as ET; ET.parse('src/ida_gazebo/models/ida_katamaran/model.sdf')"
```

```bash
colcon build --symlink-install --packages-select ida_gazebo ida_otonom
```

```bash
colcon test --packages-select ida_otonom
```

Ek doğrulama:
- Headless Gazebo smoke launch çalıştı.
- Kullanıcı GUI launch'ta spawn sonrası batmanın düzeldiğini doğruladı.
- Son commit sonrası working tree temizdi.

---

## 7. Öğrenilenler / Kararlar

- `velocity_control=true` thruster modu denenmemeli; model/solver kararsızlığına yol açtı.
- `velocity_control=false` + `p_gain=0.0` mevcut durumda daha stabil.
- Sağ propeller joint axis ve sağ thrust işareti özel dikkat istiyor; mevcut çalışan mapping korunmalı.
- `yaw_sign=-1.0` gerekli; aksi halde controller yaw komutu Gazebo tarafında ters çalışıyor.
- TALAY hidrodynamics katsayıları VRX `SimpleHydrodynamics` plugin'ine birebir taşınmamalı; özellikle roll/pitch damping çok düşerse model batma/devrilme eğilimi gösteriyor.
- `y=±0.60` Surface/itici açıklığı kullanılmadı; araç dış genişliği yaklaşık `0.78 m` olduğu için `y=±0.30` korundu.
- Gerçekçi manevra için sadece lookahead yeterli değil; keskin dönüşlerde stop-turn-go mantığı gerekiyor.
- Dönüş/yavaşlama sırasında görülen sağ-sol yalpayı azaltmak için thrust çıkışına rate limit eklendi; ilk `90.0` hâlâ yalpalı olduğu için `60.0` seçildi ve roll damping artırıldı.
- Ground-truth sim detector nihai algı değil, planner/controller/fizik zincirini izole test etmek için ara katman.

---

## 8. Açık Riskler

- Tam Parkur-1U2 evaluator PASS henüz son ayarlarla alınmadı.
- Duba/koridor planner çalışıyor ama tuning tamamlanmadı.
- `include_obstacles:=true` engel kaçınma senaryosu henüz güvenilir seviyede değil.
- Fizik modeli gerçek araç için başlangıç kalibrasyonu; gerçek testlerle doğrulanmadı.
- `thrust_rate_limit_nps=60.0` görsel olarak doğrulanmalı; yalpa sürerse `45-50`, manevra yavaşlarsa `75-90` denenebilir.
- Gerçek Degz Ultras reverse thrust bilinmiyor; sim şimdilik simetrik thrust kabul ediyor.
- Kamera şu an RGB; RealSense D456 depth/gerçek perception zinciri doğrulanmadı.
- `src/ros_gz/` ve `src/vrx/` source build ve lokal patch durumları ayrıca izlenmeli.
- ArduPilot SITL henüz bağlanmadı.

---

## 9. Yeni Hedefler

### Hedef 1: Parkur-1U2 Evaluator PASS

- `include_obstacles:=false` ile headless evaluator çalıştır.
- Başarı kriterleri:
  - Mission tamamlanmalı.
  - Max cross-track kabul edilebilir olmalı.
  - Stall oluşmamalı.
  - Waypoint min distance değerleri raporlanmalı.
- Gerekirse ayarlanacak parametreler:
  - `waypoint_stop_turn_distance_m`
  - `waypoint_stop_turn_depart_s`
  - `max_angular_speed`
  - `nR`, `nRR`
  - `parkur2_planner_node` hız/bearing rate limitleri

### Hedef 2: Duba/Koridor Takibini Stabil Hale Getir

- `enable_course_objects:=true`
- `enable_sim_buoy_detector:=true`
- `enable_corridor_planner:=true`
- Önce `include_obstacles:=false`.
- Koridor merkezinin ani sıçramalarını ve planner bearing limitlerini gözle.
- `/planner/status`, `/planner/corridor`, `/control/setpoints` topic'lerini incele.

### Hedef 3: Engel Kaçınmayı Aç

- `include_obstacles:=true` ile test et.
- `sensor_cross_validator_node`, `semantic_buoy_classifier_node`, `parkur2_planner_node` obstacle davranışını kontrol et.
- Önce sim detector ile doğrula, sonra gerçek LiDAR/perception zincirine yaklaş.

### Hedef 4: Parkur-2'ye Geç

- Parkur-1U2 PASS sonrası `parkur2.json` mission dosyasına geç.
- Parkur-2 genişlik notu:
  - Şartname hedefi yüzeyden yüzeye `8.12 m`.
  - Duba yarıçapı `0.15 m` ise merkezden merkeze yaklaşık `8.42 m`.
- Config içindeki `expected_corridor_width_m`, `course_width_m`, `corridor_keepout_margin_m` alanlarını bu ayrımı bilerek ayarla.

### Hedef 5: SITL Hazırlığı

- Gazebo + mevcut otonomi Parkur-1U2/Parkur-2 çalışmadan ArduPilot SITL'e geçme.
- İlk SITL hedefi sadece komut zinciri doğrulaması olmalı:
  - `/control/cmd_vel_safe`
  - MAVROS bridge
  - ArduPilot mode/arm/mixer
  - motor output mapping

---

## 10. Bir Sonraki Oturum İçin Önerilen İlk Komutlar

```bash
git status --short
```

```bash
source install/setup.bash
ros2 launch ida_gazebo vrx_parkur1_sim.launch.py \
  gui:=false \
  auto_start:=true \
  enable_waypoint_markers:=false \
  enable_course_objects:=true \
  enable_sim_buoy_detector:=true \
  enable_corridor_planner:=true \
  include_obstacles:=false \
  enable_evaluator:=true
```

Eğer evaluator tamamlanmazsa ilk bakılacak topic'ler:

```bash
ros2 topic echo /control/setpoints
ros2 topic echo /planner/status
ros2 topic echo /planner/corridor
ros2 topic echo /eval/status
```
