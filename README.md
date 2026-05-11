# TEKNOFEST IDA Autonomy

Bu repo, TEKNOFEST İnsansız Deniz Aracı projesi için ROS 2 Humble tabanlı
otonomi paketini içerir. Mevcut sistem Parkur-1 için GPS ve heading tabanlı
waypoint takibini simülasyonda çalıştırır. Pixhawk, kamera, depth, lidar, YKI
ve kill-chain entegrasyonları güvenli tarafta tutulmuştur; gerçek donanımda
kullanılmadan önce tek tek doğrulanmalıdır.

## Mevcut Durum

- `mission_manager_node` waypoint JSON görev dosyasını okur ve görev durumunu
  yayınlar. Gerçek kullanımda `/mission/start` gelmeden görev başlamaz.
- `gps_guidance_node` aktif waypoint için mesafe ve bearing hesaplar.
- `controller_node` `/control/cmd_vel` yayınlar. Görev başlamadıysa veya görev
  bittiyse motor komutunu sıfırlar.
- `safety_node`, latched `/safety/kill` bilgisine göre `/control/cmd_vel`
  komutunu `/control/cmd_vel_safe` çıkışına geçirir veya keser.
- `sim_gps_node` Parkur-1 için basit GPS ve pusula simülasyonu sağlar.
- `logger_node` yarış telemetrisini ayarlanabilir bir klasöre CSV olarak yazar.
- `local_costmap_node` lidar scan verisinden `/local_costmap` yayınlar ve 1 Hz
  obstacle map CSV kaydı alır.
- Parkur-2 için YOLO hazır perception ve planning zinciri eklidir:
  `buoy_detector_node`, `course_memory_node`,
  `semantic_buoy_classifier_node`, `corridor_tracker_node` ve
  `parkur2_planner_node`.
- `yki_bridge_node` UDP üzerinden telemetri gönderir. İsteğe bağlı olarak görev
  başlamadan önce mission command ve emergency kill alabilir.
- `mavros_bridge_node` gerçek MAVROS entegrasyonu için güvenli bir ara katmandır,
  fakat gerçek actuator komutu varsayılan olarak kapalıdır.
- `rc_kill_node` MAVROS RC channel verisini okur ve channel 7 bilgisini latched
  software kill olarak yorumlar.
- `power_relay_node` ileride SSR/contactor zincirini Jetson GPIO ile tetiklemek
  için hazırlanmıştır, fakat varsayılan olarak kapalıdır.

Henüz tamamlanmayanlar: eğitilmiş YOLO weights, gerçek RealSense/RPLiDAR wet-test
ayarları, Parkur-3 İHA target color handoff, gerçek YKI UI ve doğrulanmış
hardware kill switch entegrasyonu.

## Build

Workspace root içinden:

```bash
colcon build --symlink-install --packages-select ida_otonom
source install/setup.bash
```

## Parkur-1 Simülasyon

```bash
ros2 launch ida_otonom parkur1_sim.launch.py
```

Kullanışlı launch argument örnekleri:

```bash
ros2 launch ida_otonom parkur1_sim.launch.py \
  mission_file:=/path/to/mission.json \
  arrival_radius_m:=3.0 \
  log_dir:=/tmp/ida_otonom_logs \
  enable_logger:=true \
  enable_costmap_logger:=false \
  enable_yki_bridge:=false
```

Varsayılan görev dosyası package share içinden çözülür:
`share/ida_otonom/missions/mission.json`.

## Parkur-2 Planning Stack

Parkur-2 stratejisi:

```text
course corridor center + obstacle avoidance + gate validation
```

Tekne, gördüğü herhangi iki dubanın arasına kör şekilde girmez. Sistem erken
waypoint aşamasında course buoy renk profilini öğrenir, bu profile belirgin
şekilde benzemeyen dubaları obstacle candidate olarak işaretler, sol ve sağ
course boundary çizgilerini tahmin eder, corridor center hattını takip eder ve
sadece engelden kaçmak için geçici sapma yapar. YOLO çalışsa bile lidar güvenlik
katmanı her zaman aktif kalır.

`config/ida_real.yaml` içindeki varsayılan güvenlik değerleri:

- max speed: `0.25 m/s`
- approach speed: `0.15 m/s`
- danger distance: `0.60 m`
- avoidance start distance: `1.20 m`
- safe clearance: `1.50 m`
- vehicle width: `1.10 m`
- side margin: `0.35 m`
- minimum gate width: `1.80 m`

Parkur-2 stack dry launch:

```bash
ros2 launch ida_otonom parkur2.launch.py
```

Eğitilmiş YOLO model path ile launch örneği:

```bash
ros2 launch ida_otonom parkur2.launch.py \
  model_path:=/home/jetson/ida/models/buoy_yolo.pt \
  color_image_topic:=/camera/camera/color/image_raw \
  depth_image_topic:=/camera/camera/aligned_depth_to_color/image_raw \
  camera_info_topic:=/camera/camera/color/camera_info
```

YOLO model veya `ultralytics` paketi yoksa detector çökmez; boş detection
yayınlar. Bu modda lidar costmap ve safety davranışı çalışmaya devam eder, fakat
semantic corridor tracking kamera detection gelene kadar zayıf kalır.

Önemli Parkur-2 topicleri:

- `/perception/buoy_detections`: YOLO detections ile color/depth/bearing verisi.
- `/perception/course_memory`: öğrenilen course buoy color profile.
- `/perception/semantic_buoys`: course boundary, obstacle, target veya unknown
  candidate verileri.
- `/planner/corridor`: tahmini corridor center ve gate validity.
- `/planner/safe_bearing_deg`: controller için absolute bearing.
- `/planner/speed_limit_mps`: planner speed limit.
- `/planner/status`: planner mode ve karar nedeni.

## MAVROS Safety Notes

`mavros_bridge_node` aracı arm etmez, flight mode değiştirmez ve `enabled:=true`
açıkça verilmeden gerçek actuator command yayınlamaz. Mevcut donanım planında
iki arka thruster ve her thruster için ayrı ESC varsayılmıştır. Gerçek komut
açılmadan önce Pixhawk output mapping mutlaka doğrulanmalıdır. Elektronik ekip
bu düzeni korursa başlangıç varsayımı muhtemelen `MAIN OUT 1 = left ESC` ve
`MAIN OUT 2 = right ESC` olur.

Gerçek teknede açmadan önce doğrulanacaklar:

- Pixhawk Cube Orange+ beklenen ArduRover version ve mode ile çalışıyor mu?
- Jetson, Pixhawk'a UART üzerinden bağlanıyor mu ve baudrate doğrulandı mı?
- ESC PWM neutral, forward ve reverse değerleri ölçüldü mü?
- Physical emergency stop, RC kill ve YKI kill gerçek motor gücünü kesiyor mu?
- MAVROS topic seçimi firmware ayarıyla uyumlu mu?
- Command axis mapping ve scaling, pervane/impeller devre dışıyken test edildi mi?
- Command timeout ve `/safety/kill` davranışı gerçek sistemde gözlendi mi?

Dry status run örneği:

```bash
ros2 run ida_otonom mavros_bridge_node
```

Gerçek tekne stack dry launch. Bu komut hâlâ motor komutu yayınlamaz:

```bash
ros2 launch ida_otonom ida_real.launch.py
```

Sadece hardware verification tamamlandıktan sonra örnek:

```bash
ros2 run ida_otonom safety_node
ros2 run ida_otonom mavros_bridge_node --ros-args \
  -p enabled:=true \
  -p input_topic:=/control/cmd_vel_safe \
  -p output_mode:=cmd_vel \
  -p cmd_vel_output_topic:=/mavros/setpoint_velocity/cmd_vel_unstamped
```

## Şartname Uyumluluk Notları

2026 şartnamesindeki yazılım açısından kritik bazı kurallar mevcut mimariye
işlenmiştir:

- YKI üzerinde autonomy, sensor processing veya image processing çalışmamalıdır.
  Bu node'lar Jetson üzerinde kalmalıdır. YKI sadece mission upload/start,
  telemetry display ve emergency kill için kullanılmalıdır.
- IDA veya İHA tarafından ground station'a image/video stream gönderilmemelidir.
  `yki_bridge_node` sadece telemetry JSON gönderir; processed camera video lokal
  olarak kaydedilir.
- Görev başladıktan sonra YKI/RC üzerinden emergency motor power cut dışında
  komut verilemez. `yki_bridge_node`, `/mission/started` true olduktan sonra
  kill dışındaki komutları yok sayar.
- Gerçek motor yolu `/control/cmd_vel_safe` üzerinden gitmelidir. Wet test
  öncesinde `safety_node`, `mavros_bridge_node`, Pixhawk failsafe, RC kill,
  YKI kill ve physical contactor chain birlikte doğrulanmalıdır.
- Gerekli teslim verileri lokal loglarla kapsanır: processed camera MP4,
  telemetry CSV ve local costmap CSV. CSV/video artifact dosyaları git'e
  eklenmez.
- Parkur-2/3 perception modüler tutulmuştur. Bu sayede buoy/target dataset hazır
  olduğunda model training eklenebilir; mission, safety, logging ve MAVROS
  sınırları değişmek zorunda kalmaz.

Bench test için YKI command receiver örneği:

```bash
ros2 run ida_otonom yki_bridge_node --ros-args \
  -p enable_command_rx:=true \
  -p command_bind_port:=5006
```

Desteklenen command JSON payload örnekleri:

```json
{"command": "start_mission"}
{"command": "kill", "active": true}
{"command": "reset_kill"}
{"command": "set_target_color", "color": "red"}
```

`config/ida_real.yaml` içinde tutulan güncel hardware kararları:

- İki arka thruster var, ayrı ESC'ler Pixhawk üzerinden sürülecek.
- GPS ve heading Pixhawk/MAVROS üzerinden gelecek.
- RC receiver Pixhawk'a bağlanacak.
- RC kill varsayılan olarak channel 7, 1800 PWM üstü active.
- Jetson-Pixhawk bağlantısı UART, fakat baudrate hâlâ hardware TODO.
- 24 V contactor, SSR/relay ile motor akımını kesecek. GPIO control hazır ama
  pin, logic level ve harici güvenlik devresi doğrulanana kadar kapalı.

## Repository Hygiene

`build/`, `install/`, `log/`, `__pycache__/`, `*.pyc`, telemetry CSV dosyaları,
video kayıtları ve `.DS_Store` gibi generated folder/artifact dosyaları git'e
eklenmemelidir.
