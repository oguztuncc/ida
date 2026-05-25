# IDA VRX Simülasyon — Handoff Notları

> Son güncelleme: 2026-05-24
> Aktif branch: `arif/vrx-simulasyon`
> Gazebo Sim Harmonic (gz-sim8) + VRX 2.4.0 (jazzy branch) + ROS 2 Humble

---

## 1. Hedefimiz

IDA katamaranı için **tek launch** ile çalışan Gazebo Sim Harmonic + VRX Parkur-1 simülasyon ortamı:

- Gazebo Sim Harmonic'de Sydney Regatta dünyasında IDA katamaran'ı spawn etmek
- Sensör verilerini (LiDAR, kamera, IMU, GPS) ROS 2 topic'lerine aktarmak
- Otonomi yazılımının simülasyonda çalışabilmesi için MAVROS-compatible topic'lere dönüştürmek
- `/control/cmd_vel_safe` → Gazebo Thruster plugin'e diferansiyel thrust komutları göndermek
- Modelin su üzerinde **stabil, düz ve dengeli** durması

---

## 2. Mevcut Durum

### Çalışan / Tamamlanan

| Bileşen | Durum | Not |
|---------|-------|-----|
| Gazebo Sim Harmonic | ✅ | Fortress kaldırıldı, Harmonic kuruldu |
| VRX paketleri | ✅ | `vrx_gz`, `vrx_ros`, `vrx_gazebo`, `wamv_description`, `wamv_gazebo` build edildi |
| ros_gz | ✅ | Source'tan build edildi. `actuator_msgs`, `gps_msgs`, `vision_msgs` satırları CMakeLists.txt ve mappings.py'den çıkarıldı (Humble'da yok) |
| `ida_gazebo` paketi | ✅ | ament_cmake paketi, build edildi |
| Launch dosyası | ✅ | `vrx_parkur1_sim.launch.py` — tek launch'ta Gazebo + spawn + bridge + otonomi node'ları |
| Topic bridge | ✅ | `/scan`, `/camera/color/image_raw`, `/imu`, `/navsat` |
| `cmd_vel_to_thrust.py` | ✅ | `/control/cmd_vel_safe` → Gazebo Transport `cmd_thrust` (sol/sağ) |
| `sim_nav_converter.py` | ✅ | Sydney GPS → Türkiye offset, IMU yaw → pusula heading |
| `spawn_waypoint_markers.py` | ✅ | Mission `local_waypoints` noktalarını Gazebo'da renkli marker olarak spawn eder |
| Model SDF | ✅ | Yaklaşık 35kg toplam hedef, çift gövde, VRX Surface + SimpleHydrodynamics + Thruster plugin |
| COM stabilitesi | ✅ | COM TALAY varsayımına göre düşük tutuldu, model ters dönmüyor |
| Dikey damping | ✅ | `zW=88.23` ve artırılmış Surface kaldırması ile spawn sonrası batma düzeltildi |

### Bu Oturumda Yapılan Son Değişiklikler

1. **Spawn z = -0.05**: `auto_start=false` ile izole testte iyi sonuç verdi; ilk batıp çıkma/yalpa kabul edilebilir seviyeye indi.
2. **auto_start varsayılan kapalı**: VRX launch artık `auto_start:=false` ile açılıyor. Böylece fizik testi sırasında controller motor komutu üretmiyor.
3. **Yaw işareti düzeltildi**: Sola sürekli gitme fizik/momentum kaynaklı değil, controller-thrust yön zinciri kaynaklı çıktı. `cmd_vel_to_thrust.py` içine `yaw_sign=-1.0` eklendi; `auto_start=true` testinde tekne düz gitti.
4. **Thruster reaksiyon torku azaltıldı**: Thruster plugin'e `p_gain=0.0` eklendi. Motorlar çalışırken teknenin 360 derece dönüp su altına girmesi kayboldu.
5. **Komut timeout güvenliği eklendi**: `cmd_vel_to_thrust.py`, komut gelmezse periyodik sıfır thrust yayınlıyor.

### Bilinen Aktif Sorunlar

| Sorun | Açıklama | Önem |
|-------|----------|------|
| Pervane visual kaybolması | `velocity_control=false` ve `p_gain=0.0` modunda pervane render/spin gerçekçi olmayabilir | Düşük (fonksiyonelliği etkilemiyor) |
| Spawn yüksekliği hassas | `z=-0.05` iyi sonuç verdi, farklı world/dalga koşullarında tekrar doğrulanmalı | Düşük |
| GPS origin offset | Tekne spawn noktası ile `parkur1U2.json` origin'i henüz hizalanmadı | Orta |

---

## 3. Aktif Olarak Düzenlenen Dosyalar (Bu Oturumda)

```
src/ida_gazebo/launch/vrx_parkur1_sim.launch.py
  → `spawn_z` argümanı eklendi, varsayılan `-0.05`
  → `auto_start` argümanı eklendi, varsayılan `false`

src/ida_gazebo/models/ida_katamaran/model.sdf
  → Sağ propeller joint axis: <xyz>1 0 0</xyz> → <xyz>-1 0 0</xyz>
  → Thruster plugin'lere `p_gain=0.0` eklendi

src/ida_gazebo/scripts/cmd_vel_to_thrust.py
  → Sağ thrust ters işaretleme: right = -(v + w * wheelbase / 2.0)
  → `yaw_sign=-1.0` ile Gazebo yaw yönü düzeltildi
  → Komut timeout durumunda sıfır thrust publish ediliyor

src/ida_gazebo/scripts/spawn_waypoint_markers.py
  → `local_waypoints` noktalarını Gazebo'da yeşil/mavi/kırmızı marker olarak gösterir
```

---

## 4. Denenip Başarısız Olan / Öğrenilen Şeyler

### ❌ `velocity_control=true` modunda Thruster plugin
- **Sorun**: Model havaya fırlıyor, ardından DART/ODE constraint solver crash'i
- **Neden**: `velocity_control=true` modunda joint velocity constraint'leri ODE/DART ile çakışıyor
- **Çözüm**: `velocity_control=false` kullanılıyor

### ❌ PolyhedraBuoyancyDrag plugin
- **Sorun**: ODE collision detector'da abort/crash
- **Neden**: Modelin collision geometrileri ile ODE'nin Polyhedra tespiti uyumsuz
- **Çözüm**: Kaldırıldı, sadece VRX Surface plugin kullanılıyor

### ❌ Model ters dönüyordu
- **Sorun**: Spawn sonrası model ters çevriliyordu (180° roll)
- **Neden**: COM (z=0) visual/collision geometrilerinin üstünde kaldığı için yerçekimi modeli ters çeviriyordu
- **Çözüm**: COM z=0.15'e (float'ların içine) taşındı

### ❌ Spawn z'si yanlış hesaplanıyordu
- **Sorun**: `z=-0.3` modeli çok batık, `z=0.05` çok yüksek bırakıyordu
- **Neden**: `<link name="base_link"><pose>0 0 0 0 0 0</pose>` olduğu halde daha önce `z=0.3` sanılmıştı
- **Çözüm**: `z=-0.1` yapıldı (test edilmedi)

### ✅ Sola sürekli dönme (Root Cause)
- **Sorun**: Simülasyon başlar başlamaz tekne sola doğru dönüyordu
- **Neden**: İzole `auto_start=false` testinde sola kayma görülmedi. Sorun momentum değil, controller yaw komutunun Gazebo thrust yönünde ters yorumlanmasıydı.
- **Çözüm**: `cmd_vel_to_thrust.py` içine `yaw_sign=-1.0` eklendi. `auto_start=true` testinde tekne düz gitti.

### ✅ Motor çalışırken teknenin 360 derece dönmesi
- **Sorun**: Tekne ileri gidiyordu fakat motorlar dönerken gövde de dönüp su altına giriyordu.
- **Neden**: Thruster plugin `velocity_control=false` modunda pervane spin PID torkunu gövdeye aktarıyordu.
- **Çözüm**: Her iki Thruster plugin'e `p_gain=0.0` eklendi; thrust kuvveti korunurken spin reaksiyon torku bastırıldı.

---

## 5. Sonraki Adımlar

### Hemen Yapılması Gerekenler

1. **Build et ve test et**
   ```bash
   cd /home/arif/teknofest/ida
   colcon build --packages-select ida_gazebo
    ros2 launch ida_gazebo vrx_parkur1_sim.launch.py auto_start:=true spawn_z:=-0.05
   ```

2. **Spawn yüksekliğini doğrula**
   - Model suya düştükten sonra salınım yapıyor mu?
   - Float'ların batıklığı görsel olarak normal mi?
   - Varsayılan `spawn_z=-0.05` iyi sonuç verdi

3. **Sola dönme fix'ini doğrula**
   - Thrust sıfırken (hiç cmd_vel yayınlanmazken) tekne düz duruyor mu?
   - `cmd_vel` ile ileri komut verince tekne düz gidiyor mu?
   - Sola/sağa dönüş komutları düzgün çalışıyor mu?

### Kısa Vadede Yapılacaklar

4. **Otonomi node'larının simülasyonda davranışını test et**
   - `mission_manager` / `gps_guidance` doğru setpoint hesaplıyor mu?
   - `controller_node` heading error'u doğru yorumluyor mu?
   - Simülasyonda dönme komutları gerçekçi mi?

5. **Pervane visual bug'ını çöz**
   - `velocity_control=false` modunda pervane render'ı kayboluyor
   - Gazebo Sim Harmonic'de bu bilinen bir issue olabilir

6. **`.gitignore` güncelle**
    - `src/ros_gz/` ve `src/vrx/` ekle (source tree'deki değişiklikler tracking dışında kalmalı)

### Ertelenen Riskler / Sonraki Faz Planı

> Not: Aşağıdaki maddeler ilk hedef olan "stabil çalışan temel Gazebo simülasyonu" tamamlanana kadar bilinçli olarak ertelendi. Öncelik önce teknenin düzgün spawn olması, su üstünde dengeli kalması ve temel ileri/sağ/sol komutlara doğru tepki vermesi.

1. **GPS origin / spawn hizalama**
   - Şu an tekne Sydney dünyasında `x=-528.0`, `y=193.0` konumunda spawn ediliyor.
   - `sim_nav_converter.py` ise Sydney world origin'ini doğrudan Türkiye origin'ine map'liyor.
   - Sonraki fazda spawn noktası ile `parkur1U2.json` origin'i aynı mantıksal başlangıç noktasına denk gelecek şekilde GPS offset doğrulanmalı.

2. **Yaw ve thrust işaret doğrulaması**
   - `linear.x > 0`, `angular.z > 0`, `angular.z < 0` için teknenin gerçek Gazebo davranışı gözlenmeli.
   - Sentetik simülasyonlarda kullanılan heading artış yönü ile Gazebo/ENU yaw yönü aynı olmayabilir.
   - Gerekirse `cmd_vel_to_thrust.py` içindeki sol/sağ thrust işaretleri veya angular işareti tekrar ayarlanmalı.

3. **Gazebo Parkur-1 duba ortamı**
   - `parkur1U2.json` içinde boundary/obstacle dizilimi var; fakat VRX launch şu an bu objeleri Gazebo dünyasına spawn etmiyor.
   - Sonraki fazda JSON'daki boundary ve obstacle objeleri Gazebo model include/spawn mekanizmasına bağlanmalı veya özel bir world dosyası üretilmeli.

4. **Koridor / planner stack entegrasyonu**
   - Mevcut VRX launch temel zinciri başlatıyor: mission, guidance, controller, lidar_processor, safety.
   - Gerçek koridor takibi için `sensor_cross_validator_node`, `course_memory_node`, `semantic_buoy_classifier_node`, `corridor_tracker_node` ve `parkur2_planner_node` entegrasyonu ayrıca yapılmalı.

5. **Kamera ve depth eksikleri**
   - Modelde şu an RGB kamera var; RealSense benzeri depth image akışı yok.
   - Kamera Gazebo topic adları bridge tarafında doğrulanmalı.
   - Depth gerektiren perception zinciri için depth kamera, stereo/depth plugin veya LiDAR tabanlı range fallback netleştirilmeli.

6. **Safety ve thrust timeout iyileştirmesi**
   - `cmd_vel_to_thrust.py` sadece yeni `/control/cmd_vel_safe` mesajı geldiğinde Gazebo thrust publish ediyor.
   - Sonraki fazda komut timeout durumunda ve node kapanırken sıfır thrust publish eden ek güvenlik davranışı eklenmeli.

7. **Sim time ve tekrar üretilebilir ortam**
   - `/clock` bridge ediliyor fakat otonomi node'larında `use_sim_time` genel olarak ayarlı değil.
   - Gazebo pause/slow/fast testlerinde timeout davranışları için sim time stratejisi netleştirilmeli.
   - `src/ros_gz/` ve `src/vrx/` git dışında olduğu için kullanılan branch/commit ve lokal patch'ler ayrıca dokümante edilmeli.

8. **Fizik tuning**
   - Kütle, inertia, hydrodynamics, thrust coefficient ve joint damping değerleri şu an stabilite odaklı.
   - Temel simülasyon stabil olduktan sonra hızlanma, dönme, durma ve dalga tepkisi daha gerçekçi olacak şekilde kalibre edilmeli.

### Parkur-1U2 → Parkur-2 → SITL Yol Haritası

1. **Parkur-1U2 ilk hedef**
   - Yarışmaya hazırlıkta ilk çalışır hedef `parkur1U2.json` olmalı.
   - Bu parkur, şartnameye göre hazırlanan Parkur-2'ye göre daha geniş ve daha toleranslı bir başlangıç testi sağlar.
   - Başarı kriteri: sadece waypoint bilgisi verildiğinde teknenin zikzak hattı otonom takip etmesi ve engellerden kaçınması.

2. **Spawn / GPS / heading hizalama**
   - Gazebo spawn noktası mission origin kabul edilmeli.
   - Spawn anında `/mavros/global_position/global`, `parkur1U2.json` içindeki ilk waypoint ile aynı olmalı.
   - Başlangıç heading'i Parkur-1U2 ilk segmentine yakın olmalı. `local_waypoints[1] = (east=11.71, north=8.90)` yaklaşık `53°` pusula heading verir.
   - Uygulandı: VRX launch varsayılanları `spawn_x=-528.0`, `spawn_y=193.0`, `spawn_z=-0.05`, `spawn_yaw=0.646`.
   - Uygulandı: `sim_nav_converter.py`, spawn world XY noktasını Türkiye mission origin'e map'liyor.

3. **Parkur objelerini Gazebo'ya taşıma**
   - `parkur1U2.json` içindeki `boundaries` ve `obstacles` Gazebo dünyasına spawn edilmeli.
   - İlk aşamada gerçekçi dalga/yüzdürme yerine sabit collision/visual duba modelleri yeterli.
   - Amaç önce kontrol, planner ve LiDAR/algı zincirini doğrulamak.

4. **Ground-truth detection ara katmanı**
   - İlk başarılı test için JSON duba haritasından tekne pozisyonuna göre `/perception/buoy_detections_raw` üretilebilir.
   - Bu ara katman nihai perception değildir; planner/controller/Gazebo fiziğini izole test etmek içindir.
   - Daha sonra LiDAR cluster, kamera veya YOLO tabanlı gerçek algıya geçilebilir.

5. **Planner/corridor stack entegrasyonu**
   - VRX launch'a `sensor_cross_validator_node`, `course_memory_node`, `semantic_buoy_classifier_node`, `corridor_tracker_node` ve `parkur2_planner_node` eklenmeli.
   - Parkur-1U2 başarıyla çalıştıktan sonra aynı zincir `parkur2.json` üzerinde denenmeli.
   - Uygulandı: Controller, `guidance/status` üzerinden yaklaşan dönüş açısını okuyup keskin dönüşlerde waypoint'e gelmeden önce yavaşlama ve bir sonraki segmente doğru pre-turn blend yapabiliyor.

6. **Parkur-2 şartname genişliği**
   - Kullanıcı notu: Parkur-2, şartnamedeki parkur dizilimini kopyalamaya çalışır; iki kenar duba arası yüzeyden yüzeye genişlik `8.12 m` hedeflenmiştir.
   - Duba yarıçapı `0.15 m` ise merkezden merkeze genişlik `8.42 m` olur.
   - Planner/corridor parametreleri merkez koordinat mı yüzey genişliği mi kullandığına göre netleştirilmeli.

7. **ArduPilot SITL'e geçiş**
   - Gazebo fizik + mevcut otonomi ile Parkur-1U2 ve Parkur-2 çalışmadan SITL'e geçilmemeli.
   - SITL ilk aşamada sadece MAVROS/ArduPilot komut zinciri, mode ve actuator mapping doğrulaması için kullanılmalı.
   - Son aşamada `/control/cmd_vel_safe` → MAVROS bridge → ArduPilot SITL → motor/mixer zinciri kapalı çevrim denenmeli.

### Orta Vadede Yapılacaklar

7. **ros_gz değişikliklerinin kalıcılığını kontrol et**
   - CMakeLists.txt ve mappings.py'deki `actuator_msgs`, `gps_msgs`, `vision_msgs` çıkarmaları
   - Bir `ros_gz` güncellemesinde üzerine yazılabilir

8. **SimpleHydrodynamics parametrelerini tune et**
   - Drag katsayıları (`xU`, `xUU`, `nR`, `nRR` vb.) gerçekçi mi?
   - Tekne hareketleri (hızlanma, dönme, durma) gerçekçi mi?

9. **Alternatif thruster yaklaşımı değerlendir**
   - Eğer `velocity_control=false` ile visual veya stabilite sorunları devam ederse, `libcustom_thruster.so` gibi custom bir plugin yazılabilir
   - Veya Gazebo'nun `ApplyLinkWrench` sistemini kullanarak doğrudan base_link'e kuvvet/torque uygulanabilir

---

## Hızlı Referans

### Launch
```bash
ros2 launch ida_gazebo vrx_parkur1_sim.launch.py
```

### Önemli Topic'ler
| Gazebo Topic | ROS 2 Topic | Açıklama |
|-------------|-------------|----------|
| `/model/ida_katamaran/scan` | `/scan` | LiDAR |
| `/model/ida_katamaran/camera/color/image_raw` | `/camera/color/image_raw` | Kamera |
| `/model/ida_katamaran/imu` | `/imu` | IMU |
| `/model/ida_katamaran/navsat` | `/navsat` | GPS |
| — | `/control/cmd_vel` | `controller_node` çıkışı |
| — | `/control/cmd_vel_safe` | `safety_node` çıkışı (cmd_vel_to_thrust.py bunu dinliyor) |
| `/model/ida_katamaran/joint/left_prop_joint/cmd_thrust` | — | Sol thrust (Gazebo Transport) |
| `/model/ida_katamaran/joint/right_prop_joint/cmd_thrust` | — | Sağ thrust (Gazebo Transport) |

### Spawn Koordinatları
- Dünya: Sydney Regatta
- x: `-528.0`, y: `193.0`, z: `0.0`, Y: `0.646 rad` (Parkur-1U2 ilk segment heading ~53°)
- `sim_nav_converter.py`, bu spawn noktasını `40.1181, 26.4081` mission origin'e hizalar.

### Waypoint Markerları
- Varsayılan açık: `enable_waypoint_markers:=true`
- Renkler: ilk waypoint yeşil, ara waypointler mavi, son waypoint kırmızı.
- Marker konumları `mission_file` içindeki `local_waypoints` alanından ve `spawn_x/spawn_y` offsetinden hesaplanır.

### Model Kritik Parametreler
- Mass: `base_link=32.6kg`, ek housing/prop linkleriyle toplam yaklaşık `35kg`.
- Thruster: `thrust_coefficient=0.0008`, `propeller_diameter=0.10`, `velocity_control=false`, `p_gain=0.0`, `max_thrust=58.9N` (launch override, 6kgf efektif itki varsayımı).
- Surface plugin: `hull_length=1.11`, `hull_radius=0.115`, `fluid_density=1025.9`, noktalar z=0.0 (base_link frame).
- SimpleHydrodynamics: `xU=18.56`, `xUU=18.56`, `yV=36.45`, `yVV=10.93`, `zW=88.23`, `nR=8.0`, `nRR=4.0`.

### Parkur-1U2 Controller Ayarları
- Maksimum ileri hız: `0.20 m/s`
- Maksimum yaw hızı: `0.90 rad/s`
- Keskin dönüşte yerinde dönme eşiği: `45°`
- Stop-turn-go: `waypoint_stop_turn_enabled=true`, `waypoint_stop_turn_distance_m=1.1`, `waypoint_stop_turn_angle_deg=35.0`, `waypoint_stop_turn_align_error_deg=10.0`.
