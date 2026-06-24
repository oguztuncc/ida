# VRX Simülasyon Entegrasyon Planı

> Bu doküman, TEKNOFEST İDA otonomi yazılımına Gazebo Sim tabanlı VRX (Virtual RobotX) simülasyonunun entegrasyonu için adım adım yol haritasıdır. Mevcut `ida_otonom` paketine dokunmadan, yeni `ida_gazebo` paketi üzerinden ilerlenecektir.

---

## 1. Hedef ve Kapsam

### 1.1 Temel Hedef
Mevcut ROS 2 Humble tabanlı otonomi stack'ini (`ida_otonom`) fiziksel araca dokunmadan VRX simülasyon ortamında çalıştırabilmek.

### 1.2 Kapsam (MVP — Minimum Uygulanabilir Ürün)
- **Parkur-1:** GPS waypoint takibi + basit engelden kaçınma (LiDAR).
- **Parkur-2:** Duba koridoru + LiDAR tabanlı merkez takibi.
- **Parkur-3:** Renkli hedef bulma (kamera + LiDAR cross-validation).

### 1.3 Dışarıda Bırakılanlar (Sonraki Aşamalar)
- ArduPilot SITL entegrasyonu (ileriki sprint).
- YOLO ağırlıklarının simülasyonda testi (opsiyonel, model hazır olduğunda).
- Gerçekçi dalga/hava durumu (VRX'in varsayılan dalgası yeterli).

---

## 2. Mimari Özet

```text
┌─────────────────────────────────────────────────────────────┐
│  VRX Gazebo Sim Dünyası (su, dalga, dubalar, engeller)      │
│  ┌──────────────┐  ┌──────────────┐  ┌──────────────────┐  │
│  │  IDA Robot   │  │  Sensörler   │  │  Diğer Objeler │  │
│  │  (Katamaran) │  │  LiDAR, Cam  │  │  (Dubalar vb.) │  │
│  └──────┬───────┘  └──────┬───────┘  └──────────────────┘  │
│         │                 │                                  │
│  ┌──────▼─────────────────▼──────┐                           │
│  │  ros_gz_bridge / VRX Topics   │                           │
│  └──────────────┬────────────────┘                           │
└─────────────────┼─────────────────────────────────────────────┘
                  │
                  ▼
┌─────────────────────────────────────────────────────────────┐
│  ida_gazebo paketi (YENİ)                                   │
│  ├── launch/vrx_parkur1_sim.launch.py                       │
│  ├── launch/vrx_parkur2_sim.launch.py                       │
│  ├── launch/vrx_parkur3_sim.launch.py                       │
│  ├── config/vrx_bridge.yaml       # topic map               │
│  ├── models/ida_katamaran/        # SDF/URDF + mesh         │
│  └── worlds/ida_parkur1.world     # VRX tabanlı dünya      │
└─────────────────┬─────────────────────────────────────────────┘
                  │
                  ▼
┌─────────────────────────────────────────────────────────────┐
│  ida_otonom paketi (MEVCUT — DEĞİŞMİYOR)                    │
│  ├── mission_manager_node                                   │
│  ├── gps_guidance_node                                      │
│  ├── controller_node                                        │
│  ├── lidar_processor_node                                   │
│  ├── parkur2_planner_node                                   │
│  ├── parkur3_planner_node                                   │
│  ├── safety_node                                            │
│  └── mavros_bridge_node       # Simülasyonda devre dışı    │
└─────────────────────────────────────────────────────────────┘
```

> **Önemli Prensip:** `ida_otonom` kaynak kodlarına dokunulmayacak. Sadece launch parametreleri ve topic isimleri `ida_gazebo` katmanında uygun şekilde yönlendirilecek.

---

## 3. Ön Koşullar ve Bağımlılıklar

### 3.1 Sistem Gereksinimleri
- Ubuntu 22.04 (Jammy)
- ROS 2 Humble (kurulu ve `source /opt/ros/humble/setup.bash` aktif)
- `colcon` build aracı
- Gazebo Sim (Harmonic veya Fortress — VRX Humble için genellikle Harmonic önerilir)

### 3.2 ROS 2 Paket Bağımlılıkları
- `ros_gz_sim`
- `ros_gz_bridge`
- `vrx_gz` (VRX'in Gazebo Sim paketi)
- `xacro` (URDF/SDF modeller için)
- `robot_state_publisher`

### 3.3 Donanım Notu
- Geliştirme x86_64 PC üzerinde yapılacak.
- Jetson Orin Nano'da VRX çalıştırılmayacak (GUI ve GPU gereksinimi yüksek).
- Simülasyon sırasında ROS 2 topic'leri ağ üzerinden Jetson'a da yönlendirilebilir, ancak ilk aşamada tamamen yerel çalışacak.

---

## 4. Uygulama Adımları

### Adım 0: Çalışma Dizini ve Paket İskeleti (30 dk)

```bash
cd /home/arif/teknofest/ida/src

# ida_gazebo paketini oluştur (CMake — Gazebo plugin ve modeller için)
ros2 pkg create --build-type ament_cmake ida_gazebo \
  --dependencies rclcpp rclpy std_msgs geometry_msgs sensor_msgs nav_msgs \
  ros_gz_sim ros_gz_bridge xacro robot_state_publisher

# Dizin yapısını oluştur
mkdir -p ida_gazebo/{launch,config,models,worlds,urdf,meshes}
```

### Adım 1: VRX ve Gazebo Sim Kurulumu (1-2 saat)

1. Gazebo Sim Harmonic kurulumunu doğrula:
   ```bash
   gz sim --version
   ```
2. VRX depolarını `src/` altına clone et:
   ```bash
   cd /home/arif/teknofest/ida/src
   git clone https://github.com/osrf/vrx.git
   ```
3. Workspace'i build et:
   ```bash
   cd /home/arif/teknofest/ida
   colcon build --symlink-install --packages-select ida_gazebo vrx_gz
   source install/setup.bash
   ```
4. VRX örnek launch'ı test et:
   ```bash
   ros2 launch vrx_gz competition.launch.py world:=sydney_regatta
   ```

**Kriter:** Gazebo penceresi açılıyor, WAM-V su üzerinde görünüyor ve `/vrx/...` topic'leri listeleniyor.

### Adım 2: IDA Katamaran Modelinin VRX'e Adaptasyonu (3-4 saat)

Mevcut fiziksel araç özellikleri:
- Boyut: ~80 × 110 × 46 cm
- İtki: Arkada 2 adet Degz Ultras su altı itici (diferansiyel)
- Ağırlık: ~50 kg
- Sensörler: RPLiDAR S3 (üstte), Intel RealSense D456 (önde)

Yapılacaklar:
1. **WAM-V URDF/SDF'sini kopyala:** VRX'in WAM-V modelini `ida_gazebo/models/ida_katamaran/` altına kopyala.
2. **Boyutları ölçekle:** WAM-V yaklaşık 1.5m boyutlarındadır; bizimki 80cm. `scale` parametresi veya mesh ölçeklendirme ile uyarla.
3. **İtki konfigürasyonu:** VRX'teki iki jet thruster'ı (sollu-sağlı) bizim diferansiyel itki sistemimize uyarla. Thruster plugin'inin `max_force` değerini fiziksel iticilerimize göre ayarla.
4. **Sensör yerleşimi:**
   - LiDAR: Üst platform, ~30cm yükseklik, 360° scan.
   - Kamera: Ön yüzeye bakan, ~20cm yükseklik.
   - GPS/IMU: VRX'in kendi plugin'leri kullanılacak; konumunu araç merkezine ayarla.
5. **Kütle ve kütle merkezi:** Yaklaşık 50 kg, katamaran stabilitesi için düşük CG.

**Çıktı:** `models/ida_katamaran/model.sdf` ve `model.config`.

### Adım 3: Topic Mapping ve Bridge Yapılandırması (2-3 saat)

VRX/Gazebo topic'leri ile `ida_otonom`'un beklediği topic'leri eşleştir. `ida_gazebo/config/vrx_bridge.yaml` içinde:

| Gazebo/VRX Topic | ROS 2 Topic (ida_otonom Beklentisi) | Not |
|------------------|-------------------------------------|-----|
`/model/ida_katamaran/scan` | `/scan` | LiDAR (sensor_msgs/LaserScan) |
`/model/ida_katamaran/camera/image` | `/camera/color/image_raw` | Kamera (sensor_msgs/Image) |
`/model/ida_katamaran/camera/camera_info` | `/camera/color/camera_info` | Kamera kalibrasyonu |
`/model/ida_katamaran/imu` | `/mavros/imu/data` veya özel topic | IMU (sensor_msgs/Imu) |
`/model/ida_katamaran/navsat` | `/mavros/global_position/global` | GPS (sensor_msgs/NavSatFix) |
`/model/ida_katamaran/compass` | `/mavros/global_position/compass_hdg` | Pusula (std_msgs/Float32) |
`/cmd_vel` | `/model/ida_katamaran/cmd_vel` | Kontrol girişi (Twist) |

> **Not:** VRX'in kendi ROS 2 topic isimleri farklı olabilir. Gerçek isimler `ros2 topic list` ile doğrulanacak ve yaml güncellenecektir.

`vrx_bridge.yaml` örnek yapısı:
```yaml
# Gazebo Sim -> ROS 2
- topic: /model/ida_katamaran/scan
  type: sensor_msgs/LaserScan
  direction: GZ_TO_ROS

# ROS 2 -> Gazebo Sim
- topic: /cmd_vel
  type: geometry_msgs/Twist
  direction: ROS_TO_GZ
```

### Adım 4: ida_otonom Entegrasyon Launch'ları (2-3 saat)

Her parkur için ayrı launch dosyası:

**`launch/vrx_parkur1_sim.launch.py`** görevleri:
1. VRX dünyasını başlat (`ros_gz_sim` ile).
2. IDA katamaran modelini spawn et.
3. `ros_gz_bridge`'i `config/vrx_bridge.yaml` ile başlat.
4. `robot_state_publisher`'ı başlat.
5. `ida_otonom`'un ilgili node'larını başlat:
   - `mission_manager_node` (görev dosyası: `missions/parkur1.json`)
   - `gps_guidance_node`
   - `controller_node`
   - `lidar_processor_node`
   - `safety_node` (kill switch simülasyonda yazılımsal çalışacak)
6. **NOT:** `mavros_bridge_node` bu launch'ta **olmayacak** çünkü VRX'te MAVROS yok; kontrol doğrudan `cmd_vel` ile Gazebo'ya gidecek.

Parametre yönetimi:
- `config/vrx_parkur1_sim.yaml`: Simülasyona özel hız/safety limitleri (gerçek araçtan daha yüksek olabilir, test hızlandırmak için).

### Adım 5: Parkur Dünyalarının Oluşturulması (2-3 saat)

VRX'teki Sydney Regatta veya benzeri dünya temelli, kendi objelerimizi ekleyerek:

- **Parkur-1:** Açık su, 2-3 GPS waypoint, kenarlarda bariyer dubalar (görsel referans).
- **Parkur-2:** İki sıra renkli duba koridoru (~10-15m uzunluğunda). VRX'teki primitive box/sphere objeleri ile renklendirilmiş duba modelleri.
- **Parkur-3:** Parkur-2 + ortada hedef renkli duba + dağılmış engeller.

Dünya dosyaları: `worlds/ida_parkur1.world`, `ida_parkur2.world`, `ida_parkur3.world`.

Duba renkleri: Kırmızı, Yeşil, Sarı, Mavi (VRX SDF'sinde `<material><diffuse>` ile ayarlanır).

### Adım 6: Test ve Doğrulama (2-3 saat)

Her parkur için test senaryosu:

#### Parkur-1 Testleri
- [ ] Launch dosyası hatasız başlıyor.
- [ ] GPS waypoint'lerine doğru yöneliyor.
- [ ] Waypoint'e ulaştığında bir sonrakine geçiyor.
- [ ] Kill switch (`/safety/kill`) aktif olduğunda tekne duruyor.
- [ ] LiDAR `/scan`'de görünüyor ve `lidar_processor_node` çalışıyor.

#### Parkur-2 Testleri
- [ ] Dubalar `/scan`'de görünüyor.
- [ ] `corridor_tracker_node` koridor merkezini hesaplıyor.
- [ ] `parkur2_planner_node` state machine (CRUISE → PASS_COMMITTED → RETURN_TO_CENTER) çalışıyor.
- [ ] Tekne dubalara çarpmadan ilerliyor.

#### Parkur-3 Testleri
- [ ] Kamera görüntüsü `/camera/color/image_raw`'de görünüyor.
- [ ] Hedef renkli duba görsel olarak tespit edilebiliyor.
- [ ] Renkli duba etrafında dönüş manevrası yapılıyor.

---

## 5. Zaman Çizelgesi

| Gün | Görev | Tahmini Süre | Çıktı |
|-----|-------|--------------|-------|
| **Gün 1** | Adım 0 + Adım 1 | 2-3 saat | VRX kurulu, örnek WAM-V çalışıyor |
| **Gün 1** | Adım 2 (Model adaptasyonu) | 3-4 saat | `ida_katamaran` modeli SDF olarak hazır |
| **Gün 2** | Adım 3 (Topic mapping) | 2-3 saat | `vrx_bridge.yaml` hazır, topic'ler akıyor |
| **Gün 2** | Adım 4 (Launch entegrasyonu) | 2-3 saat | `vrx_parkur1_sim.launch.py` çalışıyor |
| **Gün 3** | Adım 5 (Parkur dünyaları) | 2-3 saat | 3 parkur dünyası hazır |
| **Gün 3** | Adım 6 (Test) | 2-3 saat | Parkur-1 başarıyla tamamlanıyor |
| **Gün 4** | Parkur-2/3 test ve debug | 3-4 saat | Tüm parkurlar simülasyonda çalışıyor |

**Toplam:** 3-4 gün (günde 3-4 saat aktif çalışma varsayımıyla)

---

## 6. Riskler ve Çıkmaz Sokaklar (Troubleshooting Rehberi)

| Risk | Olasılık | Çözüm Önerisi |
|------|----------|---------------|
| **VRX topic isimleri beklenenden farklı** | Yüksek | `ros2 topic list` ile anlık kontrol; `vrx_bridge.yaml`'i dinamik tut, launch parametresi olarak geçilebilir yap. |
| **WAM-V itki sistemi bizim diferansiyel itkiyle uyuşmuyor** | Orta | VRX `<plugin filename="vrx::ThrusterPlugin">` yerine Gazebo'nun `DiffDrive` plugin'ini veya özel `cmd_vel` subscriber'ı kullan. |
| **VRX'te GPS koordinatları gerçek dünya değil, simülasyon koordinatı** | Orta | `gps_guidance_node`'a simülasyon modu eklenmesi gerekebilir (parametre ile `use_sim_gps:=true`). |
| **Kill switch `/safety/kill` Gazebo'da motorları durdurmuyor** | Orta | `safety_node`'un çıkışı (`/control/cmd_vel_safe`) Gazebo'ya gidiyorsa, `Twist` mesajını sıfırlamak yeterli; fiziksel relay olmadığından ek işlem gerekmez. |
| **Gazebo Sim GUI'si yavaş çalışıyor** | Orta | `--headless-rendering` veya `ogre2` render engine ayarları; veya sadece server modunda çalışıp RViz2 ile görselleştirme. |
| **Duba renkleri kamera görüntüsünde RealSense'ten farklı çıkıyor** | Düşük | Simülasyonda ideal RGB; gerçek kamera ile renk kalibrasyonu farklıdır. Simülasyonda renk eşiklerini (HSV) ideal değerlere göre ayarla, gerçek testte tekrar kalibre et. |
| **LiDAR scan açısı/düşey çözünürlük farklı** | Düşük | RPLiDAR S3 2D 360°'dir. VRX LiDAR plugin'i de 2D ayarlanabilir; `min_angle`, `max_angle`, `samples` parametrelerini eşleştir. |

---

## 7. İleriye Dönük Notlar

- **ArduPilot SITL Geçişi:** Bu plan tamamlandıktan sonra, `mavros_bridge_node`'un birebir testi için ArduPilot SITL + Gazebo Sim planı ayrı bir doküman olarak yazılacaktır.
- **YOLO Entegrasyonu:** `buoy_detector_node` simülasyonda test edilebilir; ancak simülasyondaki ideal renkli dubalar üzerinde eğitim yapılmamış modelin performansı düşük olabilir. Gerekirse simülasyona özel basit HSV filtre ile geçici çözüm üretilebilir.
- **Çoklu Araç:** VRX çoklu tekne desteği sunar. İleride İHA-İDA koordinasyonu test etmek istersen aynı yapı genişletilebilir.

---

## 8. Checklist — Bu Planı Bitirdikten Sonra

- [ ] `colcon build --packages-select ida_gazebo` hatasız tamamlanıyor.
- [ ] `ros2 launch ida_gazebo vrx_parkur1_sim.launch.py` Gazebo penceresi açılıyor.
- [ ] `/scan`, `/camera/color/image_raw`, `/mavros/global_position/global` topic'leri aktif.
- [ ] `mission_manager_node` görev dosyasını yüklüyor ve waypoint'leri sırayla takip ediyor.
- [ ] `safety_node` kill komutu aldığında tekne duruyor.
- [ ] En az Parkur-1 başarıyla simüle ediliyor.
