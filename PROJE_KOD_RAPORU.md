# TALAY İDA Yazılım Durum ve Kod Mimarisi Raporu

Bu rapor, TEKNOFEST İnsansız Deniz Aracı projesi için şu ana kadar repoda
oluşturulan yazılım altyapısının ne yaptığını, hangi parçaların ne anlama
geldiğini ve gerçek araç üzerinde hangi aşamaya kadar hazır olduğunu özetler.

Kodlar satır satır açıklanmamıştır. Amaç, yazılımın genel mimarisini ve her
node'un projedeki görevini ekip seviyesinde anlaşılır hale getirmektir.

## 1. Projenin Genel Amacı

Bu ROS 2 paketi, Jetson Orin Nano üzerinde çalışacak otonomi yazılımının temel
omurgasını oluşturur. Ana hedef, Jetson üzerinde çalışan ROS 2 node'larının
Pixhawk Cube Orange / ArduRover ile haberleşerek insansız deniz aracını
otonom şekilde görev noktalarına götürmesi, engellerden kaçınması, görev
durumunu kaydetmesi ve güvenlik katmanları üzerinden motor komutlarını
kontrol etmesidir.

Proje üç parkura göre düşünülmüştür:

- Parkur-1: GPS ve pusula tabanlı waypoint takibi.
- Parkur-2: Waypoint takibi yaparken lidar, kamera ve depth yardımıyla
  engellerden kaçınma.
- Parkur-3: İHA'dan gelen hedef rengi bilgisine göre doğru hedefe yönelme ve
  temas etme.

Şu an yazılımda en olgun kısım Parkur-1 omurgasıdır. Parkur-2 için lidar/costmap
ve planner altyapısı başlatılmıştır. Parkur-3 için hedef rengi bilgisinin alınacağı
haberleşme altyapısı hazırlanmıştır. Görüntü işleme modeli henüz olmadığı için
YOLO tabanlı gerçek algılama kısmı daha sonra eklenecek şekilde modüler
bırakılmıştır.

## 2. Fiziksel Araç Varsayımları

TYR raporu ve ekipten gelen bilgiler doğrultusunda yazılım şu fiziksel yapı için
tasarlanmıştır:

- Araç tipi: Katamaran, çift gövdeli İDA.
- Boyut: Yaklaşık 80 x 110 x 46 cm.
- Ağırlık: 50 kg altında.
- İtki sistemi: Arkada 2 adet Degz Ultras su altı itici.
- Sürüş tipi: Diferansiyel tahrik.
- Dümen: Mekanik dümen yok; sağ-sol itici farkı ile dönüş.
- ESC: Her motor için ayrı Flycolor waterproof ESC.
- Batarya: 6S 4P Li-ion Aspilsan 18650 paketleri.
- Ana bilgisayar: NVIDIA Jetson Orin Nano.
- Otopilot: Pixhawk Cube Orange / Cube Orange+.
- Kamera: Intel RealSense D456.
- Lidar: RPLiDAR S3.
- Yazılım altyapısı: ROS 2 Humble, ArduRover, MAVROS/MAVLink.

Bu bilgiler kodda doğrudan motorları çalıştıracak şekilde tehlikeli varsayımlara
dönüştürülmemiştir. Gerçek motor kontrolü halen güvenli tarafta kapalıdır.

## 3. Genel Yazılım Akışı

Sistemin genel çalışma zinciri şöyledir:

```text
Görev dosyası / YKİ komutu
        ↓
Mission manager
        ↓
GPS guidance
        ↓
Planner / controller
        ↓
Safety layer
        ↓
MAVROS bridge
        ↓
Pixhawk
        ↓
ESC
        ↓
Motorlar
```

Bu zincirde Jetson üzerindeki ROS 2 yazılımı karar üretir. Pixhawk ise araç
kontrolü ve motor çıkışlarının fiziksel sisteme taşınmasında kullanılır.

## 4. Paket Yapısı

Ana ROS paketi:

```text
src/ida_otonom
```

Önemli klasörler:

- `ida_otonom/`: ROS 2 Python node'ları.
- `launch/`: Simülasyon ve gerçek araç launch dosyaları.
- `config/`: Gerçek araç parametreleri.
- `ida_otonom/missions/`: Örnek görev dosyası.
- `README.md`: Build, launch ve güvenlik notları.

## 5. Ortak Yardımcı Kodlar

### `common.py`

Bu dosya, birçok node tarafından kullanılan ortak yardımcı fonksiyonları içerir.

Başlıca görevleri:

- Açıyı -180 / +180 derece aralığına normalize etmek.
- Değerleri güvenli min/max aralığında sınırlamak.
- İki GPS koordinatı arasında mesafe hesaplamak.
- Bir noktadan diğerine hedef yön/bearing hesaplamak.
- JSON mesajlarını üretmek ve okumak.
- Mission dosyasının path'ini taşınabilir şekilde çözmek.
- Kayıt klasörünün varsayılan konumunu belirlemek.

Bu dosya sayesinde mission dosyası artık bilgisayara hardcoded değildir. Kod,
paket share path üzerinden veya launch parametresiyle verilen dosyadan görevi
okuyabilir.

## 6. Görev Yönetimi

### `mission_manager_node.py`

Bu node, aracın görev durumunu yönetir.

Yaptıkları:

- JSON mission dosyasından waypoint listesini okur.
- Aktif waypoint indeksini yayınlar.
- Görev yüklendi mi, başladı mı, bitti mi bilgisini yayınlar.
- `/mission/start` komutu gelmeden gerçek görev moduna geçmez.
- Waypoint tamamlandığında bir sonraki waypoint'e geçer.
- Son waypoint tamamlandığında görevi bitmiş kabul eder.

Önemli güvenlik davranışı:

Gerçek kullanımda görev otomatik başlamaz. YKİ veya test komutu üzerinden
`/mission/start` verilmesi gerekir. Simülasyon launch dosyasında ise test kolaylığı
için `auto_start` açılmıştır.

## 7. GPS Tabanlı Yönlendirme

### `gps_guidance_node.py`

Bu node, aracın mevcut GPS konumu ile aktif waypoint arasındaki ilişkiyi hesaplar.

Yaptıkları:

- Pixhawk/MAVROS üzerinden GPS konumunu dinler.
- Aktif waypoint bilgisini mission manager'dan alır.
- Hedefe kalan mesafeyi hesaplar.
- Hedefe gitmek için gereken bearing açısını hesaplar.
- Araç hedef waypoint'e yeterince yaklaşınca advance sinyali üretir.

Parkur-1 için ana navigasyon verisi bu node'dan gelir.

## 8. Kontrol Katmanı

### `controller_node.py`

Bu node, hedef yön ile mevcut heading arasındaki hataya göre hareket komutu
üretir.

Yaptıkları:

- Mevcut heading bilgisini dinler.
- Hedef bearing bilgisini dinler.
- Heading hatasına göre dönüş komutu üretir.
- Hata küçükse daha hızlı, hata büyükse daha yavaş ilerleme komutu verir.
- `/control/cmd_vel` topic'ine ileri hız ve dönüş komutu yayınlar.

Önemli güvenlik davranışları:

- Görev başlamadıysa sıfır motor komutu yayınlar.
- Görev bittiyse sıfır motor komutu yayınlar.
- Bu nedenle sistem yanlışlıkla ayağa kalksa bile start gelmeden motor komutu
  üretmez.

## 9. Simülasyon

### `sim_gps_node.py`

Bu node, gerçek GPS ve pusula yokken basit bir simülasyon ortamı sağlar.

Yaptıkları:

- Sahte GPS konumu yayınlar.
- Sahte heading/pusula bilgisi yayınlar.
- Controller'dan gelen hız ve dönüş komutuna göre sanal tekneyi hareket ettirir.

Bu node sayesinde Parkur-1 waypoint takibi masa başında test edilebilir.

## 10. Parkur-1 Launch Dosyası

### `parkur1_sim.launch.py`

Bu launch dosyası, Parkur-1 simülasyonunu tek komutla ayağa kaldırır.

Çalıştırdığı ana parçalar:

- `sim_gps_node`
- `mission_manager_node`
- `gps_guidance_node`
- `controller_node`
- `safety_node`
- `logger_node`
- İsteğe bağlı `yki_bridge_node`
- İsteğe bağlı `local_costmap_node`

Simülasyon ortamında görev otomatik başlar. Gerçek araçta ise bu davranış kapalıdır.

## 11. Güvenlik Katmanı

### `safety_node.py`

Bu node, motor komutlarının gerçek sisteme gitmeden önce geçtiği güvenlik
kapısıdır.

Yaptıkları:

- Controller'dan gelen `/control/cmd_vel` komutunu dinler.
- Kill aktif değilse komutu `/control/cmd_vel_safe` olarak geçirir.
- Kill aktifse sıfır komut yayınlar.
- Kill bilgisi latch'lidir.

Latch'li kill şu anlama gelir:

Bir kere kill gelirse sistem resetlenene kadar güvenli modda kalır. Böylece
farklı kaynaklardan gelen yanlışlıkla `false` mesajı gerçek kill durumunu söndüremez.

Bu davranış gerçek araç için önemlidir çünkü acil durdurma komutu asla kolayca
geri alınmamalıdır.

### `rc_kill_node.py`

Bu node, Pixhawk/MAVROS üzerinden gelen RC kanal bilgisinden kill üretir.

Varsayılan kabul:

- RC kill kanalı: Channel 7.
- PWM eşiği: 1800.
- RC sinyali kaybolursa güvenli olmak için kill aktif kabul edilir.

Bu değerler gerçek kumanda ayarına göre değiştirilebilir.

### `power_relay_node.py`

Bu node, ileride Jetson GPIO üzerinden SSR/röle/kontaktör zincirini kontrol etmek
için hazırlanmıştır.

Önemli nokta:

Varsayılan olarak kapalıdır. GPIO pini ve elektriksel mantık doğrulanmadan gerçek
24V motor gücünü açıp kapatmaz.

Bu node'un amacı, şartnamedeki "sadece motor sinyalini kesmek yetmez, motor
gücünün kesilmesi gerekir" gereksinimine yazılım tarafında bir entegrasyon noktası
oluşturmaktır.

## 12. MAVROS / Pixhawk Köprüsü

### `mavros_bridge_node.py`

Bu node, ROS 2 hareket komutunu Pixhawk/MAVROS tarafına göndermek için
hazırlanmıştır.

Yaptıkları:

- `/control/cmd_vel_safe` topic'ini dinler.
- Komut timeout, kill ve görev bitiş durumlarını kontrol eder.
- Komutları hız ve dönüş limitleri içinde sınırlar.
- MAVROS'a komut göndermeye hazır yapı sunar.

Önemli güvenlik kararı:

Bu node varsayılan olarak gerçek komut yayınlamaz. `enabled:=true` verilmeden
Pixhawk'a motor komutu gitmez.

Bunun sebebi, şu bilgilerin halen kesinleşmemiş olmasıdır:

- UART baudrate.
- ESC PWM nötr / ileri / geri değerleri.
- Pixhawk çıkış kanal eşleşmeleri.
- MAVROS kontrol yöntemi.
- Gerçek kill zinciri testi.

Bu bilgiler netleşmeden gerçek motor komutunu açmak tehlikelidir.

## 13. YKİ Köprüsü

### `yki_bridge_node.py`

Bu node, Yer Kontrol İstasyonu ile UDP tabanlı haberleşme için temel köprüdür.

Yaptıkları:

- Konum bilgisini YKİ'ye yollar.
- Aktif waypoint bilgisini yollar.
- Görev başladı/bitti durumunu yollar.
- Guidance ve mission status bilgisini yollar.
- İsteğe bağlı olarak YKİ'den komut alabilir.

Desteklenen YKİ komutları:

- `start_mission`
- `kill`
- `reset_kill`
- `set_target_color`

Şartnameye uygun önemli davranış:

Görev başladıktan sonra YKİ'den gelen kill dışındaki komutlar yok sayılır. Bu,
"görev başladıktan sonra dışarıdan komut verilmemesi" kuralına uyum için
tasarlanmıştır.

## 14. Kayıt ve Teslim Altyapısı

### `logger_node.py`

Bu node yarışma sonrası teslim edilecek telemetri CSV altyapısını oluşturur.

Kaydettiği başlıca bilgiler:

- Zaman etiketi.
- Konum: lat, lon.
- Yer hızı.
- Roll, pitch, heading.
- Hedef bearing.
- Hedef mesafe.
- Hız setpoint'i.
- Yaw/dönüş setpoint'i.

Şartname gereği telemetri verisinin en az 1 Hz kayıt altına alınması gerekir. Bu
node şu an 1 Hz periyotta CSV kaydı yapar.

### `perception_node.py`

Bu node şu an geçici ve basit görüntü işleme altyapısıdır.

Yaptıkları:

- Kameradan görüntü alır.
- Basit çizgi/kenar işleme dener.
- İşlenmiş video kaydı oluşturur.
- Controller'a küçük bir heading bias önerisi gönderebilir.

Bu node nihai YOLO/RealSense algılama sistemi değildir. Model ve gerçek dataset
geldiğinde bu alan büyük ölçüde değiştirilecektir.

### `local_costmap_node.py`

Bu node lidar verisinden lokal engel haritası üretir.

Yaptıkları:

- `/scan` topic'inden RPLiDAR verisini dinler.
- Tekne çevresindeki lokal grid haritasını oluşturur.
- Engel görülen hücreleri işaretler.
- `/local_costmap` yayınlar.
- CSV olarak costmap kaydı tutar.

Bu, Parkur-2 için hem engelden kaçınma hem de yarışma sonrası veri teslimi
açısından önemlidir.

## 15. Lidar ve Sensör Füzyonu

### `lidar_processor_node.py`

Bu node, lidar scan verisinden basit bir engel özeti üretir.

Yaptıkları:

- Ön, sol ve sağ açıklıkları hesaplar.
- Ön tarafta çarpışma riski var mı belirler.
- En açık yönü tahmin eder.

Bu node, Parkur-2 planner için ilk karar verici sensör özetini sağlar.

### `sensor_fusion_node.py`

Bu node, algılama ve lidar verilerini birleştirmek için başlangıç altyapısıdır.

Yaptıkları:

- Kamera/model tespitlerini dinler.
- Lidar özetini dinler.
- Potansiyel geçit veya dünya durumu mesajı üretir.

Şu an model olmadığı için bu katman tam kullanılmamaktadır. Ama YOLO modeli
geldiğinde kamera + lidar birleşimi burada daha anlamlı hale getirilecektir.

## 16. Parkur-2 İçin Mevcut Durum

Parkur-2'nin hedefi, engel bulunan ortamda waypoint takibi yapmaktır.

Şu an hazır olanlar:

- Lidar `/scan` verisini okuyacak altyapı.
- Ön/sol/sağ engel özeti.
- Lokal costmap üretimi.
- Costmap CSV kaydı.
- Güvenli hız ve engel mesafesi parametreleri için temel yapı.

Henüz yazılması gereken ana parça:

- Gerçek `local_planner_node`.

Bu planner şu işi yapacaktır:

- GPS hedef yönünü alacak.
- Lidar engellerini değerlendirecek.
- Gerekirse waypoint'e direkt gitmek yerine sağ/sol güvenli kaçış yönü seçecek.
- Hız limitini engel mesafesine göre düşürecek.
- Engel geçilince tekrar waypoint hattına dönecek.

Model yokken Parkur-2 lidar-only çalışacak şekilde tasarlanmalıdır. Model geldiğinde
YOLO tespitleri de planner'a eklenecektir.

## 17. Görüntü İşleme Planı

Görüntü işleme sistemde üç amaçla kullanılacaktır:

1. Parkur-2'de duba ve engelleri sınıflandırmak.
2. Parkur-2'de geçit/duba çiftlerini anlamaya yardımcı olmak.
3. Parkur-3'te doğru hedef rengindeki dubayı seçmek.

Tasarım kararı:

- Lidar ana güvenlik sensörüdür.
- RealSense RGB/depth yardımcı algılama sensörüdür.
- YOLO modeli duba/hedef sınıflandırması için kullanılacaktır.
- GPS genel rota hedefini verir.
- Planner tüm bu bilgileri birleştirip son hareket kararını verir.

Model henüz yoktur. Bu yüzden kodlar model sonradan takılacak şekilde yazılmalıdır.
Model yokken sistem çökmeden lidar-only güvenli moda düşmelidir.

## 18. Gerçek Araç Launch Dosyası

### `ida_real.launch.py`

Bu launch dosyası gerçek araç üzerinde çalışacak ana node'ları ayağa kaldırmak
için hazırlanmıştır.

Çalıştırdığı ana parçalar:

- Mission manager.
- GPS guidance.
- Controller.
- Safety node.
- RC kill node.
- Power relay node.
- MAVROS bridge.
- Logger.
- Local costmap node.
- İsteğe bağlı perception node.
- İsteğe bağlı YKİ bridge.

Gerçek araç launch dosyasında motor komutu varsayılan olarak kapalıdır.

## 19. Gerçek Araç Config Dosyası

### `ida_real.yaml`

Bu dosya gerçek araç parametrelerini tutar.

İçindeki önemli kararlar:

- Görev otomatik başlamaz.
- Controller hız limiti düşük tutulmuştur.
- Safety kill latch'lidir.
- MAVROS bridge disabled gelir.
- RC kill channel 7 olarak varsayılmıştır.
- Power relay disabled gelir.
- Costmap 1 Hz kayıt yapar.

Bu dosya gerçek testler ilerledikçe güncellenecek ana ayar dosyasıdır.

## 20. Şu An Reel Ortamda Ne Test Edilebilir?

Motorlar açılmadan güvenli şekilde test edilebilecekler:

- Jetson üzerinde ROS 2 node'larının çalışması.
- Pixhawk'tan GPS verisi gelip gelmediği.
- Pixhawk'tan heading verisi gelip gelmediği.
- Mission dosyasının okunması.
- `/mission/start` akışının çalışması.
- Controller'ın komut üretmesi.
- Safety node'un kill durumunda sıfır komut üretmesi.
- RC kill kanalının okunması.
- YKİ'ye UDP telemetri gönderilmesi.
- YKİ'den kill/start komutu alınması.
- Lidar `/scan` verisinin okunması.
- Costmap üretimi ve CSV kaydı.
- Telemetri CSV kaydı.

Gerçek motor sürüşü ise henüz açılmamalıdır.

## 21. Gerçek Motor Kontrolü İçin Eksik Kritik Bilgiler

Motor komutunu güvenle açmak için şu bilgiler gereklidir:

- Jetson-Pixhawk UART baudrate.
- Sol ESC'nin Pixhawk çıkışı.
- Sağ ESC'nin Pixhawk çıkışı.
- ESC PWM nötr değeri.
- ESC PWM ileri değeri.
- ESC PWM geri değeri.
- Pixhawk/ArduRover'da hangi kontrol modunun kullanılacağı.
- MAVROS üzerinden hangi komut tipinin kullanılacağı.
- Kontaktör/SSR rölesinin Jetson GPIO pini.
- Röle aktif HIGH mı, aktif LOW mu.
- Fiziksel kill butonu, RC kill ve YKİ kill gerçekten 24V motor hattını kesiyor mu.

Bu bilgiler olmadan motor sürüşünü açmak yarışma ve güvenlik açısından doğru
değildir.

## 22. Şu Ana Kadar Yapılanların Özeti

Şu ana kadar yapılan iş, projenin yazılım temelini kurmaktır.

Tamamlanan ana başlıklar:

- ROS 2 paket yapısı kuruldu.
- Mission dosyası taşınabilir hale getirildi.
- Parkur-1 GPS tabanlı waypoint takip simülasyonu oluşturuldu.
- Controller görev başlamadan ve görev bitince motor komutunu sıfırlayacak hale
  getirildi.
- Safety katmanı eklendi.
- RC kill altyapısı eklendi.
- YKİ UDP telemetri ve komut altyapısı eklendi.
- MAVROS/Pixhawk köprüsü güvenli şekilde hazırlandı.
- Gerçek motor komutu varsayılan kapalı bırakıldı.
- Lidar/costmap kayıt altyapısı eklendi.
- Telemetri CSV kayıt altyapısı genişletildi.
- Gerçek araç launch ve config dosyaları oluşturuldu.
- README ile kullanım ve güvenlik notları yazıldı.
- Kodlar GitHub'a pushlandı.

## 23. Bundan Sonraki Mantıklı Yol Haritası

Önerilen teknik yol haritası:

1. Jetson üzerinde ROS 2 Humble ortamını netleştirmek.
2. MAVROS ile Pixhawk bağlantısını kurmak.
3. GPS ve heading topic'lerinin gerçekten geldiğini doğrulamak.
4. Lidar `/scan` verisini gerçek RPLiDAR S3 ile test etmek.
5. RC kill channel 7 davranışını test etmek.
6. YKİ UDP telemetriyi laptopta görmek.
7. Motorlar bağlı değilken MAVROS dry-run yapmak.
8. ESC/PWM/Pixhawk çıkış bilgileri netleşince motor komutu yolunu kontrollü açmak.
9. Pervane/itici güvenliği sağlandıktan sonra düşük güç kara testi yapmak.
10. Parkur-2 için `local_planner_node` yazmak.
11. RealSense RGB/depth node'unu gerçek topic'lerle bağlamak.
12. YOLO modeli gelince `buoy_detector_node` eklemek.
13. Parkur-3 hedef rengi ve hedef seçme mantığını geliştirmek.

## 24. Kısa Sonuç

Mevcut yazılım, İDA'nın otonomi omurgasını oluşturur. Sistem şu an görev okuma,
GPS tabanlı yönlendirme, kontrol komutu üretme, güvenlikten geçirme, telemetri
kaydetme, YKİ ile haberleşme ve Pixhawk'a güvenli şekilde bağlanmaya hazırlanma
işlerini yapabilecek seviyededir.

Gerçek motor kontrolü bilinçli olarak kapalıdır. Bu karar, eksik donanım bilgileri ve
güvenlik gereksinimleri nedeniyle doğrudur.

Bir sonraki büyük yazılım adımı Parkur-2 için gerçek local planner yazmaktır. Bu
planner, lidar-only güvenli kaçınma ile başlayacak; YOLO ve RealSense bilgisi
geldiğinde görüntü işleme katmanı sonradan takılacaktır.
