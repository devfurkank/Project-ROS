# Project-ROS — ROS 2 ile Otonom Sürüş Simülasyonu

Robot İşletim Sistemi dersi dönem projesi. TurtleBot3 Burger, Gazebo Classic içinde
LiDAR ile engelleri algılar, SLAM Toolbox ile haritayı çalışırken çıkarır ve Nav2 ile
hedefe engellerden kaçınarak gider.

> **Durum:** Üç senaryo da tek komutla baştan sona koşar: LiDAR engel algılama
> (`obstacle_detector`), SLAM Toolbox + Nav2 ile haritalama, planlama ve kontrol, görev yöneticisi
> (`mission_manager`, çoklu hedef), hareketli engeller (`obstacle_mover`). Sonuçlar CSV'ye, her
> koşunun SLAM haritası `results/maps/`'e yazılır. Sırada deneyler (3 senaryo × 5 tekrar) ve analiz
> betiği var.

## Gereksinimler

- Ubuntu 22.04 (WSL2 üzerinde de çalışır)
- ROS 2 Humble (`ros-humble-desktop`, `ros-dev-tools`)
- Gazebo Classic 11 (`ros-humble-gazebo-ros-pkgs`)
- `ros-humble-turtlebot3`, `ros-humble-turtlebot3-gazebo`
- `ros-humble-navigation2`, `ros-humble-nav2-bringup`, `ros-humble-nav2-simple-commander`
- `ros-humble-slam-toolbox`
- `python3-pandas`, `python3-matplotlib` (sonuç analizi için)

`~/.bashrc` içinde bulunması gerekenler:

```bash
source /opt/ros/humble/setup.bash
source /usr/share/gazebo/setup.sh
export TURTLEBOT3_MODEL=burger
```

## Kurulum ve derleme

```bash
mkdir -p ~/ros2_ws/src && cd ~/ros2_ws/src
git clone https://github.com/devfurkank/Project-ROS.git
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

Doğrulama:

```bash
ros2 pkg list | grep otonom_surus
ros2 pkg executables otonom_surus
colcon test --packages-select otonom_surus && colcon test-result --verbose
```

`--symlink-install`'sız yapılmış bir kurulumun üstüne `--symlink-install` ile derlemek sorun
çıkarır: ilk derleme `[Errno 2] No such file or directory: '.../install/...'` ile düşebilir ve
eski derlemeden kalan dosyalar (`config/*.yaml`, `worlds/*.world`, `rviz/*.rviz`) kaynağa bağlantı
yerine **eski kopyalar olarak kalır**; kaynakta yapılan değişiklikler etkisiz olur. Kurulum türünü
değiştirirken paketi temiz derle ve açık terminallerde `source install/setup.bash`'i yeniden
çalıştır (yoksa `No module named 'otonom_surus'` hatası alınır):

```bash
cd ~/ros2_ws && rm -rf build/otonom_surus install/otonom_surus
colcon build --symlink-install && source install/setup.bash
```

## Senaryoyu tek komutla koşma

```bash
cd ~/ros2_ws/src/Project-ROS    # sonuçlar deponun results/ klasörüne yazılsın
ros2 launch otonom_surus scenario.launch.py scenario:=1
```

Gazebo, SLAM Toolbox + Nav2, `obstacle_detector`, RViz ve `mission_manager` birlikte açılır
(senaryo 3'te hareketli engelleri süren `obstacle_mover` da). Tıklama gerekmez:
`mission_manager` senaryonun hedeflerini sırayla gönderir, robot engellerden kaçınarak gider,
sonuçlar ve harita yazılır, launch kendiliğinden kapanır. Pencere açmadan (daha hızlı):

```bash
ros2 launch otonom_surus scenario.launch.py scenario:=1 gui:=false rviz:=false run_id:=deneme1
```

| Argüman | Varsayılan | Açıklama |
| --- | --- | --- |
| `scenario` | `1` | `config/scenarios.yaml`'daki senaryo numarası |
| `gui` | `true` | Gazebo penceresini aç |
| `rviz` | `true` | RViz2 penceresini aç |
| `run_id` | tarih-saat | Koşunun adı (CSV satırında ve yörünge dosyasının adında) |
| `goal_timeout` | `180.0` | Hedef başına süre sınırı (sim zamanı, s); aşılırsa hedef iptal edilir, sonuç `TIMEOUT` |
| `results_dir` | `results` | Sonuç klasörü; göreli yol komutun çalıştırıldığı klasöre göredir |
| `scenarios_file` | paketteki `config/scenarios.yaml` | Senaryo tanımları (dünya + hedefler) |

### Görev yöneticisi (`mission_manager`)

1. Nav2'nin hazır olmasını bekler: `waitUntilNav2Active(localizer='bt_navigator')`. Varsayılan
   `localizer='amcl'` bu projede sonsuza kadar bekler: AMCL yok, slam_toolbox da Humble'da
   lifecycle düğümü değil.
2. SLAM'ın `map → base_footprint` dönüşümünü ve `/odom`'u bekler.
3. Hedefleri `NavigateToPose` eylemiyle (`nav2_simple_commander.BasicNavigator.goToPose`) sırayla
   gönderir. Başarısız hedeften sonra sıradakine geçer (`stop_on_failure:=true` parametresiyle
   durur).
4. Bitince sonuçları yazar, SLAM haritasını `nav2_map_server`'ın `map_saver_cli` aracıyla kaydeder
   ve çıkar; `scenario.launch.py` bu çıkışta her şeyi kapatır (slam_toolbox o ana kadar
   çalıştığından harita hâlâ yayındadır).

Yol ve yörünge `/odom`'dan ölçülür. Gazebo'da `/odom` dünya gerçeğidir (`diff_drive` eklentisinin
`odometry_source` varsayılanı `world`). Robot (0, 0)'da doğduğundan koordinatlar dünya
koordinatlarıyla aynıdır. RViz'deki turuncu iz (`/traveled_path`) ise robotun SLAM'daki konumudur
(`map` çerçevesi).

| Konu | Tür | Açıklama |
| --- | --- | --- |
| `/odom` (girdi) | `nav_msgs/Odometry` | Kat edilen yol, yörünge, hedefe kalan mesafe |
| `/min_obstacle_distance` (girdi) | `std_msgs/Float32` | `obstacle_detector`'dan; koşudaki en küçük değer kaydedilir |
| `navigate_to_pose` (eylem) | `nav2_msgs/NavigateToPose` | Hedef gönderimi; geri bildirimden kurtarma sayısı |
| `/traveled_path` | `nav_msgs/Path` | Gidilen iz, `map` çerçevesinde, 5 Hz |

Çıktılar (`results_dir` altında; dosyalar yoksa oluşturulur, varsa sonlarına eklenir):

| Dosya | İçerik |
| --- | --- |
| `runs.csv` | Koşu başına bir satır |
| `goals.csv` | Hedef başına bir satır: `goal_index`, `goal_x`, `goal_y`, `goal_yaw_deg`, `result`, `duration_s`, `path_length_m`, `recoveries`, `final_error_m` (durulan nokta → hedef) |
| `trajectories/s<N>_<run_id>.csv` | Yörünge: `t` (s, sim), `x`, `y` (m), 1 cm aralıkla |
| `maps/s<N>_<run_id>.pgm`, `.yaml` | Koşu sonundaki SLAM haritası (`map_server` biçimi, 5 cm/piksel); kaydedilemezse yalnızca uyarı yazılır |

`runs.csv` sütunları:

| Sütun | Açıklama |
| --- | --- |
| `timestamp`, `scenario`, `run_id` | Koşunun kimliği |
| `result` | `SUCCEEDED`; değilse ilk başarısız hedefin sonucu: `FAILED` (Nav2 vazgeçti), `TIMEOUT`, `CANCELED`, `REJECTED` |
| `goals_reached`, `goal_count` | Ulaşılan / toplam hedef |
| `duration_s` | İlk hedefin gönderilmesinden görevin bitmesine geçen sim zamanı |
| `wall_duration_s` | Aynı aralık, gerçek zaman (`duration_s / wall_duration_s` = gerçek zaman çarpanı) |
| `path_length_m` | Kat edilen yol; 1 cm'den küçük adımlar (dururken titreşim) sayılmaz |
| `straight_line_m` | Başlangıçtan hedeflere sırayla kuş uçuşu mesafe |
| `path_efficiency` | `straight_line_m / path_length_m` (1 = dümdüz); yalnızca başarılı koşuda |
| `min_obstacle_dist_m` | LiDAR'dan en yakın yüzeye (engel ya da duvar) en küçük mesafe |
| `recoveries` | Nav2 kurtarma davranışı sayısı (hedeflerin toplamı) |
| `failure_reason` | İlk başarısız hedef, sonucu ve hedefe kalan mesafe |

## Simülasyonu çalıştırma

```bash
ros2 launch otonom_surus sim.launch.py scenario:=1             # Gazebo penceresiyle
ros2 launch otonom_surus sim.launch.py scenario:=1 gui:=false  # pencere olmadan (başsız)
```

| Argüman | Varsayılan | Açıklama |
| --- | --- | --- |
| `scenario` | `1` | Senaryonun `config/scenarios.yaml`'daki dünyasını açar |
| `gui` | `true` | `false` ise yalnızca `gzserver` çalışır |
| `scenarios_file` | paketteki `config/scenarios.yaml` | Senaryo tanımları |

Robot her senaryoda (0, 0)'da doğar. Kontrol: `ros2 topic echo /odom --once` → konum ≈ (0, 0).
Senaryoda hareketli engel tanımlıysa (senaryo 3) `sim.launch.py` onları süren `obstacle_mover`'ı da
başlatır; kutular simülasyon açılır açılmaz hareket etmeye başlar.

## Haritalama ve navigasyon (SLAM Toolbox + Nav2)

Ayrı terminallerde:

```bash
ros2 launch otonom_surus sim.launch.py scenario:=1
ros2 launch otonom_surus navigation.launch.py
```

RViz açılınca araç çubuğundaki **Nav2 Goal** ile haritada bir noktaya tıklayıp sürükleyerek hedef
ver (senaryo 1'in hedefi yeşil disk, (4, 4)). Robot rotayı planlar, engellerden kaçınarak gider;
harita robot ilerledikçe büyür.

`navigation.launch.py` şunları başlatır (simülasyon ayrıca açık olmalı):

| Bileşen | Kaynak | Görev |
| --- | --- | --- |
| `slam_toolbox` (online async) | hazır, `config/slam_params.yaml` | `/scan` → `/map`; `map → odom` TF'i (konumlama) |
| Nav2 (`navigation_launch.py`) | hazır, `config/nav2_params.yaml` | NavFn global planlayıcı, DWB yerel kontrolcü, costmap'ler, davranış ağacı, kurtarma davranışları |
| `obstacle_detector` | bu paket | LiDAR kümeleme; işaretçiler `map` çerçevesinde |
| RViz2 | `rviz/otonom_surus.rviz` | `rviz:=false` ile kapatılabilir |

| Argüman | Varsayılan | Açıklama |
| --- | --- | --- |
| `rviz` | `true` | RViz2 penceresini aç |

Harita önceden verilmez. Henüz görülmemiş alan serbest sayılır: ilk rota bilinmeyen alandan geçebilir,
robot ilerledikçe harita dolar ve rota saniyede bir yeniden planlanır. Nav2 `bringup_launch.py`
kullanılmaz; Humble'da `map` argümanı zorunludur ve SLAM modunda hata verir.

RViz'de görünenler (sabit çerçeve `map`):

| Ekran | Konu | Renk |
| --- | --- | --- |
| SLAM haritası | `/map` | gri tonlar |
| Global / yerel costmap | `/global_costmap/costmap`, `/local_costmap/costmap` | costmap renkleri (şişirme alanı) |
| Robot, ayak izi | `/robot_description`, `/local_costmap/published_footprint` | yeşil çerçeve |
| LiDAR | `/scan` | gökkuşağı |
| Algılanan engeller | `/detected_obstacles` | kırmızı silindir, gri çizgi |
| Global rota | `/plan` | yeşil |
| Yerel rota | `/local_plan` | mavi |
| Gidilen iz | `/traveled_path` | turuncu (`mission_manager` yayınlar, yalnızca `scenario.launch.py` ile) |

Zararsız mesajlar:

- Navigation 2 panelinde `Localization: inactive`: konumlamayı AMCL değil slam_toolbox yapar.
- RViz'de `GLSL link result : active samplers with a different type refer to the same texture image unit`:
  WSL'deki Mesa sürücüsünden gelir, harita yine çizilir.
- Ara sıra `Planner loop missed its desired rate of 20.0000 Hz`: 16 × 16 m'lik costmap'te tek bir
  NavFn planı ~0.1 s sürebilir; davranış ağacı zaten saniyede bir planlar.
- Ctrl+C sonrası `rviz2 ... exit code -11`: RViz2'nin kapanırken çökmesi, çalışmayı etkilemez.

Haritayı kaydetmek için:

```bash
ros2 run nav2_map_server map_saver_cli -f ~/harita_s1 --ros-args -p save_map_timeout:=10.0
```

Hazır yapılandırmalardan değiştirilen parametreler (dosyalarda `# değişti:` ile işaretli):

| Dosya | Parametre | Varsayılan → yeni | Neden |
| --- | --- | --- | --- |
| `nav2_params.yaml` | `robot_radius` (iki costmap) | 0.22 → 0.125 | Burger'in `base_footprint` etrafındaki çevrel yarıçapı (gövdenin arka köşesi 0.124 m) |
| | `inflation_radius` (iki costmap) | 0.55 → 0.35 | 0.75 m'lik geçitlerin ortası serbest kalsın |
| | `max_vel_x`, `max_speed_xy`, `velocity_smoother` hızları | 0.26 → 0.22 m/s | Burger'in azami hızı |
| | global costmap `rolling_window`, `width`, `height` | yok → `true`, 16, 16 | Başta küçük olan SLAM haritası uzak hedefi kapsamaz; robotu ortalayan 16 × 16 m pencere odanın tamamını kapsar |
| | global costmap `track_unknown_space` | true → false | Bilinmeyen alan serbest sayılır; `true` iken NavFn engel gölgelerinde zaman zaman "Failed to create a plan from potential" hatası verip kurtarma davranışı başlatıyordu |
| | DWB `forward_prune_distance` | 2.0 → 0.7 | DWB eleştirmenlere planın robottan Öklid uzaklığı min(1.5 m, bu değer)'i ilk aşan noktasına kadarki kısmını verir. Senaryo 2'de plan kapıdan geçip duvarın öbür yüzünde geri dönünce dönüş kolu robota ~0.9 m'ye kadar yaklaşıyor; yerel hedef duvarın arkasına düşüyor, GoalDist robotu duvara çekiyor ve robot kapının önünde takılıyordu |
| `slam_params.yaml` | `min_laser_range`, `max_laser_range` | 0.0, 20.0 → 0.12, 3.5 | Burger LiDAR'ının menzili |
| | `minimum_travel_distance`, `minimum_travel_heading` | 0.5 → 0.2 | Küçük odada daha sık tarama eklensin |
| | `map_update_interval` | 5.0 → 2.0 s | Harita daha sık güncellensin |
| | `do_loop_closing` | true → false | Senaryo 2'nin birbirine benzeyen paralel koridorlarında yanlış döngü kapama `map → odom`'u 2 m kaydırıp haritayı bozuyordu; oda küçük ve Gazebo'da odometri kusursuz olduğundan kapatılacak döngü kayması yok |

## Engel algılama (`obstacle_detector`)

`/scan` noktalarını açı sırasıyla gezer; ardışık iki nokta arasındaki mesafe uyarlamalı bir
eşikten (Borges ve Aldon, 2004) küçükse aynı kümeye koyar. En az 3 noktalı her küme bir
engeldir: yarıçapı `max_obstacle_radius`'tan küçükse kırmızı silindir, büyükse (duvar, köşe)
gri çizgi olarak çizilir. Merkez, kümenin iki uç noktasının orta noktasıdır.

İşaretçiler tarama anının TF'i gelene kadar bekletilip RViz'in sabit çerçevesinde (`frame_id`)
yayınlanır. RViz2'nin MarkerArray ekranı TF'i beklemediğinden, başka bir çerçevede yayınlanan
işaretçiler "extrapolation into the future" hatasıyla yanıp söner. `navigation.launch.py` düğümü
`frame_id:=map` ile başlatır.

SLAM olmadan, yalnızca algılamayı denemek için (RViz'in sabit çerçevesi `-f odom` ile değiştirilir):

```bash
ros2 launch otonom_surus sim.launch.py scenario:=1
ros2 run otonom_surus obstacle_detector --ros-args -p use_sim_time:=true
rviz2 -d $(ros2 pkg prefix otonom_surus)/share/otonom_surus/rviz/otonom_surus.rviz -f odom --ros-args -p use_sim_time:=true
ros2 run turtlebot3_teleop teleop_keyboard
```

| Konu | Tür | Açıklama |
| --- | --- | --- |
| `/scan` (girdi) | `sensor_msgs/LaserScan` | TurtleBot3 LiDAR'ı (360 ışın, 5 Hz, 0.12–3.5 m) |
| `/detected_obstacles` | `visualization_msgs/MarkerArray` | `frame_id` çerçevesinde; her mesaj `DELETEALL` ile başlar |
| `/min_obstacle_distance` | `std_msgs/Float32` | LiDAR'dan en yakın yüzeye mesafe (m); geçerli ölçüm yoksa `inf` |

| Parametre | Varsayılan | Açıklama |
| --- | --- | --- |
| `cluster_tolerance` | `0.15` | Komşu noktalar arası en küçük kümeleme eşiği (m) |
| `min_incidence_deg` | `10.0` | Uyarlamalı eşikte yüzeyin ışınla yapabileceği en dar açı (°) |
| `range_sigma` | `0.01` | LiDAR mesafe gürültüsünün standart sapması (m) |
| `min_points` | `3` | Daha az noktalı kümeler gürültü sayılır |
| `max_obstacle_radius` | `0.75` | Bundan büyük kümeler duvar olarak çizilir (m) |
| `frame_id` | `odom` | İşaretçilerin çerçevesi (SLAM ile `map`); boş bırakılırsa tarama çerçevesi (`base_scan`) |

## Senaryolar

Ortak: 6 × 6 m kapalı oda (iç alan x, y ∈ [−1, 5]), engeller 0.5 m yüksekliğinde ilkel şekiller,
geçitler ≥ 0.75 m. Robot her senaryoda (0, 0)'da doğar; harita önceden verilmez.

| Senaryo | Dünya | Hedef(ler) (x, y, yaw) | Sınadığı şey |
| --- | --- | --- | --- |
| 1 | 6 dağınık statik engel (3 kutu, 3 silindir); düz rota iki engelle kapalı | (4, 4, 90°) | Temel uçtan uca hat |
| 2 | İki iç duvarla zikzak koridor: `y = 1` duvarının kapısı doğuda (x ∈ [3.6, 4.4]), `y = 3` duvarınınki batıda (x ∈ [−0.4, 0.4]), ikisi de 0.8 m; orta koridorda duvara yaslı kutu (üstünde 0.9 m geçit), alt ve üst koridorda düz rotayı kapatan silindirler | (4, 0, 90°) → (4, 4, 180°) → (0, 4, 180°) | Dar geçit, sıralı görev: ikinci hedef için robot doğu kapısından orta koridora girip batı kapısından üst koridora çıkmak zorunda |
| 3 | 2 statik engel + robotun düz rotasını kesen 2 hareketli kutu (0.4 m, 5 kg): `engel1` (0.4, 1.5)'ten +x yönünde 1.5 m, 0.15 m/s; `engel2` (3.1, 2.7)'den +y yönünde 1.2 m, 0.12 m/s | (4, 4, 90°) | Hareketli engelden kaçınma, gerçek zamanlı yeniden planlama |

Senaryolar `config/scenarios.yaml`'da tanımlıdır; `sim.launch.py` dünyayı, `mission_manager`
hedefleri, `obstacle_mover` hareketli engelleri oradan okur:

```yaml
3:
  name: Hareketli engeller
  world: senaryo3.world          # worlds/ altındaki dosya ya da mutlak yol
  goals:                         # sırayla; [x (m), y (m), yaw (derece)], map çerçevesinde
    - [4.0, 4.0, 90.0]
  moving_obstacles:              # isteğe bağlı; dünyadaki planar_move eklentili modeller
    - {name: engel1, travel: 1.5, speed: 0.15}   # şerit uzunluğu (m), hız (m/s)
    - {name: engel2, travel: 1.2, speed: 0.12}
```

Paketi değiştirmeden yeni bir görev denemek için kendi dosyanı ver:
`scenario.launch.py scenarios_file:=/yol/benim.yaml scenario:=<N>`.

### Hareketli engeller (`obstacle_mover`)

Her hareketli kutu dünyada `gazebo_ros_planar_move` eklentili, statik olmayan bir modeldir.
Eklenti `/<ad>/cmd_vel`'i dinler ve kutunun dünyadaki konumunu `/<ad>/odom`'da yayınlar.
`obstacle_mover` kutuyu doğduğu konumdan kendi +x ekseni boyunca `travel` metre gidip getirir;
dönüş noktalarını süreyle değil bu konumla belirler, böylece zamanlama sapması ya da robotla temas
kutuyu şeridinden kaydıramaz. Kutular robottan habersizdir (yol vermez).

| Konu | Tür | Açıklama |
| --- | --- | --- |
| `/<ad>/odom` (girdi) | `nav_msgs/Odometry` | Kutunun dünyadaki konumu (eklenti, 10 Hz) |
| `/<ad>/cmd_vel` | `geometry_msgs/Twist` | Kutunun gövde ekseninde ±`speed` |

| Parametre | Varsayılan | Açıklama |
| --- | --- | --- |
| `scenario` | `3` | Hareketli engelleri okunacak senaryo |
| `scenarios_file` | paketteki `config/scenarios.yaml` | Senaryo tanımları |

Dünya dosyasında dikkat edilenler:

- Eklentide `<publish_odom_tf>false</publish_odom_tf>` **şart**: açık olsaydı eklenti
  `odom → base_footprint` dönüşümünü yayınlayıp robotun TF ağacını bozardı.
- Kutunun sürtünmesi 0: eklenti hızı doğrudan atar; zeminle sürtünme kutuyu titretmesin, devirmesin.
- Şeritler kısa (1.5 m) ve uçlarında ≥ 0.8 m boşluk var. SLAM haritası kutunun geçtiği yerleri bir
  süre dolu sayabilir ("hayalet iz"); o zaman da şeridin ucundan dolanılabilir.

## Depo yapısı

```
Project-ROS/
├── README.md
└── otonom_surus/              # ROS 2 paketi (ament_python)
    ├── package.xml · setup.py · setup.cfg
    ├── otonom_surus/
    │   ├── obstacle_detector.py   # /scan -> engel listesi (LiDAR kümeleme)
    │   ├── mission_manager.py     # hedefleri sırayla gönderir, metrikleri CSV'ye yazar
    │   ├── scenarios.py           # scenarios.yaml okuyucu (launch ve mission_manager ortak)
    │   └── obstacle_mover.py      # hareketli engelleri şeritlerinde git-gel sürer
    ├── launch/
    │   ├── sim.launch.py          # Gazebo + dünya + robot (+ obstacle_mover) (scenario:=, gui:=)
    │   ├── navigation.launch.py   # slam_toolbox + Nav2 + obstacle_detector + RViz (rviz:=)
    │   └── scenario.launch.py     # hepsi + mission_manager; görev bitince kapanır
    ├── worlds/
    │   └── senaryo1.world · senaryo2.world · senaryo3.world
    ├── config/
    │   ├── scenarios.yaml         # senaryo tanımları: dünya + hedefler
    │   ├── nav2_params.yaml       # Nav2 (planlayıcı, kontrolcü, costmap'ler)
    │   └── slam_params.yaml       # SLAM Toolbox
    ├── rviz/
    │   └── otonom_surus.rviz      # harita, costmap'ler, rotalar, engeller; Nav2 Goal aracı
    └── test/                      # birim testleri (algılama, senaryo/metrik, şerit) + ament lint
```
