# Project-ROS — ROS 2 ile Otonom Sürüş Simülasyonu

Robot İşletim Sistemi dersi dönem projesi. TurtleBot3 Burger, Gazebo Classic içinde
LiDAR ile engelleri algılar, SLAM Toolbox ile haritayı çalışırken çıkarır ve Nav2 ile
hedefe engellerden kaçınarak gider.

> **Durum:** Senaryo 1 dünyası ve simülasyon launch dosyası (Faz 3), LiDAR engel algılama
> düğümü `obstacle_detector` (Faz 4), SLAM Toolbox + Nav2 ile haritalama, planlama ve kontrol
> (Faz 5) hazır: RViz'den verilen hedefe robot engellerden kaçınarak gider. `mission_manager` ve
> `obstacle_mover` şimdilik yalnızca başlayıp log yazan iskeletlerdir; otomatik görev yürütme ve
> diğer senaryolar sonraki fazlarda eklenecek.

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

## Simülasyonu çalıştırma

```bash
ros2 launch otonom_surus sim.launch.py scenario:=1             # Gazebo penceresiyle
ros2 launch otonom_surus sim.launch.py scenario:=1 gui:=false  # pencere olmadan (başsız)
```

| Argüman | Varsayılan | Açıklama |
| --- | --- | --- |
| `scenario` | `1` | `worlds/senaryo<N>.world` dünyasını açar |
| `gui` | `true` | `false` ise yalnızca `gzserver` çalışır |

Robot her senaryoda (0, 0)'da doğar. Kontrol: `ros2 topic echo /odom --once` → konum ≈ (0, 0).

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
| Gidilen iz | `/traveled_path` | turuncu (`mission_manager` yayınlayacak) |

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
| `slam_params.yaml` | `min_laser_range`, `max_laser_range` | 0.0, 20.0 → 0.12, 3.5 | Burger LiDAR'ının menzili |
| | `minimum_travel_distance`, `minimum_travel_heading` | 0.5 → 0.2 | Küçük odada daha sık tarama eklensin |
| | `map_update_interval` | 5.0 → 2.0 s | Harita daha sık güncellensin |

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
geçitler ≥ 0.75 m.

| Senaryo | Dünya | Hedef(ler) |
| --- | --- | --- |
| 1 | 6 dağınık statik engel (3 kutu, 3 silindir); düz rota iki engelle kapalı | (4, 4) |

## Depo yapısı

```
Project-ROS/
├── README.md
└── otonom_surus/              # ROS 2 paketi (ament_python)
    ├── package.xml · setup.py · setup.cfg
    ├── otonom_surus/
    │   ├── obstacle_detector.py   # /scan -> engel listesi (LiDAR kümeleme)
    │   ├── mission_manager.py     # hedefleri sırayla gönderir, metrikleri CSV'ye yazar
    │   └── obstacle_mover.py      # hareketli engelleri sürer
    ├── launch/
    │   ├── sim.launch.py          # Gazebo + dünya + robot (scenario:=, gui:=)
    │   └── navigation.launch.py   # slam_toolbox + Nav2 + obstacle_detector + RViz (rviz:=)
    ├── worlds/
    │   └── senaryo1.world         # Gazebo dünyaları
    ├── config/
    │   ├── nav2_params.yaml       # Nav2 (planlayıcı, kontrolcü, costmap'ler)
    │   └── slam_params.yaml       # SLAM Toolbox
    ├── rviz/
    │   └── otonom_surus.rviz      # harita, costmap'ler, rotalar, engeller; Nav2 Goal aracı
    └── test/                      # birim testleri + ament lint testleri
```
