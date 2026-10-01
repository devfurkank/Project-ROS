# Project-ROS — ROS 2 ile Otonom Sürüş Simülasyonu

Robot İşletim Sistemi dersi dönem projesi. TurtleBot3 Burger, Gazebo Classic içinde
LiDAR ile engelleri algılar, SLAM Toolbox ile haritayı çalışırken çıkarır ve Nav2 ile
hedefe engellerden kaçınarak gider.

> **Durum:** Senaryo 1 dünyası, simülasyon launch dosyası (Faz 3) ve LiDAR engel algılama
> düğümü `obstacle_detector` (Faz 4) hazır. `mission_manager` ve `obstacle_mover` şimdilik
> yalnızca başlayıp log yazan iskeletlerdir; navigasyon ve diğer senaryolar sonraki fazlarda eklenecek.

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

## Engel algılama (`obstacle_detector`)

`/scan` noktalarını açı sırasıyla gezer; ardışık iki nokta arasındaki mesafe uyarlamalı bir
eşikten (Borges ve Aldon, 2004) küçükse aynı kümeye koyar. En az 3 noktalı her küme bir
engeldir: yarıçapı `max_obstacle_radius`'tan küçükse kırmızı silindir, büyükse (duvar, köşe)
gri çizgi olarak çizilir. Merkez, kümenin iki uç noktasının orta noktasıdır.

İşaretçiler tarama anının TF'i gelene kadar bekletilip `odom` çerçevesinde yayınlanır.
RViz2'nin MarkerArray ekranı TF'i beklemediğinden, `base_scan` çerçevesinde yayınlanan
işaretçiler "extrapolation into the future" hatasıyla yanıp söner.

Ayrı terminallerde:

```bash
ros2 launch otonom_surus sim.launch.py scenario:=1
ros2 run otonom_surus obstacle_detector --ros-args -p use_sim_time:=true
rviz2 -d $(ros2 pkg prefix otonom_surus)/share/otonom_surus/rviz/otonom_surus.rviz --ros-args -p use_sim_time:=true
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
| `frame_id` | `odom` | İşaretçilerin çerçevesi; boş bırakılırsa tarama çerçevesi (`base_scan`) |

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
    │   └── sim.launch.py      # Gazebo + dünya + robot (scenario:=, gui:=)
    ├── worlds/
    │   └── senaryo1.world     # Gazebo dünyaları
    ├── config/                # Nav2, SLAM ve senaryo parametreleri
    ├── rviz/
    │   └── otonom_surus.rviz  # robot, LiDAR ve algılanan engeller (sabit çerçeve: odom)
    └── test/                  # birim testleri + ament lint testleri
```
