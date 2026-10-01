# Project-ROS — ROS 2 ile Otonom Sürüş Simülasyonu

Robot İşletim Sistemi dersi dönem projesi. TurtleBot3 Burger, Gazebo Classic içinde
LiDAR ile engelleri algılar, SLAM Toolbox ile haritayı çalışırken çıkarır ve Nav2 ile
hedefe engellerden kaçınarak gider.

> **Durum:** Senaryo 1 dünyası ve simülasyon launch dosyası hazır (Faz 3). Düğümler şimdilik
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
    └── rviz/                  # RViz2 yapılandırması
```
