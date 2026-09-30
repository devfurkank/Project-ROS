# Project-ROS — ROS 2 ile Otonom Sürüş Simülasyonu

Robot İşletim Sistemi dersi dönem projesi. TurtleBot3 Burger, Gazebo Classic içinde
LiDAR ile engelleri algılar, SLAM Toolbox ile haritayı çalışırken çıkarır ve Nav2 ile
hedefe engellerden kaçınarak gider.

> **Durum:** Paket iskeleti hazır (Faz 2). Düğümler şimdilik yalnızca başlayıp log yazan
> iskeletlerdir; dünyalar, launch dosyaları ve senaryo komutları sonraki fazlarda eklenecek.

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
    ├── launch/                # launch dosyaları
    ├── worlds/                # Gazebo dünyaları
    ├── config/                # Nav2, SLAM ve senaryo parametreleri
    └── rviz/                  # RViz2 yapılandırması
```
