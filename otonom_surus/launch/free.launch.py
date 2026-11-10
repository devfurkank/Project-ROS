"""
Serbest sürüş: senaryonun dünyası ve hareketli engelleri açılır, hedefleri web paneli verir.

scenario.launch.py'nin mode:=interactive halidir; argümanları da onunkiler (scenario dünyayı ve
hareketli engelleri seçer, results_dir, run_id, gui, rviz ...). mission_manager senaryonun
hedeflerini yüklemez, panelden gelen hedefleri (/panel/goal) Nav2'ye gönderir; panelin "bitir"i
(/panel/events: finish) gelince sonuçları results_dir'e yazıp çıkar, bu launch da onunla kapanır.
Ölçüm ve kayıt senaryo koşusundakiyle aynı.

Kullanım (genellikle panel başlatır):
    ros2 launch otonom_surus free.launch.py scenario:=1 gui:=false rviz:=false
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    launch_dir = os.path.join(get_package_share_directory('otonom_surus'), 'launch')
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir, 'scenario.launch.py')),
            launch_arguments={'mode': 'interactive'}.items()),
    ])
