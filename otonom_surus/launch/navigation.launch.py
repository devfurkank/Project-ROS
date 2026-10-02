"""
SLAM Toolbox, Nav2, LiDAR engel algılama (obstacle_detector) ve RViz'i başlatır.

Simülasyon (sim.launch.py) ayrıca çalışıyor olmalıdır. Harita önceden verilmez; slam_toolbox
haritayı çalışırken çıkarır ve map -> odom dönüşümünü yayınlar, Nav2 bu haritada planlar.

Kullanım:
    ros2 launch otonom_surus navigation.launch.py rviz:=true
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('otonom_surus')
    slam_launch = os.path.join(get_package_share_directory('slam_toolbox'), 'launch')
    nav2_launch = os.path.join(get_package_share_directory('nav2_bringup'), 'launch')

    return LaunchDescription([
        DeclareLaunchArgument(
            'rviz', default_value='true', description='RViz2 penceresini aç'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(slam_launch, 'online_async_launch.py')),
            launch_arguments={
                'slam_params_file': os.path.join(share, 'config', 'slam_params.yaml'),
                'use_sim_time': 'true',
            }.items()),
        # bringup_launch.py değil: Humble'da map argümanı zorunlu, SLAM modunda hata verir.
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(nav2_launch, 'navigation_launch.py')),
            launch_arguments={
                'params_file': os.path.join(share, 'config', 'nav2_params.yaml'),
                'use_sim_time': 'true',
            }.items()),
        # İşaretçiler RViz'in sabit çerçevesi map'te yayınlanır. odom'da yayınlansa RViz'in
        # map <- odom dönüşümüne ihtiyacı olur; slam_toolbox bunu "son tarama + 0.2 s"
        # damgasıyla yayınladığından tarama aralığı 0.201 s olduğunda dönüşüm 1 ms geride kalır.
        Node(
            package='otonom_surus', executable='obstacle_detector', output='screen',
            parameters=[{'use_sim_time': True, 'frame_id': 'map'}]),
        Node(
            package='rviz2', executable='rviz2', name='rviz2', output='log',
            arguments=['-d', os.path.join(share, 'rviz', 'otonom_surus.rviz')],
            parameters=[{'use_sim_time': True}],
            condition=IfCondition(LaunchConfiguration('rviz'))),
    ])
