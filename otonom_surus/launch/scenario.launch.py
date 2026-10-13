"""
Bir senaryoyu tek komutla baştan sona koşar: simülasyon + SLAM/Nav2 + görev yöneticisi.

mission_manager senaryonun hedeflerini sırayla gönderir, sonuçları results_dir'e yazıp çıkar;
bu launch da onunla birlikte kendiliğinden kapanır.

Kullanım:
    ros2 launch otonom_surus scenario.launch.py scenario:=1 gui:=true rviz:=true run_id:=deneme1
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler, Shutdown)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = get_package_share_directory('otonom_surus')
    launch_dir = os.path.join(share, 'launch')
    arg = LaunchConfiguration

    # Türler açıkça veriliyor; yoksa launch_ros değerleri YAML olarak ayrıştırır: run_id:=007
    # tamsayı 7 olur, goal_timeout:=120 float bekleyen parametreye tamsayı olarak gidip hata verir.
    mission_manager = Node(
        package='otonom_surus', executable='mission_manager', output='screen',
        parameters=[{
            'use_sim_time': True,
            'scenario': ParameterValue(arg('scenario'), value_type=int),
            'run_id': ParameterValue(arg('run_id'), value_type=str),
            'goal_timeout': ParameterValue(arg('goal_timeout'), value_type=float),
            'results_dir': ParameterValue(arg('results_dir'), value_type=str),
            'scenarios_file': ParameterValue(arg('scenarios_file'), value_type=str),
        }])

    return LaunchDescription([
        DeclareLaunchArgument(
            'scenario', default_value='1', description='Senaryo numarası: 1 | 2 | 3'),
        DeclareLaunchArgument(
            'gui', default_value='true', description='Gazebo penceresini (gzclient) aç'),
        DeclareLaunchArgument(
            'rviz', default_value='true', description='RViz2 penceresini aç'),
        DeclareLaunchArgument(
            'run_id', default_value='',
            description='Koşunun adı; boşsa tarih-saat, örn. 20261013-142530'),
        DeclareLaunchArgument(
            'goal_timeout', default_value='180.0',
            description='Hedef başına süre sınırı (sim zamanı, s)'),
        DeclareLaunchArgument(
            'results_dir', default_value='results',
            description='Sonuç klasörü; göreli ise komutun çalıştırıldığı klasöre göre'),
        DeclareLaunchArgument(
            'scenarios_file', default_value=os.path.join(share, 'config', 'scenarios.yaml'),
            description='Senaryo tanımları (dünya + hedefler)'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir, 'sim.launch.py')),
            launch_arguments={
                'scenario': arg('scenario'),
                'gui': arg('gui'),
                'scenarios_file': arg('scenarios_file'),
            }.items()),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir, 'navigation.launch.py')),
            launch_arguments={'rviz': arg('rviz')}.items()),
        mission_manager,
        RegisterEventHandler(OnProcessExit(
            target_action=mission_manager,
            on_exit=[Shutdown(reason='mission_manager görevi bitirdi')])),
    ])
