"""
Senaryo dünyasını Gazebo Classic'te açar ve TurtleBot3 Burger'i (0, 0)'a yerleştirir.

Dünya dosyası config/scenarios.yaml'daki senaryo tanımından alınır.

Kullanım:
    ros2 launch otonom_surus sim.launch.py scenario:=1 gui:=true
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from otonom_surus.scenarios import load_scenario


def _gzserver(context):
    """Senaryonun dünya dosyasını bulur; senaryo ya da dosya yoksa anlaşılır bir hata verir."""
    scenario = LaunchConfiguration('scenario').perform(context)
    scenarios_file = LaunchConfiguration('scenarios_file').perform(context)
    try:
        world_name = load_scenario(scenarios_file, scenario)['world']
    except ValueError as e:
        raise RuntimeError(f'scenario:={scenario}: {e}') from None
    world = os.path.join(get_package_share_directory('otonom_surus'), 'worlds', world_name)
    if not os.path.isfile(world):
        raise RuntimeError(f'scenario:={scenario} için dünya dosyası yok: {world}')

    gazebo_launch = os.path.join(get_package_share_directory('gazebo_ros'), 'launch')
    return [IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(gazebo_launch, 'gzserver.launch.py')),
        launch_arguments={'world': world}.items())]


def generate_launch_description():
    # turtlebot3_gazebo launch dosyaları bu değişken yoksa KeyError verir (T1).
    os.environ.setdefault('TURTLEBOT3_MODEL', 'burger')

    gazebo_launch = os.path.join(get_package_share_directory('gazebo_ros'), 'launch')
    tb3_launch = os.path.join(get_package_share_directory('turtlebot3_gazebo'), 'launch')
    share = get_package_share_directory('otonom_surus')

    return LaunchDescription([
        DeclareLaunchArgument(
            'scenario', default_value='1', description='Senaryo numarası: 1 | 2 | 3'),
        DeclareLaunchArgument(
            'gui', default_value='true', description='Gazebo penceresini (gzclient) aç'),
        DeclareLaunchArgument(
            'scenarios_file', default_value=os.path.join(share, 'config', 'scenarios.yaml'),
            description='Senaryo tanımları (dünya + hedefler)'),

        OpaqueFunction(function=_gzserver),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(gazebo_launch, 'gzclient.launch.py')),
            condition=IfCondition(LaunchConfiguration('gui'))),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(tb3_launch, 'robot_state_publisher.launch.py')),
            launch_arguments={'use_sim_time': 'true'}.items()),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(tb3_launch, 'spawn_turtlebot3.launch.py')),
            launch_arguments={'x_pose': '0.0', 'y_pose': '0.0'}.items()),
    ])
