"""
Web kontrol panelini açar: tarayıcıda http://localhost:<port>.

Yalnızca panel_server'ı başlatır; simülasyonu panel kendisi alt süreç olarak başlatır ve kapatır.
Panelin koşuları results_dir'e yazılır (varsayılan results/panel, git dışı): Faz 8'in deney
verisine (results/runs.csv) karışmaz.

Kullanım:
    ros2 launch otonom_surus panel.launch.py
    ros2 launch otonom_surus panel.launch.py port:=8090
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    arg = LaunchConfiguration
    return LaunchDescription([
        DeclareLaunchArgument(
            'port', default_value='8080', description='HTTP portu (yalnızca 127.0.0.1)'),
        DeclareLaunchArgument(
            'results_dir', default_value='results/panel',
            description='Panel koşularının klasörü; göreli ise komutun çalıştırıldığı yere göre'),
        # Türler açıkça veriliyor (T16): launch_ros değerleri YAML olarak ayrıştırır.
        Node(
            package='otonom_surus', executable='panel_server', output='screen',
            parameters=[{
                'port': ParameterValue(arg('port'), value_type=int),
                'results_dir': ParameterValue(arg('results_dir'), value_type=str),
            }]),
    ])
