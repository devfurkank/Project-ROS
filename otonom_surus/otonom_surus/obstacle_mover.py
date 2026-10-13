"""
Senaryonun hareketli engellerini şeritleri boyunca git-gel süren düğüm (modül ③).

config/scenarios.yaml'daki senaryonun moving_obstacles listesini okur. Her engel dünyada
gazebo_ros_planar_move eklentili bir modeldir: eklenti /<ad>/cmd_vel'i dinler ve modelin
dünyadaki konumunu /<ad>/odom'da yayınlar (TF yayınlamaz; robotun TF ağacı bozulmasın, T6).
Şerit, engelin doğduğu konumdan kendi +x ekseni boyunca travel metredir; engel speed hızıyla
uçtan uca gidip gelir. Dönüş noktası süreyle değil konumla belirlenir: zamanlama sapması ya da
robotla çarpışma engeli şeridinden kaydıramaz.

Konular (her engel için):
    <ad>/odom     (nav_msgs/Odometry)    girdi: engelin dünyadaki konumu
    <ad>/cmd_vel  (geometry_msgs/Twist)  çıktı: gövde ekseninde ±speed
"""

import math
import os

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from otonom_surus.scenarios import load_scenario
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node


def yaw_of(q):
    """Kuaterniyonun z ekseni etrafındaki açısı (rad)."""
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class Lane:
    """Bir engelin şeridi: ilk konumundan ilk yönü boyunca travel metre."""

    def __init__(self, travel):
        self.travel = travel
        self.origin = None  # (x, y, yaw): ilk konum
        self.direction = 1.0  # +1 ileri (şeridin ucuna), -1 geri (başlangıca)

    def update(self, x, y, yaw):
        """
        Konuma göre hareket yönünü günceller ve döndürür (+1 ya da -1).

        İlerleme, konumun şerit eksenine izdüşümüdür; eksene dik kaymalar (örn. robot kutuyu
        yana itti) dönüş noktalarını etkilemez.
        """
        if self.origin is None:
            self.origin = (x, y, yaw)
        ox, oy, oyaw = self.origin
        progress = (x - ox) * math.cos(oyaw) + (y - oy) * math.sin(oyaw)
        if self.direction > 0.0 and progress >= self.travel:
            self.direction = -1.0
        elif self.direction < 0.0 and progress <= 0.0:
            self.direction = 1.0
        return self.direction


class ObstacleMover(Node):
    """scenarios.yaml'daki her hareketli engel için /<ad>/odom -> /<ad>/cmd_vel."""

    def __init__(self):
        super().__init__('obstacle_mover')
        scenario = self.declare_parameter('scenario', 3).value
        scenarios_file = self.declare_parameter('scenarios_file', os.path.join(
            get_package_share_directory('otonom_surus'), 'config', 'scenarios.yaml')).value
        self.obstacles = load_scenario(scenarios_file, scenario)['moving_obstacles']
        if not self.obstacles:
            self.get_logger().warn(f'Senaryo {scenario}: hareketli engel yok, çıkılıyor.')

        self.lanes = {}
        for obstacle in self.obstacles:
            name = obstacle['name']
            self.lanes[name] = Lane(obstacle['travel'])
            publisher = self.create_publisher(Twist, f'{name}/cmd_vel', 10)
            self.create_subscription(
                Odometry, f'{name}/odom',
                lambda msg, o=obstacle, p=publisher: self.on_odom(msg, o, p), 10)
            self.get_logger().info(
                f'{name}: {obstacle["travel"]:.2f} m şerit, {obstacle["speed"]:.2f} m/s')
        # Eklenti yüklenmediyse (dünyada model ya da ad alanı yanlış) engel hiç kıpırdamaz.
        self.check_timer = self.create_timer(10.0, self.check_odom)

    def on_odom(self, msg, obstacle, publisher):
        p = msg.pose.pose
        direction = self.lanes[obstacle['name']].update(
            p.position.x, p.position.y, yaw_of(p.orientation))
        cmd = Twist()
        cmd.linear.x = direction * obstacle['speed']
        publisher.publish(cmd)

    def check_odom(self):
        self.check_timer.cancel()
        for name, lane in self.lanes.items():
            if lane.origin is None:
                self.get_logger().error(
                    f'{name}/odom gelmiyor: dünyada {name} modeli ve '
                    f'<namespace>/{name}</namespace> ile planar_move eklentisi var mı?')


def main(args=None):
    rclpy.init(args=args)
    status = 0
    node = None
    try:
        node = ObstacleMover()
        if node.obstacles:
            rclpy.spin(node)
    except ValueError as e:  # scenarios.yaml'da senaryo yok ya da biçimi bozuk
        rclpy.logging.get_logger('obstacle_mover').error(str(e))
        status = 1
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()
    return status


if __name__ == '__main__':
    main()
