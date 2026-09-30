"""Hareketli engelleri git-gel süren düğüm (iskelet; Faz 7'de doldurulacak)."""

import rclpy
from rclpy.node import Node


class ObstacleMover(Node):
    """Her hareketli engelin /engelN/cmd_vel konusuna Twist yayınlar."""

    def __init__(self):
        super().__init__('obstacle_mover')
        self.get_logger().info('obstacle_mover iskeleti çalışıyor (Faz 7 bekleniyor).')


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleMover()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
