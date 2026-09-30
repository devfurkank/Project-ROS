"""LiDAR taramasından engel çıkaran düğüm (iskelet; Faz 4'te doldurulacak)."""

import rclpy
from rclpy.node import Node


class ObstacleDetector(Node):
    """/scan -> /detected_obstacles ve /min_obstacle_distance."""

    def __init__(self):
        super().__init__('obstacle_detector')
        self.get_logger().info('obstacle_detector iskeleti çalışıyor (Faz 4 bekleniyor).')


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
