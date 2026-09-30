"""Hedefleri sırayla gönderip metrik kaydeden düğüm (iskelet; Faz 6'da doldurulacak)."""

import rclpy
from rclpy.node import Node


class MissionManager(Node):
    """scenarios.yaml hedefleri -> NavigateToPose; sonuçlar -> results/runs.csv."""

    def __init__(self):
        super().__init__('mission_manager')
        self.get_logger().info('mission_manager iskeleti çalışıyor (Faz 6 bekleniyor).')


def main(args=None):
    rclpy.init(args=args)
    node = MissionManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
