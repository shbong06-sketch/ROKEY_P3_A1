"""큐브 색을 감지해 /color_id 로 발행하는 노드."""

import rclpy
from rclpy.node import Node


class ColorDetector(Node):
    def __init__(self):
        super().__init__('m0609_color_detector')
        self.get_logger().info('color detector started')


def main():
    rclpy.init()
    node = ColorDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
