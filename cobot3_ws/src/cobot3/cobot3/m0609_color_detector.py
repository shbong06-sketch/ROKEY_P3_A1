"""Wrist Camera 영상에서 큐브 색을 판별해 /color_id 로 발행한다.

    ros2 run cobot3 m0609_color_detector

구독  /rgb          sensor_msgs/Image
발행  /color_id     std_msgs/Int32    0 미감지 / 1 파랑 / 2 초록
발행  /color_debug  sensor_msgs/Image 판별 결과를 얹은 영상
"""

import cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import Int32
from cv_bridge import CvBridge

from cobot3.color_classifier import classify

IMAGE_TOPIC = '/rgb'
ID_TOPIC = '/color_id'
DEBUG_TOPIC = '/color_debug'
LOG_EVERY = 30

# 로그에 찍을 이름
LABELS = {
    0: '미감지',
    1: '파랑 -> Place 1',
    2: '초록 -> Place 2',
}

# 영상에 얹을 글자와 색 (BGR)
OVERLAY = {
    0: ('NONE', (200, 200, 200)),
    1: ('BLUE', (255, 80, 0)),
    2: ('GREEN', (0, 200, 0)),
}


class ColorDetector(Node):

    def __init__(self):
        super().__init__('m0609_color_detector')

        self.bridge = CvBridge()
        self.count = 0

        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(Image, IMAGE_TOPIC, self.on_image, qos)

        self.id_pub = self.create_publisher(Int32, ID_TOPIC, 10)
        self.debug_pub = self.create_publisher(Image, DEBUG_TOPIC, 10)

        self.get_logger().info(f'{IMAGE_TOPIC} 구독 시작')

    def on_image(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        color_id, blue_area, green_area = classify(frame)

        self.id_pub.publish(Int32(data=color_id))
        self.debug_pub.publish(self.make_debug(frame, color_id))

        self.count += 1
        if self.count % LOG_EVERY == 1:
            self.get_logger().info(
                f'color_id = {color_id} ({LABELS[color_id]})  '
                f'blue_area={blue_area}  green_area={green_area}'
            )

    def make_debug(self, frame, color_id):
        """판별 결과를 좌측 상단에 적은 영상을 만든다."""
        text, color = OVERLAY[color_id]
        vis = frame.copy()
        cv2.putText(
            vis,
            f'color_id = {color_id}  {text} (id={color_id})',
            (10, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            color,
            2,
        )
        return self.bridge.cv2_to_imgmsg(vis, encoding='bgr8')


def main():
    rclpy.init()
    node = ColorDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
