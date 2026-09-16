#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Int32
from cv_bridge import CvBridge
import cv2
import numpy as np


class ColorDetector(Node):
    def __init__(self):
        super().__init__('color_detector')
        self.sub = self.create_subscription(Image, '/rgb', self.image_callback, 10)
        self.pub = self.create_publisher(Int32, '/color_id', 10)
        self.bridge = CvBridge()
        self.get_logger().info('색상_감지_노드_시작시작')

    def image_callback(self, msg: Image):
        # BGR 이미지를 바로 가져옴 ([B, G, R] 순서)
        img = self.bridge.imgmsg_to_cv2(msg, 'bgr8')

        # BGR 채널 범위 설정 (파란색: B가 높음 / 초록색: G가 높음)
        blue_mask = cv2.inRange(img, np.array([150, 0, 0]), np.array([255, 100, 100]))
        green_mask = cv2.inRange(img, np.array([0, 150, 0]), np.array([100, 255, 100]))

        b_cnt = cv2.countNonZero(blue_mask)
        g_cnt = cv2.countNonZero(green_mask)

        # 픽셀 수 500개 이상일 때만 판별
        if b_cnt > 500 and b_cnt > g_cnt:
            self.pub.publish(Int32(data=1))
            self.get_logger().info(f'blue판정(픽셀: {b_cnt}) -> ID: 1')
        elif g_cnt > 500 and g_cnt > b_cnt:
            self.pub.publish(Int32(data=2))
            self.get_logger().info(f'green판정(픽셀: {g_cnt}) -> ID: 2')


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(ColorDetector())
    rclpy.shutdown()


if __name__ == '__main__':
    main()