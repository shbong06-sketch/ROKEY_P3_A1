#!/usr/bin/env python3

from collections import deque

import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from sensor_msgs.msg import Image
from std_msgs.msg import Int32
from cv_bridge import CvBridge


class M0609ColorDetector(Node):

    def __init__(self):
        super().__init__('m0609_color_detector')

        # ---------------------------------------------------------
        # Parameters
        # ---------------------------------------------------------
        self.declare_parameter('min_pixels', 500)
        self.declare_parameter('roi_ratio', 0.40)

        self.min_pixels = (
            self.get_parameter('min_pixels')
            .get_parameter_value()
            .integer_value
        )

        self.roi_ratio = (
            self.get_parameter('roi_ratio')
            .get_parameter_value()
            .double_value
        )

        # ---------------------------------------------------------
        # CV Bridge
        # ---------------------------------------------------------
        self.bridge = CvBridge()

        # ---------------------------------------------------------
        # 5-frame stability
        # ---------------------------------------------------------
        self.stability_buffer = deque(maxlen=5)
        self.stable_color = 0

        # 로그용. 튜닝하려면 실제 픽셀 수를 봐야 한다
        self.frame_count = 0
        self.blue_pixels = 0
        self.green_pixels = 0

        # ---------------------------------------------------------
        # QoS
        # /rgb is expected to be a sensor-data stream.
        # BEST_EFFORT avoids blocking when frames are dropped.
        # ---------------------------------------------------------
        rgb_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )

        # ---------------------------------------------------------
        # Subscribers
        # ---------------------------------------------------------
        self.rgb_sub = self.create_subscription(
            Image,
            '/rgb',
            self.rgb_callback,
            rgb_qos,
        )

        # ---------------------------------------------------------
        # Publishers
        # ---------------------------------------------------------
        self.color_pub = self.create_publisher(
            Int32,
            '/color_id',
            10,
        )

        self.debug_pub = self.create_publisher(
            Image,
            '/color_debug',
            10,
        )

        # ---------------------------------------------------------
        # 10 Hz publishing
        # ---------------------------------------------------------
        self.latest_color_id = 0

        self.timer = self.create_timer(
            0.1,
            self.publish_color_id,
        )

        self.get_logger().info(
            'M0609 Color Detector started'
        )

        self.get_logger().info(
            f'min_pixels={self.min_pixels}, '
            f'roi_ratio={self.roi_ratio}'
        )

    # =============================================================
    # RGB callback
    # =============================================================
    def rgb_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(
                msg,
                desired_encoding='bgr8'
            )

        except Exception as e:
            self.get_logger().error(
                f'Failed to convert image: {e}'
            )
            return

        color_id = self.detect_color(frame)

        # ---------------------------------------------------------
        # 5-frame stability
        # ---------------------------------------------------------
        self.stability_buffer.append(color_id)

        if len(self.stability_buffer) == 5:

            if all(
                value == self.stability_buffer[0]
                for value in self.stability_buffer
            ):
                self.stable_color = self.stability_buffer[0]

        # Current result
        self.latest_color_id = self.stable_color

        # ---------------------------------------------------------
        # Periodic log
        # ---------------------------------------------------------
        self.frame_count += 1

        if self.frame_count % 30 == 1:
            self.get_logger().info(
                f'color_id={self.stable_color} '
                f'({self.color_name(self.stable_color)})  '
                f'blue={self.blue_pixels}  '
                f'green={self.green_pixels}  '
                f'min={self.min_pixels}'
            )

        # ---------------------------------------------------------
        # Debug image
        # ---------------------------------------------------------
        debug_frame = self.create_debug_image(
            frame,
            color_id,
            self.stable_color,
        )

        try:
            debug_msg = self.bridge.cv2_to_imgmsg(
                debug_frame,
                encoding='bgr8'
            )

            debug_msg.header = msg.header

            self.debug_pub.publish(debug_msg)

        except Exception as e:
            self.get_logger().error(
                f'Failed to publish debug image: {e}'
            )

    # =============================================================
    # Color detection
    # =============================================================
    def detect_color(self, frame):

        height, width = frame.shape[:2]

        # ---------------------------------------------------------
        # Central ROI
        # roi_ratio = 0.40
        # ---------------------------------------------------------
        roi_width = int(width * self.roi_ratio)
        roi_height = int(height * self.roi_ratio)

        x1 = (width - roi_width) // 2
        y1 = (height - roi_height) // 2

        x2 = x1 + roi_width
        y2 = y1 + roi_height

        roi = frame[y1:y2, x1:x2]

        # ---------------------------------------------------------
        # BGR -> HSV
        # ---------------------------------------------------------
        hsv = cv2.cvtColor(
            roi,
            cv2.COLOR_BGR2HSV
        )

        # ---------------------------------------------------------
        # Blue
        # H: 100-130
        # S: 80-255
        # V: 50-255
        # ---------------------------------------------------------
        blue_lower = np.array(
            [100, 80, 50],
            dtype=np.uint8
        )

        blue_upper = np.array(
            [130, 255, 255],
            dtype=np.uint8
        )

        blue_mask = cv2.inRange(
            hsv,
            blue_lower,
            blue_upper
        )

        # ---------------------------------------------------------
        # Green
        # H: 40-85
        # S: 80-255
        # V: 50-255
        # ---------------------------------------------------------
        green_lower = np.array(
            [40, 80, 50],
            dtype=np.uint8
        )

        green_upper = np.array(
            [85, 255, 255],
            dtype=np.uint8
        )

        green_mask = cv2.inRange(
            hsv,
            green_lower,
            green_upper
        )

        blue_pixels = cv2.countNonZero(blue_mask)
        green_pixels = cv2.countNonZero(green_mask)

        self.blue_pixels = blue_pixels
        self.green_pixels = green_pixels

        # ---------------------------------------------------------
        # Minimum pixel threshold
        # ---------------------------------------------------------
        blue_detected = blue_pixels >= self.min_pixels
        green_detected = green_pixels >= self.min_pixels

        # ---------------------------------------------------------
        # Color decision
        # If both satisfy the threshold, select the color
        # with more detected pixels.
        # ---------------------------------------------------------
        if blue_detected and green_detected:

            if blue_pixels >= green_pixels:
                return 1

            return 2

        if blue_detected:
            return 1

        if green_detected:
            return 2

        return 0

    # =============================================================
    # Debug image
    # =============================================================
    def create_debug_image(
        self,
        frame,
        current_color,
        stable_color,
    ):

        debug = frame.copy()

        height, width = debug.shape[:2]

        roi_width = int(width * self.roi_ratio)
        roi_height = int(height * self.roi_ratio)

        x1 = (width - roi_width) // 2
        y1 = (height - roi_height) // 2

        x2 = x1 + roi_width
        y2 = y1 + roi_height

        # ROI rectangle
        cv2.rectangle(
            debug,
            (x1, y1),
            (x2, y2),
            (255, 255, 255),
            2,
        )

        # ---------------------------------------------------------
        # Text
        # ---------------------------------------------------------
        current_text = self.color_name(current_color)
        stable_text = self.color_name(stable_color)

        cv2.putText(
            debug,
            f'Current: {current_text}',
            (20, 40),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.9,
            (255, 255, 255),
            2,
        )

        cv2.putText(
            debug,
            f'Stable: {stable_text}',
            (20, 80),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.9,
            (255, 255, 255),
            2,
        )

        cv2.putText(
            debug,
            f'Pixels >= {self.min_pixels}',
            (20, 120),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (255, 255, 255),
            2,
        )

        return debug

    # =============================================================
    # Color ID -> text
    # =============================================================
    @staticmethod
    def color_name(color_id):

        if color_id == 1:
            return 'BLUE'

        if color_id == 2:
            return 'GREEN'

        return 'NONE'

    # =============================================================
    # Publish color ID at 10 Hz
    # =============================================================
    def publish_color_id(self):

        msg = Int32()
        msg.data = int(self.latest_color_id)

        self.color_pub.publish(msg)


def main(args=None):

    rclpy.init(args=args)

    node = M0609ColorDetector()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
