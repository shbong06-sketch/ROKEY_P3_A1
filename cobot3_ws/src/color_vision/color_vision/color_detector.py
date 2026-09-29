#!/usr/bin/env python3
"""Detect a stable green or blue object inside a configurable ROI."""

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import Int32

NONE, BLUE, GREEN = 0, 1, 2


class ColorDetector(Node):
    def __init__(self):
        super().__init__('color_detector')
        self._declare_parameters()

        self.bridge = CvBridge()
        self.candidate_id = NONE
        self.candidate_frames = 0
        self.stable_id = NONE
        self.last_image_time = None
        self.last_encoding = None
        self.timeout_logged = False

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )
        self.create_subscription(Image, '/rgb', self.image_callback, sensor_qos)
        self.color_pub = self.create_publisher(Int32, '/color_id', 10)
        self.debug_pub = self.create_publisher(Image, '/color_debug', sensor_qos)
        self.mask_pub = self.create_publisher(Image, '/color_mask', sensor_qos)

        rate = float(self.get_parameter('publish_rate_hz').value)
        if rate <= 0.0:
            raise ValueError('publish_rate_hz must be greater than zero')
        self.create_timer(1.0 / rate, self.timer_callback)
        self.get_logger().info('/rgb -> /color_id (0=none, 1=green, 2=blue)')

    def _declare_parameters(self):
        # OpenCV HSV: H=0..179, S/V=0..255
        self.declare_parameter('green_hsv_lower', [40, 80, 50])
        self.declare_parameter('green_hsv_upper', [85, 255, 255])
        self.declare_parameter('blue_hsv_lower', [100, 80, 50])
        self.declare_parameter('blue_hsv_upper', [130, 255, 255])
        # Normalized ROI: top-left x/y and width/height
        self.declare_parameter('roi_x', 0.30)
        self.declare_parameter('roi_y', 0.30)
        self.declare_parameter('roi_width', 0.40)
        self.declare_parameter('roi_height', 0.40)
        self.declare_parameter('min_area', 500.0)
        self.declare_parameter('stable_frames', 5)
        self.declare_parameter('image_timeout_sec', 0.5)
        self.declare_parameter('publish_rate_hz', 10.0)
        self.declare_parameter('morph_kernel_size', 5)

    def image_callback(self, msg):
        self.last_image_time = self.get_clock().now()
        self.timeout_logged = False
        try:
            frame = self._to_bgr(msg)
            result, green_mask, blue_mask, roi_box, areas = self._classify(frame)
        except (ValueError, RuntimeError) as error:
            self.get_logger().error(f'image processing failed: {error}')
            self._clear_result()
            return

        if msg.encoding != self.last_encoding:
            self.last_encoding = msg.encoding
            self.get_logger().info(
                f'/rgb encoding={msg.encoding!r}; internal order=BGR'
            )

        self._update_stability(result)
        self._publish_debug(
            msg, frame, green_mask, blue_mask, roi_box, areas, result
        )

    def _to_bgr(self, msg):
        """Convert common ROS encodings while preserving channel meaning."""
        encoding = msg.encoding.lower()
        image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        conversions = {
            'rgb8': cv2.COLOR_RGB2BGR,
            'bgra8': cv2.COLOR_BGRA2BGR,
            'rgba8': cv2.COLOR_RGBA2BGR,
            'mono8': cv2.COLOR_GRAY2BGR,
            '8uc1': cv2.COLOR_GRAY2BGR,
        }
        if encoding == 'bgr8':
            return image
        if encoding in conversions:
            return cv2.cvtColor(image, conversions[encoding])
        return self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

    def _classify(self, frame):
        image_height, image_width = frame.shape[:2]
        x1, y1, x2, y2 = self._roi_pixels(image_width, image_height)
        hsv = cv2.cvtColor(frame[y1:y2, x1:x2], cv2.COLOR_BGR2HSV)

        green_mask = cv2.inRange(
            hsv, self._hsv('green_hsv_lower'), self._hsv('green_hsv_upper')
        )
        blue_mask = cv2.inRange(
            hsv, self._hsv('blue_hsv_lower'), self._hsv('blue_hsv_upper')
        )

        kernel_size = int(self.get_parameter('morph_kernel_size').value)
        if kernel_size > 1:
            kernel = np.ones((kernel_size, kernel_size), np.uint8)
            for mask in (green_mask, blue_mask):
                cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, dst=mask)
                cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, dst=mask)

        green_area = self._largest_area(green_mask)
        blue_area = self._largest_area(blue_mask)
        minimum = float(self.get_parameter('min_area').value)
        green_found = green_area >= minimum
        blue_found = blue_area >= minimum

        # Both colors at once are ambiguous.
        if green_found == blue_found:
            result = NONE
        elif green_found:
            result = GREEN
        else:
            result = BLUE

        return (
            result,
            green_mask,
            blue_mask,
            (x1, y1, x2, y2),
            (green_area, blue_area),
        )

    def _roi_pixels(self, image_width, image_height):
        x = float(self.get_parameter('roi_x').value)
        y = float(self.get_parameter('roi_y').value)
        width = float(self.get_parameter('roi_width').value)
        height = float(self.get_parameter('roi_height').value)
        if not (0.0 <= x < 1.0 and 0.0 <= y < 1.0):
            raise ValueError('roi_x and roi_y must be in [0.0, 1.0)')
        if width <= 0.0 or height <= 0.0:
            raise ValueError('ROI width and height must be positive')

        x1, y1 = int(image_width * x), int(image_height * y)
        x2 = min(image_width, int(image_width * (x + width)))
        y2 = min(image_height, int(image_height * (y + height)))
        if x2 <= x1 or y2 <= y1:
            raise ValueError('configured ROI is outside the image')
        return x1, y1, x2, y2

    def _hsv(self, name):
        values = self.get_parameter(name).value
        if len(values) != 3:
            raise ValueError(f'{name} must contain 3 integers')
        return np.asarray(values, dtype=np.uint8)

    @staticmethod
    def _largest_area(mask):
        contours, _ = cv2.findContours(
            mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )
        return max((cv2.contourArea(c) for c in contours), default=0.0)

    def _update_stability(self, current_id):
        # Missing/ambiguous frames clear an old result immediately.
        if current_id == NONE:
            self._clear_result()
            return
        if current_id == self.candidate_id:
            self.candidate_frames += 1
        else:
            self.candidate_id = current_id
            self.candidate_frames = 1

        required = max(1, int(self.get_parameter('stable_frames').value))
        if self.candidate_frames >= required:
            self.stable_id = current_id

    def _clear_result(self):
        self.candidate_id = NONE
        self.candidate_frames = 0
        self.stable_id = NONE

    def timer_callback(self):
        timeout = float(self.get_parameter('image_timeout_sec').value)
        now = self.get_clock().now()
        stale = self.last_image_time is None or (
            (now - self.last_image_time).nanoseconds * 1e-9 > timeout
        )
        if stale:
            self._clear_result()
            if not self.timeout_logged:
                self.get_logger().warning('/rgb timeout: color_id=0')
                self.timeout_logged = True

        message = Int32()
        message.data = self.stable_id
        self.color_pub.publish(message)

    def _publish_debug(
        self, source, frame, green_mask, blue_mask, roi, areas, raw_id
    ):
        x1, y1, x2, y2 = roi
        green_area, blue_area = areas
        debug = frame.copy()
        debug_roi = debug[y1:y2, x1:x2]
        debug_roi[green_mask > 0] = (0, 255, 0)
        debug_roi[blue_mask > 0] = (255, 0, 0)
        cv2.rectangle(debug, (x1, y1), (x2, y2), (255, 255, 255), 2)
        text = (
            f'raw={self._name(raw_id)} stable={self._name(self.stable_id)} '
            f'G={green_area:.0f} B={blue_area:.0f}'
        )
        cv2.putText(
            debug, text, (20, 35), cv2.FONT_HERSHEY_SIMPLEX,
            0.7, (255, 255, 255), 2, cv2.LINE_AA
        )

        # Mono mask values: green=127, blue=255, overlap=200.
        combined = np.zeros_like(green_mask)
        combined[green_mask > 0] = 127
        combined[blue_mask > 0] = 255
        combined[(green_mask > 0) & (blue_mask > 0)] = 200

        debug_msg = self.bridge.cv2_to_imgmsg(debug, encoding='bgr8')
        debug_msg.header = source.header
        self.debug_pub.publish(debug_msg)
        mask_msg = self.bridge.cv2_to_imgmsg(combined, encoding='mono8')
        mask_msg.header = source.header
        self.mask_pub.publish(mask_msg)

    @staticmethod
    def _name(color_id):
        return {NONE: 'NONE', GREEN: 'GREEN', BLUE: 'BLUE'}[color_id]


def main(args=None):
    rclpy.init(args=args)
    node = ColorDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
