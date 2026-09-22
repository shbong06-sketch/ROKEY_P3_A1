
from __future__ import annotations

import copy
import time

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge, CvBridgeError
from rclpy.node import Node
from rclpy.qos import (
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)
from sensor_msgs.msg import Image


class ObjectDetectionNode(Node):
    """카메라 영상을 수신하고 최신 BGR 프레임을 유지한다."""

    def __init__(self) -> None:
        super().__init__("object_detection")

        self.declare_parameter("camera_topic", "/camera/image_raw")
        self.declare_parameter("image_timeout_sec", 3.0)

        self._camera_topic = (
            self.get_parameter("camera_topic")
            .get_parameter_value()
            .string_value
        )
        self._image_timeout_sec = (
            self.get_parameter("image_timeout_sec")
            .get_parameter_value()
            .double_value
        )

        if not self._camera_topic:
            raise ValueError("camera_topic must not be empty")

        if self._image_timeout_sec <= 0.0:
            raise ValueError("image_timeout_sec must be greater than 0")

        self._bridge = CvBridge()

        self._started_at = time.monotonic()
        self._last_image_received_at: float | None = None
        self._last_encoding: str | None = None
        self._last_conversion_warning_at = 0.0
        self._camera_timed_out = False

        self._latest_frame: np.ndarray | None = None
        self._latest_header = None
        self._frame_sequence = 0

        camera_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
        )

        self._image_subscription = self.create_subscription(
            Image,
            self._camera_topic,
            self._image_callback,
            camera_qos,
        )

        self._timeout_timer = self.create_timer(
            1.0,
            self._check_image_timeout,
        )

        self.get_logger().info(
            f"Waiting for camera images: topic={self._camera_topic}, "
            f"timeout={self._image_timeout_sec:.1f}s"
        )

    def _image_callback(self, message: Image) -> None:
        now = time.monotonic()
        encoding = message.encoding.strip().lower()

        self._last_image_received_at = now
        self._last_encoding = encoding

        if self._camera_timed_out:
            self._camera_timed_out = False
            self.get_logger().info(
                f"Camera image stream recovered: "
                f"topic={self._camera_topic}, encoding={encoding}"
            )

        try:
            bgr_image = self._convert_to_bgr(message)
        except (CvBridgeError, cv2.error, ValueError) as exc:
            self._log_conversion_warning(encoding, exc)
            return

        # CvBridge 결과가 원본 메시지 메모리를 참조할 가능성을 피한다.
        self._latest_frame = np.ascontiguousarray(bgr_image).copy()
        self._latest_header = copy.deepcopy(message.header)
        self._frame_sequence += 1

    def _convert_to_bgr(self, message: Image) -> np.ndarray:
        encoding = message.encoding.strip().lower()
        image = self._bridge.imgmsg_to_cv2(
            message,
            desired_encoding="passthrough",
        )

        if encoding == "bgr8":
            bgr_image = image
        elif encoding == "rgb8":
            bgr_image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
        elif encoding == "rgba8":
            bgr_image = cv2.cvtColor(image, cv2.COLOR_RGBA2BGR)
        elif encoding == "bgra8":
            bgr_image = cv2.cvtColor(image, cv2.COLOR_BGRA2BGR)
        elif encoding == "mono8":
            bgr_image = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
        else:
            raise ValueError(
                f"unsupported image encoding: {message.encoding!r}"
            )

        if (
            not isinstance(bgr_image, np.ndarray)
            or bgr_image.dtype != np.uint8
            or bgr_image.ndim != 3
            or bgr_image.shape[2] != 3
        ):
            raise ValueError(
                "converted image must be a uint8 HxWx3 BGR array"
            )

        return bgr_image

    def _log_conversion_warning(
        self,
        encoding: str,
        error: Exception,
    ) -> None:
        now = time.monotonic()

        # 잘못된 프레임마다 로그를 출력하지 않고 5초마다 한 번만 출력한다.
        if now - self._last_conversion_warning_at < 5.0:
            return

        self._last_conversion_warning_at = now
        self.get_logger().warning(
            f"Failed to convert camera image: "
            f"encoding={encoding!r}, error={error}"
        )

    def _check_image_timeout(self) -> None:
        now = time.monotonic()
        reference_time = (
            self._last_image_received_at
            if self._last_image_received_at is not None
            else self._started_at
        )

        elapsed = now - reference_time

        if elapsed <= self._image_timeout_sec:
            return

        if self._camera_timed_out:
            return

        self._camera_timed_out = True
        encoding = self._last_encoding or "unknown"

        self.get_logger().warning(
            f"Camera image timeout: no image for {elapsed:.1f}s, "
            f"topic={self._camera_topic}, "
            f"last_encoding={encoding}"
        )


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node: ObjectDetectionNode | None = None

    try:
        node = ObjectDetectionNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
