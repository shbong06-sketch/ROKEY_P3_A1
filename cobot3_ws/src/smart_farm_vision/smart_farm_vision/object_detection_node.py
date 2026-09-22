
from __future__ import annotations

from concurrent.futures import Future, ThreadPoolExecutor
import copy
import threading
import time

import cv2
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import rclpy

from rclpy.node import Node
from rclpy.qos import (
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)
from sensor_msgs.msg import Image

from .yolo_detector import Detection, DetectorConfig, YoloDetector


class ObjectDetectionNode(Node):
    """카메라 영상을 수신하고 최신 BGR 프레임을 유지한다."""

    def __init__(self) -> None:
        super().__init__('object_detection')

        # 1. parameter 선언
        self.declare_parameter('camera_topic', '/rgb')
        self.declare_parameter('image_timeout_sec', 3.0)

        self.declare_parameter('model_path', '/models/best.pt')
        self.declare_parameter('device', 'cuda:0')
        self.declare_parameter('confidence_threshold', 0.5)
        self.declare_parameter('iou_threshold', 0.45)
        self.declare_parameter('image_size', 640)
        self.declare_parameter('allow_cpu_fallback', False)

        self.declare_parameter('min_cycle_sec', 0.1)
        self.declare_parameter('continuous_inference', False)
        self.declare_parameter(
            'debug_image_topic',
            '/inspection/debug_image',
        )

        # 2. parameter 읽기
        self._camera_topic = str(
            self.get_parameter('camera_topic').value
        )
        self._image_timeout_sec = float(
            self.get_parameter('image_timeout_sec').value
        )

        self._model_path = str(
            self.get_parameter('model_path').value
        )
        self._device = str(
            self.get_parameter('device').value
        )
        self._confidence_threshold = float(
            self.get_parameter('confidence_threshold').value
        )
        self._iou_threshold = float(
            self.get_parameter('iou_threshold').value
        )
        self._image_size = int(
            self.get_parameter('image_size').value
        )
        self._allow_cpu_fallback = bool(
            self.get_parameter('allow_cpu_fallback').value
        )

        self._min_cycle_sec = float(
            self.get_parameter('min_cycle_sec').value
        )
        self._continuous_inference = bool(
            self.get_parameter('continuous_inference').value
        )
        self._debug_image_topic = str(
            self.get_parameter('debug_image_topic').value
        )

        # 3. parameter 검증
        if not self._camera_topic:
            raise ValueError('camera_topic must not be empty')

        if self._image_timeout_sec <= 0.0:
            raise ValueError('image_timeout_sec must be greater than 0')

        if not self._model_path:
            raise ValueError('model_path must not be empty')

        if self._min_cycle_sec <= 0.0:
            raise ValueError(
                'min_cycle_sec must be greater than 0'
            )

        if not self._debug_image_topic:
            raise ValueError(
                'debug_image_topic must not be empty'
            )

        detector_config = DetectorConfig(
            model_path=self._model_path,
            device=self._device,
            confidence_threshold=self._confidence_threshold,
            iou_threshold=self._iou_threshold,
            image_size=self._image_size,
            class_filter=(0, 1, 2),
            allow_cpu_fallback=self._allow_cpu_fallback,
        )

        try:
            # 4. detector 한 번만 초기화
            self._detector = YoloDetector(detector_config)
        except Exception as exc:
            self.get_logger().fatal(
                'Failed to initialize YOLO detector: '
                f'model_path={self._model_path}, '
                f'device={self._device}, '
                f'error={type(exc).__name__}: {exc}'
            )
            raise

        self.get_logger().info(
            'YOLO detector initialized: '
            f'model_path={self._model_path}, '
            f'device={self._detector.device}, '
            f'confidence={self._confidence_threshold}, '
            f'iou={self._iou_threshold}, '
            f'image_size={self._image_size}'
        )

        # 5. CvBridge 및 영상 상태 초기화
        self._bridge = CvBridge()

        self._started_at = time.monotonic()
        self._last_image_received_at: float | None = None
        self._last_encoding: str | None = None
        self._last_conversion_warning_at = 0.0
        self._camera_timed_out = False

        self._frame_lock = threading.Lock()
        self._latest_frame: np.ndarray | None = None
        self._latest_header = None
        self._frame_sequence = 0

        self._last_submitted_sequence = 0
        self._last_inference_started_at = 0.0
        self._inference_future: Future | None = None
        self._stopping = False

        self._inference_executor = ThreadPoolExecutor(
            max_workers=1,
            thread_name_prefix='yolo-inference',
        )

        # 6. QoS와 subscription 생성
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

        self._debug_publisher = self.create_publisher(
            Image,
            self._debug_image_topic,
            camera_qos,
        )

        # 7. timeout timer 생성
        self._timeout_timer = self.create_timer(
            1.0,
            self._check_image_timeout,
        )

        self._inference_timer = self.create_timer(
            self._min_cycle_sec,
            self._inference_tick,
        )

        self.get_logger().info(
            f'Waiting for camera images: topic={self._camera_topic}, '
            f'timeout={self._image_timeout_sec:.1f}s, '
            f'debug_topic={self._debug_image_topic}'
        )

    def _image_callback(self, message: Image) -> None:
        now = time.monotonic()
        encoding = message.encoding.strip().lower()

        self._last_image_received_at = now
        self._last_encoding = encoding

        if self._camera_timed_out:
            self._camera_timed_out = False
            self.get_logger().info(
                f'Camera image stream recovered: '
                f'topic={self._camera_topic}, encoding={encoding}'
            )

        try:
            bgr_image = self._convert_to_bgr(message)
        except (CvBridgeError, cv2.error, ValueError) as exc:
            self._log_conversion_warning(encoding, exc)
            return

        # CvBridge 결과가 원본 메시지 메모리를 참조할 가능성을 피한다.
        frame = np.ascontiguousarray(bgr_image).copy()
        header = copy.deepcopy(message.header)

        with self._frame_lock:
            self._latest_frame = frame
            self._latest_header = header
            self._frame_sequence += 1

    def _convert_to_bgr(self, message: Image) -> np.ndarray:
        encoding = message.encoding.strip().lower()
        image = self._bridge.imgmsg_to_cv2(
            message,
            desired_encoding='passthrough',
        )

        if encoding == 'bgr8':
            bgr_image = image
        elif encoding == 'rgb8':
            bgr_image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
        elif encoding == 'rgba8':
            bgr_image = cv2.cvtColor(image, cv2.COLOR_RGBA2BGR)
        elif encoding == 'bgra8':
            bgr_image = cv2.cvtColor(image, cv2.COLOR_BGRA2BGR)
        elif encoding == 'mono8':
            bgr_image = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
        else:
            raise ValueError(
                f'unsupported image encoding: {message.encoding!r}'
            )

        if (
            not isinstance(bgr_image, np.ndarray)
            or bgr_image.dtype != np.uint8
            or bgr_image.ndim != 3
            or bgr_image.shape[2] != 3
        ):
            raise ValueError(
                'converted image must be a uint8 HxWx3 BGR array'
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
            f'Failed to convert camera image: '
            f'encoding={encoding!r}, error={error}'
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
        encoding = self._last_encoding or 'unknown'

        self.get_logger().warning(
            f'Camera image timeout: no image for {elapsed:.1f}s, '
            f'topic={self._camera_topic}, '
            f'last_encoding={encoding}'
        )

    def _run_inference(
        self,
        frame: np.ndarray,
        header,
        frame_sequence: int,
    ):
        detections = self._detector.detect(frame)

        return (
            frame_sequence,
            frame,
            header,
            detections,
        )

    @staticmethod
    def _draw_detections(
        frame: np.ndarray,
        detections: list[Detection],
    ) -> np.ndarray:
        debug_frame = frame.copy()
        height, width = debug_frame.shape[:2]

        for detection in detections:
            x1, y1, x2, y2 = (
                int(round(value))
                for value in detection.bbox_xyxy
            )

            x1 = max(0, min(x1, width - 1))
            y1 = max(0, min(y1, height - 1))
            x2 = max(0, min(x2, width - 1))
            y2 = max(0, min(y2, height - 1))

            if x2 <= x1 or y2 <= y1:
                continue

            color = (0, 255, 0)
            label = (
                f'{detection.class_name} '
                f'{detection.confidence:.2f}'
            )
            text_origin = (x1, max(20, y1 - 8))

            cv2.rectangle(
                debug_frame,
                (x1, y1),
                (x2, y2),
                color,
                2,
            )
            cv2.putText(
                debug_frame,
                label,
                text_origin,
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (0, 0, 0),
                4,
                cv2.LINE_AA,
            )
            cv2.putText(
                debug_frame,
                label,
                text_origin,
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                color,
                2,
                cv2.LINE_AA,
            )

        return debug_frame

    def _publish_debug_image(
        self,
        frame: np.ndarray,
        header,
        detections: list[Detection],
    ) -> None:
        debug_frame = self._draw_detections(
            frame,
            detections,
        )
        debug_message = self._bridge.cv2_to_imgmsg(
            debug_frame,
            encoding='bgr8',
        )
        debug_message.header = header
        self._debug_publisher.publish(debug_message)

    def _inference_tick(self) -> None:
        if self._stopping:
            return

        self._collect_inference_result()

        if not self._continuous_inference:
            return

        if self._inference_future is not None:
            return

        now = time.monotonic()

        if (
            now - self._last_inference_started_at
            < self._min_cycle_sec
        ):
            return

        with self._frame_lock:
            if self._latest_frame is None:
                return

            if self._frame_sequence == self._last_submitted_sequence:
                return

            frame = self._latest_frame
            header = copy.deepcopy(self._latest_header)
            frame_sequence = self._frame_sequence

        try:
            future = self._inference_executor.submit(
                self._run_inference,
                frame,
                header,
                frame_sequence,
            )
        except RuntimeError as exc:
            if not self._stopping:
                self.get_logger().error(
                    f'Failed to submit inference: {exc}'
                )
            return

        self._inference_future = future
        self._last_submitted_sequence = frame_sequence
        self._last_inference_started_at = now

    def _collect_inference_result(self) -> None:
        future = self._inference_future

        if future is None or not future.done():
            return

        self._inference_future = None

        try:
            (
                frame_sequence,
                frame,
                header,
                detections,
            ) = future.result()
        except Exception as exc:
            self.get_logger().error(
                'YOLO inference failed: '
                f'{type(exc).__name__}: {exc}'
            )
            return

        try:
            self._publish_debug_image(
                frame,
                header,
                detections,
            )
        except Exception as exc:
            self.get_logger().error(
                'Failed to publish debug image: '
                f'{type(exc).__name__}: {exc}'
            )
            return

        self.get_logger().debug(
            f'Inference completed: '
            f'frame={frame_sequence}, '
            f'detections={len(detections)}'
        )

    def destroy_node(self) -> bool:
        """Stop the inference worker and destroy the ROS node."""
        self._stopping = True

        if hasattr(self, '_inference_timer'):
            self._inference_timer.cancel()

        if hasattr(self, '_inference_executor'):
            self._inference_executor.shutdown(
                wait=True,
                cancel_futures=True,
            )

        return super().destroy_node()


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


if __name__ == '__main__':
    main()
