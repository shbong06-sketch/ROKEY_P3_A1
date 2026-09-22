"""Object detection ROS node tests without a real YOLO model."""

import time

from cv_bridge import CvBridge
import numpy as np
import pytest
import rclpy

import smart_farm_vision.object_detection_node as node_module
from smart_farm_vision.yolo_detector import Detection


class FakeDetector:
    """Minimal detector used by node tests."""

    device = 'cpu'

    def __init__(self, detections=None, failures=0):
        self.detections = detections or []
        self.failures = failures
        self.detect_calls = 0

    def detect(self, _frame):
        """Return configured detections or a configured failure."""
        self.detect_calls += 1

        if self.failures > 0:
            self.failures -= 1
            raise RuntimeError('mock inference failure')

        return self.detections


class FakePublisher:
    """Collect published messages for assertions."""

    def __init__(self):
        self.messages = []

    def publish(self, message):
        """Store one published message."""
        self.messages.append(message)


@pytest.fixture
def node_factory(monkeypatch):
    """Create nodes with a mocked detector and clean ROS context."""
    nodes = []

    if not rclpy.ok():
        rclpy.init()

    def create_node(detector=None):
        fake_detector = detector or FakeDetector()
        monkeypatch.setattr(
            node_module,
            'YoloDetector',
            lambda _config: fake_detector,
        )
        node = node_module.ObjectDetectionNode()
        nodes.append(node)
        return node, fake_detector

    yield create_node

    for node in nodes:
        if not node._stopping:
            node.destroy_node()

    if rclpy.ok():
        rclpy.shutdown()


def make_image(encoding='bgr8'):
    """Create a small ROS Image with a stable header."""
    frame = np.zeros((80, 120, 3), dtype=np.uint8)
    message = CvBridge().cv2_to_imgmsg(
        frame,
        encoding='bgr8',
    )
    message.encoding = encoding
    message.header.frame_id = 'camera_rgb'
    message.header.stamp.sec = 123
    message.header.stamp.nanosec = 456
    return message


def wait_for_future(node):
    """Wait for the current worker future."""
    future = node._inference_future
    assert future is not None
    return future.result(timeout=2.0)


def test_worker_publishes_debug_image_and_skips_duplicate(
    node_factory,
):
    detection = Detection(
        class_id=0,
        class_name='lettuce_dark_green',
        confidence=0.9,
        bbox_xyxy=(10.0, 10.0, 60.0, 50.0),
    )
    node, detector = node_factory(
        FakeDetector(detections=[detection])
    )
    publisher = FakePublisher()
    node._debug_publisher = publisher
    node._continuous_inference = True
    node._min_cycle_sec = 0.001

    node._image_callback(make_image())
    node._inference_tick()
    wait_for_future(node)
    node._inference_tick()

    assert detector.detect_calls == 1
    assert len(publisher.messages) == 1

    debug_message = publisher.messages[0]
    assert debug_message.encoding == 'bgr8'
    assert debug_message.header.frame_id == 'camera_rgb'
    assert debug_message.header.stamp.sec == 123
    assert debug_message.header.stamp.nanosec == 456

    debug_frame = CvBridge().imgmsg_to_cv2(
        debug_message,
        desired_encoding='bgr8',
    )
    assert np.any(debug_frame != 0)

    node._last_inference_started_at = 0.0
    node._inference_tick()
    assert detector.detect_calls == 1


def test_min_cycle_blocks_new_frame_until_interval_passes(
    node_factory,
):
    node, detector = node_factory()
    node._continuous_inference = True

    node._image_callback(make_image())
    node._inference_tick()
    wait_for_future(node)
    node._inference_tick()
    assert detector.detect_calls == 1

    node._image_callback(make_image())
    node._last_inference_started_at = time.monotonic()
    node._inference_tick()

    assert node._inference_future is None
    assert detector.detect_calls == 1

    node._last_inference_started_at = 0.0
    node._inference_tick()
    wait_for_future(node)
    node._inference_tick()

    assert detector.detect_calls == 2


def test_inference_failure_does_not_stop_worker(node_factory):
    node, detector = node_factory(
        FakeDetector(failures=1)
    )
    node._continuous_inference = True
    node._min_cycle_sec = 0.001

    node._image_callback(make_image())
    node._inference_tick()

    with pytest.raises(RuntimeError, match='mock inference failure'):
        wait_for_future(node)

    node._inference_tick()
    assert node._inference_future is None
    assert not node._stopping

    node._image_callback(make_image())
    node._last_inference_started_at = 0.0
    node._inference_tick()
    wait_for_future(node)
    node._inference_tick()

    assert detector.detect_calls == 2
    assert not node._stopping


def test_unsupported_encoding_drops_only_that_frame(node_factory):
    node, _detector = node_factory()

    node._image_callback(make_image('unsupported'))

    assert node._frame_sequence == 0
    assert node._latest_frame is None
    assert node._last_encoding == 'unsupported'
    assert not node._stopping


def test_camera_timeout_and_recovery(node_factory):
    node, _detector = node_factory()
    node._started_at = time.monotonic() - 10.0

    node._check_image_timeout()
    assert node._camera_timed_out

    node._image_callback(make_image())
    assert not node._camera_timed_out
    assert node._last_encoding == 'bgr8'


def test_destroy_node_stops_worker(node_factory):
    node, _detector = node_factory()

    assert not node._stopping
    node.destroy_node()

    assert node._stopping
    with pytest.raises(RuntimeError):
        node._inference_executor.submit(lambda: None)
