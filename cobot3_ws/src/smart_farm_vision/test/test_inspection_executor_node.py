"""Tests for the command-driven inspection executor."""

import json
import time

from cv_bridge import CvBridge
import numpy as np
import pytest
import rclpy

from smart_farm_interfaces.msg import TaskCommand
import cobot3_ws.src.smart_farm_vision.smart_farm_vision.object_detection_node as executor_module
from smart_farm_vision.yolo_detector import Detection
from smart_farm_vision.yolo_detector import EXPECTED_CLASS_NAMES


class FakeDetector:
    """Small detector double used without loading a model."""

    def __init__(self, detections=None, error=None):
        self.class_names = EXPECTED_CLASS_NAMES
        self.device = 'cpu'
        self.detections = detections or []
        self.error = error
        self.calls = 0

    def detect(self, image):
        """Return configured detections or raise the configured error."""
        self.calls += 1
        if self.error is not None:
            raise self.error
        return list(self.detections)


class FakePublisher:
    """Capture messages and optional cross-publisher ordering."""

    def __init__(self, name, events=None):
        self.name = name
        self.events = events
        self.messages = []

    def publish(self, message):
        """Record one publication."""
        self.messages.append(message)
        if self.events is not None:
            self.events.append(self.name)


@pytest.fixture
def node_factory(monkeypatch):
    """Create nodes whose detector is replaced before construction."""
    if not rclpy.ok():
        rclpy.init()

    nodes = []

    def factory(detector=None):
        fake_detector = detector or FakeDetector()
        monkeypatch.setattr(
            executor_module,
            'YoloDetector',
            lambda config: fake_detector,
        )
        node = executor_module.InspectionExecutorNode()
        nodes.append(node)
        return node, fake_detector

    yield factory

    for node in nodes:
        if not node._stopping:
            node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


def make_image(stamp_sec=1):
    """Return a 300 x 200 BGR ROS image."""
    image = CvBridge().cv2_to_imgmsg(
        np.zeros((200, 300, 3), dtype=np.uint8),
        encoding='bgr8',
    )
    image.header.frame_id = 'camera_rgb'
    image.header.stamp.sec = stamp_sec
    return image


def make_command(
    command_id='CMD-001',
    operation='INSPECT',
    task_id='TASK-001',
    pallet_id='PALLET_001',
):
    """Return a valid inspection command unless a field is overridden."""
    message = TaskCommand()
    message.task_id = task_id
    message.command_id = command_id
    message.operation = operation
    message.pallet_id = pallet_id
    return message


def make_detections(class_names):
    """Place detections at the centers of the six default slot ROIs."""
    centers = (
        (50.0, 50.0),
        (150.0, 50.0),
        (250.0, 50.0),
        (50.0, 150.0),
        (150.0, 150.0),
        (250.0, 150.0),
    )
    class_ids = {
        name: class_id
        for class_id, name in EXPECTED_CLASS_NAMES.items()
    }
    detections = []
    for center, class_name in zip(centers, class_names):
        class_id = class_ids[class_name]
        center_u, center_v = center
        detections.append(
            Detection(
                class_id=class_id,
                class_name=class_name,
                confidence=0.9,
                bbox_xyxy=(
                    center_u - 10.0,
                    center_v - 10.0,
                    center_u + 10.0,
                    center_v + 10.0,
                ),
            )
        )
    return detections


def wait_for_future(node, timeout_sec=1.0):
    """Wait until the worker future completes."""
    deadline = time.monotonic() + timeout_sec
    while node._inference_future is not None:
        if node._inference_future.done():
            return
        if time.monotonic() >= deadline:
            pytest.fail('inference worker did not complete')
        time.sleep(0.01)


def finish_command(node, command, fresh_image=None):
    """Register a command, provide a fresh frame, and collect the worker."""
    node._image_callback(make_image(stamp_sec=1))
    node._command_callback(command)
    node._inference_tick()
    assert node._inference_future is None

    node._image_callback(fresh_image or make_image(stamp_sec=2))
    node._last_inference_started_at = 0.0
    node._inference_tick()
    wait_for_future(node)
    node._inference_tick()


def replace_outputs(node, events=None):
    """Replace output publishers with recording doubles."""
    node._detections_publisher = FakePublisher('detections', events)
    node._result_publisher = FakePublisher('result', events)
    node._debug_publisher = FakePublisher('debug', events)


def test_slot_configuration_contains_exactly_six_supported_slots(node_factory):
    """Only SLOT_01 through SLOT_06 are configured."""
    node, _ = node_factory()

    assert tuple(node._slot_rois) == executor_module.SLOT_IDS
    assert 'SLOT_07' not in node._slot_rois
    assert 'SLOT_08' not in node._slot_rois


def test_slot_assessment_uses_yaml_class_outcomes(node_factory):
    """NORMAL, DEFECT, and UNKNOWN are derived from configured outcomes."""
    node, _ = node_factory()
    detections = make_detections(
        (
            'lettuce_dark_green',
            'lettuce_brown',
            'lettuce_yellow',
            'lettuce_dark_green',
            'lettuce_dark_green',
            'lettuce_dark_green',
        )
    )

    assessment = node._assess_detections(detections, 300, 200)

    assert assessment.slot_states['SLOT_01'] == 'NORMAL'
    assert assessment.slot_states['SLOT_02'] == 'DEFECT'
    assert assessment.slot_states['SLOT_03'] == 'UNKNOWN'
    assert assessment.assigned[1].slot_id == 'SLOT_02'


def test_invalid_operation_is_rejected(node_factory):
    """Only INSPECT commands are accepted."""
    node, _ = node_factory()
    replace_outputs(node)

    node._command_callback(make_command(operation='PICK'))

    result = node._result_publisher.messages[-1]
    assert result.status == 'FAILED'
    assert result.reason == 'INVALID_COMMAND'


@pytest.mark.parametrize('field', ('task_id', 'command_id', 'pallet_id'))
def test_missing_required_id_is_rejected(node_factory, field):
    """Each required identifier is validated independently."""
    node, _ = node_factory()
    replace_outputs(node)
    overrides = {field: ''}

    node._command_callback(make_command(**overrides))

    result = node._result_publisher.messages[-1]
    assert result.status == 'FAILED'
    assert result.reason == 'INVALID_ID'


def test_busy_command_is_rejected_without_replacing_active_work(node_factory):
    """A second command receives FAILED/BUSY under its own ID."""
    node, _ = node_factory()
    replace_outputs(node)
    first = make_command(command_id='CMD-001')
    second = make_command(command_id='CMD-002')

    node._command_callback(first)
    node._command_callback(second)

    result = node._result_publisher.messages[-1]
    assert result.command_id == 'CMD-002'
    assert result.status == 'FAILED'
    assert result.reason == 'BUSY'
    assert node._active_command.command_id == 'CMD-001'


def test_duplicate_terminal_command_republishes_without_inference(node_factory):
    """A cached terminal command never performs inference twice."""
    detector = FakeDetector(
        make_detections(('lettuce_dark_green',) * 6),
    )
    node, _ = node_factory(detector)
    replace_outputs(node)
    command = make_command()

    finish_command(node, command)
    node._command_callback(command)

    assert detector.calls == 1
    assert len(node._result_publisher.messages) == 2
    assert node._result_publisher.messages[0].status == 'SUCCEEDED'
    assert node._result_publisher.messages[1].status == 'SUCCEEDED'


def test_command_waits_for_a_frame_received_after_registration(node_factory):
    """The frame present before the command is not submitted."""
    node, detector = node_factory()
    replace_outputs(node)
    node._image_callback(make_image(stamp_sec=1))

    node._command_callback(make_command())
    node._inference_tick()

    assert detector.calls == 0
    assert node._inference_future is None

    node._image_callback(make_image(stamp_sec=2))
    node._last_inference_started_at = 0.0
    node._inference_tick()
    wait_for_future(node)
    node._inference_tick()
    assert detector.calls == 1


def test_detection_precedes_result_and_preserves_image_metadata(node_factory):
    """Detection metadata comes from the inferred RGB image."""
    detector = FakeDetector(
        make_detections(
            ('lettuce_dark_green', 'lettuce_brown')
            + ('lettuce_dark_green',) * 4
        )
    )
    node, _ = node_factory(detector)
    events = []
    replace_outputs(node, events)

    finish_command(node, make_command(), make_image(stamp_sec=42))

    assert [event for event in events if event != 'debug'] == [
        'detections',
        'result',
    ]
    detection_message = node._detections_publisher.messages[-1]
    payload = json.loads(detection_message.data)
    result = node._result_publisher.messages[-1]
    assert payload['header']['stamp']['sec'] == 42
    assert payload['header']['stamp']['nanosec'] == 0
    assert payload['header']['frame_id'] == 'camera_rgb'
    assert payload['image_width'] == 300
    assert payload['image_height'] == 200
    assert payload['task_id'] == 'TASK-001'
    assert payload['command_id'] == 'CMD-001'
    assert payload['pallet_id'] == 'PALLET_001'
    assert payload['detections'][0]['center_u'] == pytest.approx(50.0)
    assert result.status == 'SUCCEEDED'
    assert list(result.defect_slots) == ['SLOT_02']


def test_missing_slot_produces_unknown_slot_failure(node_factory):
    """All six slots must contain exactly one classifiable detection."""
    detector = FakeDetector(
        make_detections(('lettuce_dark_green',) * 5),
    )
    node, _ = node_factory(detector)
    replace_outputs(node)

    finish_command(node, make_command())

    result = node._result_publisher.messages[-1]
    assert result.status == 'FAILED'
    assert result.reason == 'UNKNOWN_SLOT'
    assert list(result.unknown_slots) == ['SLOT_06']


def test_duplicate_slot_produces_unknown_slot_failure(node_factory):
    """Multiple detections assigned to one slot make it unknown."""
    detections = make_detections(('lettuce_dark_green',) * 6)
    detections.append(detections[0])
    detector = FakeDetector(detections)
    node, _ = node_factory(detector)
    replace_outputs(node)

    finish_command(node, make_command())

    result = node._result_publisher.messages[-1]
    assert result.status == 'FAILED'
    assert result.reason == 'UNKNOWN_SLOT'
    assert list(result.unknown_slots) == ['SLOT_01']


def test_inference_error_is_terminal_but_does_not_stop_node(node_factory):
    """A detector exception produces INSPECTION_FAILED and permits recovery."""
    detector = FakeDetector(error=RuntimeError('synthetic failure'))
    node, _ = node_factory(detector)
    events = []
    replace_outputs(node, events)

    finish_command(node, make_command())

    result = node._result_publisher.messages[-1]
    assert [event for event in events if event != 'debug'] == [
        'detections',
        'result',
    ]
    assert result.status == 'FAILED'
    assert result.reason == 'INSPECTION_FAILED'
    assert not node._stopping
    assert node._active_command is None


def test_active_command_image_timeout(node_factory):
    """A command without a fresh frame terminates as IMAGE_TIMEOUT."""
    node, _ = node_factory()
    replace_outputs(node)
    node._command_callback(make_command())
    node._command_deadline = time.monotonic() - 1.0

    node._inference_tick()

    result = node._result_publisher.messages[-1]
    assert result.status == 'FAILED'
    assert result.reason == 'IMAGE_TIMEOUT'
    assert node._active_command is None


def test_camera_timeout_recovers_on_next_valid_image(node_factory):
    """A valid frame returns a timed-out idle executor to READY."""
    node, _ = node_factory()
    node._started_at = time.monotonic() - node._image_timeout_sec - 1.0

    node._check_camera_timeout()
    assert node._status_state == 'STARTING'
    assert node._status_phase == 'WAITING_IMAGE'

    node._image_callback(make_image())
    assert node._status_state == 'READY'
    assert node._status_phase == 'IDLE'
    assert not node._camera_timed_out


def test_destroy_node_stops_worker(node_factory):
    """Node destruction shuts down the inference executor."""
    node, _ = node_factory()

    assert not node._inference_executor._shutdown
    node.destroy_node()

    assert node._stopping
    assert node._inference_executor._shutdown


def test_unsupported_encoding_does_not_replace_latest_frame(node_factory):
    """Unsupported images are ignored without terminating the node."""
    node, _ = node_factory()
    message = make_image()
    message.encoding = '16UC1'

    node._image_callback(message)

    assert node._frame_sequence == 0
    assert node._latest_frame is None
    assert not node._stopping
