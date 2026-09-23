"""ROS 2 Inspection Executor using the reusable YOLO detector."""

from __future__ import annotations

from concurrent.futures import Future, ThreadPoolExecutor
import copy
from dataclasses import dataclass
import json
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
from std_msgs.msg import String
from smart_farm_interfaces.msg import (
    ExecutorStatus,
    TaskCommand,
    TaskResult,
)

from .yolo_detector import (
    Detection,
    DetectorConfig,
    EXPECTED_CLASS_NAMES,
    YoloDetector,
)


SLOT_IDS = tuple(f'SLOT_{index:02d}' for index in range(1, 7))
DEFAULT_SLOT_ROIS = {
    'SLOT_01': (0.0, 0.0, 0.3333333333, 0.5),
    'SLOT_02': (0.3333333333, 0.0, 0.3333333333, 0.5),
    'SLOT_03': (0.6666666666, 0.0, 0.3333333334, 0.5),
    'SLOT_04': (0.0, 0.5, 0.3333333333, 0.5),
    'SLOT_05': (0.3333333333, 0.5, 0.3333333333, 0.5),
    'SLOT_06': (0.6666666666, 0.5, 0.3333333334, 0.5),
}
# 2026-09-23 팀 결정: 노랑도 갈색과 같이 솎아낸다.
# handoff README 2절은 보류(HOLD, CULL 아님)로 두었으나 계약에 HOLD
# 필드가 없어 DEFECT 로 합쳤다. config/object_detection.yaml 과 같은 값.
DEFAULT_CLASS_OUTCOMES = {
    'lettuce_dark_green': 'NORMAL',
    'lettuce_yellow': 'DEFECT',
    'lettuce_brown': 'DEFECT',
}
VALID_OUTCOMES = frozenset({'NORMAL', 'DEFECT', 'UNKNOWN'})


@dataclass(frozen=True)
class AssignedDetection:
    """Detection enriched with its slot and pixel center."""

    detection: Detection
    slot_id: str
    center_u: float
    center_v: float


@dataclass(frozen=True)
class SlotAssessment:
    """Slot assignment result for one source image."""

    assigned: tuple[AssignedDetection, ...]
    slot_states: dict[str, str]
    defect_slots: tuple[str, ...]
    unknown_slots: tuple[str, ...]


class InspectionExecutorNode(Node):
    """Execute command-driven inspection using fresh camera frames."""

    def __init__(self) -> None:
        super().__init__('object_detection')

        self._declare_parameters()
        self._read_parameters()
        self._validate_parameters()

        self._bridge = CvBridge()
        self._started_at = time.monotonic()
        self._last_valid_frame_at: float | None = None
        self._last_encoding: str | None = None
        self._last_conversion_warning_at = 0.0
        self._camera_timed_out = False
        self._model_ready = False
        self._image_ready = False
        self._stopping = False

        self._frame_lock = threading.Lock()
        self._latest_frame: np.ndarray | None = None
        self._latest_header = None
        self._frame_sequence = 0

        self._active_command: TaskCommand | None = None
        self._command_min_frame_sequence = 0
        self._command_deadline = 0.0
        self._terminal_results: dict[str, TaskResult] = {}

        self._last_submitted_sequence = 0
        self._last_inference_started_at = 0.0
        self._inference_future: Future | None = None

        image_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
        )

        self._debug_publisher = self.create_publisher(
            Image,
            self._debug_image_topic,
            image_qos,
        )
        self._result_publisher = self.create_publisher(
            TaskResult,
            '/inspection/result',
            10,
        )
        self._status_publisher = self.create_publisher(
            ExecutorStatus,
            '/inspection/status',
            10,
        )
        self._detections_publisher = self.create_publisher(
            String,
            '/inspection/detections_2d',
            10,
        )

        self._image_subscription = self.create_subscription(
            Image,
            self._camera_topic,
            self._image_callback,
            image_qos,
        )
        self._command_subscription = self.create_subscription(
            TaskCommand,
            '/inspection/command',
            self._command_callback,
            10,
        )

        self._status_state = 'STARTING'
        self._status_task_id = ''
        self._status_command_id = ''
        self._status_operation = 'INSPECT'
        self._status_phase = 'MODEL_LOADING'
        self._status_detail = f'loading {self._model_path}'
        self._publish_status()

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
            self._detector = YoloDetector(detector_config)
            self._validate_model_contract()
        except Exception as exc:
            self._set_status(
                state='ERROR',
                phase='MODEL_INIT_FAILED',
                detail=(
                    f'model_path={self._model_path}; '
                    f'{type(exc).__name__}: {exc}'
                ),
            )
            self.get_logger().fatal(
                'Failed to initialize inspection model: '
                f'path={self._model_path}, '
                f'device={self._device}, '
                f'error={type(exc).__name__}: {exc}'
            )
            raise

        self._model_ready = True
        self._inference_executor = ThreadPoolExecutor(
            max_workers=1,
            thread_name_prefix='inspection-inference',
        )
        self._inference_timer = self.create_timer(
            self._min_cycle_sec,
            self._inference_tick,
        )
        self._camera_timer = self.create_timer(
            1.0,
            self._check_camera_timeout,
        )
        self._status_timer = self.create_timer(
            1.0,
            self._publish_status,
        )

        self._set_status(
            state='STARTING',
            phase='WAITING_IMAGE',
            detail=f'waiting for {self._camera_topic}',
        )
        self.get_logger().info(
            'Inspection executor initialized: '
            f'model={self._model_path}, '
            f'device={self._detector.device}, '
            f'camera={self._camera_topic}'
        )

    def _declare_parameters(self) -> None:
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

        for slot_id, roi in DEFAULT_SLOT_ROIS.items():
            self.declare_parameter(
                f'slot_rois.{slot_id}',
                list(roi),
            )

        for class_name, outcome in DEFAULT_CLASS_OUTCOMES.items():
            self.declare_parameter(
                f'class_outcomes.{class_name}',
                outcome,
            )

    def _read_parameters(self) -> None:
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

        self._slot_rois = {
            slot_id: tuple(
                float(value)
                for value in self.get_parameter(
                    f'slot_rois.{slot_id}'
                ).value
            )
            for slot_id in SLOT_IDS
        }
        self._class_outcomes = {
            class_name: str(
                self.get_parameter(
                    f'class_outcomes.{class_name}'
                ).value
            ).upper()
            for class_name in EXPECTED_CLASS_NAMES.values()
        }

    def _validate_parameters(self) -> None:
        if not self._camera_topic:
            raise ValueError('camera_topic must not be empty')
        if self._image_timeout_sec <= 0.0:
            raise ValueError(
                'image_timeout_sec must be greater than 0'
            )
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
        if tuple(self._slot_rois) != SLOT_IDS:
            raise ValueError(
                f'configured slots must be exactly {SLOT_IDS}'
            )

        for slot_id, roi in self._slot_rois.items():
            if len(roi) != 4:
                raise ValueError(
                    f'{slot_id} ROI must be [x, y, width, height]'
                )

            x, y, width, height = roi
            if (
                x < 0.0
                or y < 0.0
                or width <= 0.0
                or height <= 0.0
                or x + width > 1.000001
                or y + height > 1.000001
            ):
                raise ValueError(
                    f'{slot_id} ROI is outside normalized bounds: {roi}'
                )

        expected_classes = set(EXPECTED_CLASS_NAMES.values())
        if set(self._class_outcomes) != expected_classes:
            raise ValueError(
                'class_outcomes must match EXPECTED_CLASS_NAMES'
            )

        invalid_outcomes = {
            outcome
            for outcome in self._class_outcomes.values()
            if outcome not in VALID_OUTCOMES
        }
        if invalid_outcomes:
            raise ValueError(
                f'unsupported class outcomes: {invalid_outcomes}'
            )

    def _validate_model_contract(self) -> None:
        if self._detector.class_names != EXPECTED_CLASS_NAMES:
            raise ValueError(
                'model class names do not match '
                f'EXPECTED_CLASS_NAMES: {self._detector.class_names}'
            )

        model_classes = set(self._detector.class_names.values())
        if model_classes != set(self._class_outcomes):
            raise ValueError(
                'model classes and class_outcomes differ: '
                f'model={model_classes}, '
                f'config={set(self._class_outcomes)}'
            )

    def _set_status(
        self,
        state: str,
        phase: str,
        detail: str,
        command: TaskCommand | None = None,
    ) -> None:
        self._status_state = state
        self._status_phase = phase
        self._status_detail = detail

        if command is None:
            self._status_task_id = ''
            self._status_command_id = ''
            self._status_operation = 'INSPECT'
        else:
            self._status_task_id = command.task_id
            self._status_command_id = command.command_id
            self._status_operation = command.operation

        self._publish_status()

    def _publish_status(self) -> None:
        message = ExecutorStatus()
        message.executor = 'inspection'
        message.state = self._status_state
        message.task_id = self._status_task_id
        message.command_id = self._status_command_id
        message.operation = self._status_operation
        message.phase = self._status_phase
        message.detail = self._status_detail
        self._status_publisher.publish(message)

    def _image_callback(self, message: Image) -> None:
        encoding = message.encoding.strip().lower()
        self._last_encoding = encoding

        try:
            bgr_image = self._convert_to_bgr(message)
        except (CvBridgeError, cv2.error, ValueError) as exc:
            self._log_conversion_warning(encoding, exc)
            return

        frame = np.ascontiguousarray(bgr_image).copy()
        header = copy.deepcopy(message.header)
        now = time.monotonic()

        with self._frame_lock:
            self._latest_frame = frame
            self._latest_header = header
            self._frame_sequence += 1

        self._last_valid_frame_at = now
        self._image_ready = True

        if self._camera_timed_out:
            self._camera_timed_out = False
            self.get_logger().info(
                'Camera stream recovered: '
                f'topic={self._camera_topic}, '
                f'encoding={encoding}'
            )

        if self._active_command is None:
            self._set_status(
                state='READY',
                phase='IDLE',
                detail='model and image ready',
            )

    def _convert_to_bgr(self, message: Image) -> np.ndarray:
        encoding = message.encoding.strip().lower()
        image = self._bridge.imgmsg_to_cv2(
            message,
            desired_encoding='passthrough',
        )

        conversions = {
            'rgb8': cv2.COLOR_RGB2BGR,
            'rgba8': cv2.COLOR_RGBA2BGR,
            'bgra8': cv2.COLOR_BGRA2BGR,
            'mono8': cv2.COLOR_GRAY2BGR,
        }

        if encoding == 'bgr8':
            bgr_image = image
        elif encoding in conversions:
            bgr_image = cv2.cvtColor(
                image,
                conversions[encoding],
            )
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
                'converted image must be uint8 HxWx3 BGR'
            )

        return bgr_image

    def _log_conversion_warning(
        self,
        encoding: str,
        error: Exception,
    ) -> None:
        now = time.monotonic()
        if now - self._last_conversion_warning_at < 5.0:
            return

        self._last_conversion_warning_at = now
        self.get_logger().warning(
            'Failed to convert camera image: '
            f'encoding={encoding!r}, error={error}'
        )

    def _check_camera_timeout(self) -> None:
        reference = (
            self._last_valid_frame_at
            if self._last_valid_frame_at is not None
            else self._started_at
        )
        elapsed = time.monotonic() - reference

        if elapsed <= self._image_timeout_sec:
            return

        self._image_ready = False
        if not self._camera_timed_out:
            self._camera_timed_out = True
            self.get_logger().warning(
                'Camera image timeout: '
                f'no valid image for {elapsed:.1f}s, '
                f'topic={self._camera_topic}, '
                f'last_encoding={self._last_encoding or "unknown"}'
            )

        if self._active_command is None:
            self._set_status(
                state='STARTING',
                phase='WAITING_IMAGE',
                detail=f'no valid image for {elapsed:.1f}s',
            )

    def _command_callback(self, command: TaskCommand) -> None:
        cached = self._terminal_results.get(command.command_id)
        if cached is not None:
            self._result_publisher.publish(copy.deepcopy(cached))
            self.get_logger().info(
                'Cached terminal result republished: '
                f'{command.command_id}'
            )
            return

        if (
            self._active_command is not None
            and command.command_id
            == self._active_command.command_id
        ):
            self.get_logger().info(
                'Duplicate active command ignored: '
                f'{command.command_id}'
            )
            return

        validation_reason = self._validate_command(command)
        if validation_reason is not None:
            self._publish_terminal(
                command,
                status='FAILED',
                phase='VALIDATE',
                reason=validation_reason,
            )
            return

        if self._active_command is not None:
            self._publish_terminal(
                command,
                status='FAILED',
                phase='VALIDATE',
                reason='BUSY',
            )
            return

        with self._frame_lock:
            current_sequence = self._frame_sequence

        self._active_command = copy.deepcopy(command)
        self._command_min_frame_sequence = current_sequence
        self._command_deadline = (
            time.monotonic() + self._image_timeout_sec
        )
        self._set_status(
            state='BUSY',
            phase='WAITING_FRESH_FRAME',
            detail='waiting for an image newer than the command',
            command=self._active_command,
        )

    @staticmethod
    def _validate_command(command: TaskCommand) -> str | None:
        if (
            not command.task_id
            or not command.command_id
            or not command.pallet_id
        ):
            return 'INVALID_ID'
        if command.operation != 'INSPECT':
            return 'INVALID_COMMAND'
        return None

    def _inference_tick(self) -> None:
        if self._stopping:
            return

        self._collect_inference_result()
        if self._inference_future is not None:
            return

        command = self._active_command
        if command is not None:
            if time.monotonic() >= self._command_deadline:
                self._finish_active_command(
                    status='FAILED',
                    phase='WAITING_FRESH_FRAME',
                    reason='IMAGE_TIMEOUT',
                )
                return
            minimum_sequence = self._command_min_frame_sequence
        elif self._continuous_inference:
            minimum_sequence = 0
        else:
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
            if self._frame_sequence <= minimum_sequence:
                return
            if self._frame_sequence == self._last_submitted_sequence:
                return

            frame = self._latest_frame
            header = copy.deepcopy(self._latest_header)
            frame_sequence = self._frame_sequence

        command_copy = (
            copy.deepcopy(command)
            if command is not None
            else None
        )

        try:
            future = self._inference_executor.submit(
                self._run_inference,
                frame,
                header,
                frame_sequence,
                command_copy,
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

        if command is not None:
            self._set_status(
                state='BUSY',
                phase='INFERENCE',
                detail=f'processing frame {frame_sequence}',
                command=command,
            )

    def _run_inference(
        self,
        frame: np.ndarray,
        header,
        frame_sequence: int,
        command: TaskCommand | None,
    ):
        try:
            detections = self._detector.detect(frame)
            error = None
        except Exception as exc:
            detections = []
            error = exc

        return (
            frame_sequence,
            frame,
            header,
            command,
            detections,
            error,
        )

    def _collect_inference_result(self) -> None:
        future = self._inference_future
        if future is None or not future.done():
            return

        self._inference_future = None
        (
            frame_sequence,
            frame,
            header,
            command,
            detections,
            error,
        ) = future.result()

        height, width = frame.shape[:2]
        assessment = self._assess_detections(
            detections,
            width,
            height,
        )

        try:
            self._publish_debug_image(
                frame,
                header,
                assessment,
            )
        except Exception as exc:
            self.get_logger().error(
                'Failed to publish debug image: '
                f'{type(exc).__name__}: {exc}'
            )

        if command is None:
            if error is not None:
                self.get_logger().error(
                    'Continuous inference failed: '
                    f'{type(error).__name__}: {error}'
                )
            return

        detection_message = self._build_detection_message(
            command,
            header,
            width,
            height,
            assessment,
        )
        self._detections_publisher.publish(detection_message)

        if error is not None:
            self.get_logger().error(
                'Inspection inference failed: '
                f'{type(error).__name__}: {error}'
            )
            self._finish_active_command(
                status='FAILED',
                phase='INFERENCE',
                reason='INSPECTION_FAILED',
            )
            return

        if assessment.unknown_slots:
            self._finish_active_command(
                status='FAILED',
                phase='SLOT_CLASSIFICATION',
                reason='UNKNOWN_SLOT',
                defect_slots=assessment.defect_slots,
                unknown_slots=assessment.unknown_slots,
            )
            return

        self._finish_active_command(
            status='SUCCEEDED',
            phase='INSPECTION_COMPLETE',
            reason='NONE',
            defect_slots=assessment.defect_slots,
        )
        self.get_logger().debug(
            f'Inspection completed from frame {frame_sequence}'
        )

    def _assess_detections(
        self,
        detections: list[Detection],
        image_width: int,
        image_height: int,
    ) -> SlotAssessment:
        assigned: list[AssignedDetection] = []
        detections_by_slot = {
            slot_id: []
            for slot_id in SLOT_IDS
        }
        conflict_slots: set[str] = set()

        for detection in detections:
            x_min, y_min, x_max, y_max = detection.bbox_xyxy
            center_u = (x_min + x_max) / 2.0
            center_v = (y_min + y_max) / 2.0
            normalized_u = center_u / image_width
            normalized_v = center_v / image_height

            matched_slots = [
                slot_id
                for slot_id, roi in self._slot_rois.items()
                if self._roi_contains(
                    roi,
                    normalized_u,
                    normalized_v,
                )
            ]

            slot_id = ''
            if len(matched_slots) == 1:
                slot_id = matched_slots[0]
                detections_by_slot[slot_id].append(detection)
            elif len(matched_slots) > 1:
                conflict_slots.update(matched_slots)

            assigned.append(
                AssignedDetection(
                    detection=detection,
                    slot_id=slot_id,
                    center_u=center_u,
                    center_v=center_v,
                )
            )

        slot_states: dict[str, str] = {}
        for slot_id in SLOT_IDS:
            slot_detections = detections_by_slot[slot_id]
            if (
                slot_id in conflict_slots
                or len(slot_detections) != 1
            ):
                slot_states[slot_id] = 'UNKNOWN'
                continue

            class_name = slot_detections[0].class_name
            slot_states[slot_id] = self._class_outcomes.get(
                class_name,
                'UNKNOWN',
            )

        defect_slots = tuple(
            slot_id
            for slot_id in SLOT_IDS
            if slot_states[slot_id] == 'DEFECT'
        )
        unknown_slots = tuple(
            slot_id
            for slot_id in SLOT_IDS
            if slot_states[slot_id] == 'UNKNOWN'
        )

        return SlotAssessment(
            assigned=tuple(assigned),
            slot_states=slot_states,
            defect_slots=defect_slots,
            unknown_slots=unknown_slots,
        )

    @staticmethod
    def _roi_contains(
        roi: tuple[float, ...],
        normalized_u: float,
        normalized_v: float,
    ) -> bool:
        x, y, width, height = roi
        right = x + width
        bottom = y + height

        inside_x = x <= normalized_u < right
        inside_y = y <= normalized_v < bottom

        if abs(right - 1.0) < 1e-6 and normalized_u == 1.0:
            inside_x = True
        if abs(bottom - 1.0) < 1e-6 and normalized_v == 1.0:
            inside_y = True

        return inside_x and inside_y

    def _build_detection_message(
        self,
        command: TaskCommand,
        header,
        image_width: int,
        image_height: int,
        assessment: SlotAssessment,
    ) -> String:
        payload = {
            'header': {
                'stamp': {
                    'sec': int(header.stamp.sec),
                    'nanosec': int(header.stamp.nanosec),
                },
                'frame_id': header.frame_id,
            },
            'task_id': command.task_id,
            'command_id': command.command_id,
            'pallet_id': command.pallet_id,
            'image_width': image_width,
            'image_height': image_height,
            'detections': [
                {
                    'slot_id': assigned.slot_id,
                    'class_name': assigned.detection.class_name,
                    'confidence': assigned.detection.confidence,
                    'center_u': assigned.center_u,
                    'center_v': assigned.center_v,
                    'bbox_x_min': assigned.detection.bbox_xyxy[0],
                    'bbox_y_min': assigned.detection.bbox_xyxy[1],
                    'bbox_x_max': assigned.detection.bbox_xyxy[2],
                    'bbox_y_max': assigned.detection.bbox_xyxy[3],
                }
                for assigned in assessment.assigned
            ],
        }
        message = String()
        message.data = json.dumps(
            payload,
            ensure_ascii=False,
            allow_nan=False,
            separators=(',', ':'),
        )
        return message

    def _publish_debug_image(
        self,
        frame: np.ndarray,
        header,
        assessment: SlotAssessment,
    ) -> None:
        debug_frame = frame.copy()
        height, width = debug_frame.shape[:2]
        colors = {
            'NORMAL': (0, 200, 0),
            'DEFECT': (0, 0, 255),
            'UNKNOWN': (0, 165, 255),
        }

        for slot_id, roi in self._slot_rois.items():
            x, y, roi_width, roi_height = roi
            x_min = max(0, min(int(round(x * width)), width - 1))
            y_min = max(0, min(int(round(y * height)), height - 1))
            x_max = max(
                0,
                min(int(round((x + roi_width) * width)) - 1, width - 1),
            )
            y_max = max(
                0,
                min(
                    int(round((y + roi_height) * height)) - 1,
                    height - 1,
                ),
            )
            state = assessment.slot_states[slot_id]
            color = colors[state]
            cv2.rectangle(
                debug_frame,
                (x_min, y_min),
                (x_max, y_max),
                color,
                2,
            )
            cv2.putText(
                debug_frame,
                f'{slot_id} {state}',
                (x_min + 4, min(y_max - 4, y_min + 20)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                color,
                2,
                cv2.LINE_AA,
            )

        for assigned in assessment.assigned:
            detection = assigned.detection
            x_min, y_min, x_max, y_max = (
                int(round(value))
                for value in detection.bbox_xyxy
            )
            x_min = max(0, min(x_min, width - 1))
            y_min = max(0, min(y_min, height - 1))
            x_max = max(0, min(x_max, width - 1))
            y_max = max(0, min(y_max, height - 1))
            if x_max <= x_min or y_max <= y_min:
                continue

            slot_label = assigned.slot_id or 'UNASSIGNED'
            label = (
                f'{slot_label} {detection.class_name} '
                f'{detection.confidence:.2f}'
            )
            cv2.rectangle(
                debug_frame,
                (x_min, y_min),
                (x_max, y_max),
                (255, 255, 255),
                1,
            )
            cv2.putText(
                debug_frame,
                label,
                (x_min, max(18, y_min - 6)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.45,
                (255, 255, 255),
                1,
                cv2.LINE_AA,
            )

        message = self._bridge.cv2_to_imgmsg(
            debug_frame,
            encoding='bgr8',
        )
        message.header = copy.deepcopy(header)
        self._debug_publisher.publish(message)

    def _publish_terminal(
        self,
        command: TaskCommand,
        status: str,
        phase: str,
        reason: str,
        defect_slots: tuple[str, ...] = (),
        unknown_slots: tuple[str, ...] = (),
    ) -> TaskResult:
        cached = self._terminal_results.get(command.command_id)
        if cached is not None:
            self._result_publisher.publish(copy.deepcopy(cached))
            return cached

        result = TaskResult()
        result.task_id = command.task_id
        result.command_id = command.command_id
        result.operation = command.operation
        result.status = status
        result.phase = phase
        result.reason = reason
        result.safe_to_navigate = False
        result.reached_station = ''
        result.completed_units = []
        result.defect_slots = list(defect_slots)
        result.unknown_slots = list(unknown_slots)

        if command.command_id:
            self._terminal_results[command.command_id] = copy.deepcopy(
                result
            )

        self._result_publisher.publish(result)
        return result

    def _finish_active_command(
        self,
        status: str,
        phase: str,
        reason: str,
        defect_slots: tuple[str, ...] = (),
        unknown_slots: tuple[str, ...] = (),
    ) -> None:
        command = self._active_command
        if command is None:
            return

        self._active_command = None
        self._publish_terminal(
            command,
            status=status,
            phase=phase,
            reason=reason,
            defect_slots=defect_slots,
            unknown_slots=unknown_slots,
        )

        if self._image_ready:
            self._set_status(
                state='READY',
                phase='IDLE',
                detail=f'last result {status}/{reason}',
            )
        else:
            self._set_status(
                state='STARTING',
                phase='WAITING_IMAGE',
                detail=f'last result {status}/{reason}',
            )

    def destroy_node(self) -> bool:
        """Stop timers and wait for the single inference worker."""
        self._stopping = True

        for timer_name in (
            '_inference_timer',
            '_camera_timer',
            '_status_timer',
        ):
            timer = getattr(self, timer_name, None)
            if timer is not None:
                timer.cancel()

        executor = getattr(self, '_inference_executor', None)
        if executor is not None:
            executor.shutdown(
                wait=True,
                cancel_futures=True,
            )

        return super().destroy_node()


def main(args: list[str] | None = None) -> None:
    """Run the Inspection Executor."""
    rclpy.init(args=args)
    node: InspectionExecutorNode | None = None

    try:
        node = InspectionExecutorNode()
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
