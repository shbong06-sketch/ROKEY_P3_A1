"""검사 2D 검출을 현재 준비된 팔레트와 검사 명령에 묶어 보관한다."""

import copy
import math


SLOT_IDS = tuple(f"SLOT_{index:02d}" for index in range(1, 7))
CLASS_OUTCOMES = {
    "lettuce_dark_green": "NORMAL",
    "lettuce_yellow": "DEFECT",
    "lettuce_brown": "DEFECT",
}


class DetectionContractError(ValueError):
    """검출 메시지가 현재 검사 또는 픽셀 좌표 계약과 맞지 않음."""


class CullContractError(DetectionContractError):
    """Cull 명령과 저장된 검사 결과가 맞지 않음."""

    def __init__(self, reason, detail):
        super().__init__(detail)
        self.reason = reason


class InspectionDetectionStore:
    """Cull 시작 없이 검증된 한 번의 검사 결과만 보관한다."""

    def __init__(self):
        self.reset()

    def reset(self):
        self.prepared_task_id = ""
        self.prepared_pallet_id = ""
        self.inspection_command_id = ""
        self.pending = None
        self.data = None

    def mark_prepared(self, task_id, pallet_id):
        if not task_id or not pallet_id:
            raise DetectionContractError("prepared task_id and pallet_id are required")
        self.reset()
        self.prepared_task_id = task_id
        self.prepared_pallet_id = pallet_id

    def expect_inspection(self, payload):
        if not isinstance(payload, dict):
            raise DetectionContractError("inspection context must be an object")
        command_id = payload.get("inspection_command_id")
        if (not self.prepared_task_id or not self.prepared_pallet_id
                or payload.get("operation") != "INSPECT"
                or payload.get("task_id") != self.prepared_task_id
                or payload.get("pallet_id") != self.prepared_pallet_id
                or not isinstance(command_id, str) or not command_id):
            raise DetectionContractError("inspection context does not match prepared pallet")
        if self.inspection_command_id and self.inspection_command_id != command_id:
            raise DetectionContractError("another inspection command is already expected")
        self.inspection_command_id = command_id
        if self.pending is not None:
            pending = self.pending
            self.pending = None
            return self.receive(pending)
        return False

    def receive(self, payload):
        if not isinstance(payload, dict):
            raise DetectionContractError("detections must be an object")
        if (not self.prepared_task_id
                or payload.get("task_id") != self.prepared_task_id
                or payload.get("pallet_id") != self.prepared_pallet_id):
            raise DetectionContractError("detections belong to another task or pallet")
        command_id = payload.get("inspection_command_id")
        if (not isinstance(command_id, str) or not command_id
                or payload.get("command_id") != command_id):
            raise DetectionContractError("inspection command identifiers disagree")
        if not self.inspection_command_id:
            self.pending = copy.deepcopy(payload)
            return False
        if command_id != self.inspection_command_id:
            raise DetectionContractError("detections belong to another inspection")
        self._validate_frame_and_slots(payload)
        if self.data is not None:
            raise DetectionContractError("inspection detections were already stored")
        self.data = copy.deepcopy(payload)
        self.data["detections"] = [
            detection for detection in self.data["detections"]
            if detection["slot_id"] in SLOT_IDS
        ]
        return True

    def require_cull(self, task_id, pallet_id, target_slots):
        """현재 검사에서 판정한 모든 불량 슬롯의 검출을 반환한다."""
        if self.data is None or not self.inspection_command_id:
            raise CullContractError("INSPECTION_DATA_MISSING", "no completed inspection detections")
        data = self.data
        if (task_id != self.prepared_task_id
                or pallet_id != self.prepared_pallet_id
                or data["task_id"] != task_id
                or data["pallet_id"] != pallet_id
                or data["inspection_command_id"] != self.inspection_command_id):
            raise CullContractError("INSPECTION_ID_MISMATCH", "Cull belongs to another inspection or pallet")
        targets = tuple(target_slots)
        expected = {
            slot for slot, state in data["slot_states"].items()
            if state == "DEFECT"
        }
        if (not targets or len(targets) != len(set(targets))
                or set(targets) != expected):
            raise CullContractError("CULL_TARGET_MISMATCH", "target_slots differ from inspection defects")
        by_slot = {
            detection["slot_id"]: detection
            for detection in data["detections"]
        }
        if any(slot not in by_slot for slot in targets):
            raise CullContractError("CULL_COORDINATE_MISSING", "defect slot has no detection")
        return copy.deepcopy(data)

    @classmethod
    def _validate_frame_and_slots(cls, payload):
        if payload.get("coordinate_frame") != "image_pixels":
            raise DetectionContractError("coordinate_frame must be image_pixels")
        header = payload.get("header")
        if (not isinstance(header, dict)
                or not isinstance(header.get("frame_id"), str)
                or not header["frame_id"]):
            raise DetectionContractError("camera frame_id is required")
        stamp = header.get("stamp")
        if (not isinstance(stamp, dict)
                or type(stamp.get("sec")) is not int
                or type(stamp.get("nanosec")) is not int
                or stamp["sec"] < 0
                or not 0 <= stamp["nanosec"] < 1_000_000_000
                or (stamp["sec"] == 0 and stamp["nanosec"] == 0)):
            raise DetectionContractError("nonzero image timestamp is required")
        width = payload.get("image_width")
        height = payload.get("image_height")
        if type(width) is not int or type(height) is not int or width <= 0 or height <= 0:
            raise DetectionContractError("positive image dimensions are required")
        states = payload.get("slot_states")
        if not isinstance(states, dict) or set(states) != set(SLOT_IDS):
            raise DetectionContractError("exactly six slot states are required")
        if payload.get("valid_for_cull") is not True or any(
            state not in {"NORMAL", "DEFECT"} for state in states.values()
        ):
            raise DetectionContractError("unknown or failed inspection cannot supply Cull data")
        detections = payload.get("detections")
        if not isinstance(detections, list):
            raise DetectionContractError("detections must be a list")
        by_slot = {}
        for detection in detections:
            if not isinstance(detection, dict):
                raise DetectionContractError("detection must be an object")
            slot_id = detection.get("slot_id")
            if slot_id == "":
                continue  # ROI 밖의 물체는 Cull 대상이 아니다.
            if slot_id not in SLOT_IDS or slot_id in by_slot:
                raise DetectionContractError("invalid or repeated slot_id")
            class_name = detection.get("class_name")
            if CLASS_OUTCOMES.get(class_name) != states[slot_id]:
                raise DetectionContractError("class and slot state disagree")
            values = [detection.get(key) for key in (
                "confidence", "center_u", "center_v", "bbox_x_min",
                "bbox_y_min", "bbox_x_max", "bbox_y_max",
            )]
            if any(type(value) not in (int, float) or not math.isfinite(value) for value in values):
                raise DetectionContractError("detection coordinates must be finite numbers")
            confidence, u, v, x_min, y_min, x_max, y_max = values
            if (not 0.0 <= confidence <= 1.0
                    or not 0.0 <= x_min < x_max <= width
                    or not 0.0 <= y_min < y_max <= height
                    or not x_min <= u <= x_max
                    or not y_min <= v <= y_max):
                raise DetectionContractError("detection is outside image bounds")
            by_slot[slot_id] = detection
        if set(by_slot) != set(SLOT_IDS):
            raise DetectionContractError("one detection per SLOT_01~SLOT_06 is required")
