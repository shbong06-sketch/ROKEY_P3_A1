"""ROS 없이 검사 검출의 식별자와 픽셀 계약을 검증한다."""

import copy
from pathlib import Path
import sys

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent))
from inspection_detection_store import DetectionContractError, InspectionDetectionStore


def sample_detections(task_id="TASK-001", command_id="TASK-001-CMD-007"):
    detections = []
    states = {}
    for index in range(1, 7):
        slot_id = f"SLOT_{index:02d}"
        x = float(index * 30)
        is_defect = index == 2
        states[slot_id] = "DEFECT" if is_defect else "NORMAL"
        detections.append({
            "slot_id": slot_id,
            "class_name": "lettuce_yellow" if is_defect else "lettuce_dark_green",
            "confidence": 0.9,
            "center_u": x,
            "center_v": 50.0,
            "bbox_x_min": x - 10.0,
            "bbox_y_min": 40.0,
            "bbox_x_max": x + 10.0,
            "bbox_y_max": 60.0,
        })
    return {
        "task_id": task_id,
        "command_id": command_id,
        "inspection_command_id": command_id,
        "pallet_id": "PALLET_001",
        "coordinate_frame": "image_pixels",
        "header": {"frame_id": "camera_rgb", "stamp": {"sec": 42, "nanosec": 5}},
        "image_width": 300,
        "image_height": 200,
        "slot_states": states,
        "valid_for_cull": True,
        "detections": detections,
    }


def context(task_id="TASK-001", command_id="TASK-001-CMD-007"):
    return {
        "task_id": task_id,
        "inspection_command_id": command_id,
        "pallet_id": "PALLET_001",
        "operation": "INSPECT",
    }


def prepared_store():
    store = InspectionDetectionStore()
    store.mark_prepared("TASK-001", "PALLET_001")
    return store


def test_stores_only_matching_inspection_after_preparation():
    store = prepared_store()
    assert store.expect_inspection(context()) is False
    payload = sample_detections()
    assert store.receive(payload) is True
    assert store.data == payload
    assert store.data is not payload


def test_detection_can_arrive_before_context_without_bypassing_validation():
    store = prepared_store()
    assert store.receive(sample_detections()) is False
    assert store.data is None
    assert store.expect_inspection(context()) is True
    assert store.data is not None


def test_previous_task_or_inspection_is_rejected():
    store = prepared_store()
    store.expect_inspection(context())
    with pytest.raises(DetectionContractError, match="another task"):
        store.receive(sample_detections(task_id="TASK-OLD"))
    with pytest.raises(DetectionContractError, match="another inspection"):
        store.receive(sample_detections(command_id="TASK-001-CMD-005"))
    old_pallet = sample_detections()
    old_pallet["pallet_id"] = "PALLET_002"
    with pytest.raises(DetectionContractError, match="another task or pallet"):
        store.receive(old_pallet)
    assert store.data is None


def test_context_for_another_pallet_is_rejected():
    store = prepared_store()
    wrong = context()
    wrong["pallet_id"] = "PALLET_002"
    with pytest.raises(DetectionContractError, match="does not match prepared"):
        store.expect_inspection(wrong)
    assert store.inspection_command_id == ""


def test_new_preparation_clears_old_context_and_data():
    store = prepared_store()
    store.expect_inspection(context())
    store.receive(sample_detections())
    store.mark_prepared("TASK-002", "PALLET_001")
    assert store.data is None
    assert store.inspection_command_id == ""
    with pytest.raises(DetectionContractError, match="another task"):
        store.receive(sample_detections())


@pytest.mark.parametrize("change", [
    lambda data: data.update(valid_for_cull=False),
    lambda data: data["slot_states"].update(SLOT_02="UNKNOWN"),
    lambda data: data["header"]["stamp"].update(sec=0, nanosec=0),
    lambda data: data.update(coordinate_frame="world"),
    lambda data: data["detections"][1].update(center_u=float("nan")),
    lambda data: data["detections"].pop(),
])
def test_rejects_failed_or_incomplete_cull_data(change):
    store = prepared_store()
    store.expect_inspection(context())
    payload = copy.deepcopy(sample_detections())
    change(payload)
    with pytest.raises(DetectionContractError):
        store.receive(payload)
    assert store.data is None
