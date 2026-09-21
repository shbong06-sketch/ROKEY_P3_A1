import json

import pytest

from smart_farm_manager.protocol import TaskCommandData
from smart_farm_manager.task_manager_node import TaskManagerNode


def test_sim_task_command_is_serialized_as_json_string():
    command = TaskCommandData(
        task_id="TASK-001",
        command_id="TASK-001-CMD-001",
        operation="CULL",
        recipe_id="CULL_DEFECT_SLOTS",
        pallet_id="PALLET_004",
        source="INSPECT_STATION",
        destination="INSPECT_STATION",
        target_slots=("SLOT_03", "SLOT_07"),
    )

    message = TaskManagerNode._task_command_to_json(command)
    payload = json.loads(message.data)

    assert payload == {
        "task_id": "TASK-001",
        "command_id": "TASK-001-CMD-001",
        "operation": "CULL",
        "recipe_id": "CULL_DEFECT_SLOTS",
        "pallet_id": "PALLET_004",
        "source": "INSPECT_STATION",
        "destination": "INSPECT_STATION",
        "target_slots": ["SLOT_03", "SLOT_07"],
    }


def test_sim_task_result_json_is_converted_to_internal_model():
    raw_message = json.dumps(
        {
            "task_id": "TASK-001",
            "command_id": "TASK-001-CMD-001",
            "operation": "PICK_HARVEST",
            "status": "SUCCEEDED",
            "phase": "RESULT",
            "reason": "NONE",
            "safe_to_navigate": True,
            "completed_units": [],
        }
    )

    result = TaskManagerNode._task_result_from_json(raw_message)

    assert result.task_id == "TASK-001"
    assert result.command_id == "TASK-001-CMD-001"
    assert result.operation == "PICK_HARVEST"
    assert result.status == "SUCCEEDED"
    assert result.safe_to_navigate is True
    assert result.unknown_slots == ()


def test_sim_task_status_json_is_converted_for_preflight():
    raw_message = json.dumps(
        {
            "executor": "sim_task",
            "state": "READY",
            "phase": "IDLE",
            "detail": "ready",
        }
    )

    status = TaskManagerNode._executor_status_from_json(raw_message)

    assert status.executor == "sim_task"
    assert status.state == "READY"
    assert status.phase == "IDLE"
    assert status.task_id == ""


@pytest.mark.parametrize(
    "raw_message",
    [
        "{invalid-json",
        "[]",
        json.dumps(
            {
                "command_id": "TASK-001-CMD-001",
                "operation": "TRANSFER",
                "status": "SUCCEEDED",
            }
        ),
        json.dumps(
            {
                "task_id": "TASK-001",
                "command_id": "TASK-001-CMD-001",
                "operation": "TRANSFER",
                "status": "SUCCEEDED",
                "completed_units": "not-an-array",
            }
        ),
    ],
)
def test_invalid_sim_task_result_json_is_rejected(raw_message):
    with pytest.raises((TypeError, ValueError)):
        TaskManagerNode._task_result_from_json(raw_message)
