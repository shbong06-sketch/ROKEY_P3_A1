import json

import pytest
import rclpy
from std_msgs.msg import String

from smart_farm_manager.protocol import TaskCommandData, TaskResultData
from smart_farm_manager.scenario import CycleState
from smart_farm_manager.task_manager_node import TaskManagerNode


def test_sim_task_command_is_serialized_as_json_string():
    command = TaskCommandData(
        task_id="TASK-001",
        command_id="TASK-001-CMD-001",
        operation="CULL",
        recipe_id="CULL_DEFECT_SLOTS",
        pallet_id="PALLET_001",
        source="INSPECT_STATION",
        destination="INSPECT_STATION",
        target_slots=("SLOT_03", "SLOT_06"),
    )

    message = TaskManagerNode._task_command_to_json(command)
    payload = json.loads(message.data)

    assert payload == {
        "task_id": "TASK-001",
        "command_id": "TASK-001-CMD-001",
        "operation": "CULL",
        "recipe_id": "CULL_DEFECT_SLOTS",
        "pallet_id": "PALLET_001",
        "source": "INSPECT_STATION",
        "destination": "INSPECT_STATION",
        "target_slots": ["SLOT_03", "SLOT_06"],
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


def test_inspection_dispatch_sends_sim_context_before_command():
    started_context = not rclpy.ok()
    if started_context:
        rclpy.init()
    node = TaskManagerNode()
    events = []

    class Recorder:
        def __init__(self, name):
            self.name = name

        def publish(self, message):
            events.append((self.name, message))

    try:
        node.machine.start("TASK-001", "DEMO_HARVEST_01")
        node.machine.complete_preflight(ready=True)
        node.machine.state = CycleState.INSPECT
        node.machine.command_sequence = 6
        node.inspection_context_publisher = Recorder("context")
        node.command_publishers["inspection"] = Recorder("command")

        node._dispatch_current_step()

        assert [name for name, _ in events] == ["context", "command"]
        context = json.loads(events[0][1].data)
        command = events[1][1]
        assert context == {
            "task_id": command.task_id,
            "inspection_command_id": command.command_id,
            "pallet_id": command.pallet_id,
            "operation": "INSPECT",
        }
    finally:
        node.destroy_node()
        if started_context:
            rclpy.shutdown()


def test_cull_waits_for_matching_stored_detections_after_inspection_result():
    started_context = not rclpy.ok()
    if started_context:
        rclpy.init()
    node = TaskManagerNode()
    published = []

    class Recorder:
        def publish(self, message):
            published.append(json.loads(message.data))

    try:
        node.machine.start("TASK-001", "DEMO_HARVEST_01")
        node.machine.complete_preflight(ready=True)
        node.machine.state = CycleState.INSPECT
        node.machine.command_sequence = 7
        inspect = node.machine.create_command()
        node.command_publishers["sim_task"] = Recorder()
        stored = String()
        stored.data = json.dumps({
            "state": "STORED", "task_id": "TASK-001",
            "inspection_command_id": inspect.command_id,
            "pallet_id": "PALLET_001", "reason": "NONE",
        })
        node._inspection_data_status_callback(stored)
        assert published == []
        node._handle_result(TaskResultData(
            task_id=inspect.task_id,
            command_id=inspect.command_id,
            operation="INSPECT",
            status="SUCCEEDED",
            defect_slots=("SLOT_03",),
        ), "inspection")
        assert node.machine.state == CycleState.CULL
        node._tick()
        assert len(published) == 1
        assert published[0]["operation"] == "CULL"
        assert published[0]["target_slots"] == ["SLOT_03"]
    finally:
        node.destroy_node()
        if started_context:
            rclpy.shutdown()


def test_cull_does_not_dispatch_before_sim_stores_matching_inspection():
    started_context = not rclpy.ok()
    if started_context:
        rclpy.init()
    node = TaskManagerNode()
    published = []

    class Recorder:
        def publish(self, message):
            published.append(json.loads(message.data))

    try:
        node.machine.start("TASK-001", "DEMO_HARVEST_01")
        node.machine.complete_preflight(ready=True)
        node.machine.state = CycleState.INSPECT
        node.machine.command_sequence = 7
        inspect = node.machine.create_command()
        node.command_publishers["sim_task"] = Recorder()
        node._handle_result(TaskResultData(
            task_id=inspect.task_id,
            command_id=inspect.command_id,
            operation="INSPECT",
            status="SUCCEEDED",
            defect_slots=("SLOT_03",),
        ), "inspection")
        node._tick()
        assert published == []
        stale = String()
        stale.data = json.dumps({
            "state": "STORED", "task_id": "TASK-001",
            "inspection_command_id": "TASK-001-CMD-OLD",
            "pallet_id": "PALLET_001",
        })
        node._inspection_data_status_callback(stale)
        node._tick()
        assert published == []
    finally:
        node.destroy_node()
        if started_context:
            rclpy.shutdown()


def test_cull_data_timeout_fails_without_publishing_physical_command():
    started_context = not rclpy.ok()
    if started_context:
        rclpy.init()
    node = TaskManagerNode()
    published = []

    class Recorder:
        def publish(self, message):
            published.append(message)

    try:
        node.machine.start("TASK-001", "DEMO_HARVEST_01")
        node.machine.complete_preflight(ready=True)
        node.machine.state = CycleState.INSPECT
        node.machine.command_sequence = 7
        inspect = node.machine.create_command()
        node.command_publishers["sim_task"] = Recorder()
        node._handle_result(TaskResultData(
            task_id=inspect.task_id,
            command_id=inspect.command_id,
            operation="INSPECT",
            status="SUCCEEDED",
            defect_slots=("SLOT_03",),
        ), "inspection")
        node.cull_data_deadline = 0.0
        node._tick()
        assert published == []
        assert node.machine.state == CycleState.ERROR
        assert node.machine.failure_reason == "INSPECTION_DATA_TIMEOUT"
    finally:
        node.destroy_node()
        if started_context:
            rclpy.shutdown()


def test_convey_result_keeps_pallet_and_destination():
    result = TaskManagerNode._task_result_from_json(json.dumps({
        "task_id": "TASK-001",
        "command_id": "TASK-001-CMD-005",
        "operation": "CONVEY_TO_INSPECT",
        "pallet_id": "PALLET_001",
        "status": "SUCCEEDED",
        "reached_station": "INSPECT_STOP",
    }))

    assert result.pallet_id == "PALLET_001"
    assert result.reached_station == "INSPECT_STOP"


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
