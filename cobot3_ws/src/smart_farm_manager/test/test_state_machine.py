"""CycleStateMachine 단위 테스트."""

from smart_farm_manager.protocol import TaskResultData
from smart_farm_manager.scenario import CycleState
from smart_farm_manager.state_machine import CycleStateMachine


TASK_ID = "TASK-TEST-001"


def start_machine() -> CycleStateMachine:
    machine = CycleStateMachine()
    machine.start(
        task_id=TASK_ID,
        scenario_id="DEMO_HARVEST_01",
    )
    machine.complete_preflight(ready=True)
    return machine


def success_result(command, **overrides) -> TaskResultData:
    values = {
        "task_id": command.task_id,
        "command_id": command.command_id,
        "operation": command.operation,
        "status": "SUCCEEDED",
        "phase": "RESULT",
        "reason": "NONE",
    }
    values.update(overrides)
    return TaskResultData(**values)


def complete_transfer(machine: CycleStateMachine) -> None:
    command = machine.create_command()

    result = success_result(
        command,
        completed_units=(
            "PALLET_002:RACK_L2:RACK_L3",
            "PALLET_001:RACK_L1:RACK_L2",
        ),
    )

    outcome = machine.handle_result(result)
    assert outcome.accepted is True


def complete_pick_harvest(machine: CycleStateMachine) -> None:
    command = machine.create_command()

    result = success_result(
        command,
        safe_to_navigate=True,
    )

    outcome = machine.handle_result(result)
    assert outcome.accepted is True


def complete_navigation(machine: CycleStateMachine) -> None:
    command = machine.create_command()

    result = success_result(
        command,
        reached_station="INSPECTION_DOCK",
    )

    outcome = machine.handle_result(result)
    assert outcome.accepted is True


def complete_place_inspect(machine: CycleStateMachine) -> None:
    command = machine.create_command()
    outcome = machine.handle_result(success_result(command))

    assert outcome.accepted is True


def test_full_cycle_with_defects():
    machine = start_machine()

    complete_transfer(machine)
    assert machine.state == CycleState.PICK_HARVEST
    assert machine.pallet_locations["PALLET_002"] == "RACK_L3"
    assert machine.pallet_locations["PALLET_001"] == "RACK_L2"

    complete_pick_harvest(machine)
    assert machine.state == CycleState.NAVIGATION
    assert machine.pallet_locations["PALLET_004"] == "CARRY"

    complete_navigation(machine)
    assert machine.state == CycleState.PLACE_INSPECT

    complete_place_inspect(machine)
    assert machine.state == CycleState.INSPECT
    assert machine.pallet_locations["PALLET_004"] == "INSPECT_STATION"

    inspect_command = machine.create_command()
    inspect_result = success_result(
        inspect_command,
        defect_slots=("SLOT_03", "SLOT_07"),
        unknown_slots=(),
    )
    machine.handle_result(inspect_result)

    assert machine.state == CycleState.CULL
    assert machine.defect_slots == ("SLOT_03", "SLOT_07")

    cull_command = machine.create_command()
    assert cull_command.target_slots == ("SLOT_03", "SLOT_07")

    cull_result = success_result(
        cull_command,
        completed_units=("SLOT_03", "SLOT_07"),
    )
    machine.handle_result(cull_result)

    assert machine.state == CycleState.CONVEYOR_OUT

    conveyor_command = machine.create_command()
    machine.handle_result(success_result(conveyor_command))

    assert machine.state == CycleState.COMPLETE
    assert machine.terminal_status == "SUCCEEDED"
    assert machine.pallet_locations["PALLET_004"] == "PACK_OUT"


def test_cycle_skips_cull_when_no_defect_exists():
    machine = start_machine()

    complete_transfer(machine)
    complete_pick_harvest(machine)
    complete_navigation(machine)
    complete_place_inspect(machine)

    inspect_command = machine.create_command()
    inspect_result = success_result(
        inspect_command,
        defect_slots=(),
        unknown_slots=(),
    )
    machine.handle_result(inspect_result)

    assert machine.state == CycleState.CONVEYOR_OUT

    conveyor_command = machine.create_command()
    assert conveyor_command.operation == "CONVEYOR_OUT"


def test_unknown_inspection_slot_causes_error():
    machine = start_machine()

    complete_transfer(machine)
    complete_pick_harvest(machine)
    complete_navigation(machine)
    complete_place_inspect(machine)

    command = machine.create_command()
    result = success_result(
        command,
        defect_slots=(),
        unknown_slots=("SLOT_05",),
    )

    machine.handle_result(result)

    assert machine.state == CycleState.ERROR
    assert machine.terminal_status == "FAILED"
    assert machine.failure_reason == "UNKNOWN_SLOT"


def test_pick_harvest_requires_safe_to_navigate():
    machine = start_machine()
    complete_transfer(machine)

    command = machine.create_command()
    result = success_result(
        command,
        safe_to_navigate=False,
    )

    machine.handle_result(result)

    assert machine.state == CycleState.ERROR
    assert machine.terminal_status == "RESET_REQUIRED"
    assert machine.failure_reason == "TRANSPORT_NOT_SAFE"


def test_mismatched_result_is_ignored():
    machine = start_machine()
    command = machine.create_command()

    wrong_result = TaskResultData(
        task_id=TASK_ID,
        command_id="WRONG-COMMAND-ID",
        operation=command.operation,
        status="SUCCEEDED",
    )

    outcome = machine.handle_result(wrong_result)

    assert outcome.accepted is False
    assert outcome.state_changed is False
    assert outcome.reason == "MISMATCHED_RESULT"

    assert machine.state == CycleState.TRANSFER
    assert machine.active_command == command


def test_incomplete_transfer_requires_reset():
    machine = start_machine()
    command = machine.create_command()

    result = success_result(
        command,
        completed_units=(
            "PALLET_002:RACK_L2:RACK_L3",
        ),
    )

    machine.handle_result(result)

    assert machine.state == CycleState.ERROR
    assert machine.terminal_status == "RESET_REQUIRED"
    assert machine.failure_reason == "TRANSFER_INCOMPLETE"


def test_preflight_failure():
    machine = CycleStateMachine()
    machine.start(
        task_id=TASK_ID,
        scenario_id="DEMO_HARVEST_01",
    )

    machine.complete_preflight(
        ready=False,
        reason="NAV_NOT_READY",
    )

    assert machine.state == CycleState.ERROR
    assert machine.terminal_status == "FAILED"
    assert machine.failure_reason == "NAV_NOT_READY"


def test_reset_after_complete():
    machine = CycleStateMachine()

    machine.state = CycleState.COMPLETE
    machine.terminal_status = "SUCCEEDED"

    machine.reset()

    assert machine.state == CycleState.IDLE
    assert machine.terminal_status == "IDLE"
    assert machine.task_id == ""