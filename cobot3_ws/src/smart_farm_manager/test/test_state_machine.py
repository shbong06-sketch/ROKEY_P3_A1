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
        "pallet_id": command.pallet_id,
    }
    values.update(overrides)
    return TaskResultData(**values)


def complete_transfer(machine: CycleStateMachine) -> None:
    command = machine.create_command()
    assert command.operation == "TRANSFER"

    result = success_result(
        command,
        completed_units=(
            "PALLET_002:RACK_L3:RACK_L2",
            "PALLET_003:RACK_L4:RACK_L3",
        ),
    )

    outcome = machine.handle_result(result)
    assert outcome.accepted is True


def complete_pick_harvest(machine: CycleStateMachine) -> None:
    command = machine.create_command()
    assert command.operation == "PICK_HARVEST"
    assert command.pallet_id == "PALLET_001"
    assert command.source == "RACK_L1"
    assert command.destination == "CARRY"

    result = success_result(
        command,
        safe_to_navigate=True,
    )

    outcome = machine.handle_result(result)
    assert outcome.accepted is True


def complete_navigation(machine: CycleStateMachine) -> None:
    command = machine.create_command()
    assert command.operation == "NAVIGATION"
    assert command.destination == "FEEDER_DOCK"

    result = success_result(
        command,
        reached_station="FEEDER_DOCK",
    )

    outcome = machine.handle_result(result)
    assert outcome.accepted is True


def complete_place_inspect(machine: CycleStateMachine) -> None:
    command = machine.create_command()
    outcome = machine.handle_result(success_result(command))

    assert outcome.accepted is True


def complete_inspection_preparation(machine: CycleStateMachine) -> None:
    convey = machine.create_command()
    assert convey.operation == "CONVEY_TO_INSPECT"
    assert convey.pallet_id == "PALLET_001"
    assert machine.state == CycleState.CONVEY_TO_INSPECT
    machine.handle_result(success_result(
        convey, pallet_id="PALLET_001", reached_station="INSPECT_STOP",
    ))
    assert machine.state == CycleState.PREPARE_INSPECT

    prepare = machine.create_command()
    assert prepare.operation == "PREPARE_INSPECT"
    assert prepare.pallet_id == "PALLET_001"
    machine.handle_result(success_result(
        prepare, pallet_id="PALLET_001", reached_station="INSPECT_WORK_POS",
    ))
    assert machine.state == CycleState.MOVE_TO_INSPECT

    move = machine.create_command()
    assert move.operation == "MOVE_TO_INSPECT"
    assert move.pallet_id == "PALLET_001"
    assert move.source == "INSPECT_WORK_POS"
    assert move.destination == "INSPECT_CAMERA_POSE"
    machine.handle_result(success_result(
        move, pallet_id="PALLET_001", reached_station="INSPECT_CAMERA_POSE",
    ))
    assert machine.state == CycleState.INSPECT


def test_inspection_waits_for_matching_pose_move_result():
    machine = start_machine()
    complete_transfer(machine)
    complete_pick_harvest(machine)
    complete_navigation(machine)
    complete_place_inspect(machine)
    convey = machine.create_command()
    machine.handle_result(success_result(
        convey, pallet_id="PALLET_001", reached_station="INSPECT_STOP",
    ))
    prepare = machine.create_command()
    machine.handle_result(success_result(
        prepare, pallet_id="PALLET_001", reached_station="INSPECT_WORK_POS",
    ))
    move = machine.create_command()
    assert machine.state == CycleState.MOVE_TO_INSPECT
    assert machine.handle_result(success_result(
        move, command_id="old-command", pallet_id="PALLET_001",
        reached_station="INSPECT_CAMERA_POSE",
    )).accepted is False
    assert machine.state == CycleState.MOVE_TO_INSPECT
    assert machine.handle_result(success_result(
        move, pallet_id="PALLET_001", reached_station="INSPECT_CAMERA_POSE",
    )).accepted is True
    assert machine.state == CycleState.INSPECT


def test_inspection_pose_failure_stops_before_vision_command():
    machine = start_machine()
    complete_transfer(machine)
    complete_pick_harvest(machine)
    complete_navigation(machine)
    complete_place_inspect(machine)
    convey = machine.create_command()
    machine.handle_result(success_result(
        convey, pallet_id="PALLET_001", reached_station="INSPECT_STOP",
    ))
    prepare = machine.create_command()
    machine.handle_result(success_result(
        prepare, pallet_id="PALLET_001", reached_station="INSPECT_WORK_POS",
    ))
    move = machine.create_command()
    outcome = machine.handle_result(TaskResultData(
        task_id=move.task_id,
        command_id=move.command_id,
        operation=move.operation,
        status="FAILED",
        phase="MOVE_TO_INSPECT/FAULT",
        reason="INSPECT_POSE_FAILED",
        pallet_id="PALLET_001",
    ))
    assert outcome.accepted is True
    assert machine.state == CycleState.ERROR
    assert machine.failure_reason == "INSPECT_POSE_FAILED"


def test_full_cycle_with_defects():
    machine = start_machine()

    complete_transfer(machine)
    assert machine.state == CycleState.PICK_HARVEST
    assert machine.pallet_locations["PALLET_002"] == "RACK_L2"
    assert machine.pallet_locations["PALLET_003"] == "RACK_L3"

    complete_pick_harvest(machine)
    assert machine.state == CycleState.NAVIGATION
    assert machine.pallet_locations["PALLET_001"] == "CARRY"

    complete_navigation(machine)
    assert machine.state == CycleState.PLACE_INSPECT

    complete_place_inspect(machine)
    assert machine.state == CycleState.CONVEY_TO_INSPECT
    complete_inspection_preparation(machine)
    assert machine.state == CycleState.INSPECT
    assert machine.pallet_locations["PALLET_001"] == "INSPECT_WORK_POS"

    inspect_command = machine.create_command()
    inspect_result = success_result(
        inspect_command,
        defect_slots=("SLOT_03", "SLOT_06"),
        unknown_slots=(),
    )
    machine.handle_result(inspect_result)

    assert machine.state == CycleState.CULL
    assert machine.defect_slots == ("SLOT_03", "SLOT_06")

    cull_command = machine.create_command()
    assert cull_command.target_slots == ("SLOT_03", "SLOT_06")

    cull_result = success_result(
        cull_command,
        completed_units=("SLOT_03", "SLOT_06"),
    )
    machine.handle_result(cull_result)

    assert machine.state == CycleState.RECHECK

    recheck_command = machine.create_command()
    assert recheck_command.operation == "RECHECK"
    assert recheck_command.target_slots == ("SLOT_03", "SLOT_06")
    machine.handle_result(success_result(recheck_command))
    assert machine.state == CycleState.RELEASE_INSPECT

    release_command = machine.create_command()
    assert release_command.operation == "RELEASE_INSPECT"
    machine.handle_result(success_result(release_command, reached_station="INSPECT_STOP"))
    assert machine.state == CycleState.CONVEYOR_OUT

    conveyor_command = machine.create_command()
    machine.handle_result(success_result(conveyor_command, reached_station="PACK_OUT"))

    assert machine.state == CycleState.COMPLETE
    assert machine.terminal_status == "SUCCEEDED"
    assert machine.pallet_locations["PALLET_001"] == "PACK_OUT"


def test_cycle_skips_cull_when_no_defect_exists():
    machine = start_machine()

    complete_transfer(machine)
    complete_pick_harvest(machine)
    complete_navigation(machine)
    complete_place_inspect(machine)
    complete_inspection_preparation(machine)

    inspect_command = machine.create_command()
    inspect_result = success_result(
        inspect_command,
        defect_slots=(),
        unknown_slots=(),
    )
    machine.handle_result(inspect_result)

    assert machine.state == CycleState.RECHECK

    recheck_command = machine.create_command()
    assert recheck_command.operation == "RECHECK"
    assert recheck_command.target_slots == ()
    machine.handle_result(success_result(recheck_command))
    assert machine.state == CycleState.RELEASE_INSPECT
    release_command = machine.create_command()
    machine.handle_result(success_result(release_command, reached_station="INSPECT_STOP"))
    assert machine.state == CycleState.CONVEYOR_OUT
    out_command = machine.create_command()
    machine.handle_result(success_result(out_command, reached_station="PACK_OUT"))
    assert machine.state == CycleState.COMPLETE


def test_recheck_residual_defect_and_unknown_stop_before_release():
    for slots, unknown, reason in (
        (("SLOT_03",), (), "DEFECT_REMAINS"),
        ((), ("SLOT_04",), "UNKNOWN_SLOT"),
    ):
        machine = start_machine()
        complete_transfer(machine)
        complete_pick_harvest(machine)
        complete_navigation(machine)
        complete_place_inspect(machine)
        complete_inspection_preparation(machine)
        inspect = machine.create_command()
        machine.handle_result(success_result(inspect))
        recheck = machine.create_command()
        machine.handle_result(success_result(
            recheck, defect_slots=slots, unknown_slots=unknown,
        ))
        assert machine.state == CycleState.ERROR
        assert machine.failure_reason == reason


def test_release_and_out_require_confirmed_destinations():
    machine = start_machine()
    complete_transfer(machine)
    complete_pick_harvest(machine)
    complete_navigation(machine)
    complete_place_inspect(machine)
    complete_inspection_preparation(machine)
    inspect = machine.create_command()
    machine.handle_result(success_result(inspect))
    recheck = machine.create_command()
    machine.handle_result(success_result(recheck))
    release = machine.create_command()
    machine.handle_result(success_result(release, reached_station=""))
    assert machine.state == CycleState.ERROR
    assert machine.failure_reason == "POSITION_NOT_CONFIRMED"


def test_post_inspection_failures_and_timeouts_stop_the_cycle():
    for operation, reason in (
        ("RECHECK", "DEFECT_REMAINS"),
        ("RELEASE_INSPECT", "RELEASE_FAILED"),
        ("CONVEYOR_OUT", "CONVEYOR_TIMEOUT"),
    ):
        for status, expected_reason in (("FAILED", reason), ("TIMEOUT", "RESULT_TIMEOUT")):
            machine = start_machine()
            complete_transfer(machine)
            complete_pick_harvest(machine)
            complete_navigation(machine)
            complete_place_inspect(machine)
            complete_inspection_preparation(machine)
            inspect = machine.create_command()
            machine.handle_result(success_result(inspect))
            if operation != "RECHECK":
                recheck = machine.create_command()
                machine.handle_result(success_result(recheck))
            if operation == "CONVEYOR_OUT":
                release = machine.create_command()
                machine.handle_result(success_result(release, reached_station="INSPECT_STOP"))
            command = machine.create_command()
            assert command.operation == operation
            machine.handle_result(TaskResultData(
                task_id=command.task_id,
                command_id=command.command_id,
                operation=command.operation,
                pallet_id=command.pallet_id,
                status=status,
                reason=expected_reason,
            ))
            assert machine.state == CycleState.ERROR
            assert machine.failure_reason == expected_reason
            assert machine.terminal_status == (
                "FAILED" if operation == "RECHECK" else "RESET_REQUIRED"
            )


def test_cull_does_not_succeed_with_only_some_confirmed_slots():
    machine = start_machine()
    complete_transfer(machine)
    complete_pick_harvest(machine)
    complete_navigation(machine)
    complete_place_inspect(machine)
    complete_inspection_preparation(machine)
    inspect = machine.create_command()
    machine.handle_result(success_result(
        inspect, defect_slots=("SLOT_03", "SLOT_05"),
    ))
    cull = machine.create_command()
    assert cull.target_slots == ("SLOT_03", "SLOT_05")
    machine.handle_result(success_result(
        cull, completed_units=("SLOT_03",),
    ))
    assert machine.state == CycleState.ERROR
    assert machine.failure_reason == "CULL_INCOMPLETE"


def test_unknown_inspection_slot_causes_error():
    machine = start_machine()

    complete_transfer(machine)
    complete_pick_harvest(machine)
    complete_navigation(machine)
    complete_place_inspect(machine)
    complete_inspection_preparation(machine)

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


def test_inspection_rejects_slot_outside_six_slot_contract():
    machine = start_machine()

    complete_transfer(machine)
    complete_pick_harvest(machine)
    complete_navigation(machine)
    complete_place_inspect(machine)
    complete_inspection_preparation(machine)

    command = machine.create_command()
    result = success_result(
        command,
        defect_slots=("SLOT_07",),
        unknown_slots=(),
    )

    machine.handle_result(result)

    assert machine.state == CycleState.ERROR
    assert machine.terminal_status == "FAILED"
    assert machine.failure_reason == "INVALID_SLOT_ID"


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


def test_pallet_detection_status_cannot_advance_convey_command():
    machine = start_machine()
    complete_transfer(machine)
    complete_pick_harvest(machine)
    complete_navigation(machine)
    complete_place_inspect(machine)

    command = machine.create_command()
    assert command.operation == "CONVEY_TO_INSPECT"
    # PALLET_DETECTED는 executor status이며 handle_result 호출 대상이 아니다.
    assert machine.state == CycleState.CONVEY_TO_INSPECT
    assert machine.active_command == command

    wrong = success_result(
        command, pallet_id="PALLET_999", reached_station="INSPECT_STOP",
    )
    outcome = machine.handle_result(wrong)
    assert outcome.accepted is False
    assert outcome.reason == "MISMATCHED_RESULT"
    assert machine.state == CycleState.CONVEY_TO_INSPECT
    assert machine.active_command == command

    wrong = success_result(
        command, pallet_id="PALLET_001", reached_station="",
    )
    outcome = machine.handle_result(wrong)
    assert outcome.accepted is True
    assert machine.state == CycleState.ERROR
    assert machine.failure_reason == "POSITION_NOT_CONFIRMED"


def test_prepare_failure_does_not_start_inspection():
    machine = start_machine()
    complete_transfer(machine)
    complete_pick_harvest(machine)
    complete_navigation(machine)
    complete_place_inspect(machine)

    convey = machine.create_command()
    machine.handle_result(success_result(
        convey, pallet_id="PALLET_001", reached_station="INSPECT_STOP",
    ))
    prepare = machine.create_command()
    outcome = machine.handle_result(TaskResultData(
        task_id=prepare.task_id,
        command_id=prepare.command_id,
        operation=prepare.operation,
        pallet_id=prepare.pallet_id,
        status="FAILED",
        reason="PREPARE_TIMEOUT",
    ))
    assert outcome.accepted is True
    assert machine.state == CycleState.ERROR
    assert machine.failure_reason == "PREPARE_TIMEOUT"
    assert machine.terminal_status == "RESET_REQUIRED"


def test_incomplete_transfer_requires_reset():
    machine = start_machine()
    command = machine.create_command()

    result = success_result(
        command,
        completed_units=(
            "PALLET_002:RACK_L3:RACK_L2",
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
