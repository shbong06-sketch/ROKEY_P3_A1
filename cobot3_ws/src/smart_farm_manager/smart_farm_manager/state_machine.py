"""스마트팜 통합 시연의 순수 Python 상태 머신."""

from typing import Dict, Optional, Set, Tuple

from .protocol import (
    ResultProcessOutcome,
    TaskCommandData,
    TaskResultData,
)
from .scenario import (
    CycleState,
    ScenarioDefinition,
    create_demo_harvest_scenario,
)


class CycleStateMachine:
    """시나리오 규칙에 따라 명령을 만들고 결과를 상태 전이에 반영한다."""

    VALID_PLANT_SLOTS: Set[str] = {
        f"SLOT_{index:02d}" for index in range(1, 7)
    }

    EXPECTED_TRANSFER_UNITS: Set[str] = {
        "PALLET_002:RACK_L3:RACK_L2",
        "PALLET_003:RACK_L4:RACK_L3",
    }

    PHYSICAL_OPERATIONS: Set[str] = {
        "TRANSFER",
        "PICK_HARVEST",
        "PLACE_INSPECT",
        "CONVEY_TO_INSPECT",
        "PREPARE_INSPECT",
        "CULL",
        "CONVEYOR_OUT",
    }

    def __init__(
        self,
        scenario: Optional[ScenarioDefinition] = None,
    ) -> None:
        """시나리오와 사이클 실행에 필요한 논리 상태를 초기화한다."""

        self.scenario = scenario or create_demo_harvest_scenario()

        self.state = CycleState.IDLE
        self.task_id = ""
        self.command_sequence = 0
        self.active_command: Optional[TaskCommandData] = None

        self.terminal_status = "IDLE"
        self.failure_reason = "NONE"

        self.defect_slots: Tuple[str, ...] = ()
        self.unknown_slots: Tuple[str, ...] = ()

        self.pallet_locations: Dict[str, str] = {}
        self._restore_initial_logical_state()

    def _restore_initial_logical_state(self) -> None:
        """팔레트 위치를 시나리오 시작 전 논리 상태로 복원한다."""

        self.pallet_locations = {
            "PALLET_001": "RACK_L1",
            "PALLET_002": "RACK_L3",
            "PALLET_003": "RACK_L4",
        }

    @property
    def is_terminal(self) -> bool:
        """현재 사이클이 성공 또는 오류로 종료됐는지 반환한다."""

        return self.state in {
            CycleState.COMPLETE,
            CycleState.ERROR,
        }

    @property
    def is_running(self) -> bool:
        """현재 사이클이 명령을 수행할 수 있는 실행 상태인지 반환한다."""

        return self.state not in {
            CycleState.IDLE,
            CycleState.COMPLETE,
            CycleState.ERROR,
        }

    def start(self, task_id: str, scenario_id: str) -> None:
        """새 사이클을 시작하고 PREFLIGHT로 진입한다."""

        if self.state != CycleState.IDLE:
            raise RuntimeError(
                f"IDLE 상태에서만 시작할 수 있습니다: {self.state.value}"
            )

        if scenario_id != self.scenario.scenario_id:
            raise ValueError(
                f"지원하지 않는 scenario_id입니다: {scenario_id}"
            )

        if not task_id:
            raise ValueError("task_id가 비어 있습니다.")

        self.task_id = task_id
        self.command_sequence = 0
        self.active_command = None

        self.terminal_status = "RUNNING"
        self.failure_reason = "NONE"

        self.defect_slots = ()
        self.unknown_slots = ()
        self._restore_initial_logical_state()

        self.state = CycleState.PREFLIGHT

    def complete_preflight(
        self,
        ready: bool,
        reason: str = "NONE",
    ) -> None:
        """Executor 준비 상태 검사 결과를 반영한다."""

        if self.state != CycleState.PREFLIGHT:
            raise RuntimeError(
                "PREFLIGHT 상태에서만 준비 완료 처리가 가능합니다."
            )

        if ready:
            self.state = CycleState.TRANSFER
            return

        self._fail(
            reason=reason if reason != "NONE" else "NOT_READY",
            reset_required=False,
        )

    def create_command(self) -> TaskCommandData:
        """현재 공정 단계에 대응하는 새 명령을 생성한다."""

        if self.active_command is not None:
            raise RuntimeError(
                "처리 중인 active command가 이미 존재합니다."
            )

        if self.state in {
            CycleState.IDLE,
            CycleState.PREFLIGHT,
            CycleState.COMPLETE,
            CycleState.ERROR,
        }:
            raise RuntimeError(
                f"{self.state.value} 상태에서는 명령을 생성할 수 없습니다."
            )

        step = self.scenario.step_for(self.state)

        self.command_sequence += 1
        command_id = (
            f"{self.task_id}-CMD-{self.command_sequence:03d}"
        )

        target_slots: Tuple[str, ...] = ()
        if self.state == CycleState.CULL:
            target_slots = self.defect_slots

            if not target_slots:
                raise RuntimeError(
                    "솎아내기 대상 슬롯이 없습니다."
                )

        command = TaskCommandData(
            task_id=self.task_id,
            command_id=command_id,
            operation=step.operation,
            recipe_id=step.recipe_id,
            pallet_id=step.pallet_id,
            source=step.source,
            destination=step.destination,
            target_slots=target_slots,
        )

        self.active_command = command
        return command

    def handle_result(
        self,
        result: TaskResultData,
    ) -> ResultProcessOutcome:
        """Executor의 terminal 결과를 검증하고 상태를 전이한다."""

        if self.active_command is None:
            return ResultProcessOutcome(
                accepted=False,
                state_changed=False,
                reason="NO_ACTIVE_COMMAND",
            )

        if not self._matches_active_command(result):
            return ResultProcessOutcome(
                accepted=False,
                state_changed=False,
                reason="MISMATCHED_RESULT",
            )

        command = self.active_command
        self.active_command = None

        if result.status != "SUCCEEDED":
            failure_reason = (
                result.reason
                if result.reason != "NONE"
                else result.status
            )

            self._fail(
                reason=failure_reason,
                reset_required=(
                    command.operation in self.PHYSICAL_OPERATIONS
                ),
            )

            return ResultProcessOutcome(
                accepted=True,
                state_changed=True,
                reason=failure_reason,
            )

        validation_error = self._validate_success(
            command=command,
            result=result,
        )

        if validation_error is not None:
            self._fail(
                reason=validation_error,
                reset_required=(
                    command.operation in self.PHYSICAL_OPERATIONS
                ),
            )

            return ResultProcessOutcome(
                accepted=True,
                state_changed=True,
                reason=validation_error,
            )

        self._apply_success(
            command=command,
            result=result,
        )

        return ResultProcessOutcome(
            accepted=True,
            state_changed=True,
            reason="NONE",
        )

    def _matches_active_command(
        self,
        result: TaskResultData,
    ) -> bool:
        """결과의 task, command, operation이 활성 명령과 같은지 확인한다."""

        command = self.active_command

        return (
            command is not None
            and result.task_id == command.task_id
            and result.command_id == command.command_id
            and result.operation == command.operation
            and (
                command.operation not in {"CONVEY_TO_INSPECT", "PREPARE_INSPECT"}
                or result.pallet_id == command.pallet_id
            )
        )

    def _validate_success(
        self,
        command: TaskCommandData,
        result: TaskResultData,
    ) -> Optional[str]:
        """SUCCEEDED 결과가 operation별 성공 조건을 충족하는지 검사한다."""

        if command.operation == "TRANSFER":
            completed = set(result.completed_units)

            if not self.EXPECTED_TRANSFER_UNITS.issubset(completed):
                return "TRANSFER_INCOMPLETE"

        elif command.operation in {"CONVEY_TO_INSPECT", "PREPARE_INSPECT"}:
            if result.pallet_id != command.pallet_id:
                return "PALLET_MISMATCH"
            if result.reached_station != command.destination:
                return "POSITION_NOT_CONFIRMED"

        elif command.operation == "PICK_HARVEST":
            if not result.safe_to_navigate:
                return "TRANSPORT_NOT_SAFE"

        elif command.operation == "NAVIGATION":
            if result.reached_station != command.destination:
                return "DOCKING_ERROR"

        elif command.operation == "INSPECT":
            invalid_slots = (
                set(result.defect_slots)
                | set(result.unknown_slots)
            ) - self.VALID_PLANT_SLOTS

            if invalid_slots:
                return "INVALID_SLOT_ID"

            if result.unknown_slots:
                return "UNKNOWN_SLOT"

        elif command.operation == "CULL":
            completed_slots = set(result.completed_units)
            expected_slots = set(command.target_slots)

            if not expected_slots.issubset(completed_slots):
                return "CULL_INCOMPLETE"

        return None

    def _apply_success(
        self,
        command: TaskCommandData,
        result: TaskResultData,
    ) -> None:
        """성공 결과에 따른 논리 상태와 다음 공정 상태를 반영한다."""

        if command.operation == "TRANSFER":
            self.pallet_locations["PALLET_002"] = "RACK_L2"
            self.pallet_locations["PALLET_003"] = "RACK_L3"
            self.state = CycleState.PICK_HARVEST

        elif command.operation == "PICK_HARVEST":
            self.pallet_locations["PALLET_001"] = "CARRY"
            self.state = CycleState.NAVIGATION

        elif command.operation == "NAVIGATION":
            self.state = CycleState.PLACE_INSPECT

        elif command.operation == "PLACE_INSPECT":
            self.pallet_locations["PALLET_001"] = "INSPECT_STATION"
            self.state = CycleState.CONVEY_TO_INSPECT

        elif command.operation == "CONVEY_TO_INSPECT":
            self.pallet_locations["PALLET_001"] = "INSPECT_STOP"
            self.state = CycleState.PREPARE_INSPECT

        elif command.operation == "PREPARE_INSPECT":
            self.pallet_locations["PALLET_001"] = "INSPECT_WORK_POS"
            self.state = CycleState.INSPECT

        elif command.operation == "INSPECT":
            self.defect_slots = tuple(result.defect_slots)
            self.unknown_slots = tuple(result.unknown_slots)

            if self.defect_slots:
                self.state = CycleState.CULL
            else:
                self.state = CycleState.CONVEYOR_OUT

        elif command.operation == "CULL":
            self.state = CycleState.CONVEYOR_OUT

        elif command.operation == "CONVEYOR_OUT":
            self.pallet_locations["PALLET_001"] = "PACK_OUT"
            self.state = CycleState.COMPLETE
            self.terminal_status = "SUCCEEDED"

        else:
            self._fail(
                reason="INVALID_COMMAND",
                reset_required=False,
            )

    def _fail(
        self,
        reason: str,
        reset_required: bool,
    ) -> None:
        """상태를 ERROR로 전이하고 복구 필요 여부에 맞는 상태를 기록한다."""

        self.active_command = None
        self.state = CycleState.ERROR
        self.failure_reason = reason

        if reset_required:
            self.terminal_status = "RESET_REQUIRED"
        else:
            self.terminal_status = "FAILED"

    def reset(self) -> None:
        """종료 상태를 지우고 다음 사이클을 받을 수 있도록 초기화한다."""

        if self.is_running:
            raise RuntimeError(
                "실행 중에는 상태 머신을 reset할 수 없습니다."
            )

        self.state = CycleState.IDLE
        self.task_id = ""
        self.command_sequence = 0
        self.active_command = None

        self.terminal_status = "IDLE"
        self.failure_reason = "NONE"

        self.defect_slots = ()
        self.unknown_slots = ()
        self._restore_initial_logical_state()
