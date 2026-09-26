"""DEMO_HARVEST_01 시나리오 정의."""

from dataclasses import dataclass
from enum import Enum
from typing import Tuple


class CycleState(str, Enum):
    """Task Manager가 가질 수 있는 사이클 상태."""

    IDLE = "IDLE"
    PREFLIGHT = "PREFLIGHT"

    TRANSFER = "TRANSFER"
    PICK_HARVEST = "PICK_HARVEST"
    NAVIGATION = "NAVIGATION"
    PLACE_INSPECT = "PLACE_INSPECT"
    CONVEY_TO_INSPECT = "CONVEY_TO_INSPECT"
    PREPARE_INSPECT = "PREPARE_INSPECT"
    INSPECT = "INSPECT"
    CULL = "CULL"
    CONVEYOR_OUT = "CONVEYOR_OUT"

    COMPLETE = "COMPLETE"
    ERROR = "ERROR"


class ExecutorName(str, Enum):
    """공정 명령을 수행하는 executor 식별자."""

    SIM_TASK = "sim_task"
    NAVIGATION = "navigation"
    INSPECTION = "inspection"


@dataclass(frozen=True)
class StepDefinition:
    """한 공정의 담당 executor, 명령 인자 및 제한 시간 정의."""

    state: CycleState
    executor: ExecutorName
    operation: str

    recipe_id: str = ""
    pallet_id: str = ""
    source: str = ""
    destination: str = ""

    timeout_sec: float = 60.0
    optional: bool = False


@dataclass(frozen=True)
class ScenarioDefinition:
    """시나리오 식별자와 순서가 있는 공정 정의 모음."""

    scenario_id: str
    steps: Tuple[StepDefinition, ...]

    def step_for(self, state: CycleState) -> StepDefinition:
        """주어진 사이클 상태에 대응하는 공정 정의를 반환한다."""

        for step in self.steps:
            if step.state == state:
                return step

        raise KeyError(f"시나리오에 정의되지 않은 상태입니다: {state.value}")


def create_demo_harvest_scenario() -> ScenarioDefinition:
    """현재 확정된 스마트팜 통합 시연 시나리오."""

    return ScenarioDefinition(
        scenario_id="DEMO_HARVEST_01",
        steps=(
            StepDefinition(
                state=CycleState.TRANSFER,
                executor=ExecutorName.SIM_TASK,
                operation="TRANSFER",
                recipe_id="RACK_REARRANGE_01",
                timeout_sec=400.0,
            ),
            StepDefinition(
                state=CycleState.PICK_HARVEST,
                executor=ExecutorName.SIM_TASK,
                operation="PICK_HARVEST",
                recipe_id="HARVEST_RACK_L1",
                pallet_id="PALLET_001",
                source="RACK_L1",
                destination="CARRY",
                timeout_sec=200.0,
            ),
            StepDefinition(
                state=CycleState.NAVIGATION,
                executor=ExecutorName.NAVIGATION,
                operation="NAVIGATION",
                destination="FEEDER_DOCK",
                # [navigation 2026-09-23] 속도 0.3 m/s 와 Isaac 실시간 배율 0.3 이 겹쳐
                # 접근 주행과 정밀 도킹을 합치면 벽시계로 3 분을 넘길 수 있다.
                timeout_sec=400.0,
            ),
            StepDefinition(
                state=CycleState.PLACE_INSPECT,
                executor=ExecutorName.SIM_TASK,
                operation="PLACE_INSPECT",
                recipe_id="PLACE_AT_INSPECTION",
                pallet_id="PALLET_001",
                source="CARRY",
                destination="INSPECT_STATION",
                timeout_sec=300.0,
            ),
            StepDefinition(
                state=CycleState.CONVEY_TO_INSPECT,
                executor=ExecutorName.SIM_TASK,
                operation="CONVEY_TO_INSPECT",
                recipe_id="CONVEY_TO_INSPECT",
                pallet_id="PALLET_001",
                source="INSPECT_STATION",
                destination="INSPECT_STOP",
                # Isaac 물리 시간 120초 제한보다 넉넉한 벽시계 제한.
                timeout_sec=600.0,
            ),
            StepDefinition(
                state=CycleState.PREPARE_INSPECT,
                executor=ExecutorName.SIM_TASK,
                operation="PREPARE_INSPECT",
                recipe_id="PREPARE_INSPECT",
                pallet_id="PALLET_001",
                source="INSPECT_STOP",
                destination="INSPECT_WORK_POS",
                # Isaac 물리 시간 90초 제한보다 넉넉한 벽시계 제한.
                timeout_sec=450.0,
            ),
            StepDefinition(
                state=CycleState.INSPECT,
                executor=ExecutorName.INSPECTION,
                operation="INSPECT",
                pallet_id="PALLET_001",
                source="INSPECT_WORK_POS",
                timeout_sec=300.0,
            ),
            StepDefinition(
                state=CycleState.CULL,
                executor=ExecutorName.SIM_TASK,
                operation="CULL",
                recipe_id="CULL_DEFECT_SLOTS",
                pallet_id="PALLET_001",
                source="INSPECT_STATION",
                destination="INSPECT_STATION",
                timeout_sec=400.0,
                optional=True,
            ),
            StepDefinition(
                state=CycleState.CONVEYOR_OUT,
                executor=ExecutorName.SIM_TASK,
                operation="CONVEYOR_OUT",
                recipe_id="CONVEY_TO_PACK_OUT",
                pallet_id="PALLET_001",
                source="INSPECT_STATION",
                destination="PACK_OUT",
                timeout_sec=300.0,
            ),
        ),
    )
