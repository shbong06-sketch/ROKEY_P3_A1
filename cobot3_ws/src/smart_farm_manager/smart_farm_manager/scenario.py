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
                timeout_sec=180.0,
            ),
            StepDefinition(
                state=CycleState.PICK_HARVEST,
                executor=ExecutorName.SIM_TASK,
                operation="PICK_HARVEST",
                recipe_id="HARVEST_RACK_L1",
                pallet_id="PALLET_001",
                source="RACK_L1",
                destination="CARRY",
                timeout_sec=90.0,
            ),
            StepDefinition(
                state=CycleState.NAVIGATION,
                executor=ExecutorName.NAVIGATION,
                operation="NAVIGATION",
                destination="INSPECTION_DOCK",
                timeout_sec=120.0,
            ),
            StepDefinition(
                state=CycleState.PLACE_INSPECT,
                executor=ExecutorName.SIM_TASK,
                operation="PLACE_INSPECT",
                recipe_id="PLACE_AT_INSPECTION",
                pallet_id="PALLET_001",
                source="CARRY",
                destination="INSPECT_STATION",
                timeout_sec=90.0,
            ),
            StepDefinition(
                state=CycleState.INSPECT,
                executor=ExecutorName.INSPECTION,
                operation="INSPECT",
                pallet_id="PALLET_001",
                source="INSPECT_STATION",
                timeout_sec=30.0,
            ),
            StepDefinition(
                state=CycleState.CULL,
                executor=ExecutorName.SIM_TASK,
                operation="CULL",
                recipe_id="CULL_DEFECT_SLOTS",
                pallet_id="PALLET_001",
                source="INSPECT_STATION",
                destination="INSPECT_STATION",
                timeout_sec=120.0,
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
                timeout_sec=60.0,
            ),
        ),
    )