"""Task Manager 내부 명령, 결과 데이터 모델.

이 모듈은 ROS2와 Isaac Sim에 의존하지 않는다.
4단계에서 TaskCommand.msg와 TaskResult.msg 사이를 변환하는 Adapter를 추가한다.
"""

from dataclasses import dataclass
from typing import Tuple


@dataclass(frozen=True)
class TaskCommandData:
    task_id: str
    command_id: str

    operation: str
    recipe_id: str = ""

    pallet_id: str = ""
    source: str = ""
    destination: str = ""
    target_slots: Tuple[str, ...] = ()


@dataclass(frozen=True)
class TaskResultData:
    task_id: str
    command_id: str

    operation: str
    status: str
    phase: str = ""
    reason: str = "NONE"

    safe_to_navigate: bool = False
    reached_station: str = ""

    completed_units: Tuple[str, ...] = ()
    defect_slots: Tuple[str, ...] = ()
    unknown_slots: Tuple[str, ...] = ()


@dataclass(frozen=True)
class ResultProcessOutcome:
    """수신 결과를 Task Manager가 처리한 결과."""

    accepted: bool
    state_changed: bool
    reason: str = "NONE"