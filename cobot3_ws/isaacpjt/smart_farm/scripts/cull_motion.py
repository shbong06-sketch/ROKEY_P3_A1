"""로봇 base 기준 좌표로 솎아내기 pick/place 순서를 만든다."""

from dataclasses import dataclass
from math import isfinite
from typing import Optional, Tuple


Position = Tuple[float, float, float]


@dataclass(frozen=True)
class CullStep:
    """한 단계의 TCP 목표와 그리퍼 명령."""

    name: str
    position_base: Optional[Position] = None
    gripper: Optional[str] = None


@dataclass(frozen=True)
class CullConfig:
    """솎아내기 동작에서 현장 보정이 필요한 값."""

    place_position_base: Position
    approach_clearance: float = 0.12
    transit_clearance: float = 0.18
    pick_z_offset: float = 0.0
    place_z_offset: float = 0.0

    def __post_init__(self):
        _position(self.place_position_base, "place_position_base")
        for name in ("approach_clearance", "transit_clearance"):
            value = float(getattr(self, name))
            if not isfinite(value) or value <= 0.0:
                raise ValueError(f"{name}은 0보다 큰 유한한 값이어야 합니다.")
        for name in ("pick_z_offset", "place_z_offset"):
            if not isfinite(float(getattr(self, name))):
                raise ValueError(f"{name}은 유한한 값이어야 합니다.")


def build_cull_plan(
    detected_position_base: Position,
    config: CullConfig,
) -> Tuple[CullStep, ...]:
    """탐지 위치에서 집어 고정 배출 위치에 놓는 순서를 반환한다."""

    pick = list(_position(detected_position_base, "detected_position_base"))
    place = list(_position(config.place_position_base, "place_position_base"))
    pick[2] += config.pick_z_offset
    place[2] += config.place_z_offset

    transit_z = max(pick[2], place[2]) + config.transit_clearance
    pick_approach = (pick[0], pick[1], pick[2] + config.approach_clearance)
    pick_position = tuple(pick)
    pick_transit = (pick[0], pick[1], transit_z)
    place_transit = (place[0], place[1], transit_z)
    place_position = tuple(place)

    return (
        CullStep("OPEN", gripper="open"),
        CullStep("PICK_APPROACH", pick_approach),
        CullStep("PICK_DESCEND", pick_position),
        CullStep("GRASP", gripper="close"),
        CullStep("LIFT", pick_transit),
        CullStep("PLACE_APPROACH", place_transit),
        CullStep("PLACE_DESCEND", place_position),
        CullStep("RELEASE", gripper="open"),
        CullStep("RETREAT", place_transit),
    )


def _position(value, name: str) -> Position:
    try:
        position = tuple(float(component) for component in value)
    except (TypeError, ValueError) as error:
        raise ValueError(f"{name}은 xyz 숫자 3개여야 합니다.") from error
    if len(position) != 3 or not all(isfinite(component) for component in position):
        raise ValueError(f"{name}은 유한한 xyz 숫자 3개여야 합니다.")
    return position
