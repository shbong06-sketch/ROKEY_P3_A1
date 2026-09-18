"""슬롯 pose에서 PICK의 TCP 목표를 만드는 Isaac Sim 비의존 모듈."""

from dataclasses import dataclass
from typing import Iterable

import numpy as np

from .motion_config import MotionConfig


@dataclass(frozen=True)
class Pose:
    """월드 pose. quaternion 순서는 Isaac Sim 방식인 (w, x, y, z)다."""

    position: np.ndarray
    orientation: np.ndarray

    def __post_init__(self) -> None:
        if np.asarray(self.position).shape != (3,):
            raise ValueError("position은 원소 3개여야 합니다.")
        if np.asarray(self.orientation).shape != (4,):
            raise ValueError("orientation은 wxyz 원소 4개여야 합니다.")


@dataclass(frozen=True)
class SlotPose:
    """캘리브레이션된 팔레트 포크 채널 입구 정보.

    이 객체는 확인되지 않은 USD prim path를 사용하지 않는다.
    `tcp_at_channel_entrance`는 포크 팁 중심이 채널 입구에 닿기 직전의
    월드 pose이고, `insertion_axis_world`는 랙 전면에서 내부로 향한다.
    """

    tcp_at_channel_entrance: Pose
    insertion_axis_world: np.ndarray
    slot_id: str = "UNNAMED_SLOT"


@dataclass(frozen=True)
class PickTargets:
    """PICK의 네 단계 TCP 목표와 정규화된 삽입 방향."""

    approach: Pose
    insert: Pose
    lift: Pose
    retract: Pose
    insertion_axis_world: np.ndarray


def _normalized(vector: Iterable[float]) -> np.ndarray:
    """삽입축을 길이 1의 월드 방향 벡터로 바꾼다."""

    axis = np.asarray(vector, dtype=float)
    length = np.linalg.norm(axis)
    if axis.shape != (3,) or length < 1e-9:
        raise ValueError("insertion_axis_world는 영이 아닌 3차원 벡터여야 합니다.")
    return axis / length


def build_pick_targets(slot: SlotPose, config: MotionConfig) -> PickTargets:
    """슬롯 입구 pose에서 APPROACH → INSERT → LIFT → RETRACT를 생성한다.

    INSERT와 RETRACT는 삽입축만 따라 이동하고 orientation을 바꾸지 않는다.
    LIFT만 월드 +Z 방향으로 이동한다. 즉 랙의 기울어진 로컬 축을 잘못
    위쪽으로 간주하지 않는다.
    """

    distances = (
        config.approach_distance_m,
        config.insert_depth_m,
        config.lift_distance_m,
        config.retract_distance_m,
        config.tcp_speed_m_s,
    )
    if min(distances) <= 0.0:
        raise ValueError("PICK 거리와 TCP 속도는 양수여야 합니다.")

    axis = _normalized(slot.insertion_axis_world)
    entrance = np.asarray(slot.tcp_at_channel_entrance.position, dtype=float)
    orientation = np.asarray(slot.tcp_at_channel_entrance.orientation, dtype=float)
    orientation = orientation / np.linalg.norm(orientation)
    up = np.array([0.0, 0.0, config.lift_distance_m])

    insert_position = entrance + axis * config.insert_depth_m
    return PickTargets(
        approach=Pose(entrance - axis * config.approach_distance_m, orientation),
        insert=Pose(insert_position, orientation),
        lift=Pose(insert_position + up, orientation),
        retract=Pose(
            insert_position - axis * config.retract_distance_m + up,
            orientation,
        ),
        insertion_axis_world=axis,
    )
