"""포크 PICK에서 사용하는 거리·속도·판정 기준 설정."""

from dataclasses import dataclass, field

import numpy as np


@dataclass(frozen=True)
class ForkTcp:
    """`tool0` 기준 포크 팁 중심(TCP)의 로컬 변환.

    기본값 0.2695 m는 fork_tool URDF의 기준 좌표값과 upsampling한 비율을 기반으로 계산한
    장착판 0.0275 m와 포크 길이 0.242 m에서 얻었다.
    실제 USD의 고정 조인트 변환이 달라졌다면, Isaac Sim에서 팁 중심을 측정해 이 값을 교체해야 한다.
    """

    parent_frame: str = "tool0"
    offset_m: np.ndarray = field(
        default_factory=lambda: np.array([0.0, 0.0, 0.245], dtype=float)
    )


@dataclass(frozen=True)
class MotionConfig:
    """모든 거리 단위는 m, 각도 허용오차 단위는 degree이다."""

    tcp: ForkTcp = field(default_factory=ForkTcp)

    # 슬롯 입구 기준 TCP 경로 파라미터
    approach_distance_m: float = 0.08
    insert_depth_m: float = 0.24
    lift_distance_m: float = 0.05
    retract_distance_m: float = 0.35
    tcp_speed_m_s: float = 0.04

    # IK/TCP 도달 판정 기준
    linear_sample_spacing_m: float = 0.005
    position_tolerance_m: float = 0.006
    orientation_tolerance_deg: float = 3.0
    goal_hold_seconds: float = 0.25
    # 계획된 TCP 이동시간이 끝난 뒤 실제 TCP가 도달할 때까지 추가로 기다리는 시간
    reach_timeout_seconds: float = 20.0
    joint_limit_margin_rad: float = 0.002

    # 물리 PICK 성공 판정 기준
    min_pallet_rise_m: float = 0.010
    pallet_push_tolerance_m: float = 0.015
    pallet_slip_tolerance_m: float = 0.030
