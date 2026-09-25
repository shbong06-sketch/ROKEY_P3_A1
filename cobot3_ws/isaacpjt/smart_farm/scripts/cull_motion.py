"""로봇 base 기준 좌표로 솎아내기 동작을 계획하고 실행한다.

이 파일은 장면을 열지 않는다. ``CullMotion``에 Isaac Sim 객체를 주입하면
Standalone과 통합 Runtime에서 같은 동작 모듈을 재사용할 수 있다.
"""

from dataclasses import dataclass
from math import isfinite
from typing import Callable, Optional, Tuple

import numpy as np


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


@dataclass(frozen=True)
class CullPickConfig:
    """고정 좌표 Pick 단위 동작 설정."""

    approach_clearance: float = 0.18
    lift_clearance: float = 0.22
    pick_z_offset: float = 0.05
    tcp_offset_local: Position = (0.0, 0.0, 0.19671)
    tool_orientation_base: Tuple[float, float, float, float] = (
        0.0,
        1.0,
        0.0,
        0.0,
    )
    tcp_speed_per_frame: float = 0.004
    min_move_frames: int = 60
    max_move_frames: int = 600
    gripper_open_frames: int = 90
    gripper_close_frames: int = 180
    hold_frames: int = 120
    position_tolerance: float = 0.008
    settle_frames: int = 15
    max_settle_frames: int = 600
    min_target_rise: float = 0.05

    def __post_init__(self):
        _position(self.tcp_offset_local, "tcp_offset_local")
        _quaternion(self.tool_orientation_base, "tool_orientation_base")
        if not isfinite(float(self.pick_z_offset)):
            raise ValueError("pick_z_offset은 유한한 값이어야 합니다.")
        for name in (
            "approach_clearance",
            "lift_clearance",
            "tcp_speed_per_frame",
            "position_tolerance",
            "min_target_rise",
        ):
            value = float(getattr(self, name))
            if not isfinite(value) or value <= 0.0:
                raise ValueError(f"{name}은 0보다 큰 유한한 값이어야 합니다.")
        for name in (
            "min_move_frames",
            "max_move_frames",
            "gripper_open_frames",
            "gripper_close_frames",
            "hold_frames",
            "settle_frames",
            "max_settle_frames",
        ):
            if int(getattr(self, name)) <= 0:
                raise ValueError(f"{name}은 0보다 큰 정수여야 합니다.")
        if self.max_move_frames < self.min_move_frames:
            raise ValueError("max_move_frames는 min_move_frames 이상이어야 합니다.")


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


def build_cull_pick_plan(
    detected_position_base: Position,
    config: CullPickConfig | None = None,
) -> Tuple[CullStep, ...]:
    """베이스 좌표 하나로 Pick→Lift→Hold 계획을 만든다."""

    config = config or CullPickConfig()
    pick = np.asarray(
        _position(detected_position_base, "detected_position_base"),
        dtype=float,
    )
    pick[2] += config.pick_z_offset
    approach = pick.copy()
    approach[2] += config.approach_clearance
    lift = pick.copy()
    lift[2] += config.lift_clearance

    return (
        CullStep("OPEN", gripper="open"),
        CullStep("PICK_APPROACH", tuple(approach)),
        CullStep("PICK_DESCEND", tuple(pick)),
        CullStep("GRASP", gripper="close"),
        CullStep("LIFT", tuple(lift)),
        CullStep("HOLD", tuple(lift), gripper="close"),
    )


class CullMotion:
    """좌표 기반 Pick 계획을 한 물리 프레임씩 실행한다.

    ``arm_controller``는 RMPflow처럼 ``forward(position, orientation)``을
    제공해야 한다. 장면 경로와 World 생명주기는 Standalone/Runtime 쪽에서
    관리하므로 이 클래스는 이후 ``standalone_app.py``에서도 그대로 쓸 수 있다.
    """

    def __init__(
        self,
        robot,
        gripper,
        arm_controller,
        get_end_effector_world_pose: Callable,
        get_base_world_pose: Callable,
        get_target_world_pose: Callable | None = None,
        config: CullPickConfig | None = None,
    ):
        self._robot = robot
        self._gripper = gripper
        self._arm_controller = arm_controller
        self._get_end_effector_world_pose = get_end_effector_world_pose
        self._get_base_world_pose = get_base_world_pose
        self._get_target_world_pose = get_target_world_pose
        self.config = config or CullPickConfig()
        self.cancel()

    def start_pick(self, detected_position_base: Position) -> None:
        """베이스 좌표의 물체를 집는 단위 동작을 시작한다."""

        if self.is_running:
            raise RuntimeError(f"솎아내기 동작이 이미 실행 중입니다: {self.current_stage}")

        self._plan = build_cull_pick_plan(detected_position_base, self.config)
        base_position, base_quaternion = self._get_base_world_pose()
        self._base_position = np.asarray(base_position, dtype=float)
        self._base_quaternion = _normalized_quaternion(base_quaternion, "base_quaternion")
        self._world_from_base = _quat_to_matrix(self._base_quaternion)
        self._tool_orientation_world = _quat_multiply(
            self._base_quaternion,
            _normalized_quaternion(
                self.config.tool_orientation_base,
                "tool_orientation_base",
            ),
        )
        self._tool_orientation_world /= np.linalg.norm(self._tool_orientation_world)

        self._index = 0
        self._step = 0
        self._settled = 0
        self._entered = False
        self._done = False
        self._failed = False
        self._error = None
        self._last_tcp_error = None
        self._initial_target_position = self._read_target_position()
        self._final_target_rise = None
        print(
            "[CULL] Pick 시작 base="
            f"{np.round(np.asarray(detected_position_base, dtype=float), 4).tolist()}"
        )

    def update(self) -> None:
        """현재 계획을 물리 한 프레임만큼 진행한다."""

        if not self.is_running:
            return

        try:
            if not self._entered:
                self._enter_step()

            target_tcp = self._interpolated_target()
            flange_target = _tcp_to_flange(
                target_tcp,
                self._tool_orientation_world,
                self.config.tcp_offset_local,
            )
            arm_action = self._arm_controller.forward(
                target_end_effector_position=flange_target,
                target_end_effector_orientation=self._tool_orientation_world,
            )
            self._robot.apply_action(arm_action)
            self._robot.apply_action(
                self._gripper.forward(action=self._step_spec.gripper or self._gripper_state)
            )

            self._step += 1
            if self._step < self._duration_frames:
                return

            if self._step_spec.position_base is not None:
                self._last_tcp_error = float(
                    np.linalg.norm(self._tcp_world_position() - self._goal_world)
                )
                if self._last_tcp_error <= self.config.position_tolerance:
                    self._settled += 1
                else:
                    self._settled = 0
                if self._settled < self.config.settle_frames:
                    if self._step >= self._duration_frames + self.config.max_settle_frames:
                        raise RuntimeError(
                            f"{self.current_stage}: TCP 수렴 실패 "
                            f"(오차 {self._last_tcp_error * 1000.0:.1f} mm)"
                        )
                    return

            self._complete_step()
        except Exception as error:
            self._failed = True
            self._error = str(error)
            raise

    def _enter_step(self) -> None:
        self._step_spec = self._plan[self._index]
        self._start_world = self._tcp_world_position()
        if self._step_spec.position_base is None:
            self._goal_world = self._start_world.copy()
        else:
            target_base = np.asarray(self._step_spec.position_base, dtype=float)
            self._goal_world = (
                self._base_position + self._world_from_base @ target_base
            )

        self._gripper_state = self._step_spec.gripper or self._gripper_state
        if self._step_spec.name == "OPEN":
            self._duration_frames = self.config.gripper_open_frames
        elif self._step_spec.name == "GRASP":
            self._duration_frames = self.config.gripper_close_frames
        elif self._step_spec.name == "HOLD":
            self._duration_frames = self.config.hold_frames
        else:
            distance = float(np.linalg.norm(self._goal_world - self._start_world))
            self._duration_frames = int(
                np.clip(
                    np.ceil(distance / self.config.tcp_speed_per_frame),
                    self.config.min_move_frames,
                    self.config.max_move_frames,
                )
            )

        self._step = 0
        self._settled = 0
        self._entered = True
        print(
            f"[CULL:{self.current_stage}] target_world="
            f"{np.round(self._goal_world, 4).tolist()} "
            f"frames={self._duration_frames} gripper={self._gripper_state}"
        )

    def _interpolated_target(self) -> np.ndarray:
        if self._step_spec.position_base is None:
            return self._goal_world
        alpha = min(1.0, self._step / float(self._duration_frames))
        return self._start_world + alpha * (self._goal_world - self._start_world)

    def _complete_step(self) -> None:
        print(f"[CULL:{self.current_stage}] 완료")
        completed_name = self.current_stage
        self._index += 1
        self._entered = False
        self._step = 0
        self._settled = 0

        if self._index < len(self._plan):
            return

        self._done = True
        if completed_name == "HOLD":
            self._check_physical_pick()
        print("[CULL:DONE] 고정 좌표 Pick→Lift→Hold 완료")

    def _check_physical_pick(self) -> None:
        current = self._read_target_position()
        if self._initial_target_position is None or current is None:
            print("[CULL:검증] 대상 pose getter가 없어 물리 상승 판정을 생략합니다.")
            return
        self._final_target_rise = float(current[2] - self._initial_target_position[2])
        print(f"[CULL:검증] 대상 상승량={self._final_target_rise * 1000.0:.1f} mm")
        if self._final_target_rise < self.config.min_target_rise:
            self._done = False
            self._failed = True
            self._error = (
                f"물리 Pick 실패: 대상 상승량 {self._final_target_rise * 1000.0:.1f} mm, "
                f"기준 {self.config.min_target_rise * 1000.0:.1f} mm"
            )
            raise RuntimeError(self._error)

    def _tcp_world_position(self) -> np.ndarray:
        position, quaternion = self._get_end_effector_world_pose()
        return np.asarray(position, dtype=float) + _quat_to_matrix(
            _normalized_quaternion(quaternion, "end_effector_quaternion")
        ) @ np.asarray(self.config.tcp_offset_local, dtype=float)

    def _read_target_position(self) -> np.ndarray | None:
        if self._get_target_world_pose is None:
            return None
        position, _ = self._get_target_world_pose()
        return np.asarray(position, dtype=float)

    def cancel(self) -> None:
        """진행 중 계획을 버린다. 다음 update에서는 명령을 보내지 않는다."""

        self._plan = ()
        self._index = 0
        self._step = 0
        self._settled = 0
        self._entered = False
        self._done = False
        self._failed = False
        self._error = None
        self._gripper_state = "open"
        self._last_tcp_error = None
        self._initial_target_position = None
        self._final_target_rise = None

    @property
    def is_running(self) -> bool:
        return bool(self._plan) and not self._done and not self._failed

    @property
    def is_done(self) -> bool:
        return self._done

    @property
    def is_failed(self) -> bool:
        return self._failed

    @property
    def current_stage(self) -> str:
        if self._done:
            return "DONE"
        if self._failed:
            return "FAILED"
        if not self._plan:
            return "IDLE"
        return self._plan[self._index].name

    @property
    def error(self) -> str | None:
        return self._error

    @property
    def final_target_rise(self) -> float | None:
        return self._final_target_rise


def _position(value, name: str) -> Position:
    try:
        position = tuple(float(component) for component in value)
    except (TypeError, ValueError) as error:
        raise ValueError(f"{name}은 xyz 숫자 3개여야 합니다.") from error
    if len(position) != 3 or not all(isfinite(component) for component in position):
        raise ValueError(f"{name}은 유한한 xyz 숫자 3개여야 합니다.")
    return position


def _quaternion(value, name: str) -> Tuple[float, float, float, float]:
    try:
        quaternion = tuple(float(component) for component in value)
    except (TypeError, ValueError) as error:
        raise ValueError(f"{name}은 wxyz 숫자 4개여야 합니다.") from error
    if len(quaternion) != 4 or not all(isfinite(component) for component in quaternion):
        raise ValueError(f"{name}은 유한한 wxyz 숫자 4개여야 합니다.")
    if np.linalg.norm(quaternion) <= 1e-12:
        raise ValueError(f"{name}의 크기는 0일 수 없습니다.")
    return quaternion


def _normalized_quaternion(value, name: str) -> np.ndarray:
    quaternion = np.asarray(_quaternion(value, name), dtype=float)
    return quaternion / np.linalg.norm(quaternion)


def _quat_multiply(a, b) -> np.ndarray:
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return np.array(
        [
            aw * bw - ax * bx - ay * by - az * bz,
            aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
        ],
        dtype=float,
    )


def _quat_to_matrix(quaternion) -> np.ndarray:
    w, x, y, z = quaternion
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ],
        dtype=float,
    )


def _tcp_to_flange(tcp_position, orientation, tcp_offset_local) -> np.ndarray:
    return np.asarray(tcp_position, dtype=float) - _quat_to_matrix(
        orientation
    ) @ np.asarray(tcp_offset_local, dtype=float)
