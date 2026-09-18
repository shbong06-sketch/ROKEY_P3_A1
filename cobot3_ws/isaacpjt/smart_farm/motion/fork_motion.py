"""포크 TCP의 Cartesian 경로를 IK로 실행하는 단일 PICK 상태 기계.

이 파일은 Isaac Sim 앱, USD 로딩, World.reset, 프레임 루프를 소유하지 않는다.
"""

from dataclasses import dataclass
from enum import Enum
import math

import numpy as np

from .motion_config import MotionConfig
from .target_builder import Pose, SlotPose, build_pick_targets


class MotionState(str, Enum):
    """외부 실행기가 조회할 수 있는 PICK 상태."""

    IDLE = "IDLE"
    APPROACH = "APPROACH"
    INSERT = "INSERT"
    LIFT = "LIFT"
    RETRACT = "RETRACT"
    SUCCEEDED = "SUCCEEDED"
    FAILED = "FAILED"


@dataclass(frozen=True)
class MotionResult:
    state: MotionState
    reason: str | None = None


def quaternion_to_matrix(quaternion: np.ndarray) -> np.ndarray:
    """Isaac Sim wxyz quaternion을 3x3 회전행렬로 바꾼다."""

    w, x, y, z = np.asarray(quaternion, dtype=float)
    norm = math.sqrt(w * w + x * x + y * y + z * z)
    w, x, y, z = w / norm, x / norm, y / norm, z / norm
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ]
    )


def orientation_error_deg(actual: np.ndarray, target: np.ndarray) -> float:
    """q와 -q가 같은 회전임을 고려한 회전 오차를 반환한다."""

    dot = abs(float(np.dot(actual / np.linalg.norm(actual), target / np.linalg.norm(target))))
    return math.degrees(2.0 * math.acos(np.clip(dot, 0.0, 1.0)))


class LulaTool0IK:
    """월드 TCP 목표를 M0609 Lula의 `tool0` 목표로 바꾸는 IK 어댑터.

    기존 M0609 descriptor는 tool0까지만 정의한다. 따라서 포크 TCP offset을
    먼저 제거하고, 매 PICK 시작 시 읽은 실제 base pose로 월드 목표를 base
    좌표로 바꾼 뒤 IK를 호출한다.
    """

    def __init__(self, robot, description_path, urdf_path, tcp_offset_m):
        # 모션 모듈 import 자체에는 Isaac Sim 런타임이 필요 없도록 지연 import.
        from isaacsim.robot_motion.motion_generation import LulaKinematicsSolver

        self.tcp_offset_m = np.asarray(tcp_offset_m, dtype=float)
        self.solver = LulaKinematicsSolver(
            robot_description_path=str(description_path),
            urdf_path=str(urdf_path),
        )

    def solve(self, world_tcp_target: Pose, base_pose, _seed: np.ndarray) -> np.ndarray | None:
        """월드 TCP 목표의 IK 관절 해를 반환하고, 해가 없으면 None을 반환한다."""

        base_position, base_orientation = base_pose
        base_rotation = quaternion_to_matrix(base_orientation)

        # TCP = tool0 + R_tcp * offset 이므로 tool0 목표는 offset을 빼서 구한다.
        tcp_rotation = quaternion_to_matrix(world_tcp_target.orientation)
        tool0_world = world_tcp_target.position - tcp_rotation @ self.tcp_offset_m

        # Lula kinematics는 현재 로봇 root/base 좌표계의 target을 기대한다.
        tool0_local = base_rotation.T @ (tool0_world - np.asarray(base_position))
        inverse_base = np.array(
            [base_orientation[0], -base_orientation[1], -base_orientation[2], -base_orientation[3]]
        )
        local_orientation = self._multiply(inverse_base, world_tcp_target.orientation)

        action, success = self.solver.compute_inverse_kinematics(
            "tool0", tool0_local, local_orientation
        )
        if not success or action.joint_positions is None:
            return None
        return np.asarray(action.joint_positions, dtype=float)

    @staticmethod
    def _multiply(left: np.ndarray, right: np.ndarray) -> np.ndarray:
        """wxyz Hamilton product: base 역회전 × 월드 tool 회전."""

        w, x, y, z = left
        W, X, Y, Z = right
        return np.array(
            [
                w * W - x * X - y * Y - z * Z,
                w * X + x * W + y * Z - z * Y,
                w * Y - x * Z + y * W + z * X,
                w * Z + x * Y - y * X + z * W,
            ]
        )


class ForkMotion:
    """단일 슬롯 PICK 상태 기계.

    `tcp_pose_reader`는 실제 TCP의 world (position, quaternion)을 반환해야 한다.
    IK 자체는 여기서 만들지 않아 다른 solver 또는 시험용 fake solver도 주입할 수 있다.
    """

    def __init__(
        self,
        robot,
        pallet,
        tcp_pose_reader,
        ik_solver,
        joint_indices,
        joint_limits_rad,
        config: MotionConfig = MotionConfig(),
    ):
        self.robot = robot
        self.pallet = pallet
        self.tcp_pose_reader = tcp_pose_reader
        self.ik_solver = ik_solver
        self.joint_indices = np.asarray(joint_indices)
        self.lower_limits, self.upper_limits = (np.asarray(limit) for limit in joint_limits_rad)
        self.config = config
        self.reset()

    @property
    def status(self) -> MotionResult:
        return MotionResult(self.state, self.failure_reason)

    def reset(self) -> None:
        """완료 또는 실패한 PICK을 다음 PICK 전에 초기 상태로 되돌린다."""

        self.state = MotionState.IDLE
        self.failure_reason = None
        self.targets = None
        self.base_pose_at_start = None
        self._start_tcp = None
        self._goal_tcp = None
        self._elapsed = 0.0
        self._hold_elapsed = 0.0
        self._pallet_start = None
        self._lift_start_z = None
        self._lift_start_xy = None
        self._support_offset = None

    def start_pick(self, slot: SlotPose) -> None:
        """현재 실제 base pose를 고정해 한 번의 PICK을 시작한다."""

        if self.state not in (MotionState.IDLE, MotionState.SUCCEEDED, MotionState.FAILED):
            raise RuntimeError("진행 중인 PICK이 있어 새 작업을 시작할 수 없습니다.")

        self.reset()
        self.targets = build_pick_targets(slot, self.config)
        # 승강판이 이동했더라도 현재 base pose를 기준으로 IK를 계산한다.
        self.base_pose_at_start = self.robot.get_world_pose()
        self._pallet_start = self._pallet_position()
        self._begin_stage(MotionState.APPROACH, self.targets.approach)

    def update(self, dt: float) -> MotionResult:
        """프레임마다 호출한다. 앱/World의 step은 호출자가 수행한다."""

        if self.state in (MotionState.IDLE, MotionState.SUCCEEDED, MotionState.FAILED):
            return self.status

        self._elapsed += max(0.0, dt)
        self._check_pallet()
        if self.state is MotionState.FAILED:
            return self.status

        target_tcp, path_finished = self._sample_cartesian_target()
        joint_solution = self.ik_solver.solve(
            target_tcp,
            self.base_pose_at_start,
            self.robot.get_joint_positions()[self.joint_indices],
        )
        if joint_solution is None:
            return self._fail(f"{self.state}: IK 해를 찾지 못했습니다.")
        if not self._within_joint_limits(joint_solution):
            return self._fail(f"{self.state}: IK 해가 관절 한계를 벗어났습니다.")

        # Isaac import도 실행 시점까지 늦춰, 모듈 import에 부작용을 만들지 않는다.
        from isaacsim.core.utils.types import ArticulationAction

        self.robot.apply_action(
            ArticulationAction(joint_positions=joint_solution, joint_indices=self.joint_indices)
        )

        if self._elapsed > self.config.reach_timeout_seconds:
            return self._fail(f"{self.state}: TCP 도달 시간 초과")

        if path_finished and self._tcp_reached_goal():
            self._hold_elapsed += dt
        elif path_finished:
            self._hold_elapsed = 0.0

        if self._hold_elapsed >= self.config.goal_hold_seconds:
            self._advance_stage()
        return self.status

    def _begin_stage(self, state: MotionState, goal_tcp: Pose) -> None:
        """새 단계의 실제 TCP 시작 pose를 읽고 시간·도달 판정을 초기화한다."""

        position, orientation = self.tcp_pose_reader()
        self.state = state
        self._start_tcp = Pose(np.asarray(position, dtype=float), np.asarray(orientation, dtype=float))
        self._goal_tcp = goal_tcp
        self._elapsed = 0.0
        self._hold_elapsed = 0.0

        if state is MotionState.LIFT:
            pallet = self._pallet_position()
            self._lift_start_z = pallet[2]
            self._lift_start_xy = pallet[:2].copy()

    def _sample_cartesian_target(self) -> tuple[Pose, bool]:
        """현재 단계의 직선 TCP 경로 위 목표를 반환한다.

        orientation은 단계 내내 고정한다. INSERT/RETRACT에서 관절값이 아니라
        TCP 위치를 보간하므로 포크 높이와 방향을 유지한 직선 목표가 된다.
        """

        distance = np.linalg.norm(self._goal_tcp.position - self._start_tcp.position)
        duration = max(distance / self.config.tcp_speed_m_s, 1e-6)
        alpha = min(1.0, self._elapsed / duration)
        position = self._start_tcp.position + alpha * (
            self._goal_tcp.position - self._start_tcp.position
        )
        return Pose(position, self._goal_tcp.orientation), alpha >= 1.0

    def _within_joint_limits(self, joints: np.ndarray) -> bool:
        """IK 결과 차원과 여유를 포함한 관절 범위를 확인한다."""

        margin = self.config.joint_limit_margin_rad
        return (
            joints.shape == self.lower_limits.shape
            and np.all(joints >= self.lower_limits + margin)
            and np.all(joints <= self.upper_limits - margin)
        )

    def _tcp_reached_goal(self) -> bool:
        """관절값이 아닌 실제 TCP pose로 단계 도달을 판정한다."""

        position, orientation = self.tcp_pose_reader()
        position_error = np.linalg.norm(np.asarray(position) - self._goal_tcp.position)
        rotation_error = orientation_error_deg(np.asarray(orientation), self._goal_tcp.orientation)
        return (
            position_error <= self.config.position_tolerance_m
            and rotation_error <= self.config.orientation_tolerance_deg
        )

    def _check_pallet(self) -> None:
        """관절 도달과 별개로 팔레트가 실제로 들리고 유지되는지 검사한다."""

        pallet = self._pallet_position()

        # APPROACH/INSERT 중 팔레트가 밀리면 포크 삽입이 이미 실패한 것으로 본다.
        if self._support_offset is None and self.state not in (MotionState.LIFT, MotionState.RETRACT):
            if np.linalg.norm(pallet - self._pallet_start) > self.config.pallet_push_tolerance_m:
                self._fail("인양 전 팔레트가 밀렸습니다. 포크 채널/충돌을 확인하세요.")

        # 기존 LIFT에서 발생할 수 있던 수평 밀림을 별도로 감시한다.
        if self.state is MotionState.LIFT:
            if np.linalg.norm(pallet[:2] - self._lift_start_xy) > self.config.pallet_push_tolerance_m:
                self._fail("LIFT 중 팔레트가 옆으로 밀렸습니다.")

        # 인양 확인 뒤에는 팔레트와 TCP의 상대 이동으로 슬립·이탈을 감시한다.
        if self._support_offset is not None:
            tcp_position = np.asarray(self.tcp_pose_reader()[0])
            slip = np.linalg.norm((pallet - tcp_position) - self._support_offset)
            if slip > self.config.pallet_slip_tolerance_m:
                self._fail("RETRACT 중 팔레트가 포크에서 미끄러지거나 이탈했습니다.")

    def _advance_stage(self) -> None:
        """단계별 물리 조건을 확인한 뒤 다음 TCP 목표로 전환한다."""

        if self.state is MotionState.APPROACH:
            self._begin_stage(MotionState.INSERT, self.targets.insert)
        elif self.state is MotionState.INSERT:
            self._begin_stage(MotionState.LIFT, self.targets.lift)
        elif self.state is MotionState.LIFT:
            rise = self._pallet_position()[2] - self._lift_start_z
            if rise < self.config.min_pallet_rise_m:
                self._fail(f"LIFT 실패: 팔레트 상승량 {rise * 1000:.1f} mm")
                return
            tcp_position = np.asarray(self.tcp_pose_reader()[0])
            self._support_offset = self._pallet_position() - tcp_position
            self._begin_stage(MotionState.RETRACT, self.targets.retract)
        elif self.state is MotionState.RETRACT:
            self.state = MotionState.SUCCEEDED

    def _pallet_position(self) -> np.ndarray:
        return np.asarray(self.pallet.get_world_pose()[0], dtype=float)

    def _fail(self, reason: str) -> MotionResult:
        self.state = MotionState.FAILED
        self.failure_reason = reason
        return self.status
