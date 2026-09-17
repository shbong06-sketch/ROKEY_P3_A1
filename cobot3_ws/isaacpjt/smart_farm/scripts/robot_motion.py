"""
팔레트 도킹 → 인양 → 인출: 관절 자세 표 재생

수정할 곳:
  ROBOT_BASE_POSITION : 이 자세 표를 계산한 로봇 베이스 위치
  WAYPOINTS           : 단계별 관절 목표, degree 단위
  JOINT_SPEED_DEG_S   : 관절 명령 속도

Play  : 시작 / 일시정지한 위치에서 재개
Pause : 현재 진행 상태 유지
Stop  : 다음 Play에서 처음부터 재시작

ponytail: 관절 보간은 중간 경로의 수평·직선을 보장하지 않습니다.
정밀 삽입에서 문제가 생기면 방향을 고정한 좌표 IK로 바꾸세요.
"""

from isaacsim import SimulationApp

app = SimulationApp({"headless": False})

from pathlib import Path

import numpy as np
import omni.usd
from pxr import UsdPhysics

from isaacsim.core.api import World
from isaacsim.core.prims import SingleRigidPrim
from isaacsim.core.utils.types import ArticulationAction
from isaacsim.robot.manipulators.manipulators import SingleManipulator


# ── 파일·로봇 경로 ───────────────────────────────────────
SCENE_PATH = (
    Path(__file__).resolve().parent.parent
    / "scenes/rack_pick_test/rack_pick_test.usd"
)

ROBOT_PATH = "/World/m0609"
PALLET_PATH = "/World/RecycledWoodPallet_A08_PR_NVD_01"
JOINT_NAMES = [f"joint_{i}" for i in range(1, 7)]


# ── 자세 표의 기준 배치 ──────────────────────────────────
# 이 값은 USD를 이동시키지 않습니다. 현재 배치가 맞는지만 검사합니다.
ROBOT_BASE_POSITION = np.array([-0.75, 0.0, 0.15])
BASE_POSITION_TOL = 0.005
BASE_ROTATION_TOL_DEG = 1.0


# ── 동작 순서: 관절 각도 J1~J6, degree ───────────────────
# 사용자 제공 값입니다. 현 장면에서 물리 동작 검증이 필요합니다.
WAYPOINTS = [
    ("READY",      [190.7,  -12.9,   61.2,  -15.9,   42.7,  101.8]),   # (-1.050, -0.04, 0.930) 팔레트 앞 위쪽에서 대기
    ("APPROACH",   [189.2,  -17.7,   84.4,  -22.4,   25.0,  110.5]),   # (-1.080, -0.04, 0.822) 멀리서 틈 높이로 내려오기 (갈래 끝이 앞면 8cm 앞)
    ("INSERT_1",   [187.2,  -10.2,   79.0,  -19.1,   22.3,  107.8]),   # (-1.139, -0.04, 0.822)
    ("INSERT_2",   [185.9,   -2.3,   72.1,  -16.7,   21.0,  105.6]),   # (-1.198, -0.04, 0.822)
    ("INSERT_3",   [185.0,    5.9,   63.6,  -14.1,   21.1,  103.2]),   # (-1.256, -0.04, 0.822)
    ("INSERT_4",   [184.4,   14.9,   52.7,  -11.3,   22.8,  100.5]),   # (-1.315, -0.04, 0.822) 갈래 끝 -1.56, 무게중심(-1.525) 너머
    ("LIFT",       [184.3,   19.6,   37.3,   -7.9,   33.3,   96.6]),   # (-1.315, -0.04, 0.873) 5cm 올리기
    ("RETRACT_1",  [185.4,    5.2,   55.6,  -10.9,   29.6,   99.6]),   # (-1.228, -0.04, 0.873)
    ("RETRACT_2",  [187.1,   -6.8,   67.4,  -14.1,   30.2,  102.3]),   # (-1.140, -0.04, 0.873)
    ("RETRACT_3",  [190.5,  -17.7,   75.3,  -19.2,   33.9,  106.2]),   # (-1.052, -0.04, 0.873)
    ("RETRACT_4",  [199.5,  -27.5,   80.0,  -30.2,   41.6,  113.5]),   # (-0.965, -0.04, 0.873) 35cm 빼서 선반 밖으로
]



# ── 시간·검사 기준 ───────────────────────────────────────
PHYSICS_DT = 1.0 / 60.0
JOINT_SPEED_DEG_S = 8.0
MIN_MOVE_SECONDS = 1.0
START_WAIT_SECONDS = 2.0

JOINT_REACHED_TOL_DEG = 1.0
JOINT_TRACKING_LIMIT_DEG = 8.0
HOLD_SECONDS = 0.5
REACH_TIMEOUT_SECONDS = 5.0

PALLET_PUSH_TOL = 0.015       # 인양 전에 1.5 cm 이상 움직이면 중단
MIN_PALLET_RISE = 0.010      # 인양 후 실제 상승량 최소 1 cm
PALLET_SLIP_TOL = 0.030      # 인출 중 손목 대비 상대 위치 변화
LOG_INTERVAL_SECONDS = 1.0


def read_joints_deg(robot, indices):
    """실제 관절 각도를 degree로 반환합니다."""
    return np.rad2deg(robot.get_joint_positions()[indices])


def command_joints_deg(robot, indices, target):
    """관절 목표를 radian으로 변환해 적용합니다."""
    robot.apply_action(
        ArticulationAction(
            joint_positions=np.deg2rad(target),
            joint_indices=indices,
        )
    )
    'ddd'


def check_base(robot):
    """자세 표의 기준 위치·방향과 실제 로봇 배치를 비교합니다."""
    position, quaternion = robot.get_world_pose()

    position_error = np.linalg.norm(position - ROBOT_BASE_POSITION)
    q = quaternion / np.linalg.norm(quaternion)
    rotation_error = np.degrees(
        2.0 * np.arccos(np.clip(abs(q[0]), 0.0, 1.0))
    )

    if (
        position_error > BASE_POSITION_TOL
        or rotation_error > BASE_ROTATION_TOL_DEG
    ):
        raise RuntimeError(
            f"베이스 배치 불일치: 현재 {np.round(position, 4)}, "
            f"기준 {ROBOT_BASE_POSITION}, 회전 차이 {rotation_error:.2f}°. "
            "이 자세 표는 회전 없는 기준 배치를 전제로 합니다."
        )


def validate_waypoints(stage):
    """시작 전에 표의 형식·숫자·USD 관절 한계를 검사합니다."""
    if not WAYPOINTS or sum(name == "LIFT" for name, _ in WAYPOINTS) != 1:
        raise ValueError("WAYPOINTS에는 LIFT 단계가 정확히 하나 있어야 합니다.")

    if len({name for name, _ in WAYPOINTS}) != len(WAYPOINTS):
        raise ValueError("단계 이름이 중복되어 있습니다.")

    lower, upper = [], []
    for name in JOINT_NAMES:
        joint = UsdPhysics.RevoluteJoint(
            stage.GetPrimAtPath(f"{ROBOT_PATH}/joints/{name}")
        )
        if not joint:
            raise RuntimeError(f"USD 관절을 찾지 못했습니다: {name}")
        lower.append(joint.GetLowerLimitAttr().Get())
        upper.append(joint.GetUpperLimitAttr().Get())

    for name, values in WAYPOINTS:
        q = np.asarray(values, dtype=float)
        if q.shape != (6,) or not np.all(np.isfinite(q)):
            raise ValueError(f"{name}: 유효한 관절값 6개가 필요합니다.")
        if np.any(q < lower) or np.any(q > upper):
            raise ValueError(f"{name}: USD 관절 한계를 벗어났습니다.")

    if JOINT_SPEED_DEG_S <= 0:
        raise ValueError("관절 속도는 양수여야 합니다.")


class JointSequence:
    """실제 도달을 확인하면서 자세 표를 한 단계씩 실행합니다."""

    def __init__(self, robot, pallet, indices):
        self.robot = robot
        self.pallet = pallet
        self.indices = indices

        self.index = -1
        self.name = "WAIT"
        self.elapsed = 0.0
        self.reached_seconds = 0.0
        self.done = False

        self.start = read_joints_deg(robot, indices)
        self.goal = self.start.copy()
        self.duration = START_WAIT_SECONDS

        self.pallet_start = None
        self.lift_start_z = None
        self.support_offset = None

    def pallet_position(self):
        return np.array(self.pallet.get_world_pose()[0], dtype=float)

    def relative_position(self):
        """방향을固定한다고 가정하지 않고 손목 좌표계에서 팔레트 위치 계산."""
        from isaacsim.core.utils.rotations import quat_to_rot_matrix

        position, quaternion = self.robot.end_effector.get_world_pose()
        rotation = quat_to_rot_matrix(quaternion)
        return rotation.T @ (self.pallet_position() - position)

    def begin_next_stage(self):
        self.index += 1
        self.elapsed = 0.0
        self.reached_seconds = 0.0

        if self.index == len(WAYPOINTS):
            self.name = "DONE"
            self.done = True
            print("[DONE] 자세 도달·인양·상대 위치 검사를 통과했습니다.")
            return

        self.name, values = WAYPOINTS[self.index]
        self.start = read_joints_deg(self.robot, self.indices)
        self.goal = np.asarray(values, dtype=float)

        largest_move = float(np.max(np.abs(self.goal - self.start)))
        self.duration = max(
            MIN_MOVE_SECONDS,
            largest_move / JOINT_SPEED_DEG_S,
        )

        if self.name == "LIFT":
            self.lift_start_z = self.pallet_position()[2]

        print(
            f"[{self.name}] 목표 {self.goal}, "
            f"보간 시간 {self.duration:.1f}초"
        )

    def check_pallet(self):
        if self.pallet_start is None:
            return

        if self.support_offset is not None:
            slip = np.linalg.norm(
                self.relative_position() - self.support_offset
            )
            if slip > PALLET_SLIP_TOL:
                raise RuntimeError(
                    f"{self.name}: 팔레트 상대 위치가 "
                    f"{slip * 1000:.1f} mm 변했습니다."
                )
        elif self.name != "LIFT":
            moved = np.linalg.norm(
                self.pallet_position() - self.pallet_start
            )
            if moved > PALLET_PUSH_TOL:
                raise RuntimeError(
                    f"{self.name}: 인양 전 팔레트가 "
                    f"{moved * 1000:.1f} mm 움직였습니다."
                )

    def update(self, dt):
        self.check_pallet()

        if self.done:
            command_joints_deg(self.robot, self.indices, self.goal)
            return

        if self.name == "WAIT":
            command_joints_deg(self.robot, self.indices, self.goal)
            self.elapsed += dt
            if self.elapsed >= START_WAIT_SECONDS:
                self.pallet_start = self.pallet_position()
                self.begin_next_stage()
            return

        self.elapsed += dt
        alpha = min(1.0, self.elapsed / self.duration)
        target = self.start + alpha * (self.goal - self.start)

        actual = read_joints_deg(self.robot, self.indices)
        tracking_error = np.max(np.abs(target - actual))
        if tracking_error > JOINT_TRACKING_LIMIT_DEG:
            raise RuntimeError(
                f"{self.name}: 관절 추종 오차 {tracking_error:.1f}°. "
                "충돌 또는 구동 설정을 확인하세요."
            )

        command_joints_deg(self.robot, self.indices, target)

        if alpha < 1.0:
            return

        goal_error = np.max(np.abs(self.goal - actual))
        if goal_error <= JOINT_REACHED_TOL_DEG:
            self.reached_seconds += dt
        else:
            self.reached_seconds = 0.0

        if self.reached_seconds >= HOLD_SECONDS:
            if self.name == "LIFT":
                rise = self.pallet_position()[2] - self.lift_start_z
                if rise < MIN_PALLET_RISE:
                    raise RuntimeError(
                        f"인양 실패: 실제 상승량 {rise * 1000:.1f} mm"
                    )
                self.support_offset = self.relative_position()
                print(f"[인양 확인] 실제 상승량 {rise * 1000:.1f} mm")

            self.begin_next_stage()
        elif self.elapsed > self.duration + REACH_TIMEOUT_SECONDS:
            raise RuntimeError(
                f"{self.name}: 도달 시간 초과, 관절 오차 {goal_error:.1f}°"
            )


def main():
    if not SCENE_PATH.is_file():
        raise FileNotFoundError(SCENE_PATH)

    omni.usd.get_context().open_stage(str(SCENE_PATH))
    while omni.usd.get_context().get_stage_loading_status()[2] > 0:
        app.update()

    stage = omni.usd.get_context().get_stage()
    stage.SetEditTarget(stage.GetSessionLayer())

    for path in (ROBOT_PATH, PALLET_PATH, f"{ROBOT_PATH}/link_6"):
        if not stage.GetPrimAtPath(path).IsValid():
            raise RuntimeError(f"Prim이 없습니다: {path}")

    validate_waypoints(stage)

    world = World(
        stage_units_in_meters=1.0,
        physics_dt=PHYSICS_DT,
        rendering_dt=PHYSICS_DT,
        physics_prim_path="/physicsScene",
    )
    robot = world.scene.add(
        SingleManipulator(
            prim_path=ROBOT_PATH,
            name="m0609",
            end_effector_prim_path=f"{ROBOT_PATH}/link_6",
        )
    )
    pallet = world.scene.add(
        SingleRigidPrim(prim_path=PALLET_PATH, name="pallet")
    )

    world.reset()
    world.pause()

    indices = np.array([robot.get_dof_index(n) for n in JOINT_NAMES])
    check_base(robot)
    sequence = JointSequence(robot, pallet, indices)

    needs_reset = False
    failed = False
    log_elapsed = 0.0

    print("Play: 시작/재개 | Pause: 대기 | Stop: 다음 Play에서 재시작")

    while app.is_running():
        if world.is_stopped():
            needs_reset = True
            world.render()
            continue

        if not world.is_playing():
            world.render()
            continue

        # 오류 후에는 단순 Play로 재개하지 않습니다.
        if failed and not needs_reset:
            world.pause()
            continue

        try:
            # Pause에는 초기화하지 않고, Stop 후 Play에만 초기화합니다.
            if needs_reset:
                world.reset()
                check_base(robot)
                sequence = JointSequence(robot, pallet, indices)
                needs_reset = False
                failed = False
                log_elapsed = 0.0

            sequence.update(PHYSICS_DT)
            world.step(render=True)

            log_elapsed += PHYSICS_DT
            if log_elapsed >= LOG_INTERVAL_SECONDS:
                print(
                    f"[{sequence.name}] "
                    f"관절 {np.round(read_joints_deg(robot, indices), 1)}, "
                    f"팔레트 {np.round(sequence.pallet_position(), 3)}"
                )
                log_elapsed = 0.0

        except RuntimeError as error:
            failed = True
            world.pause()
            print(f"[중단] {error}")
            print("원인을 확인하세요. Stop → Play로 처음부터 재시험합니다.")


try:
    main()
finally:
    app.close()