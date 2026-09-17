"""
도킹 자세에서 팔레트 들어 옮기기 — 상태 기계

    cd cobot3_ws/isaacpjt/smart_farm/tests
    isaac_python rack_pallet_lift_carry.py

순서 (도킹 자세에서 출발)
  0 WAIT        팔레트가 선반 위에서 자리 잡을 때까지 대기
  1 INSERT      포크 방향(수평)으로 INSERT_DISTANCE 만큼 더 넣기
  2 LIFT        LIFT_HEIGHT 만큼 들어 올리기
  3 CHECK_LIFT  팔레트가 실제로 올라왔는지 '측정'해서 확인
  4 RETRACT     든 채로 RETRACT_DISTANCE 만큼 뒤로 빼기 (선반 밖으로)
  5 CARRY       joint_1 을 CARRY_JOINT1_DEG 만큼 돌려 옮기기
  6 DONE        든 채로 유지. 내려놓기(PLACE)는 다음 단계

수업 코드 6_pick_place.py 와 같은 것
  import, SingleManipulator, find_prim_path, Lula IK 연결,
  구간 보간(lerp), 상태 기계, Play 순간 world.reset()

수업 코드와 다른 것
  - 목표 좌표를 직접 적지 않는다.
    Play 순간 link_6 의 실제 위치·자세를 읽고, 거기서 '얼마나 움직일지'만 정한다.
    자세를 그대로 두고 평행이동만 하므로 포크 끝도 똑같이 평행이동한다.
    → 포크 끝 TCP 오프셋을 몰라도 된다.
  - 성공을 가정하지 않는다. 들어 올린 뒤 팔레트 높이를 실제로 재고,
    옮기는 동안 팔레트가 포크를 따라오는지 매 프레임 잰다.
  - 문제가 생기면 그 자리에서 멈추고 이유를 출력한다 (자동 재시도 없음).

주의
  - RMPflow 가 아니라 IK 라서 충돌 회피를 하지 않는다. 선반·다리에 닿는지는 화면으로 본다.
  - 직선 보간을 해도 로봇이 완벽한 직선으로 움직인다는 보장은 없다.
  - 팔레트 질량이 USD 에 없으면 PhysX 가 부피로 계산한다. 너무 무거우면 LIFT 에서 실패한다.
  - 거리·속도 값은 시험 시작값이다. 화면을 보며 조정한다.
"""

from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": False})

from pathlib import Path
import time

import numpy as np
import omni.usd
from pxr import Usd, UsdPhysics

from isaacsim.core.api import World
from isaacsim.robot.manipulators.manipulators import SingleManipulator
from isaacsim.robot_motion.motion_generation import (
    LulaKinematicsSolver,
    ArticulationKinematicsSolver,
)

# 수업 코드에 없는 import 두 개
#   ArticulationAction : CARRY 에서 관절 각도를 직접 넣을 때 필요하다
#                        (IK 가 돌려주는 action 과 같은 종류)
#   SingleXFormPrim    : 팔레트의 '현재' 위치를 물리에서 읽을 때 필요하다
from isaacsim.core.utils.types import ArticulationAction
from isaacsim.core.prims import SingleXFormPrim


# ══════════════════════════════════════════════════════════════
#  경로
# ══════════════════════════════════════════════════════════════
#   이 파일 위치  isaacpjt/smart_farm/tests/
THIS_DIR      = Path(__file__).resolve().parent
ISAACPJT_DIR  = THIS_DIR.parents[1]
M0609_DIR     = ISAACPJT_DIR / "M0609"

SCENE_PATH       = str(THIS_DIR.parent / "scenes/rack_pick_test/rack_pick_test.usd")
URDF_PATH        = str(M0609_DIR / "doosan-robot2/urdf/m0609_isaac_sim.urdf")
DESCRIPTION_PATH = str(M0609_DIR / "descriptor/m0609_description.yaml")

for _path in (SCENE_PATH, URDF_PATH, DESCRIPTION_PATH):
    if not Path(_path).is_file():
        simulation_app.close()
        raise FileNotFoundError(f"파일이 없습니다: {_path}")


# ══════════════════════════════════════════════════════════════
#  로봇·장면 설정
# ══════════════════════════════════════════════════════════════
ROBOT_PRIM_PATH  = "/World/m0609"
EE_LINK_NAME     = "link_6"
PALLET_PRIM_PATH = "/World/RecycledWoodPallet_A08_PR_NVD_01"

ARM_JOINTS = ["joint_1", "joint_2", "joint_3",
              "joint_4", "joint_5", "joint_6"]

DRIVE_STIFFNESS = 1e8
DRIVE_DAMPING   = 1e4
DRIVE_MAX_FORCE = 1e8

# 도킹 자세 (Physics Inspector 에서 확인한 값, 도)
# Play 순간 실제 관절이 이 값과 다르면 시작하지 않는다
DOCK_JOINTS_DEG = [183.9, 24.7, -30.1, -3.0, 97.4, 90.4]
DOCK_TOL_DEG    = 2.0


# ══════════════════════════════════════════════════════════════
#  동작 설정 — 시험 시작값. 화면을 보며 조정한다
# ══════════════════════════════════════════════════════════════
WAIT_BEFORE_START_S = 2.0    # 팔레트가 선반에 자리 잡을 시간

INSERT_DISTANCE  = 0.05      # m. 도킹 위치에서 더 넣을 거리 ("조금만 더 앞으로")
LIFT_HEIGHT      = 0.03      # m. 선반에서 들어 올릴 높이
RETRACT_DISTANCE = 0.35      # m. 팔레트가 선반 밖으로 나올 만큼 (팔레트 깊이 약 0.24 + 여유)
CARRY_JOINT1_DEG = 90.0      # 옮길 때 joint_1 회전량. 반대로는 -90.0

INSERT_SPEED     = 0.02      # m/s. 넣을 때는 천천히
LIFT_SPEED       = 0.02      # m/s
RETRACT_SPEED    = 0.04      # m/s
CARRY_SPEED_DEG  = 15.0      # deg/s. 팔레트가 미끄러지지 않게 천천히

MIN_STEPS  = 30              # 짧은 구간이 순간이동하지 않도록
HOLD_S     = 0.5             # 각 구간 끝에서 멈춰 기다리는 시간


# ══════════════════════════════════════════════════════════════
#  안전 확인 기준 — 넘으면 그 자리에서 멈춘다
# ══════════════════════════════════════════════════════════════
FK_CHECK_POS_TOL   = 0.01    # m.  URDF 로 계산한 link_6 과 장면의 link_6 위치 차이
FK_CHECK_ANGLE_DEG = 5.0     # deg. 두 자세의 차이
REACH_POS_TOL      = 0.01    # m.  구간 끝에서 link_6 이 목표에서 벗어난 거리
MAX_JOINT_JUMP_DEG = 5.0     # deg. IK 결과가 한 프레임에 이보다 크게 튀면 멈춘다
IK_FAIL_LIMIT      = 10      # 연속 IK 실패 허용 횟수
PUSH_TOL           = 0.02    # m.  INSERT 중 팔레트가 이만큼 밀리면 '넣지 못하고 밀고 있음'
LIFT_OK_RATIO      = 0.6     # 팔레트가 LIFT_HEIGHT 의 60% 이상 올라와야 '들었다'
FOLLOW_TOL         = 0.03    # m.  옮기는 중 팔레트-link_6 거리가 이만큼 변하면 '미끄러짐'

LOG_INTERVAL_S = 1.0


# ══════════════════════════════════════════════════════════════
#  회전 유틸
# ══════════════════════════════════════════════════════════════
def quat_to_matrix(q):
    """쿼터니언 (w, x, y, z) 를 회전행렬로 바꾼다 (수업 코드와 같은 함수)"""
    w, x, y, z = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w),     2 * (x * z + y * w)],
        [2 * (x * y + z * w),     1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w),     2 * (y * z + x * w),     1 - 2 * (x * x + y * y)],
    ])


def matrix_to_quat(R):
    """
    회전행렬을 쿼터니언 (w, x, y, z) 로 바꾼다. quat_to_matrix 의 반대.
    IK 목표 자세로 쓰기 위해 필요하다.
    """
    trace = R[0, 0] + R[1, 1] + R[2, 2]
    if trace > 0:
        s = 0.5 / np.sqrt(trace + 1.0)
        q = [0.25 / s, (R[2, 1] - R[1, 2]) * s, (R[0, 2] - R[2, 0]) * s, (R[1, 0] - R[0, 1]) * s]
    elif R[0, 0] >= R[1, 1] and R[0, 0] >= R[2, 2]:
        s = 2.0 * np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2])
        q = [(R[2, 1] - R[1, 2]) / s, 0.25 * s, (R[0, 1] + R[1, 0]) / s, (R[0, 2] + R[2, 0]) / s]
    elif R[1, 1] >= R[2, 2]:
        s = 2.0 * np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2])
        q = [(R[0, 2] - R[2, 0]) / s, (R[0, 1] + R[1, 0]) / s, 0.25 * s, (R[1, 2] + R[2, 1]) / s]
    else:
        s = 2.0 * np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1])
        q = [(R[1, 0] - R[0, 1]) / s, (R[0, 2] + R[2, 0]) / s, (R[1, 2] + R[2, 1]) / s, 0.25 * s]
    q = np.array(q)
    return q / np.linalg.norm(q)


def rotation_angle_deg(Ra, Rb):
    """두 회전행렬이 몇 도 차이 나는지"""
    c = (np.trace(Ra.T @ Rb) - 1.0) / 2.0
    return float(np.rad2deg(np.arccos(np.clip(c, -1.0, 1.0))))


def lerp(start, goal, alpha):
    """시작점에서 목표점까지 선형 보간 (수업 코드와 같은 함수)"""
    return start + alpha * (goal - start)


def vec(v, digits=3):
    """벡터를 고정폭으로 찍는다 (수업 코드와 같은 함수)"""
    return "[" + " ".join(f"{x:+.{digits}f}" for x in v) + "]"


def section(title):
    print(f"\n{'─' * 66}")
    print(f" {title}")
    print(f"{'─' * 66}")


# ══════════════════════════════════════════════════════════════
#  장면 준비
# ══════════════════════════════════════════════════════════════
def open_scene():
    """맵 USD 를 통째로 연다"""
    omni.usd.get_context().open_stage(SCENE_PATH)
    for _ in range(15):
        simulation_app.update()

    stage = omni.usd.get_context().get_stage()
    for path in (ROBOT_PRIM_PATH, PALLET_PRIM_PATH):
        if not stage.GetPrimAtPath(path).IsValid():
            raise RuntimeError(f"prim 이 없습니다: {path}  (맵이 바뀌었으면 경로를 고치세요)")
    print(f"   맵 열기      {SCENE_PATH}")


def find_prim_path(root_path, name):
    """USD 계층에서 이름으로 prim 경로를 찾는다 (수업 코드와 같은 함수)"""
    stage = omni.usd.get_context().get_stage()
    root = stage.GetPrimAtPath(root_path)
    if not root.IsValid():
        return None

    for prim in Usd.PrimRange(root):
        if prim.GetName() == name:
            return str(prim.GetPath())
    return None


def setup_arm_drives():
    """팔 6축 Drive 를 강하게 만든다 (수업 코드와 같은 값)"""
    stage = omni.usd.get_context().get_stage()
    count = 0
    for prim in Usd.PrimRange(stage.GetPrimAtPath(ROBOT_PRIM_PATH)):
        if prim.GetName() not in ARM_JOINTS:
            continue
        drive = UsdPhysics.DriveAPI.Get(prim, "angular")
        if drive:
            drive.GetStiffnessAttr().Set(DRIVE_STIFFNESS)
            drive.GetDampingAttr().Set(DRIVE_DAMPING)
            drive.GetMaxForceAttr().Set(DRIVE_MAX_FORCE)
            count += 1
    print(f"   arm drives   {count}")


def read_joint_limits_rad():
    """USD 에 적힌 팔 관절 한계를 라디안으로 읽는다"""
    stage = omni.usd.get_context().get_stage()
    lower, upper = [], []
    for name in ARM_JOINTS:
        prim = stage.GetPrimAtPath(find_prim_path(ROBOT_PRIM_PATH, name))
        joint = UsdPhysics.RevoluteJoint(prim)
        lower.append(np.deg2rad(joint.GetLowerLimitAttr().Get()))
        upper.append(np.deg2rad(joint.GetUpperLimitAttr().Get()))
    return np.array(lower), np.array(upper)


def register_robot(world):
    """로봇을 등록한다. 포크는 그리퍼가 아니므로 gripper 는 넘기지 않는다"""
    ee_path = find_prim_path(ROBOT_PRIM_PATH, EE_LINK_NAME)
    if ee_path is None:
        raise RuntimeError(f"'{EE_LINK_NAME}' not found under {ROBOT_PRIM_PATH}")
    robot = world.scene.add(
        SingleManipulator(
            prim_path=ROBOT_PRIM_PATH,
            name="m0609_robot",
            end_effector_prim_path=ee_path,
        )
    )
    print(f"   EE frame     {ee_path}")
    return robot


def create_ik_solver(robot):
    """
    Lula 계산기를 만들고 로봇과 연결한다 (수업 코드와 같은 방법).

    수업 코드는 base 위치를 상수로 적었지만,
    이 맵은 로봇 위치를 옮겨 가며 시험하므로 장면에서 실제 값을 읽는다.
    """
    lula = LulaKinematicsSolver(
        robot_description_path=DESCRIPTION_PATH,
        urdf_path=URDF_PATH,
    )

    base_pos, base_quat = robot.get_world_pose()
    lula.set_robot_base_pose(
        robot_position=base_pos,
        robot_orientation=base_quat,
    )
    print(f"   robot base   {vec(base_pos)}  quat(w,x,y,z) {vec(base_quat)}")

    return ArticulationKinematicsSolver(
        robot_articulation=robot,
        kinematics_solver=lula,
        end_effector_frame_name=EE_LINK_NAME,
    )


# ══════════════════════════════════════════════════════════════
#  Play 직후 확인
# ══════════════════════════════════════════════════════════════
def arm_indices(robot):
    return np.array([robot.get_dof_index(n) for n in ARM_JOINTS])


def hold_all_arm_joints(robot):
    """지금 자세를 그대로 목표로 준다"""
    idx = arm_indices(robot)
    robot.apply_action(ArticulationAction(
        joint_positions=robot.get_joint_positions()[idx], joint_indices=idx))


def check_dock_pose(robot):
    """실제 관절이 도킹 자세인지 확인한다. 문제가 있으면 이유 문자열을 돌려준다"""
    q_deg = np.rad2deg(robot.get_joint_positions()[arm_indices(robot)])
    diff = np.abs(q_deg - np.array(DOCK_JOINTS_DEG))
    print(f"   현재 관절    {vec(q_deg, 1)} deg")
    if diff.max() > DOCK_TOL_DEG:
        return (f"도킹 자세가 아닙니다. 차이 {vec(diff, 1)} deg. "
                "USD 에 도킹 자세를 저장했는지, DOCK_JOINTS_DEG 가 맞는지 확인하세요.")
    return None


def check_fk_matches_scene(robot, ik_solver):
    """
    IK 가 쓰는 URDF 의 link_6 과 장면(USD)의 link_6 이 같은 곳에 있는지 확인한다.
    다르면 IK 가 엉뚱한 곳으로 팔을 보내므로 시작하지 않는다.
    """
    usd_pos, usd_quat = robot.end_effector.get_world_pose()
    fk_pos, fk_rot = ik_solver.compute_end_effector_pose()
    pos_err = float(np.linalg.norm(fk_pos - usd_pos))
    ang_err = rotation_angle_deg(quat_to_matrix(usd_quat), fk_rot)
    print(f"   link_6 장면  {vec(usd_pos)}")
    print(f"   link_6 URDF  {vec(fk_pos)}   차이 {pos_err * 1000:.1f} mm, {ang_err:.2f} deg")
    if pos_err > FK_CHECK_POS_TOL or ang_err > FK_CHECK_ANGLE_DEG:
        return "URDF 계산과 장면의 link_6 이 다릅니다. URDF/베이스 위치를 확인하세요."
    return None


# ══════════════════════════════════════════════════════════════
#  상태 기계
# ══════════════════════════════════════════════════════════════
class PalletLiftFSM:
    """
    도킹 자세에서 팔레트를 넣고, 들고, 빼고, 옮긴다.

    update() 를 물리 스텝마다 한 번 부른다.
    문제가 생기면 self.failed 에 이유를 적고 더 이상 명령하지 않는다.
    (Drive 는 마지막 목표를 유지하므로 팔은 그 자리에 멈춰 있다)
    """

    NAMES = ["WAIT", "INSERT", "LIFT", "CHECK_LIFT", "RETRACT", "CARRY", "DONE"]
    DONE_STATE = 6

    def __init__(self, robot, ik_solver, pallet, dt, joint_lower, joint_upper):
        self._robot = robot
        self._ik = ik_solver
        self._pallet = pallet
        self._dt = dt
        self._lower = joint_lower
        self._upper = joint_upper
        self.reset()

    def reset(self):
        self.state = 0
        self.step = 0
        self.failed = None
        self.ik_fail_count = 0
        self.n_steps = MIN_STEPS
        self.quat0 = None              # 유지할 link_6 자세
        self.waypoints = None          # [p0 도킹, p1 삽입, p2 인양, p3 인출]
        self.pallet_start = None
        self.pallet_z_before_lift = None
        self.follow_dist = None
        self.q_carry_start = None
        self.q_carry_goal = None

    # ── 공통 ──────────────────────────────────────────
    @property
    def done(self):
        return self.state >= self.DONE_STATE

    def fail(self, reason):
        if self.failed is None:
            self.failed = reason
            print(f"\n   !! 중단 [{self.NAMES[self.state]}] {reason}")
            print("   !! 팔은 마지막 목표에 멈춰 있습니다. Stop 후 Play 로 다시 시험하세요.\n")

    def _next(self):
        self.state += 1
        self.step = 0
        self.ik_fail_count = 0
        if self.done:
            print("\n   [DONE] 팔레트를 든 채 유지 중. 내려놓기(PLACE)는 다음 단계입니다.\n")

    def _pallet_pos(self):
        return np.array(self._pallet.get_world_pose()[0])

    def _ee_pos(self):
        return np.array(self._robot.end_effector.get_world_pose()[0])

    def _apply_ik(self, target):
        """target 으로 IK 를 풀어 적용한다. 계속 진행해도 되면 True"""
        action, success = self._ik.compute_inverse_kinematics(
            target_position=target,
            target_orientation=self.quat0,
        )
        if not success:
            self.ik_fail_count += 1
            if self.ik_fail_count > IK_FAIL_LIMIT:
                self.fail(f"IK 가 {IK_FAIL_LIMIT}번 연속 실패했습니다. 목표 {vec(target)} 가 닿지 않는 곳일 수 있습니다.")
                return False
            return True                 # 한두 프레임 실패는 건너뛴다

        self.ik_fail_count = 0

        # 해가 갑자기 튀면(예: joint_1 이 한 바퀴 반대로) 적용하지 않고 멈춘다
        idx = action.joint_indices if action.joint_indices is not None else arm_indices(self._robot)
        q_now = self._robot.get_joint_positions()[idx]
        jump_deg = float(np.rad2deg(np.max(np.abs(action.joint_positions - q_now))))
        if jump_deg > MAX_JOINT_JUMP_DEG:
            self.fail(f"IK 해가 한 프레임에 {jump_deg:.1f} deg 튀었습니다. 적용하지 않았습니다.")
            return False

        self._robot.apply_action(action)
        return True

    def _move(self, start, goal, speed):
        """
        start → goal 직선 보간으로 link_6 을 옮긴다. 자세는 quat0 유지.
        끝나면 HOLD_S 동안 기다린 뒤 실제 도달 여부를 확인한다.
        구간이 끝났으면 True.
        """
        hold_steps = int(HOLD_S / self._dt)
        if self.step == 0:
            dist = float(np.linalg.norm(goal - start))
            self.n_steps = max(MIN_STEPS, int(dist / (speed * self._dt)))
            print(f"   [{self.state}] {self.NAMES[self.state]:10s} goal {vec(goal)}"
                  f"  {dist:.3f} m  {self.n_steps} steps")

        alpha = min(1.0, (self.step + 1) / float(self.n_steps))
        if not self._apply_ik(lerp(start, goal, alpha)):
            return False

        if self.step < self.n_steps + hold_steps:
            return False

        # 구간 끝: URDF 로 계산한 실제 link_6 위치가 목표에 왔는지
        fk_pos, _ = self._ik.compute_end_effector_pose(position_only=True)
        err = float(np.linalg.norm(fk_pos - goal))
        if err > REACH_POS_TOL:
            self.fail(f"목표에 {err * 1000:.1f} mm 못 미쳤습니다. 무언가에 막혔거나 팔레트가 무거울 수 있습니다.")
            return False
        return True

    # ── 단계별 ────────────────────────────────────────
    def _wait(self):
        """팔레트가 자리 잡을 때까지 기다린 뒤, 이동 계획을 세운다"""
        if self.step * self._dt < WAIT_BEFORE_START_S:
            return False

        self.pallet_start = self._pallet_pos()

        # 출발점과 자세는 URDF 계산값을 쓴다 (IK 와 같은 기준)
        p0, rot0 = self._ik.compute_end_effector_pose()
        self.quat0 = matrix_to_quat(rot0)

        # 포크 방향 = link_6 로컬 +Z. 높이 성분을 없애 수평으로만 넣고 뺀다
        fork_dir = rot0[:, 2].copy()
        fork_dir[2] = 0.0
        if np.linalg.norm(fork_dir) < 0.5:
            self.fail(f"포크가 수평이 아닙니다. 로컬 +Z 방향 {vec(rot0[:, 2])}")
            return False
        fork_dir = fork_dir / np.linalg.norm(fork_dir)

        p1 = p0 + fork_dir * INSERT_DISTANCE
        p2 = p1 + np.array([0.0, 0.0, LIFT_HEIGHT])
        p3 = p2 - fork_dir * RETRACT_DISTANCE
        self.waypoints = [p0, p1, p2, p3]

        section("PLAN")
        print(f"   fork dir     {vec(fork_dir)}   (link_6 로컬 +Z 를 수평으로)")
        print(f"   p0 dock      {vec(p0)}")
        print(f"   p1 insert    {vec(p1)}")
        print(f"   p2 lift      {vec(p2)}")
        print(f"   p3 retract   {vec(p3)}")
        print(f"   pallet       {vec(self.pallet_start)}")
        print()
        return True

    def _insert(self):
        # 넣는 동안 팔레트가 밀리면, 구멍에 들어가지 못하고 밀고 있는 것
        pushed = float(np.linalg.norm(self._pallet_pos() - self.pallet_start))
        if pushed > PUSH_TOL:
            self.fail(f"넣는 중 팔레트가 {pushed * 100:.1f} cm 밀렸습니다. 포크 높이·방향을 확인하세요.")
            return False
        p0, p1 = self.waypoints[0], self.waypoints[1]
        return self._move(p0, p1, INSERT_SPEED)

    def _lift(self):
        if self.step == 0:
            self.pallet_z_before_lift = float(self._pallet_pos()[2])
        p1, p2 = self.waypoints[1], self.waypoints[2]
        return self._move(p1, p2, LIFT_SPEED)

    def _check_lift(self):
        """팔레트가 실제로 올라왔는지 잰다. 모의로 성공 처리하지 않는다"""
        self._apply_ik(self.waypoints[2])          # 자세 유지
        rise = float(self._pallet_pos()[2]) - self.pallet_z_before_lift
        print(f"   [{self.state}] CHECK_LIFT  팔레트 상승 {rise * 1000:.1f} mm"
              f"  (기준 {LIFT_HEIGHT * LIFT_OK_RATIO * 1000:.1f} mm 이상)")
        if rise < LIFT_HEIGHT * LIFT_OK_RATIO:
            self.fail("팔레트가 충분히 올라오지 않았습니다. 포크가 팔레트 아래에 들어가지 않았을 수 있습니다.")
            return False
        self.follow_dist = float(np.linalg.norm(self._pallet_pos() - self._ee_pos()))
        return True

    def _check_following(self):
        """옮기는 동안 팔레트가 포크를 따라오는지 잰다"""
        dist = float(np.linalg.norm(self._pallet_pos() - self._ee_pos()))
        change = abs(dist - self.follow_dist)
        if change > FOLLOW_TOL:
            self.fail(f"팔레트와 포크 거리가 {change * 100:.1f} cm 변했습니다. 미끄러졌거나 떨어졌습니다.")
            return False
        return True

    def _retract(self):
        if not self._check_following():
            return False
        p2, p3 = self.waypoints[2], self.waypoints[3]
        return self._move(p2, p3, RETRACT_SPEED)

    def _carry(self):
        """joint_1 만 돌려 옮긴다. 나머지 관절은 그대로"""
        if not self._check_following():
            return False

        idx = arm_indices(self._robot)
        hold_steps = int(HOLD_S / self._dt)

        if self.step == 0:
            self.q_carry_start = self._robot.get_joint_positions()[idx].copy()
            self.q_carry_goal = self.q_carry_start.copy()
            self.q_carry_goal[0] += np.deg2rad(CARRY_JOINT1_DEG)
            if not (self._lower[0] <= self.q_carry_goal[0] <= self._upper[0]):
                self.fail(f"joint_1 목표 {np.rad2deg(self.q_carry_goal[0]):.1f} deg 가 한계 밖입니다.")
                return False
            self.n_steps = max(MIN_STEPS, int(abs(CARRY_JOINT1_DEG) / (CARRY_SPEED_DEG * self._dt)))
            print(f"   [{self.state}] CARRY      joint_1 {np.rad2deg(self.q_carry_start[0]):+.1f}"
                  f" -> {np.rad2deg(self.q_carry_goal[0]):+.1f} deg  {self.n_steps} steps")

        alpha = min(1.0, (self.step + 1) / float(self.n_steps))
        self._robot.apply_action(ArticulationAction(
            joint_positions=lerp(self.q_carry_start, self.q_carry_goal, alpha),
            joint_indices=idx,
        ))
        return self.step >= self.n_steps + hold_steps

    # ── 매 스텝 ───────────────────────────────────────
    def update(self):
        if self.done or self.failed:
            return

        name = self.NAMES[self.state]
        if name == "WAIT":
            finished = self._wait()
        elif name == "INSERT":
            finished = self._insert()
        elif name == "LIFT":
            finished = self._lift()
        elif name == "CHECK_LIFT":
            finished = self._check_lift()
        elif name == "RETRACT":
            finished = self._retract()
        elif name == "CARRY":
            finished = self._carry()
        else:
            finished = False

        if self.failed:
            return
        self.step += 1
        if finished:
            self._next()


def print_status(fsm, robot, pallet):
    name = fsm.NAMES[min(fsm.state, fsm.DONE_STATE)]
    ee = robot.end_effector.get_world_pose()[0]
    pal = pallet.get_world_pose()[0]
    print(f"   {name:10s} link_6 {vec(ee)}   pallet {vec(pal)}")


# ══════════════════════════════════════════════════════════════
#  메인
# ══════════════════════════════════════════════════════════════
def main():
    section("SCENE")
    open_scene()
    setup_arm_drives()
    joint_lower, joint_upper = read_joint_limits_rad()

    world = World(stage_units_in_meters=1.0)
    robot = register_robot(world)
    pallet = SingleXFormPrim(prim_path=PALLET_PRIM_PATH, name="pallet")
    world.reset()
    robot.initialize()

    section("SOLVER")
    ik_solver = create_ik_solver(robot)

    fsm = PalletLiftFSM(robot, ik_solver, pallet, world.get_physics_dt(),
                        joint_lower, joint_upper)
    log_steps = max(1, int(LOG_INTERVAL_S / world.get_physics_dt()))

    section("RUN")
    print("   뷰포트에서 Play 를 누르세요\n")

    was_playing = False
    step = 0

    while simulation_app.is_running():
        world.step(render=True)
        time.sleep(0.005)

        is_playing = world.is_playing()

        # Play 를 누른 순간: 처음부터, 시작 조건 확인
        if is_playing and not was_playing:
            world.reset()
            robot.initialize()
            hold_all_arm_joints(robot)
            fsm.reset()
            step = 0

            section("START CHECK")
            reason = check_dock_pose(robot) or check_fk_matches_scene(robot, ik_solver)
            if reason:
                fsm.fail(reason)
            else:
                print("   OK — 팔레트가 자리 잡기를 기다린 뒤 시작합니다")

        if is_playing:
            fsm.update()
            if step % log_steps == 0 and not fsm.failed:
                print_status(fsm, robot, pallet)
            step += 1

        was_playing = is_playing

    simulation_app.close()


if __name__ == "__main__":
    main()
