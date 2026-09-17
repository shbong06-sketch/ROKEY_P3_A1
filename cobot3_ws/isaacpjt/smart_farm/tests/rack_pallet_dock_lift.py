"""
팔레트 도킹 → 들어올리기 — 관절 자세를 순서대로 따라가기

    cd cobot3_ws/isaacpjt/smart_farm/tests
    isaac_python rack_pallet_dock_lift.py

관절 자세 표(WAYPOINTS)를 순서대로 따라간다.
표의 값은 미리 Lula IK 로 계산해 적어 둔 숫자라, 실행 중에는 IK 를 쓰지 않는다.
자세를 바꾸고 싶으면 WAYPOINTS 표만 고친다.
  (Physics Inspector 에서 관절값을 찍어 표에 넣어도 된다)

순서 기계는 수업 코드 6_pick_place.py 의 PickPlaceFSM 과 같은 구조다.
  단계에 들어올 때 시작점과 스텝 수를 정하고,
  매 스텝 시작점 → 목표점을 보간한 값을 로봇에 준다.
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

# 수업 코드에 없는 import 두 개
#   ArticulationAction : 관절 각도를 직접 넣을 때 필요하다 (IK 가 돌려주던 action 과 같은 종류)
#   SingleXFormPrim    : 팔레트의 현재 위치를 읽을 때 필요하다
from isaacsim.core.utils.types import ArticulationAction
from isaacsim.core.prims import SingleXFormPrim


# ══════════════════════════════════════════════════════════════
#  경로
# ══════════════════════════════════════════════════════════════
THIS_DIR   = Path(__file__).resolve().parent
SCENE_PATH = str(THIS_DIR.parent / "scenes/rack_pick_test/rack_pick_test.usd")


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


# 아래 관절 자세는 로봇 베이스가 이 위치일 때 계산한 값이다.
# 베이스가 다르면 포크가 엉뚱한 곳으로 가므로 시작하지 않는다.
#   USD 에서 /World/m0609 의 Translate 를 이 값으로 맞춘다
#   (z 0.15 = 15cm 받침대 위. 바닥에서는 팔이 선반 높이까지 닿지 않는다)
ROBOT_BASE_POS = np.array([-0.75, 0.0, 0.15])
BASE_TOL       = 0.005   # m


# ══════════════════════════════════════════════════════════════
#  관절 자세 순서 — 여기만 고치면 동작이 바뀐다 (단위: 도)
# ══════════════════════════════════════════════════════════════
# 포크는 항상 수평, 월드 -x 방향을 향한다.
# 오른쪽 주석은 그 자세에서 link_6 의 월드 위치 (x, y, z).
#   y -0.04   : 팔레트 가운데 받침목 중심. 포크 두 갈래가 받침목 양옆 틈으로 들어간다
#   z 0.822   : 팔레트 틈(0.786 ~ 0.859)의 가운데
#   z 0.873   : 상판 아래에 닿고(0.853) 2cm 더 들어 올린 높이
#   갈래 끝 = link_6 x - 0.245
#
#   이름            joint_1  joint_2  joint_3  joint_4  joint_5  joint_6
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


# ══════════════════════════════════════════════════════════════
#  보간 파라미터
# ══════════════════════════════════════════════════════════════
#   가장 많이 움직이는 관절이 스텝당 JOINT_STEP_DEG 만큼 움직이도록 스텝 수를 정한다
#   (수업 코드의 TCP_SPEED 와 같은 생각. 60Hz 에서 0.2 도/스텝 = 12 도/초)
JOINT_STEP_DEG = 0.2
MIN_STEPS      = 60     # 짧은 구간이 순간이동하지 않도록
MAX_STEPS      = 1200   # 스텝 수 폭주 방지
HOLD_STEPS     = 60     # 자세 도착 후 멈춰 있는 시간
START_WAIT     = 120    # Play 후 팔레트가 선반에 자리 잡을 때까지 기다리는 스텝

LOG_INTERVAL   = 60


# ══════════════════════════════════════════════════════════════
#  보간
# ══════════════════════════════════════════════════════════════
def steps_for(start_deg, goal_deg):
    """가장 크게 움직이는 관절 기준으로 스텝 수를 정한다"""
    biggest = float(np.max(np.abs(np.array(goal_deg) - np.array(start_deg))))
    return int(np.clip(biggest / JOINT_STEP_DEG, MIN_STEPS, MAX_STEPS))


def lerp(start, goal, alpha):
    """시작점에서 목표점까지 선형 보간 (수업 코드와 같은 함수)"""
    return start + alpha * (goal - start)


class JointSequenceFSM:
    """
    WAYPOINTS 를 순서대로 따라가는 상태 기계.

      0 WAIT        팔레트가 자리 잡을 때까지 제자리
      1 ~ N         WAYPOINTS 의 자세로 이동
      N+1 DONE
    """

    def __init__(self, robot):
        self._robot = robot
        self.NAMES = ["WAIT"] + [name for name, _ in WAYPOINTS] + ["DONE"]
        self.DONE_STATE = len(self.NAMES) - 1
        self.reset()

    def reset(self):
        self.state = 0
        self.step = 0
        self.start = None
        self.goal = None
        self.n_steps = START_WAIT

    def current_target(self):
        """이번 스텝의 관절 목표 (도)"""
        if self.start is None:
            self._enter_state()
        alpha = min(1.0, self.step / float(self.n_steps))
        return lerp(self.start, self.goal, alpha)

    def advance(self):
        """한 스텝 진행한다"""
        if self.state >= self.DONE_STATE:
            return
        self.step += 1
        if self.step >= self.n_steps + HOLD_STEPS:
            self._next()

    def _enter_state(self):
        """단계에 처음 들어온 순간 시작점과 스텝 수를 정한다"""
        self.start = get_arm_joints_deg(self._robot)

        if self.state == 0 or self.state >= self.DONE_STATE:
            self.goal = self.start.copy()          # 제자리
            self.n_steps = START_WAIT if self.state == 0 else MIN_STEPS
            return

        name, joints = WAYPOINTS[self.state - 1]
        self.goal = np.array(joints, dtype=float)
        self.n_steps = steps_for(self.start, self.goal)
        print(f"   [{self.state}] {name:9s} goal {vec(self.goal, 1)}  {self.n_steps} steps")

    def _next(self):
        self.state += 1
        self.step = 0
        self.start = None
        if self.state >= self.DONE_STATE:
            print(f"   [{self.DONE_STATE}] DONE")


# ══════════════════════════════════════════════════════════════
#  장면 준비
# ══════════════════════════════════════════════════════════════
def open_scene():
    """맵 USD 를 통째로 연다"""
    omni.usd.get_context().open_stage(SCENE_PATH)
    for _ in range(15):
        simulation_app.update()
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
    """팔 관절의 Drive 를 강화한다 (수업 코드와 같은 함수)"""
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


# ══════════════════════════════════════════════════════════════
#  로봇 읽기·쓰기
# ══════════════════════════════════════════════════════════════
def check_robot_base(robot):
    """로봇 베이스가 ROBOT_BASE_POS 에 있는지 확인한다. 맞으면 True"""
    base_pos, _ = robot.get_world_pose()
    diff = float(np.linalg.norm(base_pos - ROBOT_BASE_POS))
    print(f"   robot base   {vec(base_pos)}   기대값 {vec(ROBOT_BASE_POS)}   차이 {diff * 1000:.1f} mm")
    if diff > BASE_TOL:
        print("   !! 로봇 베이스 위치가 다릅니다. WAYPOINTS 는 ROBOT_BASE_POS 기준 값이라 실행하지 않습니다.")
        print("   !! USD 에서 /World/m0609 Translate 를 ROBOT_BASE_POS 로 맞추고 저장하세요.")
        return False
    return True


def arm_indices(robot):
    return np.array([robot.get_dof_index(name) for name in ARM_JOINTS])


def get_arm_joints_deg(robot):
    """팔 관절 6개의 현재 각도 (도)"""
    return np.rad2deg(robot.get_joint_positions()[arm_indices(robot)])


def apply_arm_joints_deg(robot, joints_deg):
    """팔 관절 6개에 목표 각도 (도) 를 준다"""
    robot.apply_action(ArticulationAction(
        joint_positions=np.deg2rad(joints_deg),
        joint_indices=arm_indices(robot),
    ))


# ══════════════════════════════════════════════════════════════
#  출력
# ══════════════════════════════════════════════════════════════
def section(title):
    print(f"\n{'─' * 66}")
    print(f" {title}")
    print(f"{'─' * 66}")


def vec(v, digits=3):
    """벡터를 고정폭으로 찍는다 (수업 코드와 같은 함수)"""
    return "[" + " ".join(f"{x:+.{digits}f}" for x in v) + "]"


def print_status(robot, pallet, fsm, pallet_start):
    """현재 단계, link_6 위치, 팔레트 위치와 처음 대비 이동량"""
    name = fsm.NAMES[min(fsm.state, fsm.DONE_STATE)]
    ee_pos, _ = robot.end_effector.get_world_pose()
    pallet_pos, _ = pallet.get_world_pose()
    moved = pallet_pos - pallet_start
    print(f"   {name:9s} link_6 {vec(ee_pos)}   pallet {vec(pallet_pos)}"
          f"   이동 {vec(moved)}")


# ══════════════════════════════════════════════════════════════
#  메인
# ══════════════════════════════════════════════════════════════
def main():
    section("SCENE")
    open_scene()
    setup_arm_drives()

    world = World(stage_units_in_meters=1.0)
    robot = register_robot(world)
    pallet = SingleXFormPrim(prim_path=PALLET_PRIM_PATH, name="pallet")
    world.reset()
    robot.initialize()
    base_ok = check_robot_base(robot)

    fsm = JointSequenceFSM(robot)

    section("RUN")
    print("   뷰포트에서 Play 를 누르세요\n")

    was_playing = False
    pallet_start = None
    last_state = -1
    step = 0

    while simulation_app.is_running():
        world.step(render=True)
        time.sleep(0.005)

        is_playing = world.is_playing()

        # Play 를 누른 순간 처음부터 다시
        if is_playing and not was_playing:
            world.reset()
            robot.initialize()
            fsm.reset()
            pallet_start, _ = pallet.get_world_pose()
            last_state = -1
            step = 0
            print(f"   시작 관절   {vec(get_arm_joints_deg(robot), 1)}")
            print(f"   시작 팔레트 {vec(pallet_start)}\n")

        if is_playing and base_ok:
            apply_arm_joints_deg(robot, fsm.current_target())
            fsm.advance()

            # 단계가 바뀔 때마다, 그리고 LOG_INTERVAL 마다 상태를 찍는다
            if fsm.state != last_state or step % LOG_INTERVAL == 0:
                print_status(robot, pallet, fsm, pallet_start)
                last_state = fsm.state
            step += 1

        was_playing = is_playing

    simulation_app.close()


if __name__ == "__main__":
    main()
