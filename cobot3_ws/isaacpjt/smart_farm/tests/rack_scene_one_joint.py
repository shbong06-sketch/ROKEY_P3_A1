"""
rack_pick_test 맵에서 M0609 관절 하나만 돌려 보기

    cd cobot3_ws/isaacpjt/smart_farm/tests
    isaac_python rack_scene_one_joint.py

one_joint_90.py 와 다른 점
  1) 로봇 USD 를 붙이는 대신 맵(USD) 전체를 연다
     → /World 밖에 있는 /physicsScene, /Environment(조명)도 함께 들어온다
  2) 목표가 절대 각도가 아니라 "지금 각도에서 MOVE_DEG 만큼"
     → 이 맵은 시작 자세가 이미 꺾여 있어서 (USD 에 저장된 Drive 목표값)
       절대 90도로 보내면 팔이 크게 돌 수 있다

IK, 포크 작업, 팔레트는 건드리지 않는다.
import 와 로봇 등록(SingleManipulator)은 수업 코드 6_pick_place.py 를 따른다.
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

# 수업 코드에 없는 import 는 이것 하나다.
# IK 솔버가 돌려주던 action 이 바로 이 ArticulationAction 이다.
# IK 없이 관절 각도를 직접 넣으려면 이 상자를 우리가 만들어야 한다.
from isaacsim.core.utils.types import ArticulationAction


# ══════════════════════════════════════════════════════════════
#  경로
# ══════════════════════════════════════════════════════════════
#   이 파일 위치  isaacpjt/smart_farm/tests/
#   parent        isaacpjt/smart_farm/
THIS_DIR   = Path(__file__).resolve().parent
SCENE_PATH = str(THIS_DIR.parent / "scenes/rack_pick_test/rack_pick_test.usd")

if not Path(SCENE_PATH).is_file():
    simulation_app.close()
    raise FileNotFoundError(f"맵 USD 파일이 없습니다: {SCENE_PATH}")

ROBOT_PRIM_PATH = "/World/m0609"
EE_LINK_NAME    = "link_6"
ARM_JOINTS = ["joint_1", "joint_2", "joint_3",
              "joint_4", "joint_5", "joint_6"]


# ══════════════════════════════════════════════════════════════
#  시험 설정 — 여기만 바꾸면 된다
# ══════════════════════════════════════════════════════════════
JOINT_NAME = "joint_1"     # 바닥에 수직인 축. 팔이 바닥으로 떨어지지 않는다
MOVE_DEG   = 90.0          # 지금 각도에서 이만큼 더 돌린다. 반대로 돌리려면 -90.0

SPEED_DEG_PER_S = 30.0     # 초당 이만큼만 명령을 옮긴다
REACHED_TOL_DEG = 1.0      # 실제 각도가 목표와 이만큼 가까우면 도달로 본다

# 수업 코드와 같은 값. 메모리 안의 장면만 바뀌고 USD 파일은 그대로다
DRIVE_STIFFNESS = 1e8
DRIVE_DAMPING   = 1e4
DRIVE_MAX_FORCE = 1e8

LOG_INTERVAL = 60          # 60 스텝 = 약 1초 (이 맵은 60Hz)


# ══════════════════════════════════════════════════════════════
#  준비 함수
# ══════════════════════════════════════════════════════════════
def open_scene():
    """맵 USD 를 통째로 열고, 다 읽힐 때까지 기다린다"""
    omni.usd.get_context().open_stage(SCENE_PATH)
    for _ in range(15):
        simulation_app.update()

    stage = omni.usd.get_context().get_stage()
    if not stage.GetPrimAtPath(ROBOT_PRIM_PATH).IsValid():
        raise RuntimeError(f"맵을 열었지만 로봇 prim 이 없습니다: {ROBOT_PRIM_PATH}")
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


def find_joint_prim(joint_name):
    """로봇 아래에서 이름으로 관절 prim 을 찾는다. 없으면 None"""
    stage = omni.usd.get_context().get_stage()
    robot_prim = stage.GetPrimAtPath(ROBOT_PRIM_PATH)
    if not robot_prim.IsValid():
        raise RuntimeError(f"로봇 prim 이 없습니다: {ROBOT_PRIM_PATH}")
    for prim in Usd.PrimRange(robot_prim):
        if prim.GetName() == joint_name:
            return prim
    return None


def read_joint_limit_deg(joint_name):
    """USD 에 적힌 관절 한계(도)를 읽는다"""
    prim = find_joint_prim(joint_name)
    if prim is None:
        raise RuntimeError(f"'{joint_name}' 관절을 {ROBOT_PRIM_PATH} 아래에서 찾지 못했습니다.")
    joint = UsdPhysics.RevoluteJoint(prim)
    return joint.GetLowerLimitAttr().Get(), joint.GetUpperLimitAttr().Get()


def setup_arm_drives():
    """팔 6축 Drive 를 강하게 만든다"""
    for name in ARM_JOINTS:
        drive = UsdPhysics.DriveAPI.Get(find_joint_prim(name), "angular")
        drive.GetStiffnessAttr().Set(DRIVE_STIFFNESS)
        drive.GetDampingAttr().Set(DRIVE_DAMPING)
        drive.GetMaxForceAttr().Set(DRIVE_MAX_FORCE)
    print(f"   arm drives   {len(ARM_JOINTS)}개 강화")


# ══════════════════════════════════════════════════════════════
#  제어 함수
# ══════════════════════════════════════════════════════════════
def hold_all_arm_joints(robot):
    """지금 자세를 그대로 목표로 준다. 다른 관절이 옛 목표로 끌려가지 않게 한다"""
    indices = np.array([robot.get_dof_index(n) for n in ARM_JOINTS])
    current = robot.get_joint_positions()[indices]
    robot.apply_action(ArticulationAction(joint_positions=current, joint_indices=indices))


def move_command_toward(command_rad, goal_rad, max_step_rad):
    """명령값을 목표 쪽으로 한 스텝(max_step_rad)만 옮긴다"""
    diff = goal_rad - command_rad
    return command_rad + float(np.clip(diff, -max_step_rad, max_step_rad))


def make_goal_rad(start_rad, lower_deg, upper_deg):
    """
    시작 각도 + MOVE_DEG 를 목표로 만든다.
    한계 밖이면 None 을 돌려준다 (그러면 움직이지 않는다).
    """
    goal_deg = np.rad2deg(start_rad) + MOVE_DEG
    if not (lower_deg <= goal_deg <= upper_deg):
        print(f"   ! 목표 {goal_deg:+.1f} deg 가 한계 {lower_deg:.1f} ~ {upper_deg:.1f} 밖입니다."
              " MOVE_DEG 를 바꾸세요. 이번 Play 에서는 움직이지 않습니다.")
        return None
    return np.deg2rad(goal_deg)


# ══════════════════════════════════════════════════════════════
#  메인
# ══════════════════════════════════════════════════════════════
def main():
    print("\n── SCENE ──")
    open_scene()
    lower_deg, upper_deg = read_joint_limit_deg(JOINT_NAME)
    print(f"   {JOINT_NAME} 한계   {lower_deg:.1f} ~ {upper_deg:.1f} deg")
    setup_arm_drives()

    world = World(stage_units_in_meters=1.0)
    # 수업 코드와 같은 SingleManipulator. 포크는 그리퍼가 아니므로 gripper 는 넘기지 않는다
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
    world.reset()

    max_step_rad = np.deg2rad(SPEED_DEG_PER_S) * world.get_physics_dt()

    print("\n── RUN ──")
    print("   뷰포트에서 Play 를 누르세요\n")

    was_playing = False
    joint_index = None
    goal_rad = None
    command_rad = 0.0
    reached = False
    step = 0

    while simulation_app.is_running():
        world.step(render=True)
        time.sleep(0.005)

        is_playing = world.is_playing()

        # ── Play 를 누른 순간: 지금 자세 고정 + 목표 정하기 ──
        if is_playing and not was_playing:
            world.reset()
            joint_index = robot.get_dof_index(JOINT_NAME)
            hold_all_arm_joints(robot)
            command_rad = float(robot.get_joint_positions()[joint_index])
            goal_rad = make_goal_rad(command_rad, lower_deg, upper_deg)
            reached = False
            step = 0
            if goal_rad is not None:
                print(f"   시작  {JOINT_NAME}(index {joint_index})"
                      f"  {np.rad2deg(command_rad):+.1f} -> {np.rad2deg(goal_rad):+.1f} deg")

        # ── Play 중: 매 스텝 명령을 조금씩 옮긴다 ──
        if is_playing and goal_rad is not None:
            command_rad = move_command_toward(command_rad, goal_rad, max_step_rad)
            robot.apply_action(ArticulationAction(
                joint_positions=np.array([command_rad]),
                joint_indices=np.array([joint_index]),
            ))

            actual_rad = float(robot.get_joint_positions()[joint_index])
            error_deg = abs(np.rad2deg(goal_rad - actual_rad))

            if step % LOG_INTERVAL == 0:
                print(f"   명령 {np.rad2deg(command_rad):+8.2f}   "
                      f"실제 {np.rad2deg(actual_rad):+8.2f}   오차 {error_deg:6.2f} deg")

            if not reached and command_rad == goal_rad and error_deg <= REACHED_TOL_DEG:
                reached = True
                print(f"   도달  실제 {np.rad2deg(actual_rad):+.2f} deg  (Stop 후 Play 하면 다시 시험)")

            step += 1

        was_playing = is_playing

    simulation_app.close()


if __name__ == "__main__":
    main()
