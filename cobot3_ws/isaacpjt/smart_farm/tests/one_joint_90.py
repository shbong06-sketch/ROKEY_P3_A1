"""
M0609 관절 하나만 90도 꺾기 — 가장 단순한 시험

    cd cobot3_ws/isaacpjt/smart_farm/tests
    isaac_python one_joint_90.py

하는 일
  1) M0609 USD 를 불러온다
  2) Play 를 누르면 JOINT_NAME 관절 하나만 천천히 TARGET_DEG 까지 돌린다
  3) 명령값과 실제값을 1초마다 찍는다

나머지 관절은 Play 순간의 자세를 그대로 유지한다.
IK, 그리퍼, RMPflow 는 쓰지 않는다.
"""

from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": False})

from pathlib import Path

import numpy as np
import omni.usd
from pxr import Usd, UsdGeom, UsdPhysics

from isaacsim.core.api import World
from isaacsim.core.prims import SingleArticulation
from isaacsim.core.utils.types import ArticulationAction


# ══════════════════════════════════════════════════════════════
#  경로 — 수업 코드와 같은 USD
# ══════════════════════════════════════════════════════════════
#   이 파일 위치  isaacpjt/smart_farm/tests/
#   parents[1]    isaacpjt/
THIS_DIR  = Path(__file__).resolve().parent
M0609_DIR = THIS_DIR.parents[1] / "M0609"
USD_PATH  = str(M0609_DIR / "Collected_m0609_camera_cube/m0609_camera_cube.usd")

if not Path(USD_PATH).is_file():
    simulation_app.close()
    raise FileNotFoundError(f"USD 파일이 없습니다: {USD_PATH}")

ROBOT_PRIM_PATH = "/World/m0609"
ARM_JOINTS = ["joint_1", "joint_2", "joint_3",
              "joint_4", "joint_5", "joint_6"]


# ══════════════════════════════════════════════════════════════
#  시험 설정 — 여기만 바꾸면 된다
# ══════════════════════════════════════════════════════════════
# joint_1 은 바닥에 수직인 축이라 돌려도 팔이 바닥에 닿지 않는다. 첫 시험으로 안전하다.
JOINT_NAME = "joint_1"
TARGET_DEG = 90.0          # 절대 각도(도). "지금보다 90도 더" 가 아니다

SPEED_DEG_PER_S = 30.0     # 한 번에 확 돌지 않도록 초당 이만큼만 명령을 옮긴다
REACHED_TOL_DEG = 1.0      # 실제 각도가 목표와 이만큼 가까우면 도달로 본다

# 수업 코드와 같은 값. USD 원래 값(약 40)은 너무 약해서 팔이 중력에 처진다
DRIVE_STIFFNESS = 1e8
DRIVE_DAMPING   = 1e4
DRIVE_MAX_FORCE = 1e8

LOG_INTERVAL = 60          # 몇 스텝마다 출력할지 (60 스텝 = 약 1초)


# ══════════════════════════════════════════════════════════════
#  준비 함수
# ══════════════════════════════════════════════════════════════
def load_usd():
    """/World 에 M0609 USD 를 붙인다 (1_load_usd_standalone.py 와 같은 방법)"""
    stage = omni.usd.get_context().get_stage()
    UsdGeom.Xform.Define(stage, "/World")
    stage.GetPrimAtPath("/World").GetReferences().AddReference(USD_PATH)
    for _ in range(15):
        simulation_app.update()


def find_joint_prim(joint_name):
    """로봇 아래에서 이름으로 관절 prim 을 찾는다. 없으면 None"""
    stage = omni.usd.get_context().get_stage()
    for prim in Usd.PrimRange(stage.GetPrimAtPath(ROBOT_PRIM_PATH)):
        if prim.GetName() == joint_name:
            return prim
    return None


def check_target_in_limit():
    """목표 각도가 USD 에 적힌 관절 한계 안인지 확인한다. 밖이면 시작하지 않는다"""
    prim = find_joint_prim(JOINT_NAME)
    if prim is None:
        raise RuntimeError(f"'{JOINT_NAME}' 관절을 {ROBOT_PRIM_PATH} 아래에서 찾지 못했습니다.")

    joint = UsdPhysics.RevoluteJoint(prim)
    lower = joint.GetLowerLimitAttr().Get()     # USD 는 도(deg) 단위
    upper = joint.GetUpperLimitAttr().Get()
    print(f"   {JOINT_NAME} 한계   {lower:.1f} ~ {upper:.1f} deg")

    if not (lower <= TARGET_DEG <= upper):
        raise ValueError(f"TARGET_DEG={TARGET_DEG} 가 한계 밖입니다.")


def setup_arm_drives():
    """팔 6축 Drive 를 강하게 만든다. 메모리 안의 장면만 바뀌고 USD 파일은 그대로다"""
    for name in ARM_JOINTS:
        prim = find_joint_prim(name)
        drive = UsdPhysics.DriveAPI.Get(prim, "angular")
        drive.GetStiffnessAttr().Set(DRIVE_STIFFNESS)
        drive.GetDampingAttr().Set(DRIVE_DAMPING)
        drive.GetMaxForceAttr().Set(DRIVE_MAX_FORCE)
    print(f"   arm drives   {len(ARM_JOINTS)}개 강화")


# ══════════════════════════════════════════════════════════════
#  제어 함수
# ══════════════════════════════════════════════════════════════
def hold_all_arm_joints(robot):
    """
    지금 자세를 그대로 목표로 준다.
    이렇게 해야 Play 순간에 다른 관절이 USD 에 저장된 옛 목표로 끌려가지 않는다.
    """
    indices = np.array([robot.get_dof_index(n) for n in ARM_JOINTS])
    current = robot.get_joint_positions()[indices]
    robot.apply_action(ArticulationAction(joint_positions=current, joint_indices=indices))


def move_command_toward(command_rad, goal_rad, max_step_rad):
    """명령값을 목표 쪽으로 한 스텝(max_step_rad)만 옮긴다"""
    diff = goal_rad - command_rad
    return command_rad + float(np.clip(diff, -max_step_rad, max_step_rad))


# ══════════════════════════════════════════════════════════════
#  메인
# ══════════════════════════════════════════════════════════════
def main():
    print("\n── SCENE ──")
    world = World(stage_units_in_meters=1.0)
    load_usd()
    check_target_in_limit()
    setup_arm_drives()

    robot = world.scene.add(SingleArticulation(prim_path=ROBOT_PRIM_PATH, name="m0609"))
    world.reset()

    goal_rad = np.deg2rad(TARGET_DEG)
    max_step_rad = np.deg2rad(SPEED_DEG_PER_S) * world.get_physics_dt()

    print("\n── RUN ──")
    print("   뷰포트에서 Play 를 누르세요\n")

    was_playing = False
    joint_index = None
    command_rad = 0.0
    reached = False
    step = 0

    while simulation_app.is_running():
        world.step(render=True)
        is_playing = world.is_playing()

        # ── Play 를 누른 순간: 처음부터 다시 ──
        if is_playing and not was_playing:
            world.reset()
            joint_index = robot.get_dof_index(JOINT_NAME)
            hold_all_arm_joints(robot)
            command_rad = float(robot.get_joint_positions()[joint_index])   # 지금 각도에서 출발
            reached = False
            step = 0
            print(f"   시작  {JOINT_NAME}(index {joint_index})"
                  f"  {np.rad2deg(command_rad):+.1f} -> {TARGET_DEG:+.1f} deg")

        # ── Play 중: 매 스텝 명령을 조금씩 옮긴다 ──
        if is_playing:
            command_rad = move_command_toward(command_rad, goal_rad, max_step_rad)
            robot.apply_action(ArticulationAction(
                joint_positions=np.array([command_rad]),
                joint_indices=np.array([joint_index]),
            ))

            actual_rad = float(robot.get_joint_positions()[joint_index])
            error_deg = abs(np.rad2deg(goal_rad - actual_rad))

            if step % LOG_INTERVAL == 0:
                print(f"   명령 {np.rad2deg(command_rad):+7.2f}   "
                      f"실제 {np.rad2deg(actual_rad):+7.2f}   오차 {error_deg:6.2f} deg")

            if not reached and command_rad == goal_rad and error_deg <= REACHED_TOL_DEG:
                reached = True
                print(f"   도달  실제 {np.rad2deg(actual_rad):+.2f} deg  (Stop 후 Play 하면 다시 시험)")

            step += 1

        was_playing = is_playing

    simulation_app.close()


if __name__ == "__main__":
    main()
