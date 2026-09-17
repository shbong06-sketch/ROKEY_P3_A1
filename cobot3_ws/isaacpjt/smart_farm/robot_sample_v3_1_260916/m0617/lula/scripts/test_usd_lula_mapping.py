"""STEP 5 (가장 중요) -- USD와 Lula가 "같은 로봇"이라고 실제로 검증.

STEP 1~4가 각각 통과해도, "URDF 기준 Lula가 계산한 link_6 pose"와 "그 q를 USD에 넣었을 때
실제 link_6 prim의 world pose"가 같아야 진짜 연결이 끝난 것이다. 이 스크립트가 둘을 직접 비교한다.

절차:
  1) USD 로봇을 임의 자세(q_test)로 이동시키고 Play해서 안정시킨다
  2) 실제 link_6 prim의 world position/orientation을 읽는다 (Isaac Sim 쪽 "정답")
  3) 같은 q_test를 Lula FK에 넣어서 계산한 link_6 pose와 비교한다
  4) 위치가 몇 mm 이상 벌어지면 -- URDF와 USD가 서로 다른 로봇이거나(스케일/오프셋 문제),
     Lula의 base pose 설정이 잘못됐다는 뜻. ik_bridge.py를 실제로 쓰기 전에 이 스크립트부터 통과해야 함.

사용법 (standalone):
  ~/isaacsim/python.sh lula/scripts/test_usd_lula_mapping.py [--headless]
"""
import argparse
import os

HERE = os.path.dirname(os.path.abspath(__file__))
LULA = os.path.dirname(HERE)
PKG = os.path.dirname(LULA)
URDF_PATH = os.path.join(LULA, "dsr_description2", "urdf", "m0617.urdf")
YAML_PATH = os.path.join(LULA, "config", "m0617_robot_description.yaml")

if not os.path.isfile(YAML_PATH):
    raise SystemExit(f"FAIL: {YAML_PATH} 없음 -- yaml부터 생성하고, test_lula_fk.py/test_lula_ik.py를 먼저 통과시킬 것")

ap = argparse.ArgumentParser()
ap.add_argument("--headless", action="store_true")
args = ap.parse_args()

from isaacsim import SimulationApp
app = SimulationApp({"headless": args.headless})  # 다른 isaacsim.* import보다 반드시 먼저

import numpy as np

from isaacsim.core.api import World
try:
    from isaacsim.core.prims import SingleArticulation as _ArticulationCls
except ImportError:
    from isaacsim.core.prims import Articulation as _ArticulationCls
try:
    from isaacsim.core.prims import SingleXFormPrim as _XFormPrimCls
except ImportError:
    from isaacsim.core.prims import XFormPrim as _XFormPrimCls
from isaacsim.core.utils.stage import open_stage
from isaacsim.core.utils.types import ArticulationAction
from isaacsim.robot_motion.motion_generation import LulaKinematicsSolver

ROOT = "/World/m0617_fixed_base"   # B안으로 검증(베이스 안 움직이니 변수 적음)
ARM_PATH = ROOT + "/m0617"

open_stage(os.path.join(PKG, "scenes", "m0617_fixed_base_demo.usda"))
world = World(stage_units_in_meters=1.0)
world.reset()

arm = _ArticulationCls(ARM_PATH, name="mapping_test_arm")
arm.initialize()

q_test_deg = np.array([15.0, -20.0, 30.0, 0.0, 25.0, 0.0])  # 안전 범위 안 (J2/J5 확인)
names = arm.dof_names
order = [f"joint_{i}_joint" for i in range(1, 7)]
idx = [names.index(n) for n in order]
q_full = np.zeros(len(names))
for i, id_ in enumerate(idx):
    q_full[id_] = np.radians(q_test_deg[i])
arm.apply_action(ArticulationAction(joint_positions=q_full))
for _ in range(240):
    world.step(render=False)

# 2) USD 쪽 "정답": link_6 prim의 실제 world pose
link6_prim = _XFormPrimCls(ARM_PATH + "/link_6")
usd_pos, usd_quat = link6_prim.get_world_pose()
print("[USD] link_6 world position:", np.round(usd_pos, 5))
print("[USD] link_6 world orientation(wxyz):", np.round(usd_quat, 5))

# 3) Lula FK로 같은 q를 넣었을 때
solver = LulaKinematicsSolver(robot_description_path=YAML_PATH, urdf_path=URDF_PATH)
arm_base_pos, arm_base_quat = arm.get_world_pose()
solver.set_robot_base_pose(arm_base_pos, arm_base_quat)  # M0617 base가 world 원점이 아닐 수 있으므로 필수
lula_pos, lula_rot = solver.compute_forward_kinematics("link_6", np.radians(q_test_deg))
print("\n[Lula] link_6 position (base pose 보정 후):", np.round(lula_pos, 5))

# 4) 비교 (position + orientation 둘 다 -- 위치만 맞고 자세가 틀어져도 실제 그립에는 문제가 됨)
err_mm = np.linalg.norm(np.asarray(usd_pos) - np.asarray(lula_pos)) * 1000

def quat_angle_deg(q1, q2):
    # 쿼터니언(w,x,y,z) 사이 최소 회전각. 부호 반전(q, -q가 같은 회전)도 처리.
    q1 = np.asarray(q1) / np.linalg.norm(q1)
    q2 = np.asarray(q2) / np.linalg.norm(q2)
    dot = abs(np.dot(q1, q2))
    dot = min(1.0, max(-1.0, dot))
    return np.degrees(2 * np.arccos(dot))

# Lula의 compute_forward_kinematics가 반환하는 lula_rot이 회전행렬이면 쿼터니언으로 변환 필요.
# 3x3이면 회전행렬로 보고 변환, 이미 쿼터니언(길이4)이면 그대로 사용.
lula_rot_arr = np.asarray(lula_rot)
if lula_rot_arr.shape == (3, 3):
    from isaacsim.core.utils.numpy.rotations import rot_matrices_to_quats
    lula_quat = rot_matrices_to_quats(lula_rot_arr[None, ...])[0]
else:
    lula_quat = lula_rot_arr

orient_err_deg = quat_angle_deg(usd_quat, lula_quat)
print(f"\n위치 오차: {err_mm:.2f} mm")
print(f"자세 오차: {orient_err_deg:.2f} deg")
print("===", "PASS -- USD와 Lula가 같은 로봇으로 일치함, ik_bridge.py 써도 됨"
      if (err_mm < 5.0 and orient_err_deg < 2.0)
      else "FAIL -- 위치 5mm 또는 자세 2deg 이상 벌어짐. base pose 설정을 빼먹었거나, "
           "URDF와 USD의 스케일/오프셋/축 방향이 다를 수 있음", "===")
app.close()
