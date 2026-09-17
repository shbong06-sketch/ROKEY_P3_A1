"""STEP 3 -- Lula FK 단독 검증. Isaac Sim 안에서 실행하되 USD 로봇은 필요 없음
(LulaKinematicsSolver는 URDF+yaml만 있으면 FK/IK 계산 자체는 가능).

q=[0,0,0,0,0,0]일 때 link_6의 pose가 나오는지, q를 조금씩 바꿨을 때 pose가 상식적으로
움직이는지(예: joint_1을 90도 돌리면 x/y가 그 방향으로 회전) 확인한다.

robot_description.yaml이 아직 없으면(lula/config/ 비어있으면) 이 스크립트는
Isaac Sim을 띄우기도 전에 파일 존재 여부부터 확인해서 바로 종료한다 -- 그게 정상이다, yaml부터 만들 것.

사용법 (standalone):
  ~/isaacsim/python.sh lula/scripts/test_lula_fk.py [--headless]
"""
import argparse
import os

HERE = os.path.dirname(os.path.abspath(__file__))
LULA = os.path.dirname(HERE)  # lula/scripts -> lula
URDF_PATH = os.path.join(LULA, "dsr_description2", "urdf", "m0617.urdf")
YAML_PATH = os.path.join(LULA, "config", "m0617_robot_description.yaml")

print("[test_lula_fk] urdf:", URDF_PATH)
print("[test_lula_fk] yaml:", YAML_PATH)
if not os.path.isfile(YAML_PATH):
    raise SystemExit(
        f"FAIL: {YAML_PATH} 없음 -- XRDF Editor로 robot_description.yaml부터 생성해서 "
        f"이 경로에 저장할 것 (lula/README.md 참고). Isaac Sim을 띄우기 전에 먼저 확인함."
    )

ap = argparse.ArgumentParser()
ap.add_argument("--headless", action="store_true")
args = ap.parse_args()

from isaacsim import SimulationApp
app = SimulationApp({"headless": args.headless})  # 다른 isaacsim.* import보다 반드시 먼저

import numpy as np
from isaacsim.robot_motion.motion_generation import LulaKinematicsSolver

solver = LulaKinematicsSolver(robot_description_path=YAML_PATH, urdf_path=URDF_PATH)
print("valid frame names:", solver.get_all_frame_names())
assert "link_6" in solver.get_all_frame_names(), "link_6 프레임이 없음 -- yaml 생성이 잘못됐을 가능성"

test_configs = {
    "zero":        [0, 0, 0, 0, 0, 0],
    "joint1_90":   [90, 0, 0, 0, 0, 0],
    "small_bend":  [0, 20, -20, 0, 20, 0],
}
for label, q_deg in test_configs.items():
    q_rad = np.radians(q_deg)
    pos, rot = solver.compute_forward_kinematics("link_6", q_rad)
    print(f"\n[{label}] q={q_deg}deg -> link_6 position={np.round(pos, 4)}")
    print(f"           rotation matrix=\n{np.round(rot, 3)}")

print("\n=== 위 3개 pose가 NaN 없이 나오고, joint1_90에서 x/y가 zero 대비 회전한 것처럼 보이면 PASS ===")
app.close()
