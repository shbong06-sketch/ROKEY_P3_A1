"""STEP 4 -- Lula IK round-trip 검증. Isaac Sim 안에서 실행, USD 로봇 필요 없음.

target pose -> IK -> q -> FK(q) -> target과 다시 비교. 이게 어긋나면 robot_description.yaml
자체가 이상한 것(예: base 링크 collision sphere 반지름 0 문제)이지 USD 연결 문제가 아니다.
STEP 3(test_lula_fk.py)이 먼저 통과해야 의미 있다.

사용법 (standalone):
  ~/isaacsim/python.sh lula/scripts/test_lula_ik.py [--headless]
"""
import argparse
import os

HERE = os.path.dirname(os.path.abspath(__file__))
LULA = os.path.dirname(HERE)
URDF_PATH = os.path.join(LULA, "dsr_description2", "urdf", "m0617.urdf")
YAML_PATH = os.path.join(LULA, "config", "m0617_robot_description.yaml")

if not os.path.isfile(YAML_PATH):
    raise SystemExit(f"FAIL: {YAML_PATH} 없음 -- yaml부터 생성할 것 (test_lula_fk.py를 먼저 통과시킬 것)")

ap = argparse.ArgumentParser()
ap.add_argument("--headless", action="store_true")
args = ap.parse_args()

from isaacsim import SimulationApp
app = SimulationApp({"headless": args.headless})  # 다른 isaacsim.* import보다 반드시 먼저

import numpy as np
from isaacsim.robot_motion.motion_generation import LulaKinematicsSolver

solver = LulaKinematicsSolver(robot_description_path=YAML_PATH, urdf_path=URDF_PATH)

# ArticulationKinematicsSolver는 실제 USD Articulation 인스턴스가 있어야 생성되므로,
# IK 계산 자체만 확인할 땐 LulaKinematicsSolver.compute_inverse_kinematics를 직접 씀
# (base pose는 로봇이 원점에 있다고 가정 -- 실제 USD에 붙일 때는 ik_bridge.py의
#  set_robot_base_pose()를 반드시 거칠 것).
# 주의: 공식 문서 표현상("ArticulationKinematicsSolver가 결과를 ArticulationAction으로 감싼다")
# 이 raw 호출은 ArticulationAction이 아니라 순수 관절각 배열을 반환할 가능성이 있음 -- 둘 다 처리.
# base pose를 안 알려줬으므로 LulaKinematicsSolver는 로봇 base가 world 원점에 있다고 가정한다
# (공식 문서: "assumes robot base positioned at origin unless specified"). 즉 아래 target 좌표는
# 사실상 "M0617 base 기준" 좌표와 동일하다 -- world 좌표라고 착각하면 안 됨(실제 USD에서 M0617
# base가 원점이 아니면 ik_bridge.py처럼 set_robot_base_pose()로 반드시 보정해야 함).
targets = {
    "reachable_front": (np.array([0.5, 0.0, 0.5]), None),
    "reachable_side":  (np.array([0.3, 0.4, 0.4]), None),
}

for label, (pos, rot) in targets.items():
    warm_start = np.zeros(6)
    result, success = solver.compute_inverse_kinematics(
        frame_name="link_6", target_position=pos, target_orientation=rot,
        warm_start=warm_start,
    )
    print(f"\n[{label}] target={pos} -> success={success}")
    if not success:
        print("  FAIL: IK 미수렴 (도달 불가능한 위치이거나 yaml 리미트 문제)")
        continue
    q = np.asarray(result.joint_positions) if hasattr(result, "joint_positions") else np.asarray(result)
    # success=True라도 값 자체가 이상할 수 있어서(문서가 이 가능성을 지적함) 별도로 검증
    if q.shape[0] != 6 or not np.isfinite(q).all():
        print(f"  FAIL: success=True인데 joint_positions가 이상함 (shape={q.shape}, finite={np.isfinite(q).all()})")
        continue
    print("  q(deg)=", np.round(np.degrees(q), 2))
    fk_pos, fk_rot = solver.compute_forward_kinematics("link_6", q)
    err = np.linalg.norm(fk_pos - pos)
    print(f"  FK(IK(target)) position={np.round(fk_pos, 4)}, 오차={err:.5f}m",
          "OK" if err < 0.01 else "FAIL -- round-trip 안 맞음")

print("\n=== 전부 success=True + round-trip 오차 1cm 미만이면 PASS -- 이제 USD에 연결해도 됨 ===")
app.close()
