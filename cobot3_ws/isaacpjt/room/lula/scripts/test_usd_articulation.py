"""STEP 2 -- USD Articulation 단독 검증 (Lula 없이). Isaac Sim 안에서 실행.

Lula를 붙이기 전에 USD 쪽이 그 자체로 안정적인지 먼저 확인한다 (문서가 지적한 "로봇이 튀는
문제"는 대개 여기, Lula 연결 이전 단계에서 원인이 생긴다).

확인하는 것:
  1) DOF 이름/순서가 joint_1_joint..joint_6_joint 순서인지
  2) reset() 직후 1~2초 그대로 뒀을 때 관절이 저절로 안 움직이는지(드리프트 = 드라이브/댐핑 이상 신호)
  3) 안전 범위 안의 작은 목표 각도를 줘서 실제로 그 근처로 수렴하는지

사용법 (standalone, Script Editor 아님):
  ~/isaacsim/python.sh lula/scripts/test_usd_articulation.py            (GUI로 뜸)
  ~/isaacsim/python.sh lula/scripts/test_usd_articulation.py --headless (창 없이)
"""
import argparse

ap = argparse.ArgumentParser()
ap.add_argument("--headless", action="store_true")
args = ap.parse_args()

from isaacsim import SimulationApp
app = SimulationApp({"headless": args.headless})  # 다른 isaacsim.* import보다 반드시 먼저 실행돼야 함

import numpy as np

RUN_MODE = "fixed"   # "fixed" (B안) 또는 "mobile" (A안)
ROOT = "/World/m0617_fixed_base" if RUN_MODE == "fixed" else "/World/mobile_manipulator"
ARM_PATH = ROOT + "/m0617"

from isaacsim.core.api import World
try:
    from isaacsim.core.prims import SingleArticulation as _ArticulationCls
except ImportError:
    from isaacsim.core.prims import Articulation as _ArticulationCls
from isaacsim.core.utils.stage import open_stage

import os
HERE = os.path.dirname(os.path.abspath(__file__))
PKG = os.path.dirname(os.path.dirname(HERE))  # lula/scripts -> lula -> package root
usd = os.path.join(PKG, "scenes", "m0617_fixed_base_demo.usda" if RUN_MODE == "fixed" else "mobile_manipulator_demo.usda")
open_stage(usd)

world = World(stage_units_in_meters=1.0)
world.reset()

arm = _ArticulationCls(ARM_PATH, name="test_arm")
arm.initialize()

names = arm.dof_names
expected = [f"joint_{i}_joint" for i in range(1, 7)]
print("[1] DOF 이름/순서:", names)
print("    기대:", expected)
print("    ", "OK" if names == expected else "MISMATCH -- ik_bridge.py의 순서 재매핑 로직 확인 필요")

q0 = arm.get_joint_positions()
for _ in range(120):  # 약 1~2초, physics_dt에 따라 다름
    world.step(render=False)
q1 = arm.get_joint_positions()
drift_deg = np.degrees(np.abs(np.asarray(q1) - np.asarray(q0)))
print("\n[2] 외력 없이 방치 시 드리프트(deg):", np.round(drift_deg, 3))
print("    ", "OK (0.5deg 미만)" if (drift_deg < 0.5).all() else "FAIL -- 드라이브 stiffness/damping 확인 필요 (레이어 C)")

target_rad = np.radians([10, -10, 10, 0, 10, 0])  # 안전 범위 안 (J2/J5도 ±95/±135 안쪽)
from isaacsim.core.utils.types import ArticulationAction
arm.apply_action(ArticulationAction(joint_positions=target_rad))
for _ in range(240):
    world.step(render=False)
q2 = np.asarray(arm.get_joint_positions())
err_deg = np.degrees(np.abs(q2 - target_rad))
print("\n[3] 목표", np.degrees(target_rad), "-> 실측", np.round(np.degrees(q2), 2), "deg, 오차", np.round(err_deg, 2))
print("    ", "OK (2deg 미만)" if (err_deg < 2.0).all() else "FAIL -- 튀거나 못 따라감, Lula 붙이기 전에 이것부터 원인 파악")

app.close()
