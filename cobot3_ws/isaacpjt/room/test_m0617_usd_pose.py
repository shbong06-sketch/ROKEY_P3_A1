from isaacsim import SimulationApp

simulation_app = SimulationApp({
    "headless": True
})

import omni.usd
from pxr import UsdGeom

USD_PATH = "/home/rokey/Downloads/robot_sample_v3_1_260916/assets/m0617_rebuilt.usd"
LINK6_PATH = "/m0617/link_6"

print("\n=== M0617 USD POSE TEST ===")

# 1. USD 열기
omni.usd.get_context().open_stage(USD_PATH)

stage = omni.usd.get_context().get_stage()

# 2. link_6 확인
prim = stage.GetPrimAtPath(LINK6_PATH)

print("\n[1] Prim Check")
print("Path :", prim.GetPath())
print("Type :", prim.GetTypeName())
print("Valid:", prim.IsValid())

if not prim.IsValid():
    print("\nERROR: link_6을 찾을 수 없습니다.")
    simulation_app.close()
    raise SystemExit(1)

# 3. World Transform 계산
xform = UsdGeom.Xformable(prim)
matrix = xform.ComputeLocalToWorldTransform(0)

print("\n[2] World Transform")
print(matrix)

# 4. Translation만 출력
translation = matrix.ExtractTranslation()

print("\n[3] World Translation")
print("X =", translation[0])
print("Y =", translation[1])
print("Z =", translation[2])

print("\n=== TEST COMPLETE ===")

simulation_app.close()
