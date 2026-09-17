from isaacsim import SimulationApp

simulation_app = SimulationApp({
    "headless": True
})

import omni.usd
from pxr import UsdGeom

USD_PATH = "/home/rokey/Downloads/robot_sample_v3_1_260916/assets/m0617_rebuilt.usd"

print("\n=== M0617 USD TEST ===")

# --------------------------------------------------
# 1. USD Stage 생성
# --------------------------------------------------

stage = omni.usd.get_context().new_stage()

# --------------------------------------------------
# 2. M0617 USD 추가
# --------------------------------------------------

omni.usd.get_context().open_stage(USD_PATH)

stage = omni.usd.get_context().get_stage()

print("\n[1] USD loaded")
print("Stage:", stage)

# --------------------------------------------------
# 3. Stage 전체에서 link_6 검색
# --------------------------------------------------

print("\n[2] Search link_6")

found = []

for prim in stage.Traverse():
    if "link_6" in prim.GetName().lower():
        found.append(prim.GetPath())
        print("FOUND:", prim.GetPath())
        print("TYPE :", prim.GetTypeName())

# --------------------------------------------------
# 4. 결과
# --------------------------------------------------

print("\n[3] Result")

if len(found) == 0:
    print("ERROR: link_6 not found")
else:
    print("link_6 count =", len(found))

# --------------------------------------------------
# 종료
# --------------------------------------------------

simulation_app.close()

print("\n=== TEST COMPLETE ===")
