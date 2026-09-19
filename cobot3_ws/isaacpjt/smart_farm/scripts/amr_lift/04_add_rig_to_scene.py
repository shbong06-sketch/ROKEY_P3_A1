"""
만들어 둔 리그(amr_lift_rig.usd)를 **지금 열려 있는 장면**에 참조로 얹습니다.
Script Editor 에 붙여넣고 Ctrl+Enter.

참조(reference)라서 장면 파일에는 "이 파일을 여기에 놓아라" 한 줄만 들어갑니다.
리그를 고치면 이 장면에도 그대로 반영됩니다.

주의
  - 기존의 /Group (예전에 손으로 올려 둔 카터+리프트)은 지우지 않습니다.
    새 리그가 제대로 도는 걸 확인한 뒤 아웃라이너에서 직접 지우세요.
    (물리 엔진 입장에서 카터가 두 대 있으면 서로 부딪힙니다.)
  - 팔이 AMR 위로 올라가므로, 월드에 고정돼 있던 /World/m0609_with_fork 도
    확인 후 지워야 합니다.
"""

import os

import omni.usd
from pxr import UsdGeom, Gf


RIG_USD = os.path.expanduser(
    "~/ROKEY_P3_A1/cobot3_ws/isaacpjt/smart_farm/assets/amr_lift_rig/amr_lift_rig.usd"
)
PRIM_PATH = "/World/amr_lift_rig"

# 장면 안에서 AMR 을 어디에 세울지 (리그 전체가 함께 움직입니다)
PLACE_TRANSLATE = (0.0, 0.0, 0.0)
PLACE_YAW_DEG = 0.0


def main():
    if not os.path.exists(RIG_USD):
        raise RuntimeError("리그 파일이 없습니다. 02_build_rig.py 를 먼저 실행하세요: %s" % RIG_USD)

    stage = omni.usd.get_context().get_stage()
    if stage.GetPrimAtPath(PRIM_PATH).IsValid():
        raise RuntimeError(
            "%s 가 이미 있습니다. 다시 얹으려면 아웃라이너에서 먼저 지우세요." % PRIM_PATH
        )

    xform = UsdGeom.Xform.Define(stage, PRIM_PATH)
    xform.GetPrim().GetReferences().AddReference(RIG_USD)

    xform.ClearXformOpOrder()
    xform.AddTranslateOp().Set(Gf.Vec3d(*PLACE_TRANSLATE))
    xform.AddOrientOp(UsdGeom.XformOp.PrecisionDouble).Set(
        Gf.Quatd(Gf.Rotation(Gf.Vec3d(0, 0, 1), PLACE_YAW_DEG).GetQuat())
    )

    print("리그를 얹었습니다: %s  ->  %s" % (PRIM_PATH, RIG_USD))
    print("아티큘레이션 루트(제어용 경로): %s/Nova_Carter/chassis_link" % PRIM_PATH)
    print("리프트 조인트: %s/lift/lift_prismatic_joint" % PRIM_PATH)
    print("팔 끝(EE): %s/m0609_with_fork/link_6" % PRIM_PATH)
    print("\n확인이 끝나면 예전 /Group 과 /World/m0609_with_fork 를 지우세요.")


main()
