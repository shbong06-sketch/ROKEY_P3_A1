"""
지금 열려 있는 장면을 **읽기만** 해서 리프트/카터/팔의 물리 상태를 진단합니다.
아무것도 바꾸지 않습니다. Script Editor 에 붙여넣고 Ctrl+Enter.

무엇을 보나
  - 아티큘레이션 루트가 몇 개인지 (2개 이상이면 그 사이 조인트는 물렁해집니다)
  - kinematic 으로 잡혀 있는 바디 (물리로 안 움직이고 아티큘레이션에도 못 들어감)
  - 모든 조인트의 body0/body1 과, 지금 자세와 조인트 프레임이 어긋났는지
    (어긋나면 Play 순간 물체가 순간이동합니다 = "disjointed body transforms")
  - Prismatic 조인트의 가동 범위 / Drive 목표값이 범위 밖인지
  - 리프트 마스트와 캐리지 메시의 실제 z 범위 → 콜리전을 벗어나지 않는 행정
"""

import omni.usd
from pxr import Usd, UsdGeom, UsdPhysics, Gf


SNAP_TOL_M = 0.002       # 이보다 크면 Play 때 끌려갑니다
SNAP_TOL_DEG = 0.5


def _xf(cache, prim):
    return cache.GetLocalToWorldTransform(prim)


def _joint_frame_world(cache, stage, joint, index):
    """조인트가 선언한 local frame 을 월드로 변환."""
    j = UsdPhysics.Joint(joint)
    rel = j.GetBody0Rel() if index == 0 else j.GetBody1Rel()
    targets = rel.GetTargets()
    pos = (j.GetLocalPos0Attr() if index == 0 else j.GetLocalPos1Attr()).Get()
    rot = (j.GetLocalRot0Attr() if index == 0 else j.GetLocalRot1Attr()).Get()
    pos = Gf.Vec3d(pos) if pos is not None else Gf.Vec3d(0, 0, 0)
    rot = rot if rot is not None else Gf.Quatf(1, 0, 0, 0)

    local = Gf.Matrix4d()
    local.SetRotate(Gf.Quatd(rot.GetReal(), Gf.Vec3d(*rot.GetImaginary())))
    local.SetTranslateOnly(pos)

    if not targets:                       # body 가 비어 있으면 = 월드
        return local, None
    body = stage.GetPrimAtPath(targets[0])
    if not body.IsValid():
        return None, None
    return local * _xf(cache, body), body


def main():
    stage = omni.usd.get_context().get_stage()
    cache = UsdGeom.XformCache()

    arti_roots, kinematics, joints = [], [], []
    for prim in stage.Traverse():
        if prim.HasAPI(UsdPhysics.ArticulationRootAPI):
            arti_roots.append(prim)
        if prim.HasAPI(UsdPhysics.RigidBodyAPI):
            if UsdPhysics.RigidBodyAPI(prim).GetKinematicEnabledAttr().Get():
                kinematics.append(prim)
        if prim.IsA(UsdPhysics.Joint):
            joints.append(prim)

    print("=" * 70)
    print("아티큘레이션 루트 %d 개" % len(arti_roots))
    for p in arti_roots:
        print("   ", p.GetPath())
    if len(arti_roots) > 1:
        print("   ! 2개 이상입니다. 서로 다른 아티큘레이션을 조인트로 이으면")
        print("     그 연결은 물렁해집니다. 하나로 합치는 것을 권합니다.")

    print("\nkinematic 바디 %d 개 (물리로 안 움직임)" % len(kinematics))
    for p in kinematics:
        print("   ", p.GetPath())

    print("\n조인트 %d 개" % len(joints))
    for jp in joints:
        j = UsdPhysics.Joint(jp)
        b0 = j.GetBody0Rel().GetTargets()
        b1 = j.GetBody1Rel().GetTargets()
        active = "" if jp.IsActive() else "  [비활성]"
        print("\n  %s  (%s)%s" % (jp.GetPath(), jp.GetTypeName(), active))
        print("     body0=%s" % ([str(t) for t in b0] or "월드"))
        print("     body1=%s" % [str(t) for t in b1])

        if not jp.IsActive():
            continue

        # 지금 자세에서 두 프레임이 얼마나 벌어져 있는지
        f0, p0 = _joint_frame_world(cache, stage, jp, 0)
        f1, p1 = _joint_frame_world(cache, stage, jp, 1)
        if f0 is not None and f1 is not None:
            d = Gf.Transform(f1 * f0.GetInverse())
            gap = d.GetTranslation().GetLength()
            ang = abs(d.GetRotation().GetAngle())
            flag = ""
            if gap > SNAP_TOL_M or ang > SNAP_TOL_DEG:
                flag = "   <-- Play 때 끌려갑니다(스냅)"
            print("     프레임 어긋남: %.4f m / %.2f deg%s" % (gap, ang, flag))
            if jp.GetTypeName() == "PhysicsPrismaticJoint":
                axis = UsdPhysics.PrismaticJoint(jp).GetAxisAttr().Get()
                comp = {"X": 0, "Y": 1, "Z": 2}.get(axis, 2)
                print("     축(%s) 방향 현재 변위: %+.4f m"
                      % (axis, d.GetTranslation()[comp]))

        if jp.GetTypeName() == "PhysicsPrismaticJoint":
            pj = UsdPhysics.PrismaticJoint(jp)
            lo, hi = pj.GetLowerLimitAttr().Get(), pj.GetUpperLimitAttr().Get()
            print("     가동 범위: %.4f ~ %.4f" % (lo or 0.0, hi or 0.0))

        for token in ("linear", "angular"):
            if not jp.HasAPI(UsdPhysics.DriveAPI, token):
                continue
            d = UsdPhysics.DriveAPI(jp, token)
            tp = d.GetTargetPositionAttr().Get()
            tv = d.GetTargetVelocityAttr().Get()
            print("     Drive[%s] type=%s target=%s targetVel=%s k=%s c=%s maxF=%s"
                  % (token, d.GetTypeAttr().Get(), tp, tv,
                     d.GetStiffnessAttr().Get(), d.GetDampingAttr().Get(),
                     d.GetMaxForceAttr().Get()))
            if jp.GetTypeName() == "PhysicsPrismaticJoint":
                pj = UsdPhysics.PrismaticJoint(jp)
                lo = pj.GetLowerLimitAttr().Get() or 0.0
                hi = pj.GetUpperLimitAttr().Get() or 0.0
                if tp is not None and not (lo - 1e-6 <= tp <= hi + 1e-6):
                    print("     ! Drive 목표 %.3f 가 가동 범위(%.3f~%.3f) 밖입니다."
                          % (tp, lo, hi))
                    print("       리미트와 Drive 가 계속 싸우면서 떨거나 터집니다.")
                if tv not in (None, 0.0):
                    print("     ! 위치 제어인데 targetVelocity=%s 입니다. 0 이어야 합니다." % tv)

    # 리프트 메시 범위로 안전 행정 계산
    print("\n" + "=" * 70)
    bbox = UsdGeom.BBoxCache(
        Usd.TimeCode.Default(), [UsdGeom.Tokens.default_, UsdGeom.Tokens.render]
    )
    holder = mover = None
    for prim in stage.Traverse():
        if prim.GetName() == "lift_holder":
            holder = prim
        elif prim.GetName() == "lift_moveparts_1":
            mover = prim
    if holder and mover:
        hr = bbox.ComputeWorldBound(holder).ComputeAlignedRange()
        mr = bbox.ComputeWorldBound(mover).ComputeAlignedRange()
        print("마스트 z: %.4f ~ %.4f" % (hr.GetMin()[2], hr.GetMax()[2]))
        print("캐리지 z: %.4f ~ %.4f" % (mr.GetMin()[2], mr.GetMax()[2]))
        print("콜리전을 벗어나지 않는 행정:  위로 %.4f m / 아래로 %.4f m"
              % (hr.GetMax()[2] - mr.GetMax()[2], mr.GetMin()[2] - hr.GetMin()[2]))
    else:
        print("lift_holder / lift_moveparts_1 를 못 찾아 행정 계산을 건너뜁니다.")
    print("=" * 70)


main()
