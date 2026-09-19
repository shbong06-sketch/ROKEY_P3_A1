"""
Collected_m0609_folk_lift_test_3.usd 를 **사본으로** 고친다.
원본은 건드리지 않고 `..._lift_test_4_fixed.usd` 를 새로 만든다.

진단 결과 — 이 파일이 5.1.0 에서 제대로 안 도는 이유 (전부 실측)

  1) 아티큘레이션 루트가 **2개**다.
       /Nova_Carter/chassis_link
       /Nova_Carter/m0609_with_fork/root_joint
     팔이 독립 아티큘레이션이라 카터·리프트와 물리적으로 한 몸이 아니다.

  2) 그 `root_joint` 는 body0 가 비어 있는 FixedJoint = **월드에 못박기**이고,
     프레임이 현재 자세와 **1.1542 m** 어긋나 있다.
     → Play 누르는 순간 팔이 1.15 m 끌려간다. 이게 가장 큰 원인.

  3) `lift_holder` 가 **kinematic** 이다. 물리로 움직이지 않고 아티큘레이션
     링크도 될 수 없다. 카터에 FixedJoint 로 물려 있어도 의미가 없다.

  4) `lift_prismatic_joint` 의 프레임이 **0.0934 m / 6.19도** 어긋나 있다.
     (캐리지에 손으로 준 이동·회전이 남아 있음)
     지금 축 방향 변위는 -0.0137 m 로 리미트(0~0.309) 밖이라,
     Play 하면 리미트와 드라이브가 싸우면서 캐리지가 튄다.

  5) `base_link/FixedJoint` 도 0.0111 m 어긋나 있다 (팔이 살짝 끌려감).

  6) 바퀴 Drive 에 **targetVelocity 10.2 / 12.7 rad/s** 가 남아 있다.
     Play 하자마자 좌우 속도가 달라 카터가 제멋대로 돈다.

  7) 팔 base_link 콜라이더가 convexHull 인데 메시에 케이블이 포함돼 있어
     실제보다 큰 유령 충돌체가 된다. 마스트/캐리지도 convexHull 이면
     속 빈 마스트가 꽉 찬 상자가 되어 캐리지와 영구 충돌한다.

이 스크립트가 하는 일 (요청하신 1~5번에 그대로 대응)

  [2] lift_holder : kinematic 끄고 질량 부여, 카터 섀시와 FixedJoint (프레임 재계산)
  [3] lift_moveparts_1 : 손으로 준 이동·회전 제거 → 프리즈매틱 축과 정렬,
                         가동 범위를 마스트 폴리곤 기준 레이캐스트로 다시 계산
  [4] m0609 : root_joint 비활성(월드 고정 해제) + 캐리지 상판에 안착 +
              캐리지와 FixedJoint (프레임 재계산)
  [1] 카터 : 바퀴 Drive 목표속도 0 으로 (사용자가 조종할 때까지 가만히)
  [5] 결과적으로 아티큘레이션 1개(DOF 14) → 팔 6축을 독립적으로 명령 가능

실행
    ~/isaacsim/python.sh 10_fix_lift_test.py
  (GUI 에서는 사본 파일을 연 뒤 Script Editor 에 붙여넣어도 된다.
   그 경우 SRC/DST 는 무시하고 열려 있는 스테이지를 고친다.)
"""

import os
import shutil

import omni.usd
from pxr import Usd, UsdGeom, UsdPhysics, PhysxSchema, Sdf, Gf


ROBOTS = os.path.expanduser(
    "~/ROKEY_P3_A1/cobot3_ws/isaacpjt/smart_farm/assets/robots/Collected_m0609_folk"
)
SRC = os.path.join(ROBOTS, "Collected_m0609_folk_lift_test_3.usd")
DST = os.path.join(ROBOTS, "Collected_m0609_folk_lift_test_4_fixed.usd")

HOLDER_MASS = 10.0
MOVER_MASS = 5.0
MIN_LINK_MASS = 0.3

LIFT_DRIVE_TYPE = "acceleration"   # force 는 하중만큼 처진다
LIFT_STIFFNESS = 1.0e5
LIFT_DAMPING = 1.0e4
LIFT_MAX_FORCE = 1.0e4
MAX_JOINT_VELOCITY = 0.5           # m/s (물리 쪽 상한)
DEFAULT_SPEED = 0.08               # m/s (조작 패널 기본값)

TRAVEL_MARGIN = 0.005
TRAVEL_GRID = 0.005
SOLVER_POSITION_ITERATIONS = 32
SOLVER_VELOCITY_ITERATIONS = 1

STOP_WHEELS = True                 # 바퀴 목표속도 0 으로 (사용자가 조종)
HULL_SPREAD_LIMIT = 0.25


# ── 도구 ─────────────────────────────────────────────────
def _find(stage, name, type_name=None):
    for prim in stage.Traverse():
        if prim.GetName() == name and (type_name is None or prim.GetTypeName() == type_name):
            return prim
    return None


def _find_under(root, name):
    for prim in Usd.PrimRange(root, Usd.PrimAllPrimsPredicate):
        if prim.GetName() == name:
            return prim
    return None


def _clear_xform_overrides(layer, path):
    spec = layer.GetPrimAtPath(path)
    if spec is None:
        return []
    removed = []
    for name in list(spec.properties.keys()):
        if name.startswith("xformOp"):
            spec.RemoveProperty(spec.properties[name])
            removed.append(name)
    return removed


def _triangles(root_prim):
    cache = UsdGeom.XformCache()
    out = []
    for prim in Usd.PrimRange(root_prim, Usd.TraverseInstanceProxies(Usd.PrimAllPrimsPredicate)):
        if not prim.IsA(UsdGeom.Mesh):
            continue
        mesh = UsdGeom.Mesh(prim)
        pts = mesh.GetPointsAttr().Get()
        counts = mesh.GetFaceVertexCountsAttr().Get()
        idx = mesh.GetFaceVertexIndicesAttr().Get()
        if not pts or not counts:
            continue
        mat = cache.GetLocalToWorldTransform(prim)
        w = [mat.Transform(Gf.Vec3d(v)) for v in pts]
        o = 0
        for c in counts:
            f = idx[o:o + c]
            for k in range(1, c - 1):
                a, b, d = w[f[0]], w[f[k]], w[f[k + 1]]
                out.append((a, b, d,
                            min(a[0], b[0], d[0]), max(a[0], b[0], d[0]),
                            min(a[1], b[1], d[1]), max(a[1], b[1], d[1])))
            o += c
    return out


def _hits(tris, x, y):
    res = []
    for a, b, c, x0, x1, y0, y1 in tris:
        if x < x0 or x > x1 or y < y0 or y > y1:
            continue
        ax, ay = a[0] - x, a[1] - y
        bx, by = b[0] - x, b[1] - y
        cx, cy = c[0] - x, c[1] - y
        d1 = ax * by - ay * bx
        d2 = bx * cy - by * cx
        d3 = cx * ay - cy * ax
        if (d1 > 0 and d2 > 0 and d3 > 0) or (d1 < 0 and d2 < 0 and d3 < 0):
            tot = d1 + d2 + d3
            if abs(tot) > 1e-12:
                res.append((d2 * a[2] + d3 * b[2] + d1 * c[2]) / tot)
    res.sort()
    return res


def _set_xform(prim, translate, quat):
    xf = UsdGeom.Xformable(prim)
    existing = prim.GetAttribute("xformOp:orient")
    use_float = bool(existing) and existing.GetTypeName() == Sdf.ValueTypeNames.Quatf
    xf.ClearXformOpOrder()
    xf.AddTranslateOp().Set(Gf.Vec3d(translate))
    if use_float:
        xf.AddOrientOp(UsdGeom.XformOp.PrecisionFloat).Set(
            Gf.Quatf(float(quat.GetReal()), Gf.Vec3f(quat.GetImaginary())))
    else:
        xf.AddOrientOp(UsdGeom.XformOp.PrecisionDouble).Set(Gf.Quatd(quat))


def _enable_joint(joint_prim):
    """조인트를 켜고 아티큘레이션에 포함시킨다.

    lift_test_3.usd 의 결정적 문제:
      physics:jointEnabled = 0            → 조인트가 꺼져 있어 아무것도 안 붙는다
      physics:excludeFromArticulation = 1 → 켜도 아티큘레이션에 안 들어간다
                                            (리프트·팔이 카터와 별개 물체가 된다)
    """
    j = UsdPhysics.Joint(joint_prim)
    j.CreateJointEnabledAttr().Set(True)
    j.CreateExcludeFromArticulationAttr().Set(False)


def _joint_frames(joint_prim, body0, body1):
    cache = UsdGeom.XformCache()
    rel = Gf.Transform(cache.GetLocalToWorldTransform(body1)
                       * cache.GetLocalToWorldTransform(body0).GetInverse())
    j = UsdPhysics.Joint(joint_prim)
    j.CreateBody0Rel().SetTargets([body0.GetPath()])
    j.CreateBody1Rel().SetTargets([body1.GetPath()])
    j.CreateLocalPos0Attr().Set(Gf.Vec3f(rel.GetTranslation()))
    j.CreateLocalRot0Attr().Set(Gf.Quatf(rel.GetRotation().GetQuat()))
    j.CreateLocalPos1Attr().Set(Gf.Vec3f(0, 0, 0))
    j.CreateLocalRot1Attr().Set(Gf.Quatf(1, 0, 0, 0))


# ── 본체 ─────────────────────────────────────────────────
def _make_editable(stage, path):
    """instanceable 참조 안에 있는 프림을 이 파일 안에서 고칠 수 있게 만든다.

    원본 에셋(공용)을 건드리지 않으려고, 사본 쪽에서 인스턴스를 풀어 준다.
    """
    prim = stage.GetPrimAtPath(path)
    if prim and not prim.IsInstanceProxy():
        return prim
    chain = []
    p = stage.GetPrimAtPath(path)
    while p and p.IsValid() and not p.IsPseudoRoot():
        chain.append(p.GetPath())
        p = p.GetParent()
    for pp in reversed(chain):
        q = stage.GetPrimAtPath(pp)
        if q and q.IsValid() and q.IsInstance():
            q.SetInstanceable(False)
    return stage.GetPrimAtPath(path)


def main():
    ctx = omni.usd.get_context()
    stage = ctx.get_stage()
    opened_here = False
    if stage is None or _find(stage, "lift_prismatic_joint", "PhysicsPrismaticJoint") is None:
        if not os.path.exists(SRC):
            raise RuntimeError("원본이 없습니다: %s" % SRC)
        shutil.copy2(SRC, DST)
        print("[fix] 사본 생성: %s" % DST)
        ctx.open_stage(DST)
        stage = ctx.get_stage()
        opened_here = True

    layer = stage.GetRootLayer()
    joint = _find(stage, "lift_prismatic_joint", "PhysicsPrismaticJoint")
    if joint is None:
        raise RuntimeError("lift_prismatic_joint 를 못 찾았습니다.")
    lift = joint.GetParent()
    holder = _find_under(lift, "lift_holder")
    mover = _find_under(lift, "lift_moveparts_1")
    chassis = _find(stage, "chassis_link")
    arm = None
    for prim in stage.Traverse():
        if prim.GetName().startswith("m0609") and _find_under(prim, "base_link") is not None:
            arm = prim
            break
    arm_base = _find_under(arm, "base_link") if arm else None
    for nm, pr in (("chassis_link", chassis), ("lift_holder", holder),
                   ("lift_moveparts_1", mover), ("m0609 base_link", arm_base)):
        if pr is None:
            raise RuntimeError("%s 를 못 찾았습니다." % nm)
    print("[fix] 섀시 %s" % chassis.GetPath())
    print("[fix] 리프트 %s / 팔 %s" % (lift.GetPath(), arm.GetPath()))

    # 1) 팔의 월드 고정 해제 --------------------------------------------
    n = 0
    for prim in Usd.PrimRange(arm, Usd.PrimAllPrimsPredicate):
        if prim.GetTypeName() == "PhysicsFixedJoint":
            j = UsdPhysics.Joint(prim)
            if not j.GetBody0Rel().GetTargets():        # body0 없음 = 월드 고정
                prim.SetActive(False)
                print("[fix] 1) 월드 고정 조인트 비활성: %s" % prim.GetPath())
                n += 1
    if n == 0:
        print("[fix] 1) 월드 고정 조인트 없음 (이미 해제됨)")

    # 2) 마스트: kinematic 끄고 질량 ------------------------------------
    UsdPhysics.RigidBodyAPI(holder).CreateKinematicEnabledAttr().Set(False)
    UsdPhysics.MassAPI.Apply(holder).CreateMassAttr().Set(HOLDER_MASS)
    UsdPhysics.MassAPI.Apply(mover).CreateMassAttr().Set(MOVER_MASS)
    print("[fix] 2) lift_holder kinematic=False, 질량 %.1f / 캐리지 %.1f kg"
          % (HOLDER_MASS, MOVER_MASS))

    # 3) 캐리지에 손으로 준 이동·회전 제거 → 프리즈매틱 축과 정렬 --------
    cleared = _clear_xform_overrides(layer, mover.GetPath())
    for child in mover.GetChildren():
        cleared += _clear_xform_overrides(layer, child.GetPath())
    print("[fix] 3) 캐리지 xform 오버라이드 제거: %s" % (cleared or "없음"))

    # 4) 팔을 캐리지 상판에 안착 ----------------------------------------
    cache = UsdGeom.XformCache()
    mover_tris = _triangles(mover)
    lift_w = cache.GetLocalToWorldTransform(lift)
    axis = Gf.Transform(lift_w).GetTranslation()
    tops = []
    for dx in (-0.06, -0.03, 0.0, 0.03, 0.06):
        for dy in (-0.06, -0.03, 0.0, 0.03, 0.06):
            h = _hits(mover_tris, axis[0] + dx, axis[1] + dy)
            if h:
                tops.append(max(h))
    if not tops:
        raise RuntimeError("캐리지 상판을 못 찾았습니다 (팔 축 아래가 비어 있음).")
    plate_top = max(tops)

    target = Gf.Matrix4d(lift_w)
    target.SetTranslateOnly(Gf.Vec3d(axis[0], axis[1], plate_top))
    base_in_arm = (cache.GetLocalToWorldTransform(arm_base)
                   * cache.GetLocalToWorldTransform(arm).GetInverse())
    arm_world = base_in_arm.GetInverse() * target
    # xform 은 **부모 기준**이다. 부모(/Nova_Carter 등)가 원점이 아니면
    # 월드 행렬을 그대로 쓰면 그만큼 어긋난다.
    parent_world = cache.GetLocalToWorldTransform(arm.GetParent())
    arm_t = Gf.Transform(arm_world * parent_world.GetInverse())
    _set_xform(arm, arm_t.GetTranslation(), arm_t.GetRotation().GetQuat())
    print("[fix] 4) 캐리지 상판 z %.5f 에 팔 플랜지 안착" % plate_top)

    # 5) 조인트 프레임 재계산 (스냅 제거) --------------------------------
    cache.Clear()
    holder_joint = None
    arm_joint = None
    for prim in stage.Traverse():
        if prim.GetTypeName() != "PhysicsFixedJoint" or not prim.IsActive():
            continue
        j = UsdPhysics.Joint(prim)
        b0 = j.GetBody0Rel().GetTargets()
        b1 = j.GetBody1Rel().GetTargets()
        if not b0 or not b1:
            continue
        pair = {b0[0], b1[0]}
        if pair == {chassis.GetPath(), holder.GetPath()}:
            holder_joint = prim
        elif pair == {mover.GetPath(), arm_base.GetPath()}:
            arm_joint = prim
    if holder_joint is None:
        holder_joint = UsdPhysics.FixedJoint.Define(
            stage, lift.GetPath().AppendChild("chassis_to_lift_holder")).GetPrim()
        print("[fix] 5) 섀시<->마스트 FixedJoint 새로 생성")
    if arm_joint is None:
        arm_joint = UsdPhysics.FixedJoint.Define(
            stage, lift.GetPath().AppendChild("mover_to_m0609")).GetPrim()
        print("[fix] 5) 캐리지<->팔 FixedJoint 새로 생성")
    _joint_frames(holder_joint, chassis, holder)
    _joint_frames(arm_joint, mover, arm_base)
    for jp in (holder_joint, arm_joint, joint):
        j = UsdPhysics.Joint(jp)
        was = (j.GetJointEnabledAttr().Get(), j.GetExcludeFromArticulationAttr().Get())
        _enable_joint(jp)
        print("        %s: jointEnabled %s -> True, excludeFromArticulation %s -> False"
              % (jp.GetName(), was[0], was[1]))
    print("[fix] 5) FixedJoint 프레임 재계산: %s, %s"
          % (holder_joint.GetName(), arm_joint.GetName()))

    # 6) 가동 범위 재측정 + Drive ---------------------------------------
    cache.Clear()
    holder_tris = _triangles(holder)
    mover_tris = _triangles(mover)
    xs = [t[i][0] for t in mover_tris for i in range(3)]
    ys = [t[i][1] for t in mover_tris for i in range(3)]
    top_all = max(t[i][2] for t in mover_tris for i in range(3))
    bot_all = min(t[i][2] for t in mover_tris for i in range(3))
    up = down = float("inf")
    cols = overlap = 0
    x = min(xs) + TRAVEL_GRID / 2
    while x < max(xs):
        y = min(ys) + TRAVEL_GRID / 2
        while y < max(ys):
            mh = _hits(mover_tris, x, y)
            if len(mh) >= 2:
                cols += 1
                top, bot = mh[-1], mh[0]
                hh = _hits(holder_tris, x, y)
                above = [z for z in hh if z > top + 1e-5]
                below = [z for z in hh if z < bot - 1e-5]
                if any(bot - 1e-5 <= z <= top + 1e-5 for z in hh):
                    overlap += 1
                if above:
                    up = min(up, min(above) - top, min(above) - top_all)
                if below:
                    down = min(down, bot - max(below), bot_all - max(below))
            y += TRAVEL_GRID
        x += TRAVEL_GRID
    safe_upper = up - TRAVEL_MARGIN
    safe_lower = -(down - TRAVEL_MARGIN)

    pj = UsdPhysics.PrismaticJoint(joint)
    pj.CreateLowerLimitAttr().Set(0.0)
    pj.CreateUpperLimitAttr().Set(float(safe_upper))
    drive = UsdPhysics.DriveAPI.Apply(joint, "linear")
    drive.CreateTypeAttr().Set(LIFT_DRIVE_TYPE)
    drive.CreateTargetPositionAttr().Set(0.0)
    drive.CreateTargetVelocityAttr().Set(0.0)
    drive.CreateStiffnessAttr().Set(LIFT_STIFFNESS)
    drive.CreateDampingAttr().Set(LIFT_DAMPING)
    drive.CreateMaxForceAttr().Set(LIFT_MAX_FORCE)
    joint.CreateAttribute("lift:safeLower", Sdf.ValueTypeNames.Float).Set(float(safe_lower))
    joint.CreateAttribute("lift:safeUpper", Sdf.ValueTypeNames.Float).Set(float(safe_upper))
    joint.CreateAttribute("lift:defaultSpeed", Sdf.ValueTypeNames.Float).Set(float(DEFAULT_SPEED))
    PhysxSchema.PhysxJointAPI.Apply(joint).CreateMaxJointVelocityAttr().Set(float(MAX_JOINT_VELOCITY))
    print("[fix] 6) 칼럼 %d (마스트와 겹친 칼럼 %d)" % (cols, overlap))
    print("        폴리곤에 닿기까지 위 %.4f / 아래 %.4f → 가동 범위 0.0000 ~ %.4f"
          % (up, down, safe_upper))
    print("        Drive %s, target 0, k=%g c=%g maxF=%g, 속도 상한 %.2f m/s"
          % (LIFT_DRIVE_TYPE, LIFT_STIFFNESS, LIFT_DAMPING, LIFT_MAX_FORCE, MAX_JOINT_VELOCITY))

    # 7) 바퀴 목표속도 0 (사용자가 조종할 때까지 가만히) -----------------
    if STOP_WHEELS:
        for prim in Usd.PrimRange(chassis.GetParent(), Usd.PrimAllPrimsPredicate):
            if prim.GetTypeName() != "PhysicsRevoluteJoint":
                continue
            if not prim.HasAPI(UsdPhysics.DriveAPI, "angular"):
                continue
            d = UsdPhysics.DriveAPI(prim, "angular")
            v = d.GetTargetVelocityAttr().Get()
            if v:
                d.GetTargetVelocityAttr().Set(0.0)
                print("[fix] 7) %s 목표속도 %.2f -> 0" % (prim.GetName(), v))

    # 8) 콜라이더 --------------------------------------------------------
    bbox = UsdGeom.BBoxCache(
        Usd.TimeCode.Default(),
        [UsdGeom.Tokens.default_, UsdGeom.Tokens.render, UsdGeom.Tokens.guide, UsdGeom.Tokens.proxy],
    )
    # 먼저 훑어서 후보만 모으고(순회 중에 인스턴스를 풀면 이터레이터가 깨진다),
    # 그 다음에 하나씩 고친다.
    candidates = []
    for prim in Usd.PrimRange(stage.GetPseudoRoot(),
                              Usd.TraverseInstanceProxies(Usd.PrimAllPrimsPredicate)):
        if not prim.HasAPI(UsdPhysics.CollisionAPI):
            continue
        path = str(prim.GetPath())
        if "/chassis_link" in path or "/wheel_" in path or "/caster_" in path:
            continue                      # 카터 원본은 건드리지 않는다
        approx = (UsdPhysics.MeshCollisionAPI(prim).GetApproximationAttr().Get()
                  if prim.HasAPI(UsdPhysics.MeshCollisionAPI) else None)
        r = bbox.ComputeWorldBound(prim).ComputeAlignedRange()
        if r.IsEmpty():
            continue
        size = r.GetSize()
        spread = max(size[0], size[1], size[2])
        if approx == "convexHull" and spread > HULL_SPREAD_LIMIT:
            candidates.append((prim.GetPath(), "convexDecomposition",
                               "convexHull -> convexDecomposition", spread))
        elif approx == "none":
            candidates.append((prim.GetPath(), "convexHull", "none -> convexHull", spread))

    fixed_cols = []
    for path, want, what, spread in candidates:
        target_prim = _make_editable(stage, path)
        if target_prim and target_prim.IsValid() and not target_prim.IsInstanceProxy():
            UsdPhysics.MeshCollisionAPI.Apply(target_prim).CreateApproximationAttr().Set(want)
            fixed_cols.append((str(path), what, spread))
        else:
            fixed_cols.append((str(path), what + " (실패: 인스턴스 해제 불가)", spread))
    print("[fix] 8) 콜라이더 %d 개 수정" % len(fixed_cols))
    for path, what, spread in fixed_cols:
        print("        %s  (%s, 퍼짐 %.3f m)" % (path, what, spread))

    # 9) 가벼운 링크 질량 + 솔버 -----------------------------------------
    for prim in Usd.PrimRange(arm, Usd.PrimAllPrimsPredicate):
        if not prim.HasAPI(UsdPhysics.RigidBodyAPI):
            continue
        m = UsdPhysics.MassAPI(prim).GetMassAttr().Get()
        if m is not None and m < MIN_LINK_MASS:
            if any(q.HasAPI(UsdPhysics.CollisionAPI) for q in Usd.PrimRange(prim)):
                UsdPhysics.MassAPI.Apply(prim).CreateMassAttr().Set(MIN_LINK_MASS)
                print("[fix] 9) %s 질량 %.6f -> %.2f kg" % (prim.GetName(), m, MIN_LINK_MASS))
    arti = PhysxSchema.PhysxArticulationAPI.Apply(chassis)
    arti.CreateSolverPositionIterationCountAttr().Set(SOLVER_POSITION_ITERATIONS)
    arti.CreateSolverVelocityIterationCountAttr().Set(SOLVER_VELOCITY_ITERATIONS)

    layer.Save()
    print("\n[fix] 저장: %s" % layer.identifier)
    print("다음: 이 파일을 열고 Play → 11_control_panel.py 를 Script Editor 에 붙여넣기")
    return layer.identifier


main()
