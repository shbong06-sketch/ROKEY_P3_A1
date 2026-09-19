"""
AMR + 리프트 + M0609 리그(rig) USD 를 새로 만듭니다.

이 스크립트는 **현재 열려 있는 장면을 건드리지 않습니다.**
디스크에 새 USD 파일 하나(amr_lift_rig.usd)를 쓸 뿐입니다.

만들어지는 구조 (PhysX 아티큘레이션 1개로 통합):

    /amr_lift_rig                     (defaultPrim, 이것만 옮기면 전체가 따라감)
      Nova_Carter                     ref -> nova_carter.usd
        chassis_link                  ArticulationRootAPI (리그 전체의 뿌리)
      lift                            ref -> lift_v3_physics.usdc
        lift_holder                   마스트(고정부)  - 카터에 FixedJoint 로 결합
        lift_moveparts_1              캐리지(승강부)  - PrismaticJoint 로 상하 이동
          M0609_Mount                 팔이 붙는 자리
      m0609_with_fork                 ref -> m0609_with_fork.usd
        root_joint                    active=false (월드 고정 해제)
      Joints
        chassis_to_lift_holder        FixedJoint : 카터 섀시 <-> 마스트
        mover_to_m0609                FixedJoint : 캐리지 <-> 팔 base_link

왜 이 구조인가
  - 조인트로 이어진 바디는 PhysX 가 같은 아티큘레이션으로 묶습니다.
    그래서 카터 바퀴 + 리프트 프리즈매틱 + 팔 6축이 **하나의 아티큘레이션**이 되고,
    리프트는 "붙어 있는 척"이 아니라 실제 관절로 카터에 물려 있습니다.
    (아티큘레이션이 두 개로 갈리면 그 사이 조인트는 물렁해져서 팔이 흔들립니다.)
  - RigidBody 를 RigidBody 밑으로 reparent 하지 않습니다. 중첩 RigidBody 는
    PhysX 에서 금지입니다. 계층은 그대로 두고 조인트로만 묶습니다.
  - 조인트의 local frame 을 '지금 놓여 있는 위치'에서 역산하므로 Play 를 눌러도
    부품이 순간이동(스냅)하지 않습니다.

실행 방법 (둘 중 아무거나)
  A) Isaac Sim 5.1 GUI > Window > Script Editor 에 붙여넣고 Ctrl+Enter
  B) 터미널:  ~/isaacsim/python.sh 02_build_rig.py
"""

import os
import math
import shutil

from pxr import Usd, UsdGeom, UsdPhysics, PhysxSchema, Sdf, Gf


# ── 경로 ─────────────────────────────────────────────────
SMART_FARM = os.path.expanduser(
    "~/ROKEY_P3_A1/cobot3_ws/isaacpjt/smart_farm"
)
SCENE_DIR = os.path.join(SMART_FARM, "scenes/demo_test")

CARTER_USD = (
    "https://omniverse-content-production.s3-us-west-2.amazonaws.com"
    "/Assets/Isaac/5.1/Isaac/Robots/NVIDIA/NovaCarter/nova_carter.usd"
)
LIFT_USD = os.path.join(SCENE_DIR, "lift_v3_physics.usdc")

# 2026-09-18 21:34 에 다시 콜렉트한 **포크 달린 M0609**.
# tool0 밑에 fork_tool_base 가 붙어 있고, 관절 Drive 게인도 URDF 값(266.7/26.7 …)과
# 일치한다. 예전 m0609_with_fork.usd 보다 이쪽이 최신이다.
ARM_USD = os.path.join(
    SMART_FARM, "assets/robots/Collected_m0609_folk/Collected_m0609_folk.usd"
)
ARM_PRIM = "/World/m0609"      # 이 에셋의 defaultPrim 은 /World (Environment·Render 포함)
                               # 라서 로봇 프림만 콕 집어 참조한다.

# 리그 폴더 안에 에셋 사본을 두어 폴더째 옮겨도 열리게 한다 (카터만 웹 참조).
STAGE_ASSETS = True

OUT_DIR = os.path.join(SMART_FARM, "assets/amr_lift_rig")
OUT_USD = os.path.join(OUT_DIR, "amr_lift_rig.usd")

RIG_ROOT = "/amr_lift_rig"


# ── 배치 값 (현재 demo_test 장면에서 측정한 값) ──────────
# 리프트를 카터 기준 어디에 놓을지. GUI 에서 눈으로 맞춘 뒤
# Property 패널의 Translate/Orient 를 여기에 옮겨 적으면 됩니다.
# 카터 섀시 외피의 xy 중심, 마스트 바닥이 상판(z 0.4799)에 얹히는 높이.
# 06_fit_rig.py 가 실측으로 다시 맞추므로 여기는 출발점일 뿐이다.
LIFT_TRANSLATE = (-0.2338, 0.0, 0.1450)
LIFT_ORIENT_WXYZ = (0.7071067811865476, 0.0, 0.0, 0.7071067811865475)   # Z축 90도

# 캐리지 위에서 팔을 몇 도 돌려 얹을지 (Z축, degree)
ARM_YAW_DEG = 0.0


# ── 물리 값 ──────────────────────────────────────────────
# 질량비가 크면(무거운 팔 : 가벼운 캐리지) 솔버가 불안정해집니다.
# 원본 에셋의 4.5 / 1.5 kg 은 28 kg 짜리 팔을 얹기엔 가벼워서 올려 잡습니다.
# 콜라이더를 달고 있는데 질량이 0 에 가까운 링크(tool0 등)는 솔버를 흔든다.
# 이보다 가벼우면 이 값으로 올린다.
MIN_LINK_MASS = 0.3

HOLDER_MASS = 10.0
MOVER_MASS = 5.0

# 리프트 Drive (위치 제어). 팔+캐리지 무게 약 33 kg → 중력 약 320 N.
LIFT_STIFFNESS = 1.0e5
LIFT_DAMPING = 1.0e4
LIFT_MAX_FORCE = 1.0e4

# "acceleration" 은 질량으로 정규화된 구동이라 같은 stiffness 로도 안정적이고
# 처짐(sag)이 거의 없습니다. "force" 로 두면 하중만큼 내려앉습니다.
LIFT_DRIVE_TYPE = "acceleration"

# 캐리지가 마스트 폴리곤에 닿지 않도록 남길 여유 (m)
TRAVEL_MARGIN = 0.005

# 행정 계산용 격자 (m). 작을수록 정확하고 느립니다.
TRAVEL_GRID = 0.005

# 기본 가동 범위 (m). 안전 범위 안으로 자동으로 잘립니다.
# None 이면 안전 범위 끝까지 씁니다. 사용자는 조작 패널에서 언제든 바꿉니다.
DEFAULT_LOWER = 0.0
DEFAULT_UPPER = None

# 기본 승강 속도 (m/s). 조작 패널의 초기값으로 USD 에 같이 저장됩니다.
DEFAULT_SPEED = 0.08

# 물리 쪽 속도 상한 (m/s). 조작 패널 속도는 이 값을 넘지 못합니다.
MAX_JOINT_VELOCITY = 0.5

# 팔이 무거우므로 솔버 반복을 올립니다.
SOLVER_POSITION_ITERATIONS = 32
SOLVER_VELOCITY_ITERATIONS = 1


# ── 도우미 ───────────────────────────────────────────────
def _rel(path, start_dir):
    """OUT_USD 기준 상대 경로. http(s) 는 그대로 둡니다."""
    if path.startswith("http://") or path.startswith("https://") or path.startswith("omniverse://"):
        return path
    return os.path.relpath(path, start_dir)


def _set_xform(prim, translate=None, quat=None):
    """xformOp 를 translate + orient 로 다시 쓴다.

    참조해 온 프림에 이미 quatf 로 된 orient 가 있으면 그 정밀도를 그대로 써야 한다
    (double 로 다시 만들려고 하면 USD 가 타입 충돌로 거부한다).
    """
    xf = UsdGeom.Xformable(prim)
    existing = prim.GetAttribute("xformOp:orient")
    use_float = bool(existing) and existing.GetTypeName() == Sdf.ValueTypeNames.Quatf
    xf.ClearXformOpOrder()
    if translate is not None:
        xf.AddTranslateOp().Set(Gf.Vec3d(translate))
    if quat is not None:
        if not isinstance(quat, (Gf.Quatd, Gf.Quatf)):
            w, x, y, z = quat
            quat = Gf.Quatd(w, Gf.Vec3d(x, y, z))
        if use_float:
            xf.AddOrientOp(UsdGeom.XformOp.PrecisionFloat).Set(
                Gf.Quatf(float(quat.GetReal()), Gf.Vec3f(quat.GetImaginary()))
            )
        else:
            xf.AddOrientOp(UsdGeom.XformOp.PrecisionDouble).Set(Gf.Quatd(quat))

def _relative_transform(cache, parent_prim, child_prim):
    """parent 프레임에서 본 child 의 (위치, 회전). 조인트 frame 계산용."""
    parent_w = cache.GetLocalToWorldTransform(parent_prim)
    child_w = cache.GetLocalToWorldTransform(child_prim)
    rel = child_w * parent_w.GetInverse()
    t = Gf.Transform(rel)
    return t.GetTranslation(), t.GetRotation().GetQuat()


def _define_fixed_joint(stage, path, body0, body1, cache):
    """body0 프레임 기준으로 body1 을 '지금 그 자리에' 고정하는 FixedJoint."""
    joint = UsdPhysics.FixedJoint.Define(stage, path)
    joint.CreateBody0Rel().SetTargets([body0.GetPath()])
    joint.CreateBody1Rel().SetTargets([body1.GetPath()])

    pos, quat = _relative_transform(cache, body0, body1)
    joint.CreateLocalPos0Attr().Set(Gf.Vec3f(pos))
    joint.CreateLocalRot0Attr().Set(Gf.Quatf(quat))
    joint.CreateLocalPos1Attr().Set(Gf.Vec3f(0.0, 0.0, 0.0))
    joint.CreateLocalRot1Attr().Set(Gf.Quatf(1.0, 0.0, 0.0, 0.0))
    return joint


def _triangles(stage, root_path):
    """월드 좌표 삼각형 + 각 삼각형의 xy 바운딩 (레이캐스트 가속용)."""
    cache = UsdGeom.XformCache()
    tris = []
    root = stage.GetPrimAtPath(root_path)
    for prim in Usd.PrimRange(root, Usd.TraverseInstanceProxies(Usd.PrimAllPrimsPredicate)):
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
            face = idx[o:o + c]
            for k in range(1, c - 1):
                a, b, d = w[face[0]], w[face[k]], w[face[k + 1]]
                tris.append((
                    a, b, d,
                    min(a[0], b[0], d[0]), max(a[0], b[0], d[0]),
                    min(a[1], b[1], d[1]), max(a[1], b[1], d[1]),
                ))
            o += c
    return tris


def _hits_z(tris, x, y):
    """(x, y) 를 지나는 수직선이 만나는 z 값들 (오름차순)."""
    out = []
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
                out.append((d2 * a[2] + d3 * b[2] + d1 * c[2]) / tot)
    out.sort()
    return out


def _travel_room(stage, holder_path, mover_path, grid=TRAVEL_GRID):
    """캐리지가 마스트 폴리곤에 닿기까지 위/아래로 갈 수 있는 거리.

    캐리지 바닥면 위의 격자마다 수직선을 쏘아, 그 선 위에서 캐리지 바로 위(아래)에
    있는 마스트 면까지의 거리를 재고 그 최솟값을 취한다. 바운딩 박스로 재면
    마스트 맨 위 가로보를 놓쳐서 캐리지가 보를 뚫고 올라간다.
    """
    holder = _triangles(stage, holder_path)
    mover = _triangles(stage, mover_path)
    xs = [t[i][0] for t in mover for i in range(3)]
    ys = [t[i][1] for t in mover for i in range(3)]

    mover_top_all = max(t[i][2] for t in mover for i in range(3))
    mover_bot_all = min(t[i][2] for t in mover for i in range(3))
    up = down = float("inf")
    overlap = 0
    columns = 0
    x = min(xs) + grid / 2
    while x < max(xs):
        y = min(ys) + grid / 2
        while y < max(ys):
            mh = _hits_z(mover, x, y)
            if len(mh) >= 2:
                columns += 1
                top, bot = mh[-1], mh[0]
                hh = _hits_z(holder, x, y)
                above = [z for z in hh if z > top + 1e-5]
                below = [z for z in hh if z < bot - 1e-5]
                if any(bot - 1e-5 <= z <= top + 1e-5 for z in hh):
                    overlap += 1
                if above:
                    up = min(up, min(above) - top, min(above) - mover_top_all)
                if below:
                    down = min(down, bot - max(below), mover_bot_all - max(below))
            y += grid
        x += grid
    return up, down, columns, overlap


# ── 본체 ─────────────────────────────────────────────────
def build():
    for src in (LIFT_USD, ARM_USD):
        if not os.path.exists(src):
            raise RuntimeError("소스 에셋이 없습니다: %s" % src)

    os.makedirs(OUT_DIR, exist_ok=True)
    if os.path.exists(OUT_USD):
        os.remove(OUT_USD)          # 이 파일은 이 스크립트가 만드는 산출물입니다.

    lift_src, arm_src = LIFT_USD, ARM_USD
    if STAGE_ASSETS:
        staged = os.path.join(OUT_DIR, "assets")
        os.makedirs(staged, exist_ok=True)
        lift_src = os.path.join(staged, os.path.basename(LIFT_USD))
        shutil.copy2(LIFT_USD, lift_src)
        arm_dir_src = os.path.dirname(ARM_USD)
        arm_dir_dst = os.path.join(staged, os.path.basename(arm_dir_src))
        shutil.copytree(arm_dir_src, arm_dir_dst, dirs_exist_ok=True)
        arm_src = os.path.join(arm_dir_dst, os.path.basename(ARM_USD))
        print("[build] 에셋 사본: %s, %s" % (lift_src, arm_dir_dst))

    stage = Usd.Stage.CreateNew(OUT_USD)
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)

    rig = UsdGeom.Xform.Define(stage, RIG_ROOT)
    stage.SetDefaultPrim(rig.GetPrim())

    # 1) 세 에셋을 참조로 얹습니다 (복사가 아니라 참조 → 원본 수정이 그대로 반영).
    carter = UsdGeom.Xform.Define(stage, RIG_ROOT + "/Nova_Carter")
    carter.GetPrim().GetReferences().AddReference(_rel(CARTER_USD, OUT_DIR))

    lift = UsdGeom.Xform.Define(stage, RIG_ROOT + "/lift")
    lift.GetPrim().GetReferences().AddReference(_rel(lift_src, OUT_DIR))
    _set_xform(lift.GetPrim(), LIFT_TRANSLATE, LIFT_ORIENT_WXYZ)

    arm = UsdGeom.Xform.Define(stage, RIG_ROOT + "/m0609_with_fork")
    arm.GetPrim().GetReferences().AddReference(_rel(arm_src, OUT_DIR), ARM_PRIM)

    chassis_path = RIG_ROOT + "/Nova_Carter/chassis_link"
    holder_path = RIG_ROOT + "/lift/lift_holder"
    mover_path = RIG_ROOT + "/lift/lift_moveparts_1"
    mount_path = mover_path + "/M0609_Mount"
    joint_path = RIG_ROOT + "/lift/lift_prismatic_joint"
    arm_base_path = None
    for prim in Usd.PrimRange(arm.GetPrim()):
        if prim.GetName() == "base_link":
            arm_base_path = str(prim.GetPath())
            break
    if arm_base_path is None:
        raise RuntimeError("팔 에셋에서 base_link 를 못 찾았습니다.")

    chassis = stage.GetPrimAtPath(chassis_path)
    if not chassis.IsValid():
        raise RuntimeError(
            "chassis_link 를 못 찾았습니다. Nova Carter 에셋을 못 받아온 것 같습니다 "
            "(인터넷 또는 Isaac 에셋 경로 확인)."
        )

    holder = stage.GetPrimAtPath(holder_path)
    mover = stage.GetPrimAtPath(mover_path)
    mount = stage.GetPrimAtPath(mount_path)
    arm_base = stage.GetPrimAtPath(arm_base_path)
    for name, prim in (("lift_holder", holder), ("lift_moveparts_1", mover),
                       ("M0609_Mount", mount), ("base_link", arm_base)):
        if not prim.IsValid():
            raise RuntimeError("%s 를 못 찾았습니다." % name)

    # 2) 리프트 물리 정리 ------------------------------------------------
    #    마스트는 카터에 조인트로 물릴 것이므로 kinematic 을 끕니다.
    #    (kinematic 바디는 물리로 안 움직이고, 아티큘레이션에도 못 들어갑니다.)
    holder_rb = UsdPhysics.RigidBodyAPI(holder)
    holder_rb.CreateKinematicEnabledAttr().Set(False)
    UsdPhysics.MassAPI.Apply(holder).CreateMassAttr().Set(HOLDER_MASS)

    UsdPhysics.MassAPI.Apply(mover).CreateMassAttr().Set(MOVER_MASS)

    #    속이 빈 마스트를 convexHull 로 잡으면 꽉 찬 상자가 되어 캐리지와
    #    영구 충돌합니다. convexDecomposition 으로 바꿉니다.
    for mesh_path in (holder_path + "/Mesh", mover_path + "/Mesh"):
        mesh = stage.GetPrimAtPath(mesh_path)
        if mesh.IsValid():
            UsdPhysics.MeshCollisionAPI.Apply(mesh).CreateApproximationAttr().Set(
                "convexDecomposition"
            )

    # 3) 팔의 '월드 고정' 해제 -------------------------------------------
    #    root_joint 는 body0 가 비어 있는 FixedJoint(=월드에 못박기) 이고
    #    ArticulationRootAPI 도 여기 붙어 있습니다. 통째로 끕니다.
    for prim in Usd.PrimRange(arm.GetPrim()):
        if prim.GetName() == "root_joint" and prim.GetTypeName() == "PhysicsFixedJoint":
            prim.SetActive(False)
            print("[build] m0609 root_joint 비활성화 (월드 고정 해제): %s" % prim.GetPath())
            break

    #    콜라이더를 단 채 질량이 0 에 가까운 링크(tool0 = 포크가 붙는 자리)는 올려 준다.
    for prim in Usd.PrimRange(arm.GetPrim()):
        if not prim.HasAPI(UsdPhysics.RigidBodyAPI):
            continue
        mass_api = UsdPhysics.MassAPI(prim)
        m = mass_api.GetMassAttr().Get()
        if m is not None and m < MIN_LINK_MASS:
            has_col = any(q.HasAPI(UsdPhysics.CollisionAPI) for q in Usd.PrimRange(prim))
            if has_col:
                UsdPhysics.MassAPI.Apply(prim).CreateMassAttr().Set(MIN_LINK_MASS)
                print("[build] %s 질량 %.6f -> %.2f kg (콜라이더가 달린 링크)"
                      % (prim.GetName(), m, MIN_LINK_MASS))

    #    리프트 에셋 안에 예전에 만들어 둔 M0609_FixedJoint 가 남아 있습니다.
    #    body1 이 지금은 없는 /World/m0609/base_link 를 가리켜서
    #    "disjointed body transforms" 경고와 함께 물체를 끌어당깁니다. 끕니다.
    stale = stage.GetPrimAtPath(mover_path + "/M0609_FixedJoint")
    if stale.IsValid():
        stale.SetActive(False)
        print("[build] 리프트 에셋의 낡은 M0609_FixedJoint 비활성화")

    # 4) 팔을 캐리지의 M0609_Mount 자리에 놓습니다 ------------------------
    cache = UsdGeom.XformCache()
    mount_world = cache.GetLocalToWorldTransform(mount)
    yaw = Gf.Matrix4d().SetRotate(
        Gf.Rotation(Gf.Vec3d(0, 0, 1), ARM_YAW_DEG)
    )
    target = yaw * mount_world          # base_link 가 놓여야 할 프레임
    # 에셋에 따라 팔 프림과 base_link 가 같은 자리가 아니므로 역산해서 배치한다.
    base_in_arm = (cache.GetLocalToWorldTransform(arm_base)
                   * cache.GetLocalToWorldTransform(arm.GetPrim()).GetInverse())
    arm_t = Gf.Transform(base_in_arm.GetInverse() * target)
    _set_xform(arm.GetPrim(), tuple(arm_t.GetTranslation()), arm_t.GetRotation().GetQuat())
    cache.Clear()

    # 5) 조인트 두 개로 묶습니다 ------------------------------------------
    UsdGeom.Scope.Define(stage, RIG_ROOT + "/Joints")
    _define_fixed_joint(
        stage, RIG_ROOT + "/Joints/chassis_to_lift_holder", chassis, holder, cache
    )
    _define_fixed_joint(
        stage, RIG_ROOT + "/Joints/mover_to_m0609", mover, arm_base, cache
    )
    print("[build] FixedJoint 2개 생성: 섀시<->마스트, 캐리지<->팔")

    # 6) 프리즈매틱 조인트: 가동 범위를 실제 메시에서 계산 ----------------
    up_raw, down_raw, columns, overlap = _travel_room(stage, holder_path, mover_path)
    up_room = up_raw - TRAVEL_MARGIN
    down_room = down_raw - TRAVEL_MARGIN

    joint_prim = stage.GetPrimAtPath(joint_path)
    if not joint_prim.IsValid():
        raise RuntimeError("lift_prismatic_joint 를 못 찾았습니다.")
    pj = UsdPhysics.PrismaticJoint(joint_prim)

    # 안전 범위 = 폴리곤에 닿기 직전까지. 기본 가동 범위는 그 안에서 잡는다.
    safe_upper = up_room
    safe_lower = -down_room
    upper = safe_upper if DEFAULT_UPPER is None else min(DEFAULT_UPPER, safe_upper)
    lower = max(DEFAULT_LOWER, safe_lower)

    pj.CreateLowerLimitAttr().Set(float(lower))
    pj.CreateUpperLimitAttr().Set(float(upper))

    drive = UsdPhysics.DriveAPI.Apply(joint_prim, "linear")
    drive.CreateTypeAttr().Set(LIFT_DRIVE_TYPE)
    drive.CreateTargetPositionAttr().Set(0.0)     # 장면에 -53.4 가 들어 있었습니다.
    drive.CreateTargetVelocityAttr().Set(0.0)     # 위치 제어에서는 0 이어야 합니다.
    drive.CreateStiffnessAttr().Set(LIFT_STIFFNESS)
    drive.CreateDampingAttr().Set(LIFT_DAMPING)
    drive.CreateMaxForceAttr().Set(LIFT_MAX_FORCE)

    #    조작 패널이 읽을 값들을 조인트에 같이 저장합니다.
    #    USD 파일 안에 들어가므로 다른 PC 에서도 그대로 따라갑니다.
    joint_prim.CreateAttribute("lift:safeLower", Sdf.ValueTypeNames.Float).Set(float(safe_lower))
    joint_prim.CreateAttribute("lift:safeUpper", Sdf.ValueTypeNames.Float).Set(float(safe_upper))
    joint_prim.CreateAttribute("lift:defaultSpeed", Sdf.ValueTypeNames.Float).Set(float(DEFAULT_SPEED))

    #    속도 상한도 물리 쪽에 한 번 더 걸어 둡니다 (급격한 명령에도 튀지 않게).
    physx_joint = PhysxSchema.PhysxJointAPI.Apply(joint_prim)
    physx_joint.CreateMaxJointVelocityAttr().Set(float(MAX_JOINT_VELOCITY))

    # 7) 아티큘레이션 솔버 반복 횟수 --------------------------------------
    arti = PhysxSchema.PhysxArticulationAPI.Apply(chassis)
    arti.CreateSolverPositionIterationCountAttr().Set(SOLVER_POSITION_ITERATIONS)
    arti.CreateSolverVelocityIterationCountAttr().Set(SOLVER_VELOCITY_ITERATIONS)

    stage.GetRootLayer().Save()

    print("")
    print("=" * 62)
    print("리그 저장 완료: %s" % OUT_USD)
    print("  레이캐스트 칼럼 %d 개, 마스트와 겹친 칼럼 %d 개" % (columns, overlap))
    print("  폴리곤에 닿기까지: 위 %.4f m / 아래 %.4f m" % (up_raw, down_raw))
    print("  여유 %.3f m 를 뺀 안전 범위: %.4f ~ %.4f" % (TRAVEL_MARGIN, safe_lower, safe_upper))
    print("  적용한 가동 범위: lower=%.4f  upper=%.4f  (여유 %.3f m)"
          % (lower, upper, TRAVEL_MARGIN))
    print("  Drive: target=0.0  stiffness=%g  damping=%g  maxForce=%g"
          % (LIFT_STIFFNESS, LIFT_DAMPING, LIFT_MAX_FORCE))
    print("")
    print("다음 단계")
    print("  1) 이 파일을 열어 Play → 리프트가 제자리에 서 있는지 확인")
    print("  2) 03_lift_control.py 로 상하 조작")
    print("  3) 04_add_rig_to_scene.py 로 demo_test 장면에 얹기")
    print("=" * 62)
    return OUT_USD


if __name__ == "__main__":
    build()
