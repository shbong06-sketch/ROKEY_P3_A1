"""
이미 만들어져 있는 amr_lift_rig.usd 를 **그 파일 기준으로** 다듬는다.
새로 만들지 않고, 지금 파일의 참조·조인트 구성을 그대로 두고 자리만 고친다.

고치는 것 (순서대로)
  1) 리프트를 노바 카터 **중앙**에 놓는다 (지금은 뒤쪽/옆으로 치우쳐 있다).
     섀시 외피(visual)의 xy 중심에 맞추고, 마스트 바닥을 카터 상판 위에 얹는다.
  2) M0609 를 캐리지 **상판에 바짝** 붙인다.
     지금은 M0609_Mount(=캐리지 브래킷 꼭대기, 로컬 z 0.827889)에 붙어 있어서
     실제 상판(로컬 z 0.7528)보다 **75 mm 떠 있다.** 상판을 수직 레이캐스트로
     찾아 팔 플랜지를 거기에 앉힌다.
  3) 두 FixedJoint 의 local frame 을 새 자리에서 다시 계산한다
     (안 하면 Play 순간 제자리로 끌려간다).
  4) 프리즈매틱 가동 범위를 다시 잰다 (캐리지가 마스트 폴리곤에 닿기 직전까지).
  5) 저장한다.

건드리지 않는 것
  - 아티큘레이션 구성 (카터 섀시가 루트, 리프트·팔이 같은 아티큘레이션)
  - lift_moveparts_1 의 프리즈매틱 조인트 = 상하 운동 담당. 그대로 둔다.
  - M0609 의 6축 조인트 = 팔은 그대로 따로 움직인다.
  - 참조 경로 (assets/ 안에 복사돼 있어 다른 PC 에서도 그대로 열린다)

실행
  A) GUI 에서 amr_lift_rig.usd 를 연 상태로 Script Editor 에 붙여넣고 Ctrl+Enter
     → 고친 뒤 저장까지 한다. (Ctrl+Z 로 되돌리려면 저장 전에)
  B) 터미널:  ~/isaacsim/python.sh 06_fit_rig.py
"""

import os

import omni.usd
from pxr import Usd, UsdGeom, UsdPhysics, Sdf, Gf


# ── 설정 ─────────────────────────────────────────────────
RIG_USD = os.path.expanduser(
    "~/ROKEY_P3_A1/cobot3_ws/isaacpjt/smart_farm/assets/amr_lift_rig/amr_lift_rig.usd"
)

CENTER_ON_CARTER = True      # 리프트를 카터 섀시 중앙으로
LIFT_YAW_DEG = None          # None = 지금 회전 유지, 숫자면 그 값으로 (Z축, deg)
ARM_YAW_DEG = 0.0            # 캐리지 위에서 팔을 더 돌릴 각도 (Z축, deg)

DECK_GAP = 0.0               # 마스트 바닥을 카터 상판에서 띄울 양 (m)
SEAT_GAP = 0.0               # 팔 플랜지를 캐리지 상판에서 띄울 양 (m)
DECK_INCLUDE_SENSORS = False # 카터 상판 높이를 잴 때 라이다/카메라도 포함할지
CHECK_CARTER = True          # 마스트가 카터 부품과 실제로 겹치는지 레이캐스트로 확인하고,
                             # 캐리지 하강 한계에 카터(센서 포함)까지 반영한다

TRAVEL_MARGIN = 0.005        # 폴리곤에 닿기 전 남길 여유 (m)
TRAVEL_GRID = 0.005          # 행정 계산 격자 (m). 12mm 는 얇은 브래킷을 놓친다

SAVE = True                  # False 면 계산만 하고 저장하지 않는다


# ── 작은 도구들 ──────────────────────────────────────────
def _find_by_name(stage, name, type_name=None):
    for prim in stage.Traverse():
        if prim.GetName() == name and (type_name is None or prim.GetTypeName() == type_name):
            return prim
    return None


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

def _get_translate_orient(prim):
    xf = UsdGeom.Xformable(prim)
    t = Gf.Vec3d(0, 0, 0)
    q = Gf.Quatd(1, 0, 0, 0)
    for op in xf.GetOrderedXformOps():
        if op.GetOpType() == UsdGeom.XformOp.TypeTranslate:
            t = Gf.Vec3d(op.Get())
        elif op.GetOpType() == UsdGeom.XformOp.TypeOrient:
            v = op.Get()
            q = Gf.Quatd(v.GetReal(), Gf.Vec3d(v.GetImaginary()))
    return t, q


def _triangles(stage, root_prim, xy_box=None):
    """월드 삼각형 목록. xy_box=(x0,x1,y0,y1) 밖은 버린다."""
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
                x0, x1 = min(a[0], b[0], d[0]), max(a[0], b[0], d[0])
                y0, y1 = min(a[1], b[1], d[1]), max(a[1], b[1], d[1])
                if xy_box and (x1 < xy_box[0] or x0 > xy_box[1] or y1 < xy_box[2] or y0 > xy_box[3]):
                    pass
                else:
                    out.append((a, b, d, x0, x1, y0, y1))
            o += c
    return out


def _hits(tris, x, y):
    """(x, y) 수직선이 만나는 z 값들 (오름차순)."""
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


def _world_range(bbox_cache, prim):
    return bbox_cache.ComputeWorldBound(prim).ComputeAlignedRange()


def _fixed_joint_frames(joint, body0, body1):
    """body0 기준으로 body1 을 '지금 그 자리'에 고정하도록 local frame 을 다시 쓴다."""
    cache = UsdGeom.XformCache()
    w0 = cache.GetLocalToWorldTransform(body0)
    w1 = cache.GetLocalToWorldTransform(body1)
    rel = Gf.Transform(w1 * w0.GetInverse())
    j = UsdPhysics.Joint(joint)
    j.CreateLocalPos0Attr().Set(Gf.Vec3f(rel.GetTranslation()))
    j.CreateLocalRot0Attr().Set(Gf.Quatf(rel.GetRotation().GetQuat()))
    j.CreateLocalPos1Attr().Set(Gf.Vec3f(0, 0, 0))
    j.CreateLocalRot1Attr().Set(Gf.Quatf(1, 0, 0, 0))


# ── 본체 ─────────────────────────────────────────────────
def fit():
    ctx = omni.usd.get_context()
    stage = ctx.get_stage()
    joint_prim = _find_by_name(stage, "lift_prismatic_joint", "PhysicsPrismaticJoint") if stage else None
    if joint_prim is None:
        print("[fit] 열린 장면에 리프트가 없어 %s 를 엽니다." % RIG_USD)
        ctx.open_stage(RIG_USD)
        stage = ctx.get_stage()
        joint_prim = _find_by_name(stage, "lift_prismatic_joint", "PhysicsPrismaticJoint")
    if joint_prim is None:
        raise RuntimeError("lift_prismatic_joint 를 못 찾았습니다.")

    lift = joint_prim.GetParent()
    rig = lift.GetParent()
    holder = stage.GetPrimAtPath(lift.GetPath().AppendChild("lift_holder"))
    mover = stage.GetPrimAtPath(lift.GetPath().AppendChild("lift_moveparts_1"))
    chassis = _find_by_name(stage, "chassis_link")
    arm = arm_base = None
    for child in rig.GetChildren():
        for prim in Usd.PrimRange(child):
            if prim.GetName() == "base_link":
                arm, arm_base = child, prim
                break
        if arm is not None:
            break
    if arm is None or chassis is None or not holder.IsValid() or not mover.IsValid():
        raise RuntimeError("카터 섀시 / 마스트 / 캐리지 / 팔 중 하나를 못 찾았습니다.")

    print("[fit] 리그 루트 : %s" % rig.GetPath())
    print("[fit] 카터 섀시 : %s" % chassis.GetPath())
    print("[fit] 팔        : %s" % arm.GetPath())

    bbox = UsdGeom.BBoxCache(
        Usd.TimeCode.Default(), [UsdGeom.Tokens.default_, UsdGeom.Tokens.render]
    )

    # ── 1) 리프트를 카터 중앙에 ──────────────────────────
    lift_t, lift_q = _get_translate_orient(lift)
    if LIFT_YAW_DEG is not None:
        lift_q = Gf.Rotation(Gf.Vec3d(0, 0, 1), LIFT_YAW_DEG).GetQuat()

    holder_r = _world_range(bbox, holder)
    holder_bottom_local = holder_r.GetMin()[2] - lift_t[2]      # yaw 뿐이라 z 는 그대로
    foot = (holder_r.GetMin()[0], holder_r.GetMax()[0],
            holder_r.GetMin()[1], holder_r.GetMax()[1])

    shell = stage.GetPrimAtPath(chassis.GetPath().AppendChild("visual"))
    shell_r = _world_range(bbox, shell if shell.IsValid() else chassis)
    center_x = (shell_r.GetMin()[0] + shell_r.GetMax()[0]) / 2.0
    center_y = (shell_r.GetMin()[1] + shell_r.GetMax()[1]) / 2.0
    if not CENTER_ON_CARTER:
        center_x, center_y = lift_t[0], lift_t[1]

    # 새 발자국 (중앙으로 옮긴 뒤)
    dx, dy = center_x - lift_t[0], center_y - lift_t[1]
    new_foot = (foot[0] + dx, foot[1] + dx, foot[2] + dy, foot[3] + dy)

    # 발자국 안에 들어오는 카터 메시들의 높이 (프림 단위 bbox 라 빠르다)
    deck_top = None
    tall_sensors = []
    for prim in Usd.PrimRange(chassis, Usd.TraverseInstanceProxies(Usd.PrimAllPrimsPredicate)):
        if not prim.IsA(UsdGeom.Mesh):
            continue
        r = _world_range(bbox, prim)
        if r.IsEmpty():
            continue
        mn, mx = r.GetMin(), r.GetMax()
        if mx[0] < new_foot[0] or mn[0] > new_foot[1] or mx[1] < new_foot[2] or mn[1] > new_foot[3]:
            continue
        is_sensor = "/sensors/" in str(prim.GetPath())
        if is_sensor and not DECK_INCLUDE_SENSORS:
            tall_sensors.append((mx[2], str(prim.GetPath())))
            continue
        deck_top = mx[2] if deck_top is None else max(deck_top, mx[2])
    if deck_top is None:
        raise RuntimeError("카터 상판 높이를 못 쟀습니다.")

    new_lift_t = Gf.Vec3d(center_x, center_y, deck_top + DECK_GAP - holder_bottom_local)
    _set_xform(lift, new_lift_t, lift_q)
    print("[fit] 1) 리프트 위치 %s -> %s" % (
        tuple(round(v, 4) for v in lift_t), tuple(round(v, 4) for v in new_lift_t)))
    print("        카터 상판(발자국 안 외피 최고점) z = %.4f, 마스트 바닥을 여기에 얹음" % deck_top)
    # 마스트 바닥보다 높이 솟은 카터 부품이 **실제로** 마스트와 겹치는지 확인한다.
    # (겹치지 않으면 마스트 프레임의 빈 공간 안에 들어간 것이라 그냥 두면 된다)
    if CHECK_CARTER:
        holder_tris_chk = _triangles(stage, holder)
        for z, path in sorted(tall_sensors, reverse=True):
            if z <= deck_top:
                continue
            part = stage.GetPrimAtPath(path)
            pr = _world_range(bbox, part)
            part_tris = _triangles(stage, part)
            hit_cols = 0
            gx = pr.GetMin()[0]
            while gx <= pr.GetMax()[0]:
                gy = pr.GetMin()[1]
                while gy <= pr.GetMax()[1]:
                    ph = _hits(part_tris, gx, gy)
                    if len(ph) >= 2:
                        hh = _hits(holder_tris_chk, gx, gy)
                        if any(ph[0] - 1e-5 <= v <= ph[-1] + 1e-5 for v in hh):
                            hit_cols += 1
                    gy += TRAVEL_GRID
                gx += TRAVEL_GRID
            name = path.split("/")[-2]
            if hit_cols:
                print("        ! %s (z %.4f) 가 마스트와 겹칩니다 — 칼럼 %d 개" % (name, z, hit_cols))
                print("          마스트를 올리거나(DECK_GAP) 그 부품을 옮겨야 합니다.")
            else:
                print("        %s 가 마스트 바닥(%.4f)보다 높지만(z %.4f) 프레임 안쪽이라 겹치지 않음"
                      % (name, deck_top, z))

    # ── 2) M0609 를 캐리지 상판에 안착 ───────────────────
    mover_tris = _triangles(stage, mover)
    lift_world = UsdGeom.XformCache().GetLocalToWorldTransform(lift)
    axis = Gf.Transform(lift_world).GetTranslation()      # 캐리지 마운트 축 (로컬 x=y=0)

    plate_tops = []
    holes = 0
    for ddx in (-0.06, -0.03, 0.0, 0.03, 0.06):
        for ddy in (-0.06, -0.03, 0.0, 0.03, 0.06):
            h = _hits(mover_tris, axis[0] + ddx, axis[1] + ddy)
            if h:
                plate_tops.append(max(h))
            else:
                holes += 1
    if not plate_tops:
        raise RuntimeError("캐리지 상판을 못 찾았습니다 (마운트 축 아래가 비어 있음).")
    plate_top = max(plate_tops)
    plate_top_local = plate_top - new_lift_t[2]

    mount = stage.GetPrimAtPath(mover.GetPath().AppendChild("M0609_Mount"))
    mount_local_z = None
    if mount.IsValid():
        ops = UsdGeom.Xformable(mount).GetOrderedXformOps()
        if ops:
            mount_local_z = Gf.Vec3d(ops[0].Get())[2]

    # base_link 가 놓여야 할 프레임 (캐리지 상판 위, 리프트 좌표계 기준)
    seat = Gf.Matrix4d().SetRotate(Gf.Rotation(Gf.Vec3d(0, 0, 1), ARM_YAW_DEG))
    seat.SetTranslateOnly(Gf.Vec3d(0, 0, plate_top_local + SEAT_GAP))
    target = seat * lift_world

    # 팔 프림과 base_link 가 같은 자리가 아닐 수 있으므로(에셋마다 다름),
    # base_link 가 target 에 오도록 팔 프림 자체의 위치를 역산한다.
    cache2 = UsdGeom.XformCache()
    base_in_arm = (cache2.GetLocalToWorldTransform(arm_base)
                   * cache2.GetLocalToWorldTransform(arm).GetInverse())
    # xform 은 부모 기준이므로, 부모가 원점이 아니면 월드 행렬을 로컬로 바꿔야 한다.
    parent_world = cache2.GetLocalToWorldTransform(arm.GetParent())
    arm_world = Gf.Transform((base_in_arm.GetInverse() * target) * parent_world.GetInverse())
    _set_xform(arm, arm_world.GetTranslation(), arm_world.GetRotation().GetQuat())

    print("[fit] 2) 캐리지 상판 월드 z = %.5f (구멍 %d/25)" % (plate_top, holes))
    if mount_local_z is not None:
        print("        M0609_Mount 로컬 z %.5f 는 브래킷 꼭대기라 %.1f mm 떠 있었음"
              % (mount_local_z, (mount_local_z - plate_top_local) * 1000))
    print("        팔을 상판 로컬 z %.5f 에 안착 (여유 %.3f m)" % (plate_top_local, SEAT_GAP))

    # ── 3) 조인트 프레임 다시 계산 ───────────────────────
    fixed_joints = []
    for prim in stage.Traverse():
        if prim.GetTypeName() != "PhysicsFixedJoint" or not prim.IsActive():
            continue
        j = UsdPhysics.Joint(prim)
        b0 = j.GetBody0Rel().GetTargets()
        b1 = j.GetBody1Rel().GetTargets()
        if not b0 or not b1:
            continue
        p0, p1 = stage.GetPrimAtPath(b0[0]), stage.GetPrimAtPath(b1[0])
        pair = {p0.GetPath(), p1.GetPath()}
        if pair == {chassis.GetPath(), holder.GetPath()} or pair == {mover.GetPath(), arm_base.GetPath()}:
            _fixed_joint_frames(prim, p0, p1)
            fixed_joints.append(str(prim.GetPath()))
    print("[fit] 3) FixedJoint local frame 재계산: %s" % fixed_joints)

    # ── 4) 가동 범위 다시 측정 ───────────────────────────
    holder_tris = _triangles(stage, holder)
    mover_tris = _triangles(stage, mover)
    xs = [t[i][0] for t in mover_tris for i in range(3)]
    ys = [t[i][1] for t in mover_tris for i in range(3)]
    # 칼럼마다 두 가지로 잰다.
    #   per-column : 그 칼럼의 캐리지 윗면 기준 (정확하지만 격자가 성기면 얇은
    #                브래킷을 놓쳐서 값이 과하게 커진다)
    #   global     : 캐리지 전체의 최고점 기준 (격자와 무관하게 항상 안전측)
    # 둘 중 작은 값을 쓴다.
    mover_top_all = max(t[i][2] for t in mover_tris for i in range(3))
    mover_bot_all = min(t[i][2] for t in mover_tris for i in range(3))
    up = down = float("inf")
    columns = overlap = 0
    x = min(xs) + TRAVEL_GRID / 2
    while x < max(xs):
        y = min(ys) + TRAVEL_GRID / 2
        while y < max(ys):
            mh = _hits(mover_tris, x, y)
            if len(mh) >= 2:
                columns += 1
                top, bot = mh[-1], mh[0]
                hh = _hits(holder_tris, x, y)
                above = [z for z in hh if z > top + 1e-5]
                below = [z for z in hh if z < bot - 1e-5]
                if any(bot - 1e-5 <= z <= top + 1e-5 for z in hh):
                    overlap += 1
                if above:
                    up = min(up, min(above) - top, min(above) - mover_top_all)
                if below:
                    down = min(down, bot - max(below), mover_bot_all - max(below))
            y += TRAVEL_GRID
        x += TRAVEL_GRID

    # 캐리지가 내려올 때 카터(센서 포함)에 닿지 않도록, 카터 부품까지 한계에 반영한다.
    carter_down = float("inf")
    carter_hit = None
    if CHECK_CARTER:
        mx0 = min(t[i][0] for t in mover_tris for i in range(3))
        mx1 = max(t[i][0] for t in mover_tris for i in range(3))
        my0 = min(t[i][1] for t in mover_tris for i in range(3))
        my1 = max(t[i][1] for t in mover_tris for i in range(3))
        mz0 = min(t[i][2] for t in mover_tris for i in range(3))
        for prim in Usd.PrimRange(chassis, Usd.TraverseInstanceProxies(Usd.PrimAllPrimsPredicate)):
            if not prim.IsA(UsdGeom.Mesh):
                continue
            r = _world_range(bbox, prim)
            if r.IsEmpty():
                continue
            mn, mxr = r.GetMin(), r.GetMax()
            if mxr[0] < mx0 or mn[0] > mx1 or mxr[1] < my0 or mn[1] > my1:
                continue
            gap = mz0 - mxr[2]
            if 0 <= gap < carter_down:
                carter_down, carter_hit = gap, str(prim.GetPath())

    safe_upper = up - TRAVEL_MARGIN
    safe_lower = -(min(down, carter_down) - TRAVEL_MARGIN)
    pj = UsdPhysics.PrismaticJoint(joint_prim)
    pj.CreateLowerLimitAttr().Set(0.0)
    pj.CreateUpperLimitAttr().Set(float(safe_upper))
    joint_prim.CreateAttribute("lift:safeLower", Sdf.ValueTypeNames.Float).Set(float(safe_lower))
    joint_prim.CreateAttribute("lift:safeUpper", Sdf.ValueTypeNames.Float).Set(float(safe_upper))
    drive = UsdPhysics.DriveAPI.Get(joint_prim, "linear")
    if drive:
        drive.GetTargetPositionAttr().Set(0.0)
        drive.GetTargetVelocityAttr().Set(0.0)
    print("[fit] 4) 칼럼 %d, 마스트와 겹친 칼럼 %d" % (columns, overlap))
    print("        마스트 기준: 위 %.4f / 아래 %.4f" % (up, down))
    if carter_hit:
        print("        카터 기준: 캐리지 밑면 → %s 까지 %.4f"
              % (carter_hit.split("/")[-1], carter_down))
    print("        → 안전 범위 %.4f ~ %.4f (여유 %.3f)" % (safe_lower, safe_upper, TRAVEL_MARGIN))
    print("        조인트 리미트 0.0 ~ %.4f 적용" % safe_upper)

    # ── 5) 저장 ──────────────────────────────────────────
    if SAVE:
        stage.GetRootLayer().Save()
        print("[fit] 5) 저장 완료: %s" % stage.GetRootLayer().identifier)
    else:
        print("[fit] 5) SAVE=False 라 저장하지 않았습니다.")

    print("\n확인 순서")
    print("  - Play → 팔이 캐리지 상판에 붙어 있고 제자리에 서 있는지")
    print("  - 05_lift_panel.py 로 위아래 (0 ~ %.3f m)" % safe_upper)
    print("  - 팔은 그대로 따로 움직입니다 (joint_1~6)")


fit()
