"""
장면 파일(combine_test_1.usd 등)에 얹은 리그를 정상 상태로 되돌린다.

combine_test_1.usd 를 뜯어보고 찾은 문제 (이 스크립트가 고치는 것)

  1) 리그 안 `lift_moveparts_1` 에 장면 레이어에서 손으로 준 이동값이 남아 있다.
        lift_moveparts_1        translate z = +0.0383
        lift_moveparts_1/Mesh   translate z = +0.1055
     프리즈매틱 조인트는 마스트 원점과 캐리지 원점이 겹친 상태(localPos0=localPos1=0)
     를 0 으로 본다. 여기에 이동값을 덧칠하면 Play 하는 순간 그만큼 끌려가고,
     메시만 따로 올려 두면 보이는 위치와 물리 위치가 어긋난다.
     → 장면 레이어의 xformOp 오버라이드를 지워서 리그 값(=조인트와 일치)으로 되돌린다.

  2) 리그 안 팔(`amr_lift_rig/m0609_with_fork`)이 active = false 로 꺼져 있고,
     대신 `/World/m0609_with_fork` 라는 **별도의 팔**이 손으로 놓여 있다.
     이 별도 팔은 자기 root_joint 로 **월드에 고정**돼 있어서 리프트가 올라가도
     같이 올라가지 않는다. 카터가 움직여도 따라가지 않는다.
     → 리그 안 팔을 다시 켜고(캐리지에 FixedJoint 로 물려 있음),
       손으로 놓은 별도 팔은 끈다(지우지 않고 active=false, 되돌릴 수 있게).

  3) 별도 팔의 링크마다 `visuals` 와 `collisions` 양쪽에
     PhysxTriangleMeshCollisionAPI + approximation = "none" 이 적용돼 있다.
     approximation "none" 은 삼각형 메시 그대로 쓰겠다는 뜻인데, PhysX 에서
     **움직이는(dynamic) 바디에는 쓸 수 없다.** 콜라이더가 무시되거나 에러가 난다.
     게다가 visuals 에까지 콜리전을 걸어 콜라이더가 두 겹이 된다.
     → 07_fix_colliders.py 가 처리한다 (이 스크립트는 알려만 준다).

실행
  A) GUI 에서 장면을 연 채 Script Editor 에 붙여넣고 Ctrl+Enter
  B) 터미널:  ~/isaacsim/python.sh 08_fit_scene.py  (아래 SCENE_USD 를 엽니다)
"""

import os

import omni.usd
from pxr import Usd, UsdGeom, UsdPhysics, Gf


SCENE_USD = os.path.expanduser(
    "~/ROKEY_P3_A1/cobot3_ws/isaacpjt/smart_farm/scenes/demo_test/combine_test_1.usd"
)

CLEAR_RIG_OVERRIDES = True    # 리그 내부 프림에 장면에서 덧칠한 xform 을 지운다
ENABLE_RIG_ARM = True         # 리그 안 팔을 켠다
DISABLE_LOOSE_ARMS = True     # 리그 밖에 따로 놓인 팔은 끈다
SAVE = True


def _open_stage():
    ctx = omni.usd.get_context()
    stage = ctx.get_stage()
    ok = False
    if stage:
        for prim in stage.Traverse():
            if prim.GetName() == "lift_prismatic_joint":
                ok = True
                break
    if not ok:
        print("[scene] 열린 장면에 리프트가 없어 %s 를 엽니다." % SCENE_USD)
        ctx.open_stage(SCENE_USD)
        stage = ctx.get_stage()
    return stage


def _rig_root(stage):
    for prim in stage.Traverse():
        if prim.GetName() == "lift_prismatic_joint" and prim.GetTypeName() == "PhysicsPrismaticJoint":
            return prim.GetParent().GetParent()
    raise RuntimeError("리그를 못 찾았습니다.")


def _all_children(prim):
    """비활성(active=false) 프림까지 포함해서 자식을 본다."""
    return prim.GetFilteredChildren(Usd.PrimAllPrimsPredicate)


def _clear_layer_field(layer, path, field):
    spec = layer.GetPrimAtPath(path)
    if spec is not None and spec.HasInfo(field):
        spec.ClearInfo(field)
        return True
    return False


def _clear_layer_xform(layer, path):
    """루트 레이어에 덧칠된 xformOp 속성들을 지운다 (리그 원본 값이 다시 보이게)."""
    spec = layer.GetPrimAtPath(path)
    if spec is None:
        return []
    removed = []
    for name in list(spec.properties.keys()):
        if name.startswith("xformOp"):
            spec.RemoveProperty(spec.properties[name])
            removed.append(name)
    return removed


def main():
    stage = _open_stage()
    layer = stage.GetRootLayer()
    rig = _rig_root(stage)
    print("[scene] 장면       : %s" % layer.identifier)
    print("[scene] 리그 프림   : %s" % rig.GetPath())

    lift = None
    for child in _all_children(rig):
        if stage.GetPrimAtPath(child.GetPath().AppendChild("lift_moveparts_1")).IsValid():
            lift = child
    if lift is None:
        raise RuntimeError("리그 안 lift 프림을 못 찾았습니다.")
    mover = stage.GetPrimAtPath(lift.GetPath().AppendChild("lift_moveparts_1"))
    holder = stage.GetPrimAtPath(lift.GetPath().AppendChild("lift_holder"))

    # ── 1) 리그 안 팔 켜기 / 밖에 놓인 팔 끄기 ───────────
    #     (비활성 프림은 기본 순회에서 안 보이므로 먼저 켜고 나서 오버라이드를 지운다)
    # 꺼져 있는 프림은 자식이 합성되지 않아 base_link 로 못 찾는다.
    # 그래서 (a) base_link 가 보이거나 (b) 이름에 m0609 가 들어가면 팔로 본다.
    rig_arm = None
    for child in _all_children(rig):
        if stage.GetPrimAtPath(child.GetPath().AppendChild("base_link")).IsValid():
            rig_arm = child
        elif not child.IsActive() and "m0609" in child.GetName().lower():
            rig_arm = child
    print("\n[scene] 1) 팔 정리")
    if rig_arm is not None:
        if ENABLE_RIG_ARM and not rig_arm.IsActive():
            # 장면 레이어에 박힌 active=false 를 지워서 리그 값(켜짐)으로 되돌린다
            if not _clear_layer_field(layer, rig_arm.GetPath(), "active"):
                rig_arm.SetActive(True)
            print("        리그 안 팔 켬: %s (active=%s)" % (rig_arm.GetPath(), rig_arm.IsActive()))
        else:
            print("        리그 안 팔 상태: active=%s  (%s)" % (rig_arm.IsActive(), rig_arm.GetPath()))
    else:
        print("        ! 리그 안에서 팔을 못 찾았습니다.")

    loose = []
    for prim in stage.Traverse():
        if not stage.GetPrimAtPath(prim.GetPath().AppendChild("base_link")).IsValid():
            continue
        if str(prim.GetPath()).startswith(str(rig.GetPath())):
            continue
        loose.append(prim)
    for prim in loose:
        root_joint = stage.GetPrimAtPath(prim.GetPath().AppendChild("root_joint"))
        world_fixed = False
        if root_joint.IsValid():
            world_fixed = not UsdPhysics.Joint(root_joint).GetBody0Rel().GetTargets()
        print("        리그 밖 팔: %s (root_joint 월드고정=%s)" % (prim.GetPath(), world_fixed))
        if DISABLE_LOOSE_ARMS and prim.IsActive():
            prim.SetActive(False)
            print("           → active=false 로 껐습니다 (아웃라이너에서 되돌릴 수 있음)")

    # ── 2) 리그 내부에 덧칠된 xform 오버라이드 제거 ──────
    if CLEAR_RIG_OVERRIDES:
        cleared = []
        for prim in Usd.PrimRange(rig, Usd.PrimAllPrimsPredicate):
            if prim == rig or prim == lift:
                continue          # 리그 자체 배치와 리프트 위치는 각자 관리한다
            removed = _clear_layer_xform(layer, prim.GetPath())
            if removed:
                cleared.append((str(prim.GetPath()), removed))
        print("\n[scene] 2) 리그 내부 xform 오버라이드 제거 %d 곳" % len(cleared))
        for path, names in cleared:
            print("        %s  %s" % (path, names))
        if not cleared:
            print("        (덧칠된 것 없음)")

    # ── 3) 결과 확인: 팔이 캐리지 상판에 붙어 있는지 ─────
    stage_cache = UsdGeom.XformCache()
    bbox = UsdGeom.BBoxCache(
        Usd.TimeCode.Default(), [UsdGeom.Tokens.default_, UsdGeom.Tokens.render]
    )
    print("\n[scene] 3) 확인")
    mover_r = bbox.ComputeWorldBound(mover).ComputeAlignedRange()
    holder_r = bbox.ComputeWorldBound(holder).ComputeAlignedRange()
    print("        마스트 z %.4f ~ %.4f / 캐리지 z %.4f ~ %.4f"
          % (holder_r.GetMin()[2], holder_r.GetMax()[2],
             mover_r.GetMin()[2], mover_r.GetMax()[2]))

    if rig_arm is not None and rig_arm.IsActive():
        arm_base = stage.GetPrimAtPath(rig_arm.GetPath().AppendChild("base_link"))
        arm_w = Gf.Transform(stage_cache.GetLocalToWorldTransform(arm_base)).GetTranslation()
        # 캐리지 상판을 팔 축 바로 아래에서 찾는다
        tris = []
        for prim in Usd.PrimRange(mover, Usd.TraverseInstanceProxies(Usd.PrimAllPrimsPredicate)):
            if not prim.IsA(UsdGeom.Mesh):
                continue
            mesh = UsdGeom.Mesh(prim)
            pts = mesh.GetPointsAttr().Get()
            counts = mesh.GetFaceVertexCountsAttr().Get()
            idx = mesh.GetFaceVertexIndicesAttr().Get()
            if not pts or not counts:
                continue
            mat = stage_cache.GetLocalToWorldTransform(prim)
            w = [mat.Transform(Gf.Vec3d(v)) for v in pts]
            o = 0
            for c in counts:
                f = idx[o:o + c]
                for k in range(1, c - 1):
                    tris.append((w[f[0]], w[f[k]], w[f[k + 1]]))
                o += c
        best = None
        for a, b, c in tris:
            x, y = arm_w[0], arm_w[1]
            ax, ay = a[0] - x, a[1] - y
            bx, by = b[0] - x, b[1] - y
            cx, cy = c[0] - x, c[1] - y
            d1 = ax * by - ay * bx
            d2 = bx * cy - by * cx
            d3 = cx * ay - cy * ax
            if (d1 > 0 and d2 > 0 and d3 > 0) or (d1 < 0 and d2 < 0 and d3 < 0):
                tot = d1 + d2 + d3
                if abs(tot) > 1e-12:
                    z = (d2 * a[2] + d3 * b[2] + d1 * c[2]) / tot
                    best = z if best is None else max(best, z)
        if best is None:
            print("        ! 팔 축 아래에 캐리지 상판이 없습니다 (팔 위치 확인 필요)")
        else:
            print("        팔 플랜지 z %.5f / 캐리지 상판 z %.5f → 간극 %+.5f m"
                  % (arm_w[2], best, arm_w[2] - best))

    joint = stage.GetPrimAtPath(lift.GetPath().AppendChild("lift_prismatic_joint"))
    pj = UsdPhysics.PrismaticJoint(joint)
    drive = UsdPhysics.DriveAPI.Get(joint, "linear")
    print("        리프트 가동 범위 %.4f ~ %.4f m, Drive target %s"
          % (pj.GetLowerLimitAttr().Get(), pj.GetUpperLimitAttr().Get(),
             drive.GetTargetPositionAttr().Get() if drive else "없음"))

    if SAVE:
        layer.Save()
        print("\n[scene] 저장 완료: %s" % layer.identifier)
    else:
        print("\n[scene] SAVE=False 라 저장하지 않았습니다.")
    print("다음: 07_fix_colliders.py 로 콜라이더 점검 → Play → 05_lift_panel.py")


main()
