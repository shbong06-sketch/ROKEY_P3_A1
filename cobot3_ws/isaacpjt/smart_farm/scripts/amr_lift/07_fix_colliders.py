"""
M0609 / 리프트 / 카터의 콜라이더 상태를 점검하고, 문제가 있는 것만 고친다.
amr_lift_rig.usd 를 기준으로 동작한다 (열려 있으면 그 스테이지, 아니면 파일을 연다).

왜 필요한가 — 실측으로 확인된 문제
  * `base_link` 의 콜라이더가 **convexHull** 인데, 그 메시에는 베이스 옆으로
    빠져나온 **케이블**이 포함돼 있다 (xy 로 y = -0.340 까지, 반경 0.356 m).
    convexHull 은 그 케이블 끝까지 통째로 감싸므로, 실제 로봇보다 훨씬 큰
    '유령 충돌체'가 생긴다. 팔이 랙/팔레트 근처로 갈 때 눈에 안 보이는 것에
    부딪힌다.
  * 리프트 마스트/캐리지도 원본 에셋은 convexHull 이다. 속이 빈 마스트를
    convexHull 로 잡으면 꽉 찬 상자가 되어 캐리지와 영구 충돌한다.
    (리그 빌더가 이미 convexDecomposition 으로 덮어썼지만, 여기서 다시 확인한다.)

고치는 방법
  메시를 건드리지 않고 **근사 방식만 바꾼다**: convexHull -> convexDecomposition.
  볼록 덩어리 여러 개로 쪼개므로 케이블은 케이블대로, 베이스는 베이스대로
  잡힌다. 물리 엔진은 그대로 쓰고, 충돌 모양만 실제 형상에 맞춘다.

  fork_tool 은 Cube 프림(box) 콜라이더라 이미 정확하다 — 건드리지 않는다.
  얇은 포크 날은 box 가 convexDecomposition 보다 낫다.

옵션
  ARM_DRIVE_STIFFNESS 를 숫자로 주면 팔 6축 Drive 게인을 그 값으로 올린다.
  URDF 에서 넘어온 기본값(k=40~1160, c=0.012~0.46)은 자세 유지에 약해서
  팔을 뻗으면 처진다. test.py 는 실행할 때마다 자기 값(1e8/1e4)으로 덮어쓰므로
  기본값은 None(=건드리지 않음)으로 둔다.
"""

import os

import omni.usd
from pxr import Usd, UsdGeom, UsdPhysics, Sdf


RIG_USD = os.path.expanduser(
    "~/ROKEY_P3_A1/cobot3_ws/isaacpjt/smart_farm/assets/amr_lift_rig/amr_lift_rig.usd"
)

# 이 크기(m)보다 넓게 퍼진 메시를 convexHull 로 잡고 있으면 문제로 본다.
HULL_SPREAD_LIMIT = 0.25

# 카터 원본 에셋은 건드리지 않는다. NVIDIA 가 의도적으로 외피 하나를 convexHull 로
# 잡아 둔 것이고, 5백만 삼각형짜리 메시를 convexDecomposition 으로 바꾸면
# 쿠킹에 한참 걸리고 성능도 나빠진다. 상태만 보고한다.
SKIP_NAMES = ("Nova_Carter",)

# approximation = "none"(삼각형 메시 그대로)은 움직이는 바디에 쓸 수 없다.
# PhysX 가 거부해서 콜라이더가 무시되거나 에러가 난다. 발견하면 볼록 근사로 바꾼다.
FIX_NONE_APPROX = True

# visuals 에까지 콜리전이 걸려 있으면 그쪽을 끈다 (collisions 쪽이 진짜다).
DISABLE_VISUAL_COLLIDERS = True

ARM_DRIVE_STIFFNESS = None      # 예: 1.0e8 로 주면 팔 Drive 를 그 값으로
ARM_DRIVE_DAMPING = 1.0e4

SAVE = True


def _open_stage():
    ctx = omni.usd.get_context()
    stage = ctx.get_stage()
    found = False
    if stage:
        for prim in stage.Traverse():
            if prim.GetName() == "lift_prismatic_joint":
                found = True
                break
    if not found:
        print("[col] 열린 장면에 리그가 없어 %s 를 엽니다." % RIG_USD)
        ctx.open_stage(RIG_USD)
        stage = ctx.get_stage()
    return stage


def _author(stage, prim, prop_name, value, value_type, api_schemas=()):
    """속성 하나를 쓴다.

    콜라이더가 instanceable 참조 안에 있으면(=instance proxy) 스테이지에서는
    쓸 수 없다. 그럴 때는 그 프림을 정의한 **원본 레이어**(에셋 파일)에 직접 써서
    그 에셋을 쓰는 모든 인스턴스에 반영되게 한다. 리그 폴더 안의 사본이므로
    다른 PC 로 옮겨도 그대로 따라간다.

    반환: 어디에 썼는지 설명 문자열
    """
    if not prim.IsInstanceProxy():
        attr = prim.GetAttribute(prop_name)
        if not attr:
            attr = prim.CreateAttribute(prop_name, value_type)
        attr.Set(value)
        return "스테이지"

    proto = prim.GetPrimInPrototype()
    if not proto:
        return None
    for spec in proto.GetPrimStack():
        layer, path = spec.layer, spec.path
        attr_spec = layer.GetAttributeAtPath(path.AppendProperty(prop_name))
        if attr_spec is not None:
            attr_spec.default = value
            layer.Save()
            return "원본 레이어 %s : %s" % (os.path.basename(layer.identifier), path)
    # 그 속성이 아직 없으면 가장 강한 spec 에 새로 만든다
    spec = proto.GetPrimStack()[0]
    layer, path = spec.layer, spec.path
    prim_spec = Sdf.CreatePrimInLayer(layer, path)
    attr_spec = Sdf.AttributeSpec(prim_spec, prop_name, value_type)
    attr_spec.default = value
    for api in api_schemas:
        items = prim_spec.GetInfo("apiSchemas") if prim_spec.HasInfo("apiSchemas") else Sdf.TokenListOp()
        if api not in items.prependedItems:
            items.prependedItems = list(items.prependedItems) + [api]
            prim_spec.SetInfo("apiSchemas", items)
    layer.Save()
    return "원본 레이어 %s : %s (새로 생성)" % (os.path.basename(layer.identifier), path)


def _is_dynamic(prim):
    """이 콜라이더가 움직이는(dynamic) 바디에 속하는지."""
    p = prim
    while p and p.IsValid() and not p.IsPseudoRoot():
        if p.HasAPI(UsdPhysics.RigidBodyAPI):
            return not bool(UsdPhysics.RigidBodyAPI(p).GetKinematicEnabledAttr().Get())
        p = p.GetParent()
    return False


def main():
    stage = _open_stage()
    bbox = UsdGeom.BBoxCache(
        Usd.TimeCode.Default(),
        [UsdGeom.Tokens.default_, UsdGeom.Tokens.render,
         UsdGeom.Tokens.proxy, UsdGeom.Tokens.guide],
    )

    print("=" * 72)
    print("%-52s %-18s %s" % ("콜라이더", "근사", "퍼짐(최대변, m)"))
    print("-" * 72)

    fixed, checked, skipped = [], 0, []
    for prim in Usd.PrimRange(stage.GetPseudoRoot(),
                              Usd.TraverseInstanceProxies(Usd.PrimAllPrimsPredicate)):
        if not prim.HasAPI(UsdPhysics.CollisionAPI):
            continue
        path = str(prim.GetPath())
        if any(("/%s/" % n) in path or path.endswith("/%s" % n) for n in SKIP_NAMES):
            skipped.append(path)
            continue
        checked += 1
        enabled = UsdPhysics.CollisionAPI(prim).GetCollisionEnabledAttr().Get()
        approx = None
        if prim.HasAPI(UsdPhysics.MeshCollisionAPI):
            approx = UsdPhysics.MeshCollisionAPI(prim).GetApproximationAttr().Get()
        r = bbox.ComputeWorldBound(prim).ComputeAlignedRange()
        size = r.GetSize() if not r.IsEmpty() else None
        spread = max(size[0], size[1], size[2]) if size else 0.0

        mark = ""
        # 문제 판정: convexHull 인데 넓게 퍼져 있음 = 실제보다 큰 덩어리가 된다
        # 콜리전은 Mesh 가 아니라 그 위 Xform 에 붙어 있는 경우도 있어서
        # IsA(Mesh) 로 거르지 않는다. approximation 이 convexHull 인 것만 본다.
        if approx == "convexHull" and spread > HULL_SPREAD_LIMIT:
            where = _author(stage, prim, "physics:approximation", "convexDecomposition",
                            Sdf.ValueTypeNames.Token, ("PhysicsMeshCollisionAPI",))
            fixed.append((str(prim.GetPath()), "convexHull(너무 넓음)", spread, where))
            mark = "  -> convexDecomposition (%s)" % where

        elif FIX_NONE_APPROX and approx == "none" and _is_dynamic(prim):
            new_approx = "convexDecomposition" if spread > HULL_SPREAD_LIMIT else "convexHull"
            where = _author(stage, prim, "physics:approximation", new_approx,
                            Sdf.ValueTypeNames.Token, ("PhysicsMeshCollisionAPI",))
            fixed.append((str(prim.GetPath()), "none(움직이는 바디엔 불가)", spread, where))
            mark = "  -> %s (%s)" % (new_approx, where)

        # visuals 에까지 콜리전이 걸려 있으면(콜라이더 두 겹) 그쪽을 끈다
        if (DISABLE_VISUAL_COLLIDERS and enabled is not False
                and "/visuals" in path
                and stage.GetPrimAtPath(
                    prim.GetPath().GetParentPath().AppendChild("collisions")).IsValid()):
            where = _author(stage, prim, "physics:collisionEnabled", False,
                            Sdf.ValueTypeNames.Bool, ("PhysicsCollisionAPI",))
            fixed.append((str(prim.GetPath()), "visuals 중복 콜라이더", spread, where))
            mark += "  -> collisionEnabled=False (%s)" % where

        if enabled is False:
            mark += "  (collisionEnabled=False)"

        short = str(prim.GetPath())
        if len(short) > 50:
            short = "..." + short[-47:]
        print("%-52s %-18s %.3f%s" % (short, approx or "(mesh 아님/box)", spread, mark))

    print("-" * 72)
    print("콜라이더 %d 개 점검, %d 개 수정 (카터 쪽 %d 개는 건드리지 않음)"
          % (checked, len(fixed), len(skipped)))
    for path, before, spread, where in fixed:
        print("   %s\n      (%s, 퍼짐 %.3f m) -> %s" % (path, before, spread, where))

    # 팔 Drive 게인 (옵션)
    if ARM_DRIVE_STIFFNESS is not None:
        n = 0
        for prim in stage.Traverse():
            if prim.GetTypeName() != "PhysicsRevoluteJoint":
                continue
            if not prim.GetName().startswith("joint_"):
                continue
            if not prim.HasAPI(UsdPhysics.DriveAPI, "angular"):
                continue
            d = UsdPhysics.DriveAPI(prim, "angular")
            d.GetStiffnessAttr().Set(float(ARM_DRIVE_STIFFNESS))
            d.GetDampingAttr().Set(float(ARM_DRIVE_DAMPING))
            n += 1
        print("팔 관절 Drive %d 개를 k=%g c=%g 로 올림" % (n, ARM_DRIVE_STIFFNESS, ARM_DRIVE_DAMPING))

    if SAVE and fixed:
        stage.GetRootLayer().Save()
        print("저장 완료: %s" % stage.GetRootLayer().identifier)
    elif not fixed:
        print("고칠 것이 없어 저장하지 않았습니다.")
    print("=" * 72)


main()
