import os
import shutil
from datetime import datetime

import omni.usd
from pxr import PhysxSchema, Sdf, Usd, UsdPhysics, UsdShade


# ── 설정 ──────────────────────────────────────────────
FORK_ROOT_PATH = (
    "/World/SmartFarm/Placed/LiftRig/Asset/nova_carter_ROS"
    "/m0609_with_fork/fork_tool"
)
PALLET_ROOT_PATHS = [
    "/World/SmartFarm/Placed/Pallet_1/Asset",
    "/World/SmartFarm/Placed/Pallet_2/Asset",
    "/World/SmartFarm/Placed/Pallet_3/Asset",
]

MATERIAL_PATH = "/World/IntegrationMaterials/ForkPalletGrip"

STATIC_FRICTION = 1.5
DYNAMIC_FRICTION = 1.2
RESTITUTION = 0.0
FRICTION_COMBINE_MODE = "max"

OVERRIDE_FILENAME = "integration_v1_physics.usda"
CREATE_ROOT_BACKUP = True


# ── Stage 확인 ────────────────────────────────────────
context = omni.usd.get_context()
stage = context.get_stage()

if stage is None:
    raise RuntimeError("열려 있는 USD Stage가 없습니다.")

root_layer = stage.GetRootLayer()
root_path = root_layer.realPath

if not root_path:
    raise RuntimeError("현재 Stage의 root layer가 파일로 저장되어 있지 않습니다.")

scene_dir = os.path.dirname(root_path)
override_path = os.path.join(scene_dir, OVERRIDE_FILENAME)

print(f"[Friction] root layer: {root_path}")
print(f"[Friction] override layer: {override_path}")


# ── Root layer 백업 ───────────────────────────────────
if CREATE_ROOT_BACKUP:
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    backup_path = f"{root_path}.before_friction_{timestamp}.bak"
    shutil.copy2(root_path, backup_path)
    print(f"[Friction] root backup: {backup_path}")


# ── Override layer 생성 및 연결 ───────────────────────
override_layer = Sdf.Layer.FindOrOpen(override_path)

if override_layer is None:
    override_layer = Sdf.Layer.CreateNew(override_path)
    if override_layer is None:
        raise RuntimeError(f"Override layer 생성 실패: {override_path}")

relative_override_path = os.path.relpath(override_path, scene_dir)

if relative_override_path not in root_layer.subLayerPaths:
    # 앞에 배치하여 기존 sublayer보다 강한 opinion으로 사용합니다.
    root_layer.subLayerPaths.insert(0, relative_override_path)
    print(f"[Friction] sublayer 연결: {relative_override_path}")
else:
    print("[Friction] sublayer가 이미 연결되어 있습니다.")

stage.SetEditTarget(override_layer)


# ── Physics Material 생성 ─────────────────────────────
stage.DefinePrim("/World/IntegrationMaterials", "Scope")
usd_material = UsdShade.Material.Define(stage, MATERIAL_PATH)
material_prim = usd_material.GetPrim()

physics_material = UsdPhysics.MaterialAPI.Apply(material_prim)
physics_material.CreateStaticFrictionAttr().Set(STATIC_FRICTION)
physics_material.CreateDynamicFrictionAttr().Set(DYNAMIC_FRICTION)
physics_material.CreateRestitutionAttr().Set(RESTITUTION)

physx_material = PhysxSchema.PhysxMaterialAPI.Apply(material_prim)
physx_material.CreateFrictionCombineModeAttr().Set(
    FRICTION_COMBINE_MODE
)

print(
    "[Friction] material: "
    f"static={STATIC_FRICTION}, "
    f"dynamic={DYNAMIC_FRICTION}, "
    f"restitution={RESTITUTION}, "
    f"combine={FRICTION_COMBINE_MODE}"
)


# ── Material 바인딩 ───────────────────────────────────
def bind_material_to_collision_tree(label, root_prim_path):
    root_prim = stage.GetPrimAtPath(root_prim_path)

    if not root_prim.IsValid():
        raise RuntimeError(f"{label} prim이 없습니다: {root_prim_path}")

    # Root에도 강한 inherited binding을 적용합니다.
    UsdShade.MaterialBindingAPI.Apply(root_prim).Bind(
        usd_material,
        UsdShade.Tokens.strongerThanDescendants,
        "physics",
    )

    collision_paths = []

    # 포크 충돌체는 인스턴스 프록시입니다. 기본 순회는 프록시 안으로
    # 들어가지 않아 0개가 잡히므로 TraverseInstanceProxies 가 필요합니다.
    for prim in Usd.PrimRange(
        root_prim, Usd.TraverseInstanceProxies(Usd.PrimAllPrimsPredicate)
    ):
        if not prim.HasAPI(UsdPhysics.CollisionAPI):
            continue

        collision_paths.append(str(prim.GetPath()))

        # Instance proxy에는 직접 authoring할 수 없으므로 root binding을 상속시킵니다.
        if prim.IsInstanceProxy():
            print(f"  [inherited] {prim.GetPath()}")
            continue

        UsdShade.MaterialBindingAPI.Apply(prim).Bind(
            usd_material,
            UsdShade.Tokens.strongerThanDescendants,
            "physics",
        )
        print(f"  [bound] {prim.GetPath()}")

    if not collision_paths:
        raise RuntimeError(
            f"{label} 아래에서 CollisionAPI prim을 찾지 못했습니다: "
            f"{root_prim_path}"
        )

    print(f"[Friction] {label}: collider {len(collision_paths)}개")


bind_material_to_collision_tree("fork", FORK_ROOT_PATH)
for _pallet_path in PALLET_ROOT_PATHS:
    _label = _pallet_path.split("/")[-2]      # 예: Pallet_1
    bind_material_to_collision_tree(f"pallet {_label}", _pallet_path)


# ── 저장 ──────────────────────────────────────────────
override_layer.Save()
root_layer.Save()

print("[Friction] 저장 완료")
print(f"  override: {override_path}")
print("  Timeline을 Play하여 CARRY_ROTATE를 다시 시험하세요.")
