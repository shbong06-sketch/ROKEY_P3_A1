import os
import sys
import shutil
import datetime
import argparse
from pxr import Usd, UsdGeom, UsdPhysics, UsdShade

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from preflight import run_preflight_or_raise
from mesh_utils import triangulate_mesh
from physics_presets import PRESETS


def apply_physics_test_cabbage(input_path: str, preset_name: str = "cabbage_v1",
                                overwrite: bool = False, add_debug_color: bool = True,
                                triangulate: bool = False):
    run_preflight_or_raise(require_physx=False)

    cfg = PRESETS[preset_name]
    if not os.path.exists(input_path):
        raise FileNotFoundError(input_path)

    backup_path = input_path + f".bak_{datetime.datetime.now().strftime('%Y%m%d%H%M%S')}"
    shutil.copy2(input_path, backup_path)
    print(f"[정보] 백업: {backup_path}")

    stage = Usd.Stage.Open(input_path)
    default_prim = stage.GetDefaultPrim()
    if not default_prim or not default_prim.IsValid():
        raise RuntimeError("defaultPrim이 없습니다.")

    mesh_prims = [p for p in stage.Traverse() if p.IsA(UsdGeom.Mesh)]
    print(f"[정보] RigidBody 대상: {default_prim.GetPath()} / Mesh {len(mesh_prims)}개")

    if triangulate:
        for mesh_prim in mesh_prims:
            triangulate_mesh(mesh_prim)

    UsdPhysics.RigidBodyAPI.Apply(default_prim)
    UsdPhysics.MassAPI.Apply(default_prim).CreateMassAttr(cfg["mass_kg"])

    material_path = default_prim.GetPath().AppendChild(f"PhysicsMaterial_{preset_name}")
    material_prim = stage.DefinePrim(material_path, "Material")
    phys_mat = UsdPhysics.MaterialAPI.Apply(material_prim)
    phys_mat.CreateStaticFrictionAttr(cfg["static_friction"])
    phys_mat.CreateDynamicFrictionAttr(cfg["dynamic_friction"])
    phys_mat.CreateRestitutionAttr(cfg["restitution"])

    for mesh_prim in mesh_prims:
        UsdPhysics.CollisionAPI.Apply(mesh_prim)
        mesh_col = UsdPhysics.MeshCollisionAPI.Apply(mesh_prim)
        mesh_col.CreateApproximationAttr("convexHull")
        UsdShade.MaterialBindingAPI.Apply(mesh_prim).Bind(
            UsdShade.Material(material_prim),
            UsdShade.Tokens.strongerThanDescendants, "physics",
        )
        if add_debug_color:
            UsdGeom.Mesh(mesh_prim).CreateDisplayColorAttr([cfg["debug_color"]])
        print(f"[적용] {mesh_prim.GetPath()} -> convexHull + physics material")

    if overwrite:
        stage.Save()
        print(f"[완료] 덮어쓰기: {input_path}")
    else:
        out_path = os.path.splitext(input_path)[0] + f"_{preset_name}_physics.usd"
        stage.Export(out_path)
        print(f"[완료] 저장: {out_path}")


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("input_path")
    parser.add_argument("--preset", default="cabbage_v1")
    parser.add_argument("--overwrite", action="store_true")
    parser.add_argument("--triangulate", action="store_true")
    parser.add_argument("--no-debug-color", action="store_true")
    args = parser.parse_args()

    apply_physics_test_cabbage(
        args.input_path,
        preset_name=args.preset,
        overwrite=args.overwrite,
        add_debug_color=not args.no_debug_color,
        triangulate=args.triangulate,
    )
