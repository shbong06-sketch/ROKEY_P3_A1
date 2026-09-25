"""Make a cabbage variant of a Collected_smartfarm scene without touching the original files.

  D:\\isaacsim\\python.bat run_in_isaac.py 06_make_cabbage_scene.py SCENE_USD ASSET_DIR [OUT_NAME]
  (run through run_in_isaac.py: the root layers are binary .usd and must be written by Isaac Sim's own USD)

* copies ASSET_DIR (cabbage_pallet_6.usd, empty tray, single heads, textures, build_info.json)
  to <scene dir>/assets/cabbage_pallet_6
* copies the root layer to OUT_NAME (default <scene>_cabbage.usd) and, in that copy only, replaces every
  reference to romaine_pallet_6_v005*.usd with cabbage_pallet_6.usd and sets xformOp:scale to 1
  (the cabbage tray has the scenes' x0.6 tray width baked in).
  This must be done in the root layer itself: the root layer is stronger than any of its sublayers, so a swap
  authored in a sublayer is merged with (not replacing) the root's prepended romaine reference.
* verifies the composed copy: no Romaine_* prims left, every tray has 6 Cabbage_* bodies, scale 1.
"""
import os, sys, shutil, json
from pxr import Usd, Sdf, UsdPhysics

SCENE, ASSET_DIR = os.path.abspath(sys.argv[1]), os.path.abspath(sys.argv[2])
scene_dir = os.path.dirname(SCENE)
out = os.path.join(scene_dir, sys.argv[3] if len(sys.argv) > 3 else os.path.splitext(os.path.basename(SCENE))[0] + "_cabbage.usd")
NEW_REF = "./assets/cabbage_pallet_6/cabbage_pallet_6.usd"
INSPECT_REF = "./assets/cabbage_pallet_6/cabbage_pallet_6_inspect.usd"
# user request 2026-09-25: the rack trays the fork carries to the conveyor must need vision sorting too, so they use the
# inspection asset with the team's inspection patterns (Inspection_Node 설계정리 tray table).
RACK_PATTERNS = {
    "Pallet_01": ["green", "yellow", "brown", "green", "yellow", "green"],   # = Pallet_Inspect: defect 03, hold 02 05
    "Pallet_02": ["green", "yellow", "brown", "green", "yellow", "brown"],   # = Pallet_Inspect_03: defect 03 06
    "Pallet_03": ["green", "green", "yellow", "green", "yellow", "green"],   # = Pallet_Inspect_01: hold 03 05
}

dst_assets = os.path.join(scene_dir, "assets", "cabbage_pallet_6")
os.makedirs(os.path.join(dst_assets, "textures"), exist_ok=True)
for f in ("cabbage_pallet_6.usd", "cabbage_pallet_6_inspect.usd", "cabbage_pallet_empty.usd", "cabbage_A.usd",
          "cabbage_B.usd", "cabbage_C.usd", "build_info.json"):
    shutil.copy2(os.path.join(ASSET_DIR, f), os.path.join(dst_assets, f))
for f in os.listdir(os.path.join(ASSET_DIR, "textures")):
    shutil.copy2(os.path.join(ASSET_DIR, "textures", f), os.path.join(dst_assets, "textures", f))

# per-slot colour of every romaine head in the ORIGINAL scene (inspection trays carry yellow / brown heads)
from pxr import UsdGeom, UsdShade
orig = Usd.Stage.Open(SCENE)
slot_cond = {}
for p in orig.Traverse():
    if p.GetName().startswith("Romaine_"):
        meshes = [c for c in Usd.PrimRange(p) if c.IsA(UsdGeom.Mesh) and c.GetName().startswith("head")]
        mat = UsdShade.MaterialBindingAPI(meshes[0]).ComputeBoundMaterial()[0] if meshes else None
        name = mat.GetPath().name.lower() if mat else ""
        slot_cond[str(p.GetPath())] = "yellow" if "yellow" in name else "brown" if "brown" in name else "green"
del orig

shutil.copy2(SCENE, out)
layer = Sdf.Layer.FindOrOpen(out)
swapped = []
def visit(path):
    spec = layer.GetObjectAtPath(path)
    if not isinstance(spec, Sdf.PrimSpec):
        return
    rl = spec.referenceList
    for field in ("explicitItems", "prependedItems", "appendedItems"):
        items = list(getattr(rl, field))
        if any("romaine_pallet_6_v005" in r.assetPath for r in items):
            ref = INSPECT_REF if (path.name in RACK_PATTERNS or "_inspect" in "".join(r.assetPath for r in items)) else NEW_REF
            setattr(rl, field, [Sdf.Reference(ref) if "romaine_pallet_6_v005" in r.assetPath else r for r in items])
            sc = spec.attributes.get("xformOp:scale")
            if sc is not None:
                sc.default = type(sc.default)(1.0, 1.0, 1.0)
            swapped.append(str(path))
layer.Traverse(Sdf.Path.absoluteRootPath, visit)
# user request 2026-09-25: the three old vision test cubes (Placed/Lettuce_1..3) sit on the inspection belt next to
# the vision stop line; deactivate them in the copy (the original scene keeps them).
removed = []
for name in ("Lettuce_1", "Lettuce_2", "Lettuce_3"):
    path = Sdf.Path(f"/World/SmartFarm/Placed/{name}")
    spec = Sdf.CreatePrimInLayer(layer, path)     # an inactive over is harmless where the prim does not exist
    spec.active = False
    removed.append(str(path))

conditions = {}
for tray in swapped:                          # carry the romaine colour layout over as condition variant selections
    for i in range(1, 7):
        name = tray.split("/")[-1]
        cond = RACK_PATTERNS[name][i - 1] if name in RACK_PATTERNS else slot_cond.get(f"{tray}/root_001/Romaine_{i:02d}", "green")
        head = Sdf.CreatePrimInLayer(layer, f"{tray}/root_001/Cabbage_{i:02d}")
        head.variantSelections["condition"] = cond
        conditions[f"{tray.split('/')[-1]}/SLOT_{i:02d}"] = cond
layer.Save()

st = Usd.Stage.Open(out)
romaine = [str(p.GetPath()) for p in st.Traverse() if p.GetName().startswith("Romaine_")]
bad_cond = [k for k, v in conditions.items()
            if st.GetPrimAtPath("/World/SmartFarm/Placed/" + k.replace("SLOT_", "root_001/Cabbage_")).GetVariantSets().GetVariantSelection("condition") != v]
problems = []
for path in swapped:
    p = st.GetPrimAtPath(path)
    heads = [c for c in Usd.PrimRange(p) if c.GetName().startswith("Cabbage_") and c.HasAPI(UsdPhysics.RigidBodyAPI)]
    sc = p.GetAttribute("xformOp:scale").Get()
    if len(heads) != 6 or (sc is not None and tuple(sc) != (1.0, 1.0, 1.0)):
        problems.append((path, len(heads), sc))
for p in swapped:
    print("swapped", p)
print("romaine prims left:", len(romaine), romaine[:5])
print("problems:", problems)
print("scene", out)
json.dump({"scene": out, "swapped": swapped, "removed": removed, "conditions": conditions, "romaine_left": romaine, "problems": [list(map(str, p)) for p in problems], "condition_mismatch": bad_cond},
          open(out + ".swap_report.json", "w"), indent=1)
if romaine or problems or bad_cond:
    raise SystemExit("cabbage scene verification FAILED")
