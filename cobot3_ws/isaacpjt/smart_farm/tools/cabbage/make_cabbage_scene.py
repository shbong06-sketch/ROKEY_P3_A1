"""원본 v013 씬에서 양배추 씬 v014 를 만든다 (원본 파일은 그대로).

  isaacsim/python.sh make_cabbage_scene.py SCENE_USD ASSET_DIR [OUT_NAME]
  (Windows: isaacsim\\python.bat). 씬 파일은 Isaac Sim 5.1 의 USD 로 써야 해서 이 스크립트가 Isaac 을 headless 로 띄운다.
  기본 출력: 씬 폴더에 Collected_smartfarm_v014_room_core_cabbage.usd (원본 이름의 v013 -> v014, 뒤에 _cabbage)

* copies ASSET_DIR (cabbage_pallet_6.usd, empty tray, single heads, textures, build_info.json)
  to <scene dir>/assets/cabbage_pallet_6
* copies the root layer to OUT_NAME and, in that copy only, replaces every
  reference to romaine_pallet_6_v005*.usd with cabbage_pallet_6.usd and sets xformOp:scale to 1
  (the cabbage tray has the scenes' x0.6 tray width baked in).
  This must be done in the root layer itself: the root layer is stronger than any of its sublayers, so a swap
  authored in a sublayer is merged with (not replacing) the root's prepended romaine reference.
* verifies the composed copy: no Romaine_* prims left, every tray has 6 Cabbage_* bodies, scale 1.
"""
import atexit, os, sys, shutil, json, traceback

if "omni.kit.app" not in sys.modules:              # 혼자 실행: Isaac 을 headless 로 띄운다 (씬 파일은 Isaac USD 로 저장)
    from isaacsim import SimulationApp
    _app = SimulationApp({"headless": True})
    atexit.register(_app.close)

    def _excepthook(kind, value, tb):              # Kit 종료 때 stderr 가 사라질 수 있어 파일로도 남긴다
        open(os.path.abspath(__file__) + ".error.txt", "w", encoding="utf-8").write(
            "".join(traceback.format_exception(kind, value, tb)))
        sys.__excepthook__(kind, value, tb)
    sys.excepthook = _excepthook
from pxr import Usd, Sdf, UsdPhysics, Gf  # noqa: E402

SCENE, ASSET_DIR = os.path.abspath(sys.argv[1]), os.path.abspath(sys.argv[2])
scene_dir = os.path.dirname(SCENE)
default_name = os.path.splitext(os.path.basename(SCENE))[0].replace("_v013", "_v014") + "_cabbage.usd"
out = os.path.join(scene_dir, sys.argv[3] if len(sys.argv) > 3 else default_name)
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

# user request 2026-09-25: vision-room layout for the inspection -> cull pick-and-place.
# The conveyor line runs at y -6.73, 0.89 m from the vision M0609 (top-down pick + 18 cm approach reaches ~0.6 m),
# so the pedestal + arm move up to the conveyor frame (+0.146 m, north face at y -7.33) and the two SortBoxes stand
# on the floor beside the pedestal (0.41 m from the base). World-space moves, applied in this root-layer copy only.
STATION_LAYOUT = {
    "/World/SmartFarm/Placed/M0609": (0.0, 0.146, 0.0),
    "/World/SmartFarm/RobotZone/Pedestal": (0.0, 0.146, 0.0),
    "/World/SmartFarm/RobotZone/SortBox_1": ("to", -1.017, -7.700),
    "/World/SmartFarm/RobotZone/SortBox_2": ("to", -0.327, -7.700),
}
st = Usd.Stage.Open(out)
st.SetEditTarget(st.GetRootLayer())
xc = UsdGeom.XformCache(0)
layout = {}
for path, move in STATION_LAYOUT.items():
    prim = st.GetPrimAtPath(path)
    if not prim:
        layout[path] = "missing"
        continue
    world = xc.GetLocalToWorldTransform(prim).ExtractTranslation()
    delta = (move[1] - world[0], move[2] - world[1], 0.0) if move[0] == "to" else move
    parent = xc.GetLocalToWorldTransform(prim.GetParent()).RemoveScaleShear()
    local = parent.GetInverse().TransformDir(Gf.Vec3d(*delta))
    op = next((o for o in UsdGeom.Xformable(prim).GetOrderedXformOps() if o.GetOpType() == UsdGeom.XformOp.TypeTranslate), None)
    if op is None:
        layout[path] = "no translate op"
        continue
    v = op.Get()
    op.Set(type(v)(v[0] + local[0], v[1] + local[1], v[2] + local[2]))
    layout[path] = [round(world[0] + delta[0], 4), round(world[1] + delta[1], 4), round(world[2], 4)]
    print("layout", path, layout[path])

# user request 2026-09-25: (a) the conveyor openings of the vision-room walls had a tall notch in the middle of the
# lintel (BackWall_03_Head / BackWall_04_Head, 16-point meshes) -> replace each lintel with a plain box over the
# opening; (b) SortBox_1/2 were solid cubes (culled heads came to rest on the lid) -> hide them and build an
# open-top bin (floor + 4 walls, same footprint / colour) as SortBin_1/2. Low-poly, static colliders.
from pxr import UsdShade
bbc = UsdGeom.BBoxCache(0, [UsdGeom.Tokens.default_, UsdGeom.Tokens.render])

def copy_look(src, dst):
    mat = UsdShade.MaterialBindingAPI(src).GetDirectBinding().GetMaterial()
    if mat:
        UsdShade.MaterialBindingAPI.Apply(dst).Bind(mat)
    col = UsdGeom.Gprim(src).GetDisplayColorAttr().Get() if src.IsA(UsdGeom.Gprim) else None
    if col:
        UsdGeom.Gprim(dst).CreateDisplayColorAttr(col)

def world_box(path, lo, hi, look_from, parent_path):
    parent = st.GetPrimAtPath(parent_path)
    pinv = xc.GetLocalToWorldTransform(parent).GetInverse() if parent_path != "/" else Gf.Matrix4d(1)
    c = UsdGeom.Cube.Define(st, path)
    c.CreateSizeAttr(1.0)
    centre = pinv.Transform(Gf.Vec3d(*[(a + b) / 2 for a, b in zip(lo, hi)]))
    c.AddTranslateOp().Set(centre)
    c.AddScaleOp().Set(Gf.Vec3f(*[b - a for a, b in zip(lo, hi)]))
    UsdPhysics.CollisionAPI.Apply(c.GetPrim())
    copy_look(look_from, c.GetPrim())
    return c

scene_fix = {}
for wall in (() if os.environ.get("SKIP_SCENEFIX") else ("BackWall_03_Head", "BackWall_04_Head")):
    old = st.GetPrimAtPath(f"/{wall}")
    if not old:
        scene_fix[wall] = "missing"
        continue
    rng = bbc.ComputeWorldBound(old).ComputeAlignedRange()
    lo, hi = list(rng.GetMin()), list(rng.GetMax())
    world_box(f"/{wall}_Flat", lo, hi, old, "/")
    old.SetActive(False)
    scene_fix[wall] = {"replaced_by": f"/{wall}_Flat", "min": [round(v, 3) for v in lo], "max": [round(v, 3) for v in hi]}
T = 0.02      # bin wall / floor thickness
for i in (() if os.environ.get("SKIP_SCENEFIX") else (1, 2)):
    old = st.GetPrimAtPath(f"/World/SmartFarm/RobotZone/SortBox_{i}")
    if not old:
        scene_fix[f"SortBox_{i}"] = "missing"
        continue
    rng = bbc.ComputeWorldBound(old).ComputeAlignedRange()
    (x0, y0, z0), (x1, y1, z1) = rng.GetMin(), rng.GetMax()
    root = f"/World/SmartFarm/RobotZone/SortBin_{i}"
    UsdGeom.Xform.Define(st, root)
    parts = {"Floor": ((x0, y0, z0), (x1, y1, z0 + 0.05)),     # 바닥은 두껍게: 2 cm 에서 떨어진 포기가 뚫고 내려감
             "WallW": ((x0, y0, z0), (x0 + T, y1, z1)), "WallE": ((x1 - T, y0, z0), (x1, y1, z1)),
             "WallS": ((x0, y0, z0), (x1, y0 + T, z1)), "WallN": ((x0, y1 - T, z0), (x1, y1, z1))}
    for name, (lo, hi) in parts.items():
        world_box(f"{root}/{name}", lo, hi, old, root)
    UsdGeom.Imageable(old).MakeInvisible()
    UsdPhysics.CollisionAPI(old).CreateCollisionEnabledAttr().Set(False)
    scene_fix[f"SortBox_{i}"] = {"replaced_by": root, "min": [round(v, 3) for v in (x0, y0, z0)], "max": [round(v, 3) for v in (x1, y1, z1)]}
# Conveyor junction exit: the collider of the feeder frame part SM_ConveyorBelt_A49_01 stands ~2 mm above the line
# roller tops at x -1.68. Once the junction wheels drop, the tray's rear skids rest on it and the tray stalls
# (CONVEYOR_FAILED at x -1.424, PhysX contact report 2026-09-25). Trays are carried by rollers / wheels, so this frame
# part does not need a collider.
lip = st.GetPrimAtPath("/World/SmartFarm/Placed/Conveyor/Feeder/Geometry/SM_ConveyorBelt_A49_01")
if lip and lip.HasAPI(UsdPhysics.CollisionAPI):
    UsdPhysics.CollisionAPI(lip).CreateCollisionEnabledAttr().Set(False)
    scene_fix["SM_ConveyorBelt_A49_01"] = "collision disabled (junction exit lip stalled the trays)"
layout["scene_fix"] = scene_fix
# user request 2026-09-25: one fixed camera per process, selectable in the viewport camera list and recorded by
# aio_wrapper (CABBAGE_CAPTURE_CAMS). Cam3 (inspection screen) is the vision M0609 wrist RealSense itself.
PROCESS_CAMERAS = {
    "Cam1_Harvest": ((1.3, -0.6, 2.6), (-1.0, 1.0, 0.9)),
    "Cam2_Nav2Place": ((1.0, -5.2, 3.2), (-1.7, -1.6, 0.5)),
    "Cam4_CullPickPlace": ((0.25, -5.95, 2.25), (-0.72, -7.45, 0.40)),
    "Cam5_Pusher": ((-1.35, -6.05, 1.55), (-0.69, -6.95, 0.80)),     # 이송 프레임(PlateN·PlateS) 동작
}
UsdGeom.Xform.Define(st, "/World/ProcessCameras")
for name, (eye, target) in PROCESS_CAMERAS.items():
    cam = UsdGeom.Camera.Define(st, f"/World/ProcessCameras/{name}")
    cam.CreateFocalLengthAttr(12.0 if name.startswith("Cam4") else 16.0)
    cam.CreateClippingRangeAttr(Gf.Vec2f(0.05, 1000.0))
    m = Gf.Matrix4d().SetLookAt(Gf.Vec3d(*eye), Gf.Vec3d(*target), Gf.Vec3d(0, 0, 1)).GetInverse()
    xf = UsdGeom.Xformable(cam)
    xf.ClearXformOpOrder()
    xf.AddTransformOp().Set(m)
layout["process_cameras"] = {k: {"eye": v[0], "target": v[1]} for k, v in PROCESS_CAMERAS.items()}
# user request 2026-09-25: a camera at the viewport Perspective position. The scene's saved Perspective
# (customLayerData cameraSettings) and every view saved from the running GUI with aio_wrapper's SAVE_VIEW_CAMERA
# trigger (out/saved_view_cameras.json) become cameras too.
cs = dict(st.GetRootLayer().customLayerData).get("cameraSettings", {})
views = []
if "Perspective" in cs and "target" in cs["Perspective"]:
    views.append(("Cam0_Perspective", tuple(cs["Perspective"]["position"]), tuple(cs["Perspective"]["target"]), 18.147))
saved = os.environ.get("SAVED_VIEW_CAMERAS", os.path.join(scene_dir, "saved_view_cameras.json"))
if os.path.exists(saved):
    for i, v in enumerate(json.load(open(saved))):
        views.append((v.get("name", f"Cam0_View{i + 1}"), None, v["matrix"], v.get("focal", 18.147)))
for name, eye, target, focal in views:
    cam = UsdGeom.Camera.Define(st, f"/World/ProcessCameras/{name}")
    cam.CreateFocalLengthAttr(float(focal))
    cam.CreateClippingRangeAttr(Gf.Vec2f(0.05, 1000.0))
    xf = UsdGeom.Xformable(cam)
    xf.ClearXformOpOrder()
    if eye is None:                                   # full world matrix saved from the GUI viewport (row-major)
        xf.AddTransformOp().Set(Gf.Matrix4d(*[float(x) for x in target]))
    else:
        xf.AddTransformOp().Set(Gf.Matrix4d().SetLookAt(Gf.Vec3d(*eye), Gf.Vec3d(*target), Gf.Vec3d(0, 0, 1)).GetInverse())
    layout["process_cameras"][name] = {"eye": eye, "target": target if eye is not None else "matrix"}

# user request 2026-09-25: the Nova Carter + M0609 rig's lift frame (lift_holder: two posts + a top crossbar up to
# z 1.48) stood 0.44 m above the arm base and the M0609 swung through it (same articulation -> no contact, it just
# passes through). Keep the robot position and the lift travel, make the mast telescopic:
#   * outer posts of lift_holder cut down so their top (with the crossbar) is at MAST_TOP, just under the lowest
#     lift plate (lift_moveparts_1 bottom 0.788 at joint 0)
#   * inner posts added to the lift plate (move with it), long enough to overlap the outer posts over the whole
#     0.61 m travel (their lower part is hidden inside the Carter body at low lift). Visual only, no collider.
import numpy as np
RIG = "/World/SmartFarm/Placed/LiftRig/Asset/nova_carter_ROS/lift_v3_physics"
MAST_CUT_ABOVE, MAST_TOP, INNER_LEN = 0.95, 0.78, 0.63
holder = st.GetPrimAtPath(f"{RIG}/lift_holder/Mesh")
plate = st.GetPrimAtPath(f"{RIG}/lift_moveparts_1")
mast = {}
if os.environ.get("SKIP_MAST"):
    holder = None
    mast = {"skipped": True}
if holder and plate:
    xc2 = UsdGeom.XformCache(0)
    M = np.array(xc2.GetLocalToWorldTransform(holder)).T                  # column convention
    mesh = UsdGeom.Mesh(holder)
    pts = np.array(mesh.GetPointsAttr().Get(), dtype=float)
    w = (M[:3, :3] @ pts.T).T + M[:3, 3]
    top_old = float(w[:, 2].max())
    sel = w[:, 2] > MAST_CUT_ABOVE
    w[sel, 2] -= top_old - MAST_TOP
    local = (np.linalg.inv(M[:3, :3]) @ (w - M[:3, 3]).T).T
    mesh.GetPointsAttr().Set([Gf.Vec3f(*map(float, p)) for p in local])
    mesh.GetExtentAttr().Set([Gf.Vec3f(*map(float, local.min(0))), Gf.Vec3f(*map(float, local.max(0)))])
    posts = []                                                           # footprints of the moved (post top) vertices
    for xs in (w[sel & (w[:, 0] < -0.4)], w[sel & (w[:, 0] >= -0.4)]):
        posts.append(((xs[:, 0].min(), xs[:, 0].max()), (xs[:, 1].min(), xs[:, 1].max())))
    plate_bottom = float(np.array(UsdGeom.BBoxCache(0, [UsdGeom.Tokens.default_]).ComputeWorldBound(plate)
                                  .ComputeAlignedRange().GetMin())[2])
    Pw = xc2.GetLocalToWorldTransform(plate)                               # Gf row convention
    for i, ((x0, x1), (y0, y1)) in enumerate(posts):
        cube = UsdGeom.Cube.Define(st, f"{RIG}/lift_moveparts_1/MastInner_{i}")
        cube.CreateSizeAttr(1.0)
        sx, sy = max(x1 - x0, 0.01) * 0.8, max(y1 - y0, 0.01) * 0.8
        world = Gf.Matrix4d().SetScale(Gf.Vec3d(sx, sy, INNER_LEN)) * \
            Gf.Matrix4d().SetTranslate(Gf.Vec3d((x0 + x1) / 2, (y0 + y1) / 2, plate_bottom - INNER_LEN / 2))
        xf = UsdGeom.Xformable(cube)
        xf.ClearXformOpOrder()
        xf.AddTransformOp().Set(world * Pw.GetInverse())
        copy_look(holder, cube.GetPrim())
    mast = {"old_top": round(top_old, 3), "new_top": MAST_TOP, "moved_vertices": int(sel.sum()),
            "posts_xy": [[round(float(v), 3) for v in (x0, x1, y0, y1)] for (x0, x1), (y0, y1) in posts],
            "inner_post_length": INNER_LEN, "plate_bottom": round(plate_bottom, 3)}
layout["telescopic_mast"] = mast or "lift_holder not found"

# user request 2026-09-25: make the shared scene lighter without changing anything visible or functional.
#   * Nova Carter chassis_link/visual/internal_components: 1.52 M points of parts inside the closed body shell
#     (skirt + top_body stay). It is an instance, so only this Carter's visual is de-instanced to switch it off.
#   * hidden meshes that are not colliders (viewport camera gizmos, an unused hidden mesh in the fork robot base).
lighten = {}
visual = st.GetPrimAtPath("/World/SmartFarm/Placed/LiftRig/Asset/nova_carter_ROS/chassis_link/visual")
if visual:
    visual.SetInstanceable(False)
    inner = st.GetPrimAtPath(str(visual.GetPath()) + "/internal_components")
    if inner:
        inner.SetActive(False)
        lighten[str(inner.GetPath())] = "deactivated (inside the chassis shell, no collider)"

def carries_physics(prim):
    """collider / body on the prim or any ancestor, or joints / articulations below it"""
    q = prim
    while q and q.GetPath() != Sdf.Path.absoluteRootPath:
        if q.HasAPI(UsdPhysics.CollisionAPI) or q.HasAPI(UsdPhysics.RigidBodyAPI):
            return True
        q = q.GetParent()
    return False

for prim in st.Traverse():
    if not prim.IsA(UsdGeom.Gprim) or prim.IsInstanceProxy():
        continue
    img = UsdGeom.Imageable(prim)
    hidden = img.ComputeVisibility() == UsdGeom.Tokens.invisible
    if hidden and not carries_physics(prim) and "/SortBox_" not in str(prim.GetPath()):
        prim.SetActive(False)
        lighten[str(prim.GetPath())] = "deactivated (invisible, no physics)"
layout["lighten"] = lighten
st.GetRootLayer().Save()
del st

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
# user request 2026-09-25: the old test trays on the conveyor (Pallet_Inspect, _01.._03: showed for a few seconds on
# Play until the all-in-one moved them off the belt) and the Lettuce_1..3 cubes are deleted from this copy: their
# prim specs are removed from the root layer; anything still composed from a sublayer is deactivated.
del st
DELETE = [f"/World/SmartFarm/Placed/Pallet_Inspect{s}" for s in ("", "_01", "_02", "_03")] + \
         [f"/World/SmartFarm/Placed/Lettuce_{i}" for i in (1, 2, 3)]
root_layer = Sdf.Layer.FindOrOpen(out)
deleted = {}
for path in DELETE:
    spec = root_layer.GetPrimAtPath(path)
    if spec is not None:
        parent = root_layer.GetPrimAtPath(Sdf.Path(path).GetParentPath())
        del parent.nameChildren[spec.name]
        deleted[path] = "removed"
root_layer.Save()
st = Usd.Stage.Open(out)
st.SetEditTarget(st.GetRootLayer())
for path in DELETE:
    p = st.GetPrimAtPath(path)
    if p and p.IsActive():
        p.SetActive(False)
        deleted[path] = "deactivated (defined in a sublayer)"
    deleted.setdefault(path, "not present")
st.GetRootLayer().Save()
swapped = [p for p in swapped if p not in DELETE]
conditions = {k: v for k, v in conditions.items() if not k.startswith("Pallet_Inspect")}
layout["deleted"] = deleted
for p in swapped:
    print("swapped", p)
print("romaine prims left:", len(romaine), romaine[:5])
print("problems:", problems)
print("scene", out)
json.dump({"scene": out, "swapped": swapped, "removed": removed, "station_layout": layout, "conditions": conditions, "romaine_left": romaine, "problems": [list(map(str, p)) for p in problems], "condition_mismatch": bad_cond},
          open(out + ".swap_report.json", "w"), indent=1)
if romaine or problems or bad_cond or any(isinstance(v, str) for v in layout.values()):
    raise SystemExit("cabbage scene verification FAILED")
