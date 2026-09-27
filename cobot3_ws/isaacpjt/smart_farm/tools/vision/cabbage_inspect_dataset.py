"""YOLO dataset for the cabbage sorting station (v014 scene): cabbage heads seated in the 6-hole tray in front of the
vision M0609, seen by its wrist RealSense - the same geometry the station uses (inspection_cull_station.py).

Replaces the romaine dataset (62_v011_tray_inspect_dataset.py) the current best.pt was trained on. Every frame:
  * tray = cabbage_pallet_empty.usd at the station (x STATION_X +-6 cm, y between the lane -6.73 and the inspection
    spot -7.0, long side along the belt, yaw 90/270 +-4 deg), static prop (physics off)
  * each of the 6 seats shows one of 9 heads: shape A/B/C x condition green/yellow/brown (the asset's own variant and
    'condition' semantic label, so box labels come straight from the asset), random yaw, scale 0.95-1.05
  * 30 % of frames a second tray queued on the belt next to it (partial trays at the image edge)
  * the transfer frame (built by the station's own build_transfer_frame) around the tray, lowered in 70 % of frames
  * camera: 50 % station inspection pose (+- jitter), 30 % close views of one head, 20 % wide random views;
    Lula IK puts the real wrist camera there (the gripper fingers are in view as in the station)
  * lighting: vision-room panel, ring light, dome intensity + small colour-temperature tint
Class names/index order are kept (lettuce_dark_green / lettuce_yellow / lettuce_brown) so the model is a drop-in for
the team's station code (CULL_CLASSES / CLASS_SHORT). The scene file is never saved (session layer only).

    D:\\isaacsim\\python.bat cabbage_inspect_dataset.py --frames 2000 --out D:\\smartfarm-sim\\out\\yolo_cabbage_v1
    (--test: held-out set with another seed, station views only, stronger lighting range)
"""

import argparse
import json
import math
import os
import shutil
import sys
import time

REPO = r"D:\smartfarm-sim\ROKEY_P3_A1\cobot3_ws\isaacpjt\smart_farm"
SCENE_DIR = REPO + r"\scenes\Collected_smartfarm_v014"
STAGE = SCENE_DIR + r"\Collected_smartfarm_v014_room_core_cabbage.usd"
ASSETS = SCENE_DIR + r"\assets\cabbage_pallet_6"
URDF = r"D:\smartfarm-sim\robots\doosan-robot2-humble\dsr_description2\urdf\m0609_isaac_sim.urdf"
DESC = r"D:\smartfarm-sim\lula\m0609_robot_description.yaml"

ARM_ART = "/World/SmartFarm/Placed/M0609/Asset/root_joint"
ARM_ROOT = "/World/SmartFarm/Placed/M0609/Asset"
BRACKET = "/World/SmartFarm/Placed/M0609/Asset/onrobot_rg2ft/angle_bracket"
COLOR = BRACKET + "/realsense_d455/RSD455/Camera_OmniVision_OV9782_Color"
RING = BRACKET + "/vision_ring_light"
PANEL = "/World/VisionRoom/Lights/Panel_0"
DOME = "/World/SmartFarm/Lighting/DomeLight"
TOOL0 = "/World/SmartFarm/Placed/M0609/Asset/link_6/tool0"

CLASSES = ["lettuce_dark_green", "lettuce_yellow", "lettuce_brown"]      # contract with the station code
COND2CLS = {"green": "lettuce_dark_green", "yellow": "lettuce_yellow", "brown": "lettuce_brown"}
CLASS_ID = {c: i for i, c in enumerate(CLASSES)}
SHAPES, CONDS = ("A", "B", "C"), ("green", "yellow", "brown")

ROLLER_TOP = 0.769
TRAY_Z = ROLLER_TOP + 0.02592913 + 0.001
TRAY_LEN = 0.552
STATION_X, LANE_Y, INSPECT_Y = -0.686, -6.73, -7.00
HEAD_TOP_ABOVE_TRAY = 0.105
INSPECT_EYE = (-0.20, 0.36)                     # toward the arm, above the head tops (station constant)

parser = argparse.ArgumentParser()
parser.add_argument("--frames", type=int, default=2000)
parser.add_argument("--res", type=int, default=640)
parser.add_argument("--out", default=r"D:\smartfarm-sim\out\yolo_cabbage_v1")
parser.add_argument("--val-split", type=float, default=0.2)
parser.add_argument("--seed", type=int, default=25)
parser.add_argument("--subframes", type=int, default=8)
parser.add_argument("--min-box-px", type=int, default=12)
parser.add_argument("--max-occlusion", type=float, default=0.6)
parser.add_argument("--save-debug", type=int, default=24)
parser.add_argument("--test", action="store_true")
parser.add_argument("--verbose", action="store_true")
parser.add_argument("--no-frame", action="store_true")
parser.add_argument("--no-tint", action="store_true")
args = parser.parse_args()
if args.test:
    args.val_split = 1.0

if os.path.exists(args.out):
    shutil.rmtree(args.out)
for split in ("train", "val"):
    for kind in ("images", "labels"):
        os.makedirs(os.path.join(args.out, kind, split), exist_ok=True)
os.makedirs(os.path.join(args.out, "debug"), exist_ok=True)
t0 = time.time()

from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": True, "renderer": "RaytracedLighting", "width": args.res, "height": args.res})

import numpy as np
import omni.replicator.core as rep
import omni.usd
from isaacsim.core.api import World
from isaacsim.core.prims import SingleArticulation
from isaacsim.core.utils.types import ArticulationAction
from isaacsim.robot_motion.motion_generation import LulaKinematicsSolver
from PIL import Image, ImageDraw
from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics

sys.path.insert(0, REPO + r"\scripts")
import inspection_cull_station as station  # noqa: E402  (build_transfer_frame + constants)

rng = np.random.default_rng(args.seed)
omni.usd.get_context().open_stage(STAGE)
while omni.usd.get_context().get_stage_loading_status()[2] > 0:
    simulation_app.update()
for _ in range(30):
    simulation_app.update()
stage = omni.usd.get_context().get_stage()
stage.SetEditTarget(stage.GetSessionLayer())

info = json.load(open(os.path.join(ASSETS, "build_info.json")))
SLOTS = [tuple(s["slot_xy"]) for s in info["slots"]]
ORIGIN_Z = {}
for s in info["slots"]:
    ORIGIN_Z.setdefault(s["variant"], s["origin_z"])


def static(prim):
    for p in Usd.PrimRange(prim):
        if p.IsInstanceable():            # toggling visibility of many instanced heads crashed Kit (heap corruption)
            p.SetInstanceable(False)
        if p.HasAPI(UsdPhysics.RigidBodyAPI):
            p.CreateAttribute("physics:rigidBodyEnabled", Sdf.ValueTypeNames.Bool).Set(False)
        if p.HasAPI(UsdPhysics.CollisionAPI):
            p.CreateAttribute("physics:collisionEnabled", Sdf.ValueTypeNames.Bool).Set(False)


# ---- capture trays: empty tray + 9 head options per seat (labels fixed at creation) -------------
UsdGeom.Scope.Define(stage, "/World/VisionCapture")
trays = []
for k in range(2):
    path = f"/World/VisionCapture/Tray_{k}"
    xf = UsdGeom.Xform.Define(stage, path)
    ops = (xf.AddTranslateOp(), xf.AddRotateZOp())
    ops[0].Set(Gf.Vec3d(0.0, 0.0, -20.0))
    body = stage.DefinePrim(f"{path}/Body")
    body.GetReferences().AddReference(os.path.join(ASSETS, "cabbage_pallet_empty.usd"))
    static(body)
    seats = []
    for si, (sx, sy) in enumerate(SLOTS):
        per = {}
        for shp in SHAPES:
            for cond in CONDS:
                hx = UsdGeom.Xform.Define(stage, f"{path}/Seat{si}_{shp}_{cond}")
                hops = (hx.AddTranslateOp(), hx.AddRotateZOp(), hx.AddScaleOp())
                hops[0].Set(Gf.Vec3d(sx, sy, ORIGIN_Z[shp]))
                head = stage.DefinePrim(f"{path}/Seat{si}_{shp}_{cond}/Head")
                head.GetReferences().AddReference(os.path.join(ASSETS, f"cabbage_{shp}.usd"))
                head.GetVariantSets().GetVariantSet("condition").SetVariantSelection(cond)
                # class label fixed at creation on the referencing prim (overrides the asset's 'cabbage');
                # labels edited later never reach the bbox annotator, and a custom semanticTypes filter crashed Kit
                head.CreateAttribute("semantics:labels:class", Sdf.ValueTypeNames.TokenArray).Set([COND2CLS[cond]])
                static(head)
                img = UsdGeom.Imageable(hx.GetPrim())
                img.MakeInvisible()
                per[(shp, cond)] = (img, hops)
        seats.append(per)
    trays.append({"img": UsdGeom.Imageable(xf.GetPrim()), "ops": ops, "seats": seats})
print(f"[t] trays {len(trays)}, 6 seats x 9 heads", flush=True)


def set_tray(t, x, y, yaw, picks):
    t["ops"][0].Set(Gf.Vec3d(float(x), float(y), TRAY_Z))
    t["ops"][1].Set(float(yaw))
    for per, pick in zip(t["seats"], picks):
        for key, (img, hops) in per.items():
            if key == pick:
                hops[1].Set(float(rng.uniform(0, 360)))
                s = float(rng.uniform(0.95, 1.05))
                hops[2].Set(Gf.Vec3f(s, s, s))
                img.MakeVisible()
            else:
                img.MakeInvisible()
    t["img"].MakeVisible()


def hide_tray(t):
    t["ops"][0].Set(Gf.Vec3d(0.0, 0.0, -20.0))
    t["img"].MakeInvisible()


def random_picks(empty_p=0.0):
    """One (shape, condition) per seat; None = empty seat (after culling). Empty seats are unlabeled negatives:
    the romaine model called the bare conical hole 'brown' (conf ~0.36) in recheck frames."""
    picks = [(SHAPES[rng.integers(3)], CONDS[rng.integers(3)]) for _ in range(6)]
    if empty_p and rng.random() < empty_p:
        for i in rng.choice(6, size=int(rng.integers(1, 4)), replace=False):
            picks[int(i)] = None
    return picks


# ---- transfer frame (station's own builder) ----------------------------------------------------
if args.no_frame:
    def set_frame(*_):
        pass
else:
  station.build_transfer_frame(stage)
  frame_ops = UsdGeom.Xformable(stage.GetPrimAtPath(f"{station.PUSHER_ROOT}/Frame")).GetOrderedXformOps()[0]
  carr_ops = UsdGeom.Xformable(stage.GetPrimAtPath(f"{station.PUSHER_ROOT}/Carriage")).GetOrderedXformOps()[0]
  static(stage.GetPrimAtPath(station.PUSHER_ROOT))

  def set_frame(x, y, z):
      frame_ops.Set(type(frame_ops.Get())(float(x), float(y), float(z)))
      c = carr_ops.Get()
      carr_ops.Set(type(c)(float(x), c[1], float(z)))


# ---- robot, IK, render product -------------------------------------------------------------
world = World(stage_units_in_meters=1.0)
world.reset()
arm = SingleArticulation(prim_path=ARM_ART, name="m0609")
arm.initialize()
names = list(arm.dof_names)
ai = [names.index(f"joint_{i}") for i in range(1, 7)]
fi = names.index("finger_joint") if "finger_joint" in names else None
ik = LulaKinematicsSolver(robot_description_path=DESC, urdf_path=URDF)
bm = UsdGeom.XformCache().GetLocalToWorldTransform(stage.GetPrimAtPath(ARM_ROOT))
bt, bq = bm.ExtractTranslation(), bm.ExtractRotationQuat()
ik.set_robot_base_pose(np.array([bt[0], bt[1], bt[2]]), np.array([bq.GetReal(), *bq.GetImaginary()]))
ring, panel, dome = (stage.GetPrimAtPath(p) for p in (RING, PANEL, DOME))
BASE = {p: (p.GetAttribute("inputs:intensity").Get() if p.IsValid() else None) for p in (ring, panel, dome)}
print(f"[t] lights {[(str(p.GetPath()), v) for p, v in BASE.items()]}", flush=True)

_xc = UsdGeom.XformCache()
_mc = Gf.Matrix4d(_xc.GetLocalToWorldTransform(stage.GetPrimAtPath(COLOR))).RemoveScaleShear()
_mt = Gf.Matrix4d(_xc.GetLocalToWorldTransform(stage.GetPrimAtPath(TOOL0))).RemoveScaleShear()
TOOL_IN_CAM = (_mc * _mt.GetInverse()).GetInverse()


def tool_target_for_camera(eye, look_at, roll_deg, up=(0, 0, 1)):
    cam = Gf.Matrix4d().SetLookAt(Gf.Vec3d(*eye), Gf.Vec3d(*look_at), Gf.Vec3d(*up)).GetInverse()
    cam = Gf.Matrix4d().SetRotate(Gf.Rotation(Gf.Vec3d(0, 0, 1), roll_deg)) * cam
    tool = TOOL_IN_CAM * cam
    q, t = tool.ExtractRotationQuat(), tool.ExtractTranslation()
    return np.array([t[0], t[1], t[2]]), np.array([q.GetReal(), *q.GetImaginary()])


rp = rep.create.render_product(COLOR, (args.res, args.res))
ann = {"rgb": rep.AnnotatorRegistry.get_annotator("rgb"),
       "bbox": rep.AnnotatorRegistry.get_annotator("bounding_box_2d_tight")}
for a in ann.values():
    a.attach([rp])


def hold(arm_q, finger=None, steps=32):
    q = np.array(arm.get_joint_positions(), dtype=float)
    q[ai] = arm_q
    if finger is not None and fi is not None:
        q[fi] = finger
    arm.set_joint_positions(q)
    arm.set_joint_velocities(np.zeros_like(q))
    arm.apply_action(ArticulationAction(joint_positions=q))
    for _ in range(steps):
        world.step(render=False)


def seats_world(x, y, yaw):
    c, s = math.cos(math.radians(yaw)), math.sin(math.radians(yaw))
    return [(x + c * px - s * py, y + s * px + c * py) for px, py in SLOTS]


HUE_RANGE = {"lettuce_dark_green": (60.0, 170.0), "lettuce_yellow": (33.0, 75.0), "lettuce_brown": (5.0, 45.0)}


def hue_deg(rgb):
    r, g, b = [float(v) for v in rgb]
    mx, mn = max(r, g, b), min(r, g, b)
    if mx - mn < 1e-6:
        return -1.0
    if mx == r:
        return 60.0 * (((g - b) / (mx - mn)) % 6)
    if mx == g:
        return 60.0 * ((b - r) / (mx - mn) + 2)
    return 60.0 * ((r - g) / (mx - mn) + 4)


def label_of(raw):
    if isinstance(raw, dict):
        raw = raw.get("class", "")
    raw = str(raw)
    return raw if raw in CLASS_ID else None


seed_q = np.deg2rad([85.0, 50.0, 30.0, 0.0, 100.9, 0.0])
records, fails, ik_fails, mismatch, debug_saved, modes = [], 0, 0, 0, 0, {"station": 0, "close": 0, "wide": 0}
odd_boxes = [0]
for frame in range(args.frames):
    x_main = STATION_X + rng.uniform(-0.06, 0.06)
    at_lane = rng.random() < 0.25                       # tray still on the lane (frame up above it) vs pushed in
    y_main = LANE_Y + rng.uniform(-0.02, 0.02) if at_lane else INSPECT_Y + rng.uniform(-0.03, 0.03)
    yaw_main = (90.0 if rng.random() < 0.5 else 270.0) + rng.uniform(-4, 4)
    set_tray(trays[0], x_main, y_main, yaw_main, random_picks(empty_p=0.35))   # 35 %: recheck-like tray
    if rng.random() < 0.30:
        side = -1.0 if rng.random() < 0.5 else 1.0
        set_tray(trays[1], x_main + side * (TRAY_LEN + rng.uniform(0.04, 0.20)), LANE_Y + rng.uniform(-0.02, 0.02),
                 (90.0 if rng.random() < 0.5 else 270.0) + rng.uniform(-4, 4), random_picks())
    else:
        hide_tray(trays[1])
    # as in the station: on the lane the frame is up above the tray or just lowered around it; pushed in = down
    if at_lane:
        set_frame(x_main, y_main, station.FRAME_Z_UP if rng.random() < 0.5 else station.FRAME_Z_DOWN)
    else:
        set_frame(x_main, y_main - station.FRAME_GAP + rng.uniform(-0.004, 0.004), station.FRAME_Z_DOWN)

    # lighting: intensity + small colour temperature tint (test set: wider range)
    wide_l = 1.6 if args.test else 1.0
    for p, lo, hi in ((panel, 0.6, 1.4), (ring, 0.5, 1.3), (dome, 0.6, 1.4)):
        if p.IsValid() and BASE[p]:
            f = rng.uniform(1 - (1 - lo) * wide_l, 1 + (hi - 1) * wide_l)
            p.GetAttribute("inputs:intensity").Set(float(BASE[p] * max(0.2, f)))
            tint = rng.uniform(-0.06, 0.06) * wide_l
            if not args.no_tint:
                p.CreateAttribute("inputs:color", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(1.0 + tint, 1.0, 1.0 - tint))

    r_ = rng.random()
    mode = "station" if (args.test or r_ < 0.5) else ("close" if r_ < 0.8 else "wide")
    centre = np.array([x_main, y_main, TRAY_Z + HEAD_TOP_ABOVE_TRAY])   # station: tray origin + head-top height
    seats = seats_world(x_main, y_main, yaw_main)
    ok = False
    for attempt in range(8):
        if mode == "station":
            away = centre[:2] - np.array([bt[0], bt[1]])
            away /= np.linalg.norm(away)
            eye = centre.copy()
            eye[:2] += away * (INSPECT_EYE[0] + rng.uniform(-0.05, 0.05))
            eye[:2] += np.array([-away[1], away[0]]) * rng.uniform(-0.05, 0.05)
            eye[2] += INSPECT_EYE[1] + rng.uniform(-0.06, 0.06)
            look = centre + np.array([*rng.uniform(-0.03, 0.03, 2), 0.0])
            up = (away[0], away[1], 0.0)
            roll = rng.uniform(-8, 8)
        else:
            if mode == "close":
                sx_, sy_ = seats[int(rng.integers(6))]
                look = np.array([sx_, sy_, ROLLER_TOP + 0.10])
                dist = rng.uniform(0.25, 0.45)
            else:
                look = np.array([x_main, y_main, ROLLER_TOP + 0.08])
                dist = rng.uniform(0.45, 0.70)
            look[:2] += rng.uniform(-0.04, 0.04, 2)
            to_arm = math.atan2(bt[1] - look[1], bt[0] - look[0])
            az = to_arm + math.radians(rng.uniform(-35, 35))
            el = math.radians(rng.uniform(45, 88))
            eye = look + dist * np.array([math.cos(el) * math.cos(az), math.cos(el) * math.sin(az), math.sin(el)])
            # camera 'up' = away from the arm base (as the station): the arm stays behind/below the image
            aw = look[:2] - np.array([bt[0], bt[1]])
            aw /= np.linalg.norm(aw)
            up, roll = (aw[0], aw[1], 0.0), rng.uniform(-20, 20)
        pos, quat = tool_target_for_camera(eye, look, roll, up)
        sol, ok = ik.compute_inverse_kinematics(frame_name="tool0", target_position=pos, target_orientation=quat,
                                                warm_start=seed_q)
        if ok:
            break
    if not ok:
        ik_fails += 1
        fails += 1
        continue
    sol = np.asarray(getattr(sol, "joint_positions", sol), dtype=float)
    hold(sol, finger=float(rng.uniform(0.0, 0.55)))
    seed_q = sol

    if args.verbose:
        print(f"[v] frame {frame} {mode} render", flush=True)
    rep.orchestrator.step(rt_subframes=2)
    rep.orchestrator.step(rt_subframes=args.subframes)
    rgb = np.asarray(ann["rgb"].get_data())
    if rgb.size == 0:
        fails += 1
        continue
    bb = ann["bbox"].get_data()
    id2lab = bb.get("info", {}).get("idToLabels", {})
    lines, boxes_px = [], []
    for row in bb.get("data", []):
        lab = label_of(id2lab.get(str(int(row["semanticId"])), {}))
        if lab is None:
            continue
        x0b, y0b = max(0, int(row["x_min"])), max(0, int(row["y_min"]))
        x1b, y1b = min(args.res, int(row["x_max"])), min(args.res, int(row["y_max"]))
        w, h = x1b - x0b, y1b - y0b
        if w < args.min_box_px or h < args.min_box_px:
            continue
        edge = x0b <= 1 or y0b <= 1 or x1b >= args.res - 1 or y1b >= args.res - 1
        if not edge and not (0.6 <= w / h <= 1.67):       # one head is roughly round; a merged/odd box is dropped
            odd_boxes[0] += 1
            continue
        if w * h > 0.2 * args.res * args.res:
            odd_boxes[0] += 1
            continue
        if "occlusionRatio" in row.dtype.names and float(row["occlusionRatio"]) > args.max_occlusion:
            continue
        line = f"{CLASS_ID[lab]} {(x0b + w / 2) / args.res:.6f} {(y0b + h / 2) / args.res:.6f} {w / args.res:.6f} {h / args.res:.6f}"
        if line not in lines:
            lines.append(line)
            boxes_px.append((lab, x0b, y0b, x1b, y1b))
    if not lines:
        fails += 1
        continue
    bad = 0
    for lab, a_, b_, c_, e_ in boxes_px:
        cw, ch = (c_ - a_) // 4, (e_ - b_) // 4
        patch = rgb[b_ + ch:e_ - ch, a_ + cw:c_ - cw, :3].reshape(-1, 3).astype(float)
        if len(patch) and not (HUE_RANGE[lab][0] <= hue_deg(np.median(patch, axis=0)) <= HUE_RANGE[lab][1]):
            bad += 1
            if mismatch < 6:
                print(f"[audit] frame {frame} {lab} median {np.median(patch, axis=0).round(0).tolist()}", flush=True)
    if bad:
        mismatch += 1
        fails += 1
        continue

    split = "val" if rng.random() < args.val_split else "train"
    stem = f"cabbage_{'test' if args.test else 'v1'}_{frame:05d}"
    img = Image.fromarray(rgb[..., :3].astype(np.uint8))
    img.save(os.path.join(args.out, "images", split, f"{stem}.jpg"), quality=92)
    with open(os.path.join(args.out, "labels", split, f"{stem}.txt"), "w", encoding="utf-8") as fh:
        fh.write("\n".join(lines) + "\n")
    if debug_saved < args.save_debug:
        d = ImageDraw.Draw(img)
        for lab, a_, b_, c_, e_ in boxes_px:
            col = {"lettuce_yellow": (255, 210, 40), "lettuce_dark_green": (60, 230, 90), "lettuce_brown": (215, 140, 60)}[lab]
            d.rectangle([a_, b_, c_, e_], outline=col, width=3)
        img.save(os.path.join(args.out, "debug", f"{stem}_{mode}.jpg"), quality=90)
        debug_saved += 1
    modes[mode] += 1
    records.append({"split": split, "labels": [l.split()[0] for l in lines], "mode": mode})
    if args.verbose or frame % 50 == 0 or frame == args.frames - 1:
        print(f"[t] frame {frame + 1}/{args.frames} kept={len(records)} t={time.time() - t0:.0f}s", flush=True)

with open(os.path.join(args.out, "data.yaml"), "w", encoding="utf-8") as fh:
    fh.write("# cabbage sorting inspection (v014 scene): 6-hole tray at the vision station, M0609 wrist RealSense.\n")
    fh.write("# Class names kept from the romaine model for drop-in use: 0 green = keep, 1 yellow / 2 brown = cull.\n")
    fh.write("train: images/train\nval: images/val\n")
    fh.write(f"nc: {len(CLASSES)}\nnames:\n")
    for i, c in enumerate(CLASSES):
        fh.write(f"  {i}: {c}\n")
counts = {c: 0 for c in CLASSES}
per_split = {"train": 0, "val": 0}
for r in records:
    per_split[r["split"]] += 1
    for cid in r["labels"]:
        counts[CLASSES[int(cid)]] += 1
summary = {"frames_kept": len(records), "split": per_split, "modes": modes, "skipped": fails, "ik_fail": ik_fails,
           "colour_audit_fail": mismatch, "odd_boxes_dropped": odd_boxes[0], "boxes": counts, "seconds": round(time.time() - t0), "seed": args.seed,
           "test": args.test}
json.dump(summary, open(os.path.join(args.out, "summary.json"), "w"), indent=1)
print(f"\n[t] {summary}", flush=True)
simulation_app.close()
