"""Generate a Nav2 occupancy map (PNG + YAML) from a USD scene's prim bounding boxes.

    <usd-core python> make_map_from_usd.py <scene.usd> <out_dir> <map_name>

Obstacles: every prim listed in OBSTACLE_GROUPS (children of /World/SmartFarm/Placed except the
robot rig and loose pallets, the inspection-cell walls /BackWall_03..07, room walls) whose vertical
span intersects [Z_MIN, Z_MAX].  Each becomes one solid block; the conveyor is rasterised per
segment so the TurnTable/Feeder get their own boxes.  Same extent/origin/resolution as
maps/Collected_smartfarm_v005.yaml so stations.yaml stays valid.
"""

import math
import os
import sys

from PIL import Image
from pxr import Usd, UsdGeom

RES = 0.05
ORIGIN = (-4.525, -10.025)          # same as Collected_smartfarm_v005.yaml
W, H = 285, 460
Z_MIN, Z_MAX = 0.10, 1.50           # anything standing between these heights blocks the robot
SKIP = ("liftrig", "lettuce", "pallet_inspect", "materials", "lighting", "markers", "floor", "groundcollider")


def boxes_of(stage):
    bb = UsdGeom.BBoxCache(Usd.TimeCode.Default(), ["default", "render", "proxy"], useExtentsHint=True)
    out = []

    def add(prim, label):
        r = bb.ComputeWorldBound(prim).ComputeAlignedRange()
        if r.IsEmpty():
            return
        lo, hi = r.GetMin(), r.GetMax()
        if hi[2] < Z_MIN or lo[2] > Z_MAX or hi[0] - lo[0] > 30 or hi[1] - lo[1] > 30:
            return
        out.append((lo[0], lo[1], hi[0], hi[1], label))

    placed = stage.GetPrimAtPath("/World/SmartFarm/Placed")
    for p in placed.GetChildren():
        n = p.GetName().lower()
        if any(s in n for s in SKIP):
            continue
        if n == "conveyor":
            for seg in p.GetChildren():
                add(seg, f"Conveyor/{seg.GetName()}")
        elif n.startswith("pallet"):
            continue                    # pallets on the rack sit inside the rack box
        else:
            add(p, p.GetName())
    for p in stage.GetPrimAtPath("/World/SmartFarm/Room").GetChildren():
        if "wall" in p.GetName().lower():
            add(p, "Room/" + p.GetName())
    for p in stage.GetPseudoRoot().GetChildren():
        if p.GetName().lower().startswith("backwall"):
            add(p, p.GetName())
    return out


def main():
    scene, out_dir, name = sys.argv[1:4]
    stage = Usd.Stage.Open(scene, Usd.Stage.LoadAll)
    boxes = boxes_of(stage)
    img = Image.new("L", (W, H), 254)
    px = img.load()
    for x0, y0, x1, y1, label in boxes:
        c0 = int(math.floor((x0 - ORIGIN[0]) / RES)); c1 = int(math.ceil((x1 - ORIGIN[0]) / RES)) - 1
        r0 = int(math.floor((y0 - ORIGIN[1]) / RES)); r1 = int(math.ceil((y1 - ORIGIN[1]) / RES)) - 1
        c0, c1 = max(0, c0), min(W - 1, max(c0, c1)); r0, r1 = max(0, r0), min(H - 1, max(r0, r1))
        if c1 < 0 or r1 < 0 or c0 > W - 1 or r0 > H - 1:
            continue
        for row in range(r0, r1 + 1):
            for col in range(c0, c1 + 1):
                px[col, H - 1 - row] = 0
        print(f"obstacle {label}: x {x0:.2f}..{x1:.2f} y {y0:.2f}..{y1:.2f}")
    os.makedirs(out_dir, exist_ok=True)
    img.save(os.path.join(out_dir, name + ".png"))
    with open(os.path.join(out_dir, name + ".yaml"), "w") as f:
        f.write(f"image: {name}.png\nmode: trinary\nresolution: {RES}\norigin: [{ORIGIN[0]}, {ORIGIN[1]}, 0.0]\n"
                "negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n")
    print(f"wrote {name}.png ({W}x{H}, {len(boxes)} boxes) to {out_dir}")


if __name__ == "__main__":
    main()
