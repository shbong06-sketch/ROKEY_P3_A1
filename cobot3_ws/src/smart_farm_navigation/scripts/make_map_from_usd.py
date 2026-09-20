"""Generate a Nav2 occupancy map (PGM + YAML) from a USD scene's collider bounding boxes.

Usage (any python with usd-core / pxr):
    python make_map_from_usd.py <scene.usd> <out_dir> <map_name> [--assets <dir>]

Obstacles = the world-space bounding box of every top-level /World child whose
vertical span intersects [Z_MIN, Z_MAX_SOLID] and whose name does not match the
exclude words (robots, arm, lift, ground).  Each becomes one solid block.  Cells are
0 (occupied) or 254 (free); the whole map is treated as free ground.
"""

import argparse
import math
import os
import sys

from pxr import Usd, UsdGeom, UsdPhysics

EXCLUDE = ("carter", "nova", "m0609", "lift", "groundplane", "physicsscene", "ros_clock")
RES = 0.05
X_MIN, X_MAX, Y_MIN, Y_MAX = -8.0, 8.0, -8.0, 12.0
Z_MIN, Z_MAX_SOLID = 0.10, 1.50    # anything standing between 0.1 m and 1.5 m blocks the robot


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("scene"); ap.add_argument("out_dir"); ap.add_argument("name")
    a = ap.parse_args()
    stage = Usd.Stage.Open(a.scene, Usd.Stage.LoadAll)
    bb = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_, UsdGeom.Tokens.render], useExtentsHint=True)
    boxes = []
    # One solid block per top-level /World child (rack frame, conveyor, ...): the lidar only sees
    # the rack posts, but the shelves above the lidar band must still be impassable for planning.
    for prim in stage.GetPrimAtPath("/World").GetChildren():
        path = str(prim.GetPath())
        if any(w in path.lower() for w in EXCLUDE):
            continue
        r = bb.ComputeWorldBound(prim).ComputeAlignedRange()
        if r.IsEmpty():
            continue
        lo, hi = r.GetMin(), r.GetMax()
        if hi[2] < Z_MIN or lo[2] > Z_MAX_SOLID:
            continue
        if hi[0] - lo[0] > 20.0 or hi[1] - lo[1] > 20.0:   # ground-plane sized things are not obstacles
            continue
        boxes.append((lo[0], lo[1], hi[0], hi[1], path))
    if not boxes:
        sys.exit("no obstacle boxes found")
    w = int(round((X_MAX - X_MIN) / RES)); h = int(round((Y_MAX - Y_MIN) / RES))
    grid = bytearray([254]) * (w * h)
    for x0, y0, x1, y1, path in boxes:
        c0 = max(0, int(math.floor((x0 - X_MIN) / RES))); c1 = min(w - 1, max(c0, int(math.ceil((x1 - X_MIN) / RES)) - 1))
        r0 = max(0, int(math.floor((y0 - Y_MIN) / RES))); r1 = min(h - 1, max(r0, int(math.ceil((y1 - Y_MIN) / RES)) - 1))
        for row in range(r0, r1 + 1):
            img_row = h - 1 - row               # image row 0 is the top (max y)
            base = img_row * w
            for col in range(c0, c1 + 1):
                grid[base + col] = 0
        print(f"obstacle {path}: x {x0:.2f}..{x1:.2f} y {y0:.2f}..{y1:.2f}")
    os.makedirs(a.out_dir, exist_ok=True)
    pgm = os.path.join(a.out_dir, a.name + ".pgm")
    with open(pgm, "wb") as f:
        f.write(f"P5\n# generated from {os.path.basename(a.scene)}\n{w} {h}\n255\n".encode())
        f.write(bytes(grid))
    with open(os.path.join(a.out_dir, a.name + ".yaml"), "w") as f:
        f.write(f"image: {a.name}.pgm\nmode: trinary\nresolution: {RES}\norigin: [{X_MIN}, {Y_MIN}, 0.0]\n"
                "negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n")
    print(f"wrote {pgm} ({w}x{h}, {len(boxes)} boxes)")


if __name__ == "__main__":
    main()
