"""Score the team's colour YOLO (best.pt) on env_capture.py images: is every head on the inspection trays found and
put in the right colour class?

  python 08_yolo_color_check.py CAPTURE_DIR WEIGHTS OUT_JSON [--annotate]
(needs ultralytics + torch + usd-core on PYTHONPATH)

Ground truth comes from the captured stage itself: cabbage heads -> their 'condition' variant (green/yellow/brown),
romaine heads -> their bound material (M_Romaine_001 / _Yellow / _Brown). Each head's visual bbox centre is projected
with the captured camera pose (cam_to_world, fx) and matched to the detection box that contains it.
"""
import json, os, sys
import numpy as np
from pxr import Usd, UsdGeom, UsdShade
from ultralytics import YOLO

CAP, WEIGHTS, OUT = sys.argv[1], sys.argv[2], sys.argv[3]
ANNOTATE = "--annotate" in sys.argv
CLS = {"green": "lettuce_dark_green", "yellow": "lettuce_yellow", "brown": "lettuce_brown"}

meta = json.load(open(os.path.join(CAP, "capture.json")))
W, H, fx = meta["width"], meta["height"], meta["fx"]
st = Usd.Stage.Open(meta["stage"])
bb = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_, UsdGeom.Tokens.render])
heads = []
for p in st.Traverse():
    path = str(p.GetPath())
    if "/Pallet_Inspect" not in path:
        continue
    name = p.GetName()
    if name.startswith("Cabbage_") and p.GetParent().GetName() == "root_001":
        cond = p.GetVariantSets().GetVariantSelection("condition") or "green"
    elif name.startswith("Romaine_") and p.GetParent().GetName() == "root_001":
        mesh = [c for c in Usd.PrimRange(p) if c.IsA(UsdGeom.Mesh) and c.GetName().startswith("head")][0]
        m = UsdShade.MaterialBindingAPI(mesh).ComputeBoundMaterial()[0].GetPath().name.lower()
        cond = "yellow" if "yellow" in m else "brown" if "brown" in m else "green"
    else:
        continue
    r = bb.ComputeWorldBound(p).ComputeAlignedRange()
    heads.append({"path": path.replace("/World/SmartFarm/Placed/", ""), "cond": cond,
                  "c": np.array((r.GetMin() + r.GetMax()) / 2, float)})
print("ground-truth heads on inspection trays:", len(heads), {k: sum(h["cond"] == k for h in heads) for k in CLS})

model = YOLO(WEIGHTS)
names = model.names
rows, conf_mat = [], {g: {n: 0 for n in list(CLS.values()) + ["MISSED"]} for g in CLS}
false_pos = 0
for pose in meta["poses"]:
    img = os.path.join(CAP, pose["id"] + ".png")
    M = np.array(pose["cam_to_world"], float)
    inv = np.linalg.inv(M)
    res = model.predict(img, imgsz=640, conf=0.25, verbose=False)[0]
    boxes = res.boxes.xyxy.cpu().numpy(); cls = res.boxes.cls.cpu().numpy().astype(int); cf = res.boxes.conf.cpu().numpy()
    used = set()
    for h in heads:
        pc = np.r_[h["c"], 1.0] @ inv            # USD row-vector convention; camera looks down -Z
        if pc[2] > -0.05:
            continue
        u = W / 2 + fx * pc[0] / -pc[2]; v = H / 2 - fx * pc[1] / -pc[2]
        if not (15 <= u <= W - 15 and 15 <= v <= H - 15):
            continue
        inside = [i for i, b in enumerate(boxes) if b[0] <= u <= b[2] and b[1] <= v <= b[3]]
        if inside:
            i = max(inside, key=lambda k: cf[k]); used.add(i)
            pred = names[cls[i]]
            rows.append({"img": pose["id"], "head": h["path"], "truth": h["cond"], "pred": pred, "conf": round(float(cf[i]), 3)})
            conf_mat[h["cond"]][pred] += 1
        else:
            rows.append({"img": pose["id"], "head": h["path"], "truth": h["cond"], "pred": "MISSED", "conf": 0.0})
            conf_mat[h["cond"]]["MISSED"] += 1
    false_pos += len([i for i in range(len(boxes)) if i not in used])
    if ANNOTATE:
        res.save(filename=os.path.join(CAP, pose["id"] + "_yolo.jpg"))

total = sum(sum(v.values()) for v in conf_mat.values())
correct = sum(conf_mat[g][CLS[g]] for g in CLS)
summary = {"weights": WEIGHTS, "images": len(meta["poses"]), "head_views": total, "correct": correct,
           "accuracy": round(correct / max(total, 1), 3),
           "per_class_recall": {g: round(conf_mat[g][CLS[g]] / max(sum(conf_mat[g].values()), 1), 3) for g in CLS},
           "confusion": conf_mat, "unmatched_detections": false_pos}
json.dump({"summary": summary, "rows": rows}, open(OUT, "w"), indent=1)
print(json.dumps(summary, indent=1))
