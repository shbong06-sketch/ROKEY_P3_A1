"""Fine-tune the station's YOLO11n (romaine model) on the cabbage dataset from cabbage_inspect_dataset.py, then
compare old vs new on the held-out test set (another seed, wider lighting, station views only).

  set PYTHONPATH=D:\\smartfarm-sim\\team_ref\\pylib_yolo;D:\\isaacsim\\exts\\omni.isaac.ml_archive\\pip_prebundle;D:\\smartfarm-sim\\team_ref\\pylib
  D:\\isaacsim\\kit\\python\\python.exe train_eval_yolo.py train
  D:\\isaacsim\\kit\\python\\python.exe train_eval_yolo.py eval OLD.pt NEW.pt [more.pt ...]

Station-style metric (eval): every labelled head in a test image is matched to the best-IoU detection (IoU >= 0.5,
conf >= 0.35 = the station's CONF_MIN) -> correct class / wrong class / missed, plus the cull decision
(yellow|brown -> cull) right or wrong. False detections with no head are counted too.
"""
import json
import sys
from pathlib import Path

OUT = Path(r"D:\smartfarm-sim\out")
DATA = OUT / "yolo_cabbage_v1" / "data.yaml"
TEST = OUT / "yolo_cabbage_test"
BASE = r"C:\Users\kangm\Downloads\best.pt"          # romaine3_v011_yolo11n (current station model)
CONF_MIN = 0.35
CULL = {1, 2}


def train():
    from ultralytics import YOLO
    model = YOLO(BASE)
    model.train(data=str(DATA), epochs=60, imgsz=640, batch=16, device=0, workers=2, patience=20,
                hsv_h=0.01, hsv_s=0.5, hsv_v=0.45, degrees=5.0, translate=0.1, scale=0.3, fliplr=0.5, mosaic=1.0,
                project=str(OUT / "yolo_runs"), name="cabbage_v1", exist_ok=True, plots=True, verbose=False)


def iou(a, b):
    x0, y0, x1, y1 = max(a[0], b[0]), max(a[1], b[1]), min(a[2], b[2]), min(a[3], b[3])
    inter = max(0.0, x1 - x0) * max(0.0, y1 - y0)
    return inter / ((a[2] - a[0]) * (a[3] - a[1]) + (b[2] - b[0]) * (b[3] - b[1]) - inter + 1e-9)


def station_metric(model, imgs):
    import numpy as np  # noqa: F401
    res = {"heads": 0, "correct": 0, "wrong_class": 0, "missed": 0, "false_det": 0, "cull_right": 0, "cull_wrong": 0,
           "confusion": [[0] * 4 for _ in range(3)], "conf_correct": []}
    for img in imgs:
        lab = img.parent.parent.parent / "labels" / img.parent.name / (img.stem + ".txt")
        W = H = 640
        gt = []
        for line in lab.read_text().split("\n"):
            if line.strip():
                c, x, y, w, h = map(float, line.split())
                gt.append((int(c), [(x - w / 2) * W, (y - h / 2) * H, (x + w / 2) * W, (y + h / 2) * H]))
        r = model.predict(str(img), imgsz=640, conf=CONF_MIN, verbose=False)[0]
        det = [(int(c), b, float(s)) for b, c, s in zip(r.boxes.xyxy.tolist(), r.boxes.cls.tolist(), r.boxes.conf.tolist())]
        used = set()
        for c, b in gt:
            res["heads"] += 1
            best, bi = 0.0, None
            for j, (dc, db, ds) in enumerate(det):
                v = iou(b, db)
                if j not in used and v > best:
                    best, bi = v, j
            if bi is None or best < 0.5:
                res["missed"] += 1
                res["confusion"][c][3] += 1
                continue
            used.add(bi)
            dc, _, ds = det[bi]
            res["confusion"][c][dc] += 1
            if dc == c:
                res["correct"] += 1
                res["conf_correct"].append(ds)
            else:
                res["wrong_class"] += 1
            if (dc in CULL) == (c in CULL):
                res["cull_right"] += 1
            else:
                res["cull_wrong"] += 1
        res["false_det"] += len(det) - len(used)
    cc = res.pop("conf_correct")
    res["accuracy"] = round(res["correct"] / max(1, res["heads"]), 4)
    res["cull_decision_accuracy"] = round(res["cull_right"] / max(1, res["heads"]), 4)
    res["mean_conf_correct"] = round(sum(cc) / max(1, len(cc)), 3)
    return res


def evaluate(weights):
    from ultralytics import YOLO
    test_yaml = TEST / "data.yaml"
    imgs = sorted((TEST / "images" / "val").glob("*.jpg"))
    out = {}
    for w in weights:
        model = YOLO(w)
        v = model.val(data=str(test_yaml), imgsz=640, batch=16, device=0, workers=2, plots=False, verbose=False,
                      project=str(OUT / "yolo_runs"), name="eval_" + Path(w).parent.parent.name, exist_ok=True)
        per_class = {model.names[i]: round(float(v.box.maps[i]), 4) for i in range(len(model.names))}
        out[w] = {"mAP50": round(float(v.box.map50), 4), "mAP50_95": round(float(v.box.map), 4),
                  "precision": round(float(v.box.mp), 4), "recall": round(float(v.box.mr), 4),
                  "mAP50_95_per_class": per_class, "station": station_metric(model, imgs), "test_images": len(imgs)}
        print(w, json.dumps(out[w], ensure_ascii=False), flush=True)
    (OUT / "yolo_runs").mkdir(exist_ok=True)
    json.dump(out, open(OUT / "yolo_runs" / "eval_compare.json", "w"), indent=1, ensure_ascii=False)


if __name__ == "__main__":
    if sys.argv[1] == "train":
        train()
    else:
        evaluate(sys.argv[2:])
