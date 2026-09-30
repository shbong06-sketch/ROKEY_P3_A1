#!/usr/bin/env python3
"""저장된 PatchCore로 마스크 없이 ROI의 이미지 단위 이상 점수를 평가한다."""

import argparse
import csv
import json
import subprocess
import sys
from collections import Counter
from datetime import datetime
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from PIL import Image
from sklearn.metrics import roc_auc_score
import torch
from torch.utils.data import DataLoader, Dataset
from torchvision import transforms


PROJECT = Path(__file__).resolve().parents[1]
DATA = PROJECT / "data"
DEFAULT_REPO = PROJECT.parents[2] / "patchcore-inspection"
LABELS = ("good", "yellow", "brown")
SLOTS = tuple(f"SLOT_{index:02d}" for index in range(1, 7))
IMAGE_SIZE = 96


class ImageDataset(Dataset):
    """마스크 없이 ROI 이미지만 읽고 학습과 같은 전처리를 적용한다."""

    def __init__(self, records, mean, std):
        self.records = records
        self.transform = transforms.Compose((
            transforms.Resize(IMAGE_SIZE),
            transforms.CenterCrop(IMAGE_SIZE),
            transforms.ToTensor(),
            transforms.Normalize(mean=mean, std=std),
        ))

    def __len__(self):
        return len(self.records)

    def __getitem__(self, index):
        path = self.records[index]["path"]
        with Image.open(path) as image:
            if image.size != (IMAGE_SIZE, IMAGE_SIZE):
                raise ValueError(f"96x96 ROI가 아닙니다: {path}: {image.size}")
            return self.transform(image.convert("RGB"))


def load_records(data_dir, manifest_path, limit_per_label):
    """검증·테스트 ROI를 manifest의 원본 장면·슬롯·라벨과 연결한다."""
    with manifest_path.open(newline="") as file:
        manifest = list(csv.DictReader(file))
    by_path = {}
    for row in manifest:
        key = row["roi_image"]
        if key in by_path:
            raise ValueError(f"manifest에 중복된 ROI가 있습니다: {key}")
        by_path[key] = row

    records = {"val": [], "test": []}
    for split, base in (("val", data_dir / "validation"),
                        ("test", data_dir / "mvtec/cabbage/test")):
        for label in LABELS:
            folder = base / label
            if not folder.is_dir():
                raise FileNotFoundError(f"평가 폴더가 없습니다: {folder}")
            paths = sorted(folder.glob("*.png"))
            if not paths:
                raise ValueError(f"평가 이미지가 없습니다: {folder}")
            if limit_per_label is not None:
                paths = paths[:limit_per_label]
            for path in paths:
                # 파일명만 믿지 않고 생성 당시 기록한 split·라벨을 대조한다.
                key = path.relative_to(data_dir).as_posix()
                row = by_path.get(key)
                if row is None:
                    raise ValueError(f"manifest에 연결되지 않은 평가 이미지: {path}")
                if row["split"] != split or row["label"] != label or row["slot"] not in SLOTS:
                    raise ValueError(f"manifest의 분할·라벨·슬롯이 다릅니다: {key}")
                source = data_dir / row["source_image"]
                if not source.is_file():
                    raise FileNotFoundError(f"원본 장면이 없습니다: {source}")
                records[split].append({
                    "split": split, "filename": path.name, "roi_image": key,
                    "source_scene": row["source_image"], "slot": row["slot"],
                    "label": label, "binary_label": int(label != "good"), "path": path,
                })
    return records


def score_records(model, records, mean, std, batch_size):
    """ROI 순서를 유지하며 이미지 점수와 예측 히트맵을 수집한다."""
    loader = DataLoader(ImageDataset(records, mean, std), batch_size=batch_size,
                        shuffle=False, num_workers=0)
    maps = {}
    offset = 0
    for images in loader:
        scores, heatmaps = model.predict(images)
        if len(scores) != len(images) or len(heatmaps) != len(images):
            raise RuntimeError("PatchCore 출력 수가 배치 이미지 수와 다릅니다")
        for record, score, heatmap in zip(records[offset:offset + len(images)], scores, heatmaps):
            score = float(score)
            heatmap = np.asarray(heatmap, dtype=np.float32)
            if not np.isfinite(score) or heatmap.shape != (IMAGE_SIZE, IMAGE_SIZE) or not np.isfinite(heatmap).all():
                raise ValueError(f"잘못된 PatchCore 출력: {record['roi_image']}")
            record["raw_score"] = score
            maps[record["roi_image"]] = heatmap
        offset += len(images)
    if offset != len(records):
        raise RuntimeError("평가 결과 수가 이미지 수와 다릅니다")
    return maps


def choose_threshold(records):
    """검증 balanced accuracy 최대화. score >= threshold면 이상; 동점이면 높은 threshold."""
    good = sum(record["binary_label"] == 0 for record in records)
    anomaly = len(records) - good
    if good == 0 or anomaly == 0:
        raise ValueError("임계값 선정에는 정상과 이상 검증 이미지가 모두 필요합니다")
    scores = sorted({record["raw_score"] for record in records})
    # 최대 점수보다 큰 후보도 넣어 '모두 정상' 판정을 비교한다.
    candidates = scores + [float(np.nextafter(scores[-1], np.inf))]

    def rank(threshold):
        tp = sum(record["binary_label"] == 1 and record["raw_score"] >= threshold
                 for record in records)
        fp = sum(record["binary_label"] == 0 and record["raw_score"] >= threshold
                 for record in records)
        # Youden J = TPR - FPR. 정수로 비교하면 부동소수점 동점 판정이 일정하다.
        return tp * good - fp * anomaly, threshold

    return max(candidates, key=rank)


def metrics(records, threshold):
    """한 split의 이미지 단위 순위 성능과 임계값 기준 오분류 수를 계산한다."""
    counts = Counter((record["binary_label"], record["raw_score"] >= threshold)
                     for record in records)
    tp, fp, tn, fn = counts[1, True], counts[0, True], counts[0, False], counts[1, False]
    labels = [record["binary_label"] for record in records]
    scores = [record["raw_score"] for record in records]
    return {
        "n": len(records), "good": tn + fp, "anomaly": tp + fn,
        "auroc": float(roc_auc_score(labels, scores)) if len(set(labels)) == 2 else None,
        "tp": tp, "fp": fp, "tn": tn, "fn": fn,
        "false_positive_rate": fp / (fp + tn) if fp + tn else None,
        "anomaly_recall": tp / (tp + fn) if tp + fn else None,
    }


def score_summary(records):
    """라벨·슬롯별 점수 분포를 건수와 함께 요약한다."""
    if not records:
        return None
    values = np.asarray([record["raw_score"] for record in records])
    return {"n": len(records), "min": float(values.min()),
            "median": float(np.median(values)), "max": float(values.max())}


def summarize_groups(records, threshold):
    """결함 라벨과 슬롯별 오탐·미탐 건수를 분리한다."""
    slots = {}
    for slot in SLOTS:
        group = [record for record in records if record["slot"] == slot]
        good = [record for record in group if record["binary_label"] == 0]
        anomaly = [record for record in group if record["binary_label"] == 1]
        slots[slot] = {
            "good_scores": score_summary(good), "anomaly_scores": score_summary(anomaly),
            "false_positives": sum(record["raw_score"] >= threshold for record in good),
            "misses": sum(record["raw_score"] < threshold for record in anomaly),
        }
    defects = {}
    for label in ("yellow", "brown"):
        group = [record for record in records if record["label"] == label]
        defects[label] = {"n": len(group),
                          "misses": sum(record["raw_score"] < threshold for record in group)}
    return slots, defects


def save_example(record, heatmap, folder, vmin, vmax):
    """ROI·예측 히트맵·오버레이를 같은 색상 범위로 그린다."""
    folder.mkdir(parents=True, exist_ok=True)
    with Image.open(record["path"]) as image:
        rgb = np.asarray(image.convert("RGB"))
    fig, axes = plt.subplots(1, 3, figsize=(9, 3), layout="constrained")
    axes[0].imshow(rgb)
    axes[0].set_title("ROI")
    colored = axes[1].imshow(heatmap, cmap="inferno", vmin=vmin, vmax=vmax)
    axes[1].set_title("Predicted heatmap")
    axes[2].imshow(rgb)
    axes[2].imshow(heatmap, cmap="inferno", vmin=vmin, vmax=vmax, alpha=0.45)
    axes[2].set_title("Overlay")
    for axis in axes:
        axis.axis("off")
    fig.colorbar(colored, ax=axes, shrink=0.72, label="PatchCore score")
    fig.savefig(folder / record["filename"], dpi=140)
    plt.close(fig)


def write_report(output, summaries, groups, defects, distribution, threshold, example_paths, full_data):
    """숫자와 해석 범위를 함께 읽을 수 있는 Markdown 보고서를 만든다."""
    lines = ["# PatchCore 이미지 단위 평가", "",
             f"임계값: `{threshold:.8g}`. 검증 balanced accuracy 최대; 점수 `>=`이면 이상, 동점이면 높은 임계값 선택.",
             "최종 테스트 점수는 임계값 선정에 사용하지 않았다.", "",
             "| 분할 | 정상/이상 | AUROC | TP | FP | TN | FN | 정상 오탐률 | 이상 재현율 |",
             "| --- | --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |"]
    for split in ("val", "test"):
        item = summaries[split]
        lines.append(f"| {split} | {item['good']}/{item['anomaly']} | {item['auroc']:.4f} | "
                     f"{item['tp']} | {item['fp']} | {item['tn']} | {item['fn']} | "
                     f"{item['false_positive_rate']:.4f} | {item['anomaly_recall']:.4f} |")
    lines.extend(["", "## 검증 점수 분포", "",
                  "| 라벨 | 이미지 수 | 최소 | 중앙값 | 최대 |",
                  "| --- | ---: | ---: | ---: | ---: |"])
    for label, item in distribution.items():
        lines.append(f"| {label} | {item['n']} | {item['min']:.4f} | "
                     f"{item['median']:.4f} | {item['max']:.4f} |")
    for split in ("val", "test"):
        lines.extend(["", f"## {split}: 라벨과 슬롯", "",
                      "| 결함 라벨 | 이미지 수 | 미탐 |", "| --- | ---: | ---: |"])
        for label, item in defects[split].items():
            lines.append(f"| {label} | {item['n']} | {item['misses']} |")
        lines.extend(["", "| 슬롯 | 정상 수·점수 중앙값 | 이상 수·점수 중앙값 | 오탐 | 미탐 |",
                      "| --- | ---: | ---: | ---: | ---: |"])
        for slot, item in groups[split].items():
            good, anomaly = item["good_scores"], item["anomaly_scores"]
            good_text = f"{good['n']} · {good['median']:.4f}" if good else "0 · —"
            anomaly_text = f"{anomaly['n']} · {anomaly['median']:.4f}" if anomaly else "0 · —"
            lines.append(f"| {slot} | {good_text} | {anomaly_text} | "
                         f"{item['false_positives']} | {item['misses']} |")
    lines.extend(["", "## 시각화", "",
                  "히트맵은 PatchCore 예측 점수이며 정답 마스크가 아니다. 모든 그림은 검증 히트맵 픽셀의 99백분위수를 공통 vmax로 사용하고 vmin=0, inferno 색상표, 오버레이 alpha=0.45를 사용한다."])
    for category, paths in example_paths.items():
        lines.append(f"- {category}: {len(paths)}장" + (f" — {', '.join(paths)}" if paths else " (해당 사례 없음)"))
    lines.extend(["", "## 해석 범위", "",
                  "검증에서 정한 한 임계값으로 최종 테스트를 평가했다. 픽셀 정확도 지표는 계산하지 않았다.",
                  "현재 이상 장면은 SLOT_03·04=yellow, SLOT_05=brown으로 고정되어 있어 슬롯 위치·배경과 색상 차이를 분리해 평가할 수 없다.",
                  "같은 원본 장면의 여러 ROI는 서로 독립 표본이 아니다. 아래 하위 그룹은 건수가 작아 비율보다 건수를 우선해 읽어야 한다."])
    if not full_data:
        lines.append("이번 실행은 `--limit-per-label` 표본 검사이며 전체 데이터셋 성능 추정이 아니다.")
    (output / "report.md").write_text("\n".join(lines) + "\n")


def main():
    """전체 학습 모델을 확인한 뒤 검증 임계값으로 두 split을 평가한다."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--model-dir", type=Path, default=PROJECT / "models/patchcore")
    parser.add_argument("--data-dir", type=Path, default=DATA)
    parser.add_argument("--manifest", type=Path, default=DATA / "metadata/roi_manifest.csv")
    parser.add_argument("--patchcore-repo", type=Path, default=DEFAULT_REPO)
    parser.add_argument("--run-dir", type=Path, default=PROJECT / "results" /
                        datetime.now().strftime("image-level-%Y%m%d-%H%M%S-%f"))
    parser.add_argument("--limit-per-label", type=int, help="각 분할·라벨에서 앞의 N장만 평가")
    parser.add_argument("--batch-size", type=int, default=8)
    parser.add_argument("--examples-per-category", type=int, default=3)
    parser.add_argument("--device", choices=("auto", "cuda", "cpu"), default="auto")
    args = parser.parse_args()
    if args.limit_per_label is not None and args.limit_per_label < 1:
        parser.error("--limit-per-label은 1 이상이어야 합니다")
    if args.batch_size < 1 or args.examples_per_category < 1:
        parser.error("배치 크기와 사례 수는 1 이상이어야 합니다")

    model_dir = args.model_dir.resolve()
    data_dir = args.data_dir.resolve()
    manifest_path = args.manifest.resolve()
    repo = args.patchcore_repo.resolve()
    output = args.run_dir.resolve()
    if output.exists():
        parser.error(f"실행 결과 폴더가 이미 있습니다: {output}")
    if not (repo / "src/patchcore/patchcore.py").is_file():
        parser.error(f"원본 PatchCore 저장소가 없습니다: {repo}")
    for name in ("nnscorer_search_index.faiss", "patchcore_params.pkl", "training_metadata.json"):
        if not (model_dir / name).is_file():
            parser.error(f"모델 파일이 없습니다: {model_dir / name}")
    with (model_dir / "training_metadata.json").open() as file:
        training = json.load(file)
    # 4장 smoke 모델이나 다른 입력 변환으로 만든 모델의 결과를 섞지 않는다.
    expected = {
        "normal_image_count": 1020,
        "input_image_size": IMAGE_SIZE,
        "resize": IMAGE_SIZE,
        "center_crop": IMAGE_SIZE,
        "backbone": "wideresnet50",
        "layers": ["layer2", "layer3"],
        "faiss_device": "cpu",
    }
    for key, value in expected.items():
        if training.get(key) != value:
            parser.error(f"학습 설정 불일치: {key}={training.get(key)!r}, 기대값={value!r}")
    commit = subprocess.check_output(
        ["git", "-C", str(repo), "rev-parse", "HEAD"], text=True
    ).strip()
    if training.get("patchcore_commit") != commit:
        parser.error("현재 PatchCore 소스 commit이 학습 때와 다릅니다")
    if not manifest_path.is_file():
        parser.error(f"ROI manifest가 없습니다: {manifest_path}")
    records = load_records(data_dir, manifest_path, args.limit_per_label)

    available = torch.cuda.is_available()
    if args.device == "cuda" and not available:
        parser.error("CUDA를 사용할 수 없습니다")
    device = torch.device("cuda" if args.device == "auto" and available else
                          args.device if args.device != "auto" else "cpu")
    print(f"GPU 사용 가능: {available}; 평가 장치: {device}", flush=True)
    print(f"모델: {model_dir}; 결과: {output}", flush=True)
    print(f"검증 {len(records['val'])}장, 테스트 {len(records['test'])}장", flush=True)

    sys.path.insert(0, str(repo / "src"))
    import patchcore.common
    import patchcore.datasets.mvtec
    import patchcore.patchcore

    model = patchcore.patchcore.PatchCore(device)
    model.load_from_path(str(model_dir), device, patchcore.common.FaissNN(False, 4))
    indexed_patches = model.anomaly_scorer.nn_method.search_index.ntotal
    if (
        tuple(model.input_shape) != (3, IMAGE_SIZE, IMAGE_SIZE)
        or model.backbone.name != training["backbone"]
        or list(model.layers_to_extract_from) != training["layers"]
        or indexed_patches != training["indexed_patches"]
    ):
        raise ValueError("저장 모델과 학습 메타데이터의 입력·백본·레이어·인덱스 설정이 다릅니다")
    mean, std = patchcore.datasets.mvtec.IMAGENET_MEAN, patchcore.datasets.mvtec.IMAGENET_STD

    # 임계값을 정하기 전에는 검증 ROI만 추론한다. 테스트 점수는 이 결정에 사용하지 않는다.
    val_maps = score_records(model, records["val"], mean, std, args.batch_size)
    threshold = choose_threshold(records["val"])
    test_maps = score_records(model, records["test"], mean, std, args.batch_size)
    all_records = records["val"] + records["test"]
    for record in all_records:
        record["predicted_anomaly"] = int(record["raw_score"] >= threshold)

    summaries = {split: metrics(rows, threshold) for split, rows in records.items()}
    validation_distribution = {
        label: score_summary([record for record in records["val"] if record["label"] == label])
        for label in LABELS
    }
    groups, defects = {}, {}
    for split, rows in records.items():
        groups[split], defects[split] = summarize_groups(rows, threshold)

    output.mkdir(parents=True, exist_ok=False)
    with (output / "scores.csv").open("w", newline="") as file:
        fields = ("split", "filename", "roi_image", "source_scene", "slot", "label",
                  "binary_label", "raw_score", "predicted_anomaly")
        writer = csv.DictWriter(file, fieldnames=fields)
        writer.writeheader()
        writer.writerows({field: record[field] for field in fields} for record in all_records)

    # 예시 그림의 색상 범위도 검증 히트맵만으로 정해 테스트 그림에 동일하게 적용한다.
    validation_pixels = np.concatenate([value.ravel() for value in val_maps.values()])
    vmax = float(np.percentile(validation_pixels, 99))
    vmax = max(vmax, 1e-12)
    example_paths = {}
    for split, rows in records.items():
        categories = {
            "false_positive": sorted(
                (r for r in rows if r["binary_label"] == 0 and r["predicted_anomaly"]),
                key=lambda r: r["raw_score"], reverse=True,
            ),
            "false_negative": sorted(
                (r for r in rows if r["binary_label"] == 1 and not r["predicted_anomaly"]),
                key=lambda r: r["raw_score"],
            ),
            "true_positive": sorted(
                (r for r in rows if r["binary_label"] == 1 and r["predicted_anomaly"]),
                key=lambda r: r["raw_score"], reverse=True,
            ),
        }
        for category, examples in categories.items():
            key = f"{split}/{category}"
            example_paths[key] = []
            for record in examples[:args.examples_per_category]:
                folder = output / "examples" / split / category
                heatmap = (val_maps if split == "val" else test_maps)[record["roi_image"]]
                save_example(record, heatmap, folder, 0.0, vmax)
                example_paths[key].append((folder / record["filename"]).relative_to(output).as_posix())

    threshold_info = {
        "threshold": threshold,
        "rule": "predict anomaly when raw_score >= threshold",
        "selection": "maximize validation balanced accuracy (Youden J)",
        "tie_break": "choose the highest threshold; scores equal to threshold are anomaly",
        "selection_split": "val",
        "validation_counts": {
            label: sum(r["label"] == label for r in records["val"])
            for label in LABELS
        },
        "validation_score_distribution": validation_distribution,
        "heatmap_display": {
            "vmin": 0.0,
            "vmax": vmax,
            "vmax_source": "validation predicted heatmap pixel 99th percentile",
            "colormap": "inferno",
            "overlay_alpha": 0.45,
        },
        "model_dir": str(model_dir),
        "model_training_images": training["normal_image_count"],
        "patchcore_commit": commit,
    }
    (output / "threshold.json").write_text(json.dumps(threshold_info, indent=2) + "\n")
    (output / "metrics.json").write_text(
        json.dumps({"splits": summaries, "slots": groups, "defects": defects}, indent=2) + "\n"
    )
    full_data = args.limit_per_label is None
    write_report(output, summaries, groups, defects, validation_distribution,
                 threshold, example_paths, full_data)
    print(f"임계값: {threshold:.8g}; 검증 AUROC {summaries['val']['auroc']:.4f}; "
          f"테스트 AUROC {summaries['test']['auroc']:.4f}", flush=True)
    print(f"결과 저장: {output}", flush=True)


if __name__ == "__main__":
    main()
