#!/usr/bin/env python3
"""팔레트 원본을 장면별로 나눈 뒤 여섯 슬롯 ROI 데이터셋을 만든다."""

import argparse
import csv
import random
from pathlib import Path

from PIL import Image
import yaml

ROOT = Path(__file__).resolve().parent.parent
DATA = ROOT / "data"
ROI_CONFIG = ROOT.parent.parent / "cobot3_ws/src/smart_farm_vision/config/object_detection.yaml"
SPLITS = {
    "lighting_only": {"train": 100, "val": 30, "test": 30, "reserve": 40},
    "prim_rotation": {"train": 70, "val": 15, "test": 15},
    "synthetic_defect": {"val": 25, "test": 25},
}
# 현재 synthetic_defect 원본 50장의 슬롯 배치. 새 배치를 추가하면 이 라벨을 갱신한다.
SYNTHETIC_LABELS = {"SLOT_03": "yellow", "SLOT_04": "yellow", "SLOT_05": "brown"}
# 640x640 Isaac Sim 원본에서 확인한 배추 중심. 슬롯 판정 ROI의 중심과는 다르다.
CROP_CENTERS = {
    "SLOT_01": (191, 283), "SLOT_02": (319, 283), "SLOT_03": (453, 283),
    "SLOT_04": (166, 380), "SLOT_05": (319, 380), "SLOT_06": (474, 380),
}
CROP_SIZE = 96
IMAGE_SUFFIXES = {".png", ".jpg", ".jpeg", ".webp", ".bmp", ".tif", ".tiff"}


def load_rois():
    parameters = yaml.safe_load(ROI_CONFIG.read_text())["object_detection"]["ros__parameters"]
    rois = {}
    for index in range(1, 7):
        slot = f"SLOT_{index:02d}"
        x, y, width, height = parameters[f"slot_rois.{slot}"]
        if not (0 <= x < x + width <= 1.000001 and 0 <= y < y + height <= 1.000001):
            raise ValueError(f"잘못된 ROI: {slot}")
        rois[slot] = (x, y, width, height)
    return rois


def crop_box(slot, roi, width, height):
    if (width, height) != (640, 640):
        raise ValueError(f"{slot}: 640x640 이미지만 지원합니다: {width}x{height}")
    center_x, center_y = CROP_CENTERS[slot]
    x, y, roi_width, roi_height = roi
    if not (x <= center_x / width < x + roi_width and
            y <= center_y / height < y + roi_height):
        raise ValueError(f"{slot}: 배추 중심이 검출 슬롯 ROI 밖에 있습니다")
    half = CROP_SIZE // 2
    return center_x - half, center_y - half, center_x + half, center_y + half


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--seed", type=int, default=42, help="원본 이미지 분할용 난수 시드")
    args = parser.parse_args()

    try:
        rois = load_rois()
    except (OSError, KeyError, TypeError, ValueError, yaml.YAMLError) as exc:
        parser.error(f"ROI 설정을 읽을 수 없습니다: {exc}")

    split_file = DATA / "metadata/scene_split.csv"
    manifest_file = DATA / "metadata/roi_manifest.csv"
    output_dirs = [DATA / "mvtec/cabbage/train/good"]
    output_dirs += [DATA / f"mvtec/cabbage/test/{label}" for label in ("good", "brown", "yellow")]
    output_dirs += [DATA / f"validation/{label}" for label in ("good", "brown", "yellow")]
    if split_file.exists() or manifest_file.exists() or any(
        folder.exists() and any(folder.iterdir()) for folder in output_dirs
    ):
        parser.error("출력 CSV 또는 ROI 이미지가 이미 있습니다. 기존 데이터를 확인하세요")

    sources = {}
    excluded_prim = []
    for category, split_counts in SPLITS.items():
        folder = DATA / "raw" / category
        images = sorted(path for path in folder.iterdir()
                        if path.is_file() and path.suffix.lower() in IMAGE_SUFFIXES) if folder.is_dir() else []
        if category == "prim_rotation":
            expected_names = {f"rgb_{index:04d}.png" for index in range(140)}
            if {image.name for image in images} != expected_names:
                parser.error(f"{folder}: 정상 0000~0099장과 이상 0100~0139장을 확인하세요")
            excluded_prim = images[100:]
            images = images[:100]
        expected = sum(split_counts.values())
        if len(images) != expected:
            parser.error(f"{folder}: 이미지 {expected}장이 필요하지만 {len(images)}장입니다")
        sources[category] = images

    rng = random.Random(args.seed)
    scenes = []
    for category, split_counts in SPLITS.items():
        images = sources[category].copy()
        rng.shuffle(images)
        offset = 0
        for split, count in split_counts.items():
            scenes.extend((image, category, split) for image in images[offset:offset + count])
            offset += count
    scenes.extend((image, "prim_rotation", "excluded_defect") for image in excluded_prim)

    for image, _, _ in scenes:
        try:
            with Image.open(image) as source:
                source.verify()
        except (OSError, ValueError) as exc:
            parser.error(f"원본 이미지를 읽을 수 없습니다: {image}: {exc}")

    split_file.parent.mkdir(parents=True, exist_ok=True)
    with split_file.open("w", newline="") as scene_csv, manifest_file.open("w", newline="") as roi_csv:
        scene_writer = csv.writer(scene_csv)
        roi_writer = csv.writer(roi_csv)
        scene_writer.writerow(("source_image", "category", "split"))
        roi_writer.writerow(("roi_image", "source_image", "category", "split", "slot", "label",
                             "x_min", "y_min", "x_max", "y_max"))
        for image, category, split in scenes:
            source_path = image.relative_to(DATA).as_posix()
            scene_writer.writerow((source_path, category, split))
            if split in ("reserve", "excluded_defect"):
                continue
            with Image.open(image) as source:
                width, height = source.size
                for slot, roi in rois.items():
                    if category == "synthetic_defect" and slot not in SYNTHETIC_LABELS:
                        continue
                    label = SYNTHETIC_LABELS[slot] if category == "synthetic_defect" else "good"
                    folder = (DATA / "validation" / label if split == "val" else
                              DATA / "mvtec/cabbage" / split / label)
                    folder.mkdir(parents=True, exist_ok=True)
                    output = folder / f"{category}_{image.stem}_{slot}.png"
                    box = crop_box(slot, roi, width, height)
                    source.crop(box).save(output)
                    roi_writer.writerow((output.relative_to(DATA).as_posix(), source_path,
                                         category, split, slot, label, *box))

    roi_count = sum(0 if split in ("reserve", "excluded_defect") else
                    3 if category == "synthetic_defect" else 6
                    for _, category, split in scenes)
    print(f"완료: 원본 {len(scenes)}장 → ROI {roi_count}장")
    print(f"분할 기록: {split_file}")
    print(f"ROI 목록: {manifest_file}")


if __name__ == "__main__":
    main()
