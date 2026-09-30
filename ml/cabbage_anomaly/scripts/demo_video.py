#!/usr/bin/env python3
"""녹화한 640x640 팔레트 영상의 여섯 ROI에 PatchCore 결과를 그린다."""

import argparse
import json
import sys
from pathlib import Path

import cv2
import numpy as np
from PIL import Image
import torch
from torchvision import transforms

from build_dataset import CROP_CENTERS, CROP_SIZE, crop_box, load_rois


PROJECT = Path(__file__).resolve().parents[1]
DEFAULT_REPO = PROJECT.parents[2] / "patchcore-inspection"


def draw_frame(frame, model, transform, boxes, threshold, heatmap_max):
    """한 프레임의 여섯 ROI를 함께 추론하고 예측 히트맵·판정을 그린다."""
    # 배추 crop에 학습·평가와 같은 RGB/96×96/ImageNet 변환을 적용한다.
    tensors = []
    for box in boxes.values():
        x0, y0, x1, y1 = box
        rgb = cv2.cvtColor(frame[y0:y1, x0:x1], cv2.COLOR_BGR2RGB)
        tensors.append(transform(Image.fromarray(rgb)))
    scores, heatmaps = model.predict(torch.stack(tensors))
    if len(scores) != len(boxes) or len(heatmaps) != len(boxes):
        raise RuntimeError("PatchCore 출력 ROI 수가 여섯 개가 아닙니다")

    overlay = frame.copy()
    for (slot, box), score, heatmap in zip(boxes.items(), scores, heatmaps):
        score = float(score)
        heatmap = np.asarray(heatmap, dtype=np.float32)
        if not np.isfinite(score) or heatmap.shape != (CROP_SIZE, CROP_SIZE) or not np.isfinite(heatmap).all():
            raise ValueError(f"잘못된 PatchCore 출력: {slot}")
        x0, y0, x1, y1 = box
        # 프레임마다 자동 정규화하면 같은 점수가 다른 색으로 보여 비교가 어려워진다.
        scale = np.uint8(np.clip(heatmap / heatmap_max, 0, 1) * 255)
        color_map = cv2.applyColorMap(scale, cv2.COLORMAP_INFERNO)
        roi = overlay[y0:y1, x0:x1]
        overlay[y0:y1, x0:x1] = cv2.addWeighted(roi, 0.55, color_map, 0.45, 0)
        anomaly = score >= threshold
        color = (0, 0, 255) if anomaly else (0, 255, 0)
        cv2.rectangle(overlay, (x0, y0), (x1 - 1, y1 - 1), color, 2)
        cv2.rectangle(overlay, (x0, y0 - 19), (x1 - 1, y0 - 1), (0, 0, 0), -1)
        cv2.putText(overlay, f"{slot[-2:]} {score:.3f} {'NG' if anomaly else 'OK'}",
                    (x0 + 2, y0 - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.36, color, 1, cv2.LINE_AA)
    return overlay


def main():
    """녹화 영상을 읽어 전체 학습 모델의 판정 결과를 mp4로 저장한다."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", type=Path, required=True, help="녹화한 영상 파일(mp4 등)")
    parser.add_argument("--output", type=Path, required=True, help="오버레이 결과 mp4 파일")
    parser.add_argument("--model-dir", type=Path, default=PROJECT / "models/patchcore")
    parser.add_argument("--threshold-file", type=Path,
                        default=PROJECT / "results/image-level-full/threshold.json")
    parser.add_argument("--patchcore-repo", type=Path, default=DEFAULT_REPO)
    parser.add_argument("--device", choices=("auto", "cuda", "cpu"), default="auto")
    parser.add_argument("--max-frames", type=int, help="짧은 시연용 처리 프레임 수 제한")
    args = parser.parse_args()

    source = args.input.resolve()
    output = args.output.resolve()
    model_dir = args.model_dir.resolve()
    repo = args.patchcore_repo.resolve()
    if not source.is_file():
        parser.error(f"입력 영상이 없습니다: {source}")
    if output.exists() or output.suffix.lower() != ".mp4":
        parser.error(f"출력은 기존 파일이 아닌 .mp4 경로여야 합니다: {output}")
    if args.max_frames is not None and args.max_frames < 1:
        parser.error("--max-frames는 1 이상이어야 합니다")
    for name in ("nnscorer_search_index.faiss", "patchcore_params.pkl", "training_metadata.json"):
        if not (model_dir / name).is_file():
            parser.error(f"모델 파일이 없습니다: {model_dir / name}")
    if not (repo / "src/patchcore/patchcore.py").is_file():
        parser.error(f"PatchCore 저장소가 없습니다: {repo}")
    with (model_dir / "training_metadata.json").open() as file:
        training = json.load(file)
    with args.threshold_file.open() as file:
        selection = json.load(file)
    # 소수 이미지로 만든 smoke 모델과 다른 모델의 임계값 사용을 막는다.
    if (training.get("normal_image_count") != 1020 or
            training.get("input_image_size") != CROP_SIZE or
            training.get("resize") != CROP_SIZE or
            training.get("center_crop") != CROP_SIZE or
            selection.get("model_training_images") != 1020 or
            selection.get("patchcore_commit") != training.get("patchcore_commit")):
        parser.error("모델·임계값의 학습 설정이 현재 ROI와 다릅니다")
    threshold = float(selection["threshold"])
    heatmap_max = float(selection["heatmap_display"]["vmax"])
    if not np.isfinite(threshold) or not np.isfinite(heatmap_max) or heatmap_max <= 0:
        parser.error("임계값 또는 히트맵 색상 범위가 잘못됐습니다")

    video = cv2.VideoCapture(str(source))
    if not video.isOpened():
        parser.error(f"영상을 열 수 없습니다: {source}")
    fps = video.get(cv2.CAP_PROP_FPS)
    width = int(video.get(cv2.CAP_PROP_FRAME_WIDTH))
    height = int(video.get(cv2.CAP_PROP_FRAME_HEIGHT))
    # 고정 crop 중심은 640×640 Isaac Sim 장면을 기준으로 측정했다.
    if (width, height) != (640, 640) or not np.isfinite(fps) or fps <= 0:
        video.release()
        parser.error(f"학습 장면과 같은 640x640 영상과 유효한 FPS가 필요합니다: {width}x{height}, {fps}")
    rois = load_rois()
    boxes = {slot: crop_box(slot, rois[slot], width, height) for slot in CROP_CENTERS}

    available = torch.cuda.is_available()
    if args.device == "cuda" and not available:
        parser.error("CUDA를 사용할 수 없습니다")
    device = torch.device("cuda" if args.device == "auto" and available else
                          args.device if args.device != "auto" else "cpu")
    print(f"GPU 사용 가능: {available}; 추론 장치: {device}", flush=True)
    print(f"입력: {source}; 출력: {output}; 임계값: {threshold:.6f}", flush=True)

    sys.path.insert(0, str(repo / "src"))
    import patchcore.common
    import patchcore.datasets.mvtec
    import patchcore.patchcore

    model = patchcore.patchcore.PatchCore(device)
    model.load_from_path(str(model_dir), device, patchcore.common.FaissNN(False, 4))
    if (tuple(model.input_shape) != (3, CROP_SIZE, CROP_SIZE) or
            model.backbone.name != training["backbone"] or
            list(model.layers_to_extract_from) != training["layers"] or
            model.anomaly_scorer.nn_method.search_index.ntotal != training["indexed_patches"]):
        raise ValueError("저장 모델과 학습 메타데이터가 다릅니다")
    mean, std = patchcore.datasets.mvtec.IMAGENET_MEAN, patchcore.datasets.mvtec.IMAGENET_STD
    transform = transforms.Compose((
        transforms.Resize(CROP_SIZE),
        transforms.CenterCrop(CROP_SIZE),
        transforms.ToTensor(),
        transforms.Normalize(mean=mean, std=std),
    ))

    output.parent.mkdir(parents=True, exist_ok=True)
    writer = cv2.VideoWriter(str(output), cv2.VideoWriter_fourcc(*"mp4v"), fps, (width, height))
    if not writer.isOpened():
        video.release()
        raise RuntimeError(f"출력 영상을 만들 수 없습니다: {output}")
    count = 0
    try:
        # 입력의 FPS를 유지하되 --max-frames가 있으면 앞부분만 시연한다.
        while args.max_frames is None or count < args.max_frames:
            ok, frame = video.read()
            if not ok:
                break
            writer.write(draw_frame(frame, model, transform, boxes, threshold, heatmap_max))
            count += 1
            if count % 30 == 0:
                print(f"처리: {count}프레임", flush=True)
    finally:
        writer.release()
        video.release()
    if count == 0:
        raise RuntimeError("입력 영상에서 프레임을 읽지 못했습니다")
    print(f"완료: {count}프레임 → {output}", flush=True)


if __name__ == "__main__":
    main()
