#!/usr/bin/env python3
"""원본 PatchCore 구현으로 배추의 정상 ROI만 학습해 FAISS 인덱스를 저장한다."""

import argparse
import gc
import json
import random
import subprocess
import sys
from pathlib import Path

import numpy as np
import torch


# 실행 위치가 달라도 프로젝트와 외부 저장소를 찾도록 스크립트 위치를 기준으로 한다.
PROJECT = Path(__file__).resolve().parents[1]
DEFAULT_REPO = PROJECT.parents[2] / "patchcore-inspection"
IMAGE_SIZE = 96
BACKBONE = "wideresnet50"
LAYERS = ("layer2", "layer3")
CORESET_RATIO = 0.01


def main() -> None:
    """정상 ROI의 패치 특징을 모아 모델을 저장하고 재로드로 검증한다."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--patchcore-repo", type=Path, default=DEFAULT_REPO)
    parser.add_argument("--data-root", type=Path, default=PROJECT / "data/mvtec")
    parser.add_argument("--output", type=Path, default=PROJECT / "models/patchcore")
    parser.add_argument("--limit", type=int, help="smoke test에 사용할 정상 이미지 수")
    parser.add_argument("--batch-size", type=int, default=8)
    parser.add_argument("--seed", type=int, default=42)
    parser.add_argument("--device", choices=("auto", "cuda", "cpu"), default="auto")
    args = parser.parse_args()

    if args.limit is not None and args.limit < 1:
        parser.error("--limit은 1 이상이어야 합니다")
    if args.batch_size < 1:
        parser.error("--batch-size는 1 이상이어야 합니다")

    repo = args.patchcore_repo.expanduser().resolve()
    data_root = args.data_root.expanduser().resolve()
    output = args.output.expanduser().resolve()
    source = repo / "src/patchcore/patchcore.py"
    if not source.is_file():
        parser.error(f"PatchCore 소스가 없습니다: {source}")
    if not (data_root / "cabbage/train/good").is_dir():
        parser.error(f"정상 학습 폴더가 없습니다: {data_root / 'cabbage/train/good'}")
    if output.exists() and (not output.is_dir() or any(output.iterdir())):
        parser.error(f"출력 폴더가 비어 있지 않습니다: {output}")
    # 저장 모델이 어느 버전의 외부 PatchCore 구현으로 만들어졌는지 남긴다.
    try:
        commit = subprocess.check_output(
            ["git", "-C", str(repo), "rev-parse", "HEAD"], text=True
        ).strip()
    except (OSError, subprocess.CalledProcessError) as exc:
        parser.error(f"PatchCore commit을 확인할 수 없습니다: {exc}")

    cuda_available = torch.cuda.is_available()
    if args.device == "cuda" and not cuda_available:
        parser.error("CUDA를 사용할 수 없습니다")
    device = torch.device("cuda" if args.device == "auto" and cuda_available else
                          args.device if args.device != "auto" else "cpu")
    print(f"GPU 사용 가능: {cuda_available}; 학습 장치: {device}", flush=True)
    print(f"입력: {data_root / 'cabbage/train/good'}", flush=True)
    print(f"출력: {output}", flush=True)
    print(f"PatchCore: {repo} @ {commit}", flush=True)

    # 설치된 다른 patchcore가 아닌 지정한 원본 저장소의 src를 import한다.
    sys.path.insert(0, str(repo / "src"))
    import patchcore.backbones
    import patchcore.common
    import patchcore.datasets.mvtec
    import patchcore.patchcore
    import patchcore.sampler

    random.seed(args.seed)
    np.random.seed(args.seed)
    torch.manual_seed(args.seed)
    if cuda_available:
        torch.cuda.manual_seed_all(args.seed)

    # 현재 TRAIN 폴더에는 good만 있다. resize와 crop을 96으로 맞춰 ROI 가장자리를 보존한다.
    dataset = patchcore.datasets.mvtec.MVTecDataset(
        str(data_root), classname="cabbage", resize=IMAGE_SIZE,
        imagesize=IMAGE_SIZE, split=patchcore.datasets.mvtec.DatasetSplit.TRAIN,
    )
    # 원본 Dataset의 내부 목록에서 이미지 라벨과 마스크 경로를 직접 확인한다.
    if not dataset or any(item[1] != "good" or item[3] is not None
                          for item in dataset.data_to_iterate):
        parser.error("학습 Dataset에 정상 ROI 이외의 데이터가 있습니다")
    image_count = len(dataset)
    input_shape = dataset.imagesize
    if args.limit is not None:
        # smoke test도 전체 학습과 같은 순서의 앞부분 이미지를 사용한다.
        if args.limit > image_count:
            parser.error(f"--limit {args.limit}이 정상 이미지 수 {image_count}보다 큽니다")
        image_count = args.limit
        dataset = torch.utils.data.Subset(dataset, range(image_count))
    loader = torch.utils.data.DataLoader(
        dataset, batch_size=args.batch_size, shuffle=False, num_workers=0,
        pin_memory=device.type == "cuda",
    )
    print(f"정상 이미지: {image_count}장; 입력 변환: resize {IMAGE_SIZE}, center crop {IMAGE_SIZE}", flush=True)

    # ImageNet 사전학습 CNN에서 layer2/3 특징을 추출한다. fit()은 CNN 가중치를 갱신하지 않는다.
    backbone = patchcore.backbones.load(BACKBONE)
    backbone.name = BACKBONE
    model = patchcore.patchcore.PatchCore(device)
    model.load(
        backbone=backbone, layers_to_extract_from=LAYERS, device=device,
        input_shape=input_shape,
        pretrain_embed_dimension=1024, target_embed_dimension=384,
        patchsize=3, anomaly_score_num_nn=1,
        featuresampler=patchcore.sampler.ApproximateGreedyCoresetSampler(
            CORESET_RATIO, device=device,
        ),
        nn_method=patchcore.common.FaissNN(False, 4),  # 특징 추출 장치와 무관하게 FAISS는 CPU 사용
    )
    # fit()은 정상 패치 특징의 1%를 대표 집합으로 골라 FAISS 인덱스를 만든다.
    model.fit(loader)
    output.mkdir(parents=True, exist_ok=True)
    model.save_to_path(str(output))
    for name in ("nnscorer_search_index.faiss", "patchcore_params.pkl"):
        path = output / name
        if not path.is_file() or path.stat().st_size == 0:
            raise RuntimeError(f"모델 출력이 없습니다: {path}")

    # 저장한 설정과 인덱스를 재로드해 인덱스가 실제로 다시 읽히는지 확인한다.
    del model, backbone
    gc.collect()
    if device.type == "cuda":
        torch.cuda.empty_cache()
    restored = patchcore.patchcore.PatchCore(device)
    restored.load_from_path(str(output), device, patchcore.common.FaissNN(False, 4))
    indexed_patches = restored.anomaly_scorer.nn_method.search_index.ntotal
    if indexed_patches < 1:
        raise RuntimeError("재로드한 FAISS 인덱스가 비어 있습니다")

    # pkl은 PatchCore 복원 설정이고, 이 JSON은 학습 조건과 원본 코드 버전을 남긴다.
    metadata = {
        "patchcore_commit": commit,
        "patchcore_repo": str(repo),
        "training_data": str(data_root / "cabbage/train/good"),
        "normal_image_count": image_count,
        "batch_size": args.batch_size,
        "seed": args.seed,
        "device": str(device),
        "backbone": BACKBONE,
        "pretrained": True,
        "layers": LAYERS,
        "input_image_size": IMAGE_SIZE,
        "resize": IMAGE_SIZE,
        "center_crop": IMAGE_SIZE,
        "pretrain_embed_dimension": 1024,
        "target_embed_dimension": 384,
        "patchsize": 3,
        "anomaly_score_num_nn": 1,
        "coreset_method": "approximate_greedy",
        "coreset_ratio": CORESET_RATIO,
        "faiss_device": "cpu",
        "indexed_patches": indexed_patches,
    }
    (output / "training_metadata.json").write_text(
        json.dumps(metadata, ensure_ascii=False, indent=2) + "\n"
    )
    print(f"저장·재로드 완료: {indexed_patches}개 패치, {output}", flush=True)


if __name__ == "__main__":
    main()
