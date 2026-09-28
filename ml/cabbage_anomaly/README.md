# 배추 이상 탐지: PatchCore

정상 배추 ROI로 PatchCore 특징 인덱스를 만들고, 검증 데이터에서 이미지 단위 판정 임계값을 정한 뒤 최종 테스트와 녹화 영상에 적용한다. 데이터셋의 구성·분할·ROI 규칙은 [DATASET.md](data/DATASET.md)에 정리했다.

## 설치 준비

다음 명령은 `ROKEY_P3_A1` 프로젝트 루트에서 실행한다. 원본 PatchCore 저장소는 프로젝트 **안이 아니라 같은 상위 디렉터리**에 둔다.

```text
작업 디렉터리/
├── ROKEY_P3_A1/
└── patchcore-inspection/
```

```bash
cd /path/to/ROKEY_P3_A1
git clone https://github.com/amazon-science/patchcore-inspection.git ../patchcore-inspection
# 저장된 전체 학습 모델이 사용한 원본 코드 버전
git -C ../patchcore-inspection checkout fcaa92f124fb1ad74a7acf56726decd4b27cbcad

python3.12 -m venv ml/cabbage_anomaly/.venv
ml/cabbage_anomaly/.venv/bin/python -m pip install --upgrade pip
ml/cabbage_anomaly/.venv/bin/python -m pip install -r ../patchcore-inspection/requirements.txt
ml/cabbage_anomaly/.venv/bin/python -m pip install timm PyYAML opencv-python-headless
ml/cabbage_anomaly/.venv/bin/python -c "import torch, faiss, timm, yaml, cv2; print('dependencies OK')"
```

원본 저장소의 파일명은 `requirements.txt`이며, `timm`은 원본 코드가 import하지만 해당 파일에 없어서 별도로 설치한다. `PyYAML`은 데이터셋 생성, `opencv-python-headless`는 녹화 영상 시연에 사용한다. Python 3.12는 현재 확인한 환경의 버전이다. 스크립트는 형제 저장소의 `src`를 직접 import하므로 원본 저장소를 프로젝트 안으로 복사하거나 수정하지 않는다.

`data/` 원본 이미지, `models/patchcore/`의 전체 학습 모델, `results/image-level-full/threshold.json`은 Git에 포함되지 않는다. 다른 머신에서는 해당 파일을 별도로 준비하거나 아래 순서대로 데이터셋 생성 → 전체 학습 → 전체 평가를 실행해야 영상 시연까지 가능하다.

## 데이터셋 준비

원본 이미지 배치, 장면 분할, ROI 파일명과 수량은 [DATASET.md](data/DATASET.md)를 참고한다. `data/raw/`에 원본 이미지를 준비한 뒤 프로젝트 루트에서 생성한다. 이미 생성된 데이터셋은 덮어쓰지 않는다.

```bash
ml/cabbage_anomaly/.venv/bin/python ml/cabbage_anomaly/scripts/build_dataset.py
```

## PatchCore 정상 데이터 학습

`scripts/train_patchcore.py`는 외부 저장소 `../patchcore-inspection/src`의 원본 PatchCore 구현을 import한다. 기본 외부 저장소 경로는 프로젝트와 같은 상위 폴더의 `patchcore-inspection`이며, 다른 위치에 있으면 `--patchcore-repo`로 지정한다. 학습에는 `mvtec/cabbage/train/good`의 정상 ROI만 사용한다. validation·test 이미지와 `ground_truth` 마스크는 읽지 않는다.

입력 ROI는 이미 96×96이므로 원본 MVTec 학습 Dataset에 `resize=96`, `imagesize=96`을 전달한다. 즉 **96×96 resize → 96×96 center crop → ImageNet 정규화** 순서이며 가장자리 픽셀을 잘라내지 않는다. 사전학습 WideResNet50의 `layer2`, `layer3` 특징을 사용하고, embedding 차원은 1024→384, 패치 크기는 3, approximate greedy coreset 비율은 1%다. CNN 특징 추출과 coreset은 CUDA가 있으면 GPU를 사용하고, 최근접 이웃 검색 인덱스는 CPU FAISS로 만든다. 기본 시드는 42, 배치 크기는 8이다.

프로젝트 루트에서 가상환경 Python으로 실행한다. `--output`은 빈 폴더를 지정해야 하며, 기존 모델을 덮어쓰지 않는다.

```bash
# 정상 ROI 4장 smoke test
ml/cabbage_anomaly/.venv/bin/python ml/cabbage_anomaly/scripts/train_patchcore.py \
  --limit 4 --output ml/cabbage_anomaly/models/patchcore-smoke

# 정상 ROI 1,020장 전체 학습
ml/cabbage_anomaly/.venv/bin/python ml/cabbage_anomaly/scripts/train_patchcore.py \
  --output ml/cabbage_anomaly/models/patchcore
```

필요하면 `--data-root`로 `data/mvtec`의 위치를, `--device cpu`로 실행 장치를 바꿀 수 있다. 실행 전에 GPU 사용 가능 여부와 입력·출력 경로를 출력한다. 사전학습 가중치가 로컬에 없으면 첫 실행에서 torchvision이 다운로드할 수 있다.

PatchCore의 `fit()`은 CNN 가중치를 새로 학습하지 않는다. 정상 이미지에서 특징을 추출해 대표 패치를 고른 뒤 FAISS 인덱스에 저장한다. 출력 폴더에는 `nnscorer_search_index.faiss`, `patchcore_params.pkl`, `training_metadata.json`이 생긴다. 마지막 JSON에는 설정, 정상 이미지 수, 시드, 외부 PatchCore commit SHA와 인덱스 크기를 기록한다. 스크립트는 저장 후 모델을 재로드하고 인덱스가 비어 있지 않은지도 확인한다.

현재 환경에서 4장 smoke test와 1,020장 전체 학습·재로드를 완료했다. 전체 모델의 FAISS 인덱스에는 대표 패치 1,468개가 저장됐다. 이 단계에서는 결함 평가 지표를 계산하지 않는다.

## 마스크 없는 이미지 단위 평가

`scripts/evaluate_image_level.py`는 **1,020장 전체 학습 모델**의 `training_metadata.json`과 PatchCore commit, 입력 크기를 확인한 뒤 `load_from_path()`로 모델을 복원한다. 검증은 `data/validation/{good,yellow,brown}`, 최종 테스트는 `data/mvtec/cabbage/test/{good,yellow,brown}`의 ROI 이미지만 직접 읽는다. 원본 MVTec TEST Dataset과 `ground_truth`는 사용하지 않는다. ROI 경로를 `data/metadata/roi_manifest.csv`의 원본 장면·슬롯·라벨과 대조하며, 연결되지 않거나 다른 항목이 있으면 오류로 중단한다.

학습과 동일하게 RGB 변환, **96×96 resize → 96×96 center crop → ImageNet 정규화**를 적용한다. 실제 ROI가 96×96이 아니거나 저장 모델·메타데이터의 입력 설정이 다르면 평가하지 않는다. 원본 PatchCore 저장소가 다른 곳에 있으면 `--patchcore-repo`로 지정한다.

프로젝트 루트에서 실행한다. `--run-dir`은 새 경로여야 하며 결과를 덮어쓰지 않는다. 생략하면 `results/` 아래에 실행 시각이 포함된 폴더를 만든다.

```bash
# 입출력·CSV·히트맵 확인: 분할과 라벨마다 2장
ml/cabbage_anomaly/.venv/bin/python ml/cabbage_anomaly/scripts/evaluate_image_level.py \
  --limit-per-label 2 --run-dir ml/cabbage_anomaly/results/image-level-smoke

# 전체 검증 345장과 최종 테스트 345장
ml/cabbage_anomaly/.venv/bin/python ml/cabbage_anomaly/scripts/evaluate_image_level.py \
  --run-dir ml/cabbage_anomaly/results/image-level-full
```

검증 이미지의 원본 이상 점수만 사용해 **balanced accuracy가 최대**인 단일 임계값을 고른다. 이는 `TPR - FPR`(Youden J) 최대화와 같으며, 동점이면 더 높은 임계값을 택한다. 점수가 임계값과 **같으면 이상**으로 판정한다. 최종 테스트 점수는 임계값 선정에 사용하지 않는다. 임계값과 규칙, 검증 점수 분포는 `threshold.json`에 저장한다.

각 실행 폴더의 `scores.csv`에는 분할, ROI 파일명, 원본 장면, 슬롯, 라벨, 이진 라벨, 원본 이상 점수와 판정을 기록한다. `metrics.json`과 `report.md`에는 검증·테스트의 이미지 AUROC, TP/FP/TN/FN, 정상 오탐률, 이상 재현율, yellow·brown별 미탐 수와 슬롯별 점수·오탐·미탐 건수를 기록한다. `examples/`에는 오탐·미탐·올바르게 탐지한 이상 ROI의 예측 히트맵과 오버레이를 저장한다. 히트맵은 **정답 마스크가 아닌 PatchCore 예측**이다. 모든 그림에 공통으로 `vmin=0`, **검증 히트맵 픽셀의 99백분위수**를 `vmax`로 적용하며 값은 `threshold.json`에 기록한다.

현재 전체 실행에서는 임계값 `0.3270235062`를 얻었다. 검증은 AUROC **0.9929**, TP/FP/TN/FN **75/5/265/0**이고, 최종 테스트는 AUROC **0.9999**, TP/FP/TN/FN **74/1/269/1**이다. 이 수치는 현재 합성 장면에 한정된다. 이상 배추가 항상 `SLOT_03·04=yellow`, `SLOT_05=brown`에 있어서 슬롯 위치·배경이 점수에 영향을 줄 수 있고, 같은 원본에서 나온 여러 ROI는 독립 표본이 아니다. 하위 그룹은 건수와 함께 읽어야 한다. 픽셀 AUROC, PRO, IoU 등 위치 정확도 지표는 계산하지 않는다.

## 녹화 영상 시연

`scripts/demo_video.py`는 `/rgb`에서 별도로 녹화한 **640×640 영상 파일**을 읽는다. 기존 ROS 2 노드와 연결하지 않는다. 데이터셋 생성 때와 같은 고정 좌표로 여섯 슬롯을 96×96으로 자르고, 전체 학습 PatchCore 모델과 검증에서 정한 임계값으로 각 슬롯의 이미지 단위 점수를 계산한다. 결과 영상에는 ROI별 예측 히트맵, 점수, `OK`/`NG`가 표시된다. 히트맵은 정답 마스크가 아니며 검증에서 정한 공통 색상 범위를 사용한다.

입력은 녹화한 mp4 등 OpenCV가 읽는 영상 파일이며 결과는 mp4다. 기존 출력 파일은 덮어쓰지 않는다. 위 설치 절차로 OpenCV를 가상환경에 설치했다면 다음 명령을 사용한다.

```bash
ml/cabbage_anomaly/.venv/bin/python ml/cabbage_anomaly/scripts/demo_video.py \
  --input /path/to/recorded_rgb.mp4 \
  --output ml/cabbage_anomaly/results/recorded_overlay.mp4 \
  --max-frames 30
```

현재 개발 환경의 가상환경에는 OpenCV가 없고 시스템에만 있다. 이 환경에서 바로 실행할 때는 명령 앞에 `PYTHONPATH=/usr/lib/python3/dist-packages`를 붙인다.

전체 영상을 처리할 때는 `--max-frames`를 생략한다. 고정 crop은 현재 640×640 Isaac Sim 장면에서만 확인됐으며, 영상 압축이나 카메라 구도 변화는 학습·평가 결과와 점수를 다르게 만들 수 있다. 이 시연은 녹화 영상의 동작 확인용으로, 실시간 처리 속도나 실제 카메라 성능을 증명하지 않는다.
