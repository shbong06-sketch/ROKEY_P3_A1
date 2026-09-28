# 배추 이상 탐지 데이터셋

Isaac Sim 팔레트 RGB 원본에서 배추별 ROI를 잘라 PatchCore 학습·평가에 사용한다. 원본 한 장에는 배추 6개가 있으며, 같은 원본에서 나온 ROI는 항상 같은 분할에 둔다.

## 디렉터리

```text
data/
├── raw/                         # 640×640 원본; 빌드 과정에서 수정하지 않음
│   ├── lighting_only/           # 200장
│   ├── prim_rotation/           # 140장
│   ├── synthetic_defect/        # 50장
│   └── real/                    # 향후 실제 카메라 원본
├── metadata/
│   ├── scene_split.csv          # 원본 이미지별 분할
│   └── roi_manifest.csv         # ROI 파일과 원본·슬롯·crop 좌표 연결
├── validation/                  # 임계값 결정용; MVTec 로더와 별도 사용
│   ├── good/
│   ├── yellow/
│   └── brown/
└── mvtec/cabbage/               # PatchCore용 MVTec 형태
    ├── train/good/
    ├── test/{good,yellow,brown}/
    └── ground_truth/{yellow,brown}/
```

## 분할 기준

원본 팔레트 이미지를 먼저 난수 시드 42로 분할한 뒤 ROI를 생성한다.

| 원본 폴더 | train | validation | test | 출력 제외 |
| --- | ---: | ---: | ---: | ---: |
| `lighting_only` | 100 | 30 | 30 | 예비 40 |
| `prim_rotation` | 70 | 15 | 15 | 이상 포함 40 |
| `synthetic_defect` | 0 | 25 | 25 | 0 |

`prim_rotation/rgb_0000.png`부터 `rgb_0099.png`까지만 정상 원본으로 사용한다. `rgb_0100.png`부터 `rgb_0139.png`까지는 이상 배추가 섞여 있어 `good`에 넣지 않고 `scene_split.csv`에 `excluded_defect`로 기록한다. `lighting_only` 예비 40장은 `reserve`로 기록하며 원본 상태로 보관한다.

정상 원본에서는 여섯 슬롯을 모두 `good`으로 저장한다. 현재 `synthetic_defect` 원본의 배치는 `SLOT_03`, `SLOT_04`가 `yellow`, `SLOT_05`가 `brown`이다. 이 세 슬롯만 validation과 test에 저장한다. 이 배치가 바뀌면 생성 스크립트의 슬롯별 라벨도 확인해야 한다.

## ROI와 파일명

슬롯 번호는 `SLOT_01`~`SLOT_03`이 위쪽 왼쪽부터 오른쪽, `SLOT_04`~`SLOT_06`이 아래쪽 왼쪽부터 오른쪽 순서다. `scripts/build_dataset.py`는 object detection 설정의 슬롯 영역을 확인하고, 각 영역 안의 배추 중심을 기준으로 **96×96 픽셀**을 자른다. 중심 좌표는 현재 640×640 Isaac Sim 카메라에 맞춰져 있다.

파일명은 `<원본 폴더>_<확장자를 뺀 원본 파일명>_<슬롯>.png` 형식이다. 예: `synthetic_defect_rgb_0000_SLOT_03.png`. `roi_manifest.csv`의 `x_min`, `y_min`, `x_max`, `y_max`는 원본 이미지 기준 crop 좌표이며 오른쪽·아래쪽 경계는 포함하지 않는다. CSV의 파일 경로는 `data/` 기준 상대 경로다.

## 현재 생성 수량

| 위치 | good | yellow | brown | 합계 |
| --- | ---: | ---: | ---: | ---: |
| `mvtec/cabbage/train` | 1,020 | 0 | 0 | 1,020 |
| `validation` | 270 | 50 | 25 | 345 |
| `mvtec/cabbage/test` | 270 | 50 | 25 | 345 |
| **합계** | **1,560** | **100** | **50** | **1,710** |

`ground_truth` 픽셀 마스크는 아직 없다. 현재 데이터셋은 이미지 단위 이상 판정에 사용하며, 픽셀 단위 평가는 마스크를 추가한 뒤 진행한다.

## 생성

저장소 루트에서 실행한다. Python 패키지 `Pillow`, `PyYAML`이 필요하다.

```bash
python3 ml/cabbage_anomaly/scripts/build_dataset.py
```

기본 시드는 42이며 `--seed`로 변경할 수 있다. 출력 CSV나 ROI 이미지가 이미 있으면 스크립트가 중단하므로 기존 분할을 덮어쓰지 않는다.

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
