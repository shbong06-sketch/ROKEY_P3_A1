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
