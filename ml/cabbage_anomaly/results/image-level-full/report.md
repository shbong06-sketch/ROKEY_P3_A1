# PatchCore 이미지 단위 평가

임계값: `0.32702351`. 검증 balanced accuracy 최대; 점수 `>=`이면 이상, 동점이면 높은 임계값 선택.
최종 테스트 점수는 임계값 선정에 사용하지 않았다.

| 분할 | 정상/이상 | AUROC | TP | FP | TN | FN | 정상 오탐률 | 이상 재현율 |
| --- | --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| val | 270/75 | 0.9929 | 75 | 5 | 265 | 0 | 0.0185 | 1.0000 |
| test | 270/75 | 0.9999 | 74 | 1 | 269 | 1 | 0.0037 | 0.9867 |

## 검증 점수 분포

| 라벨 | 이미지 수 | 최소 | 중앙값 | 최대 |
| --- | ---: | ---: | ---: | ---: |
| good | 270 | 0.2215 | 0.2764 | 0.3906 |
| yellow | 50 | 0.3270 | 0.3549 | 0.4654 |
| brown | 25 | 0.4660 | 0.4772 | 0.5530 |

## val: 라벨과 슬롯

| 결함 라벨 | 이미지 수 | 미탐 |
| --- | ---: | ---: |
| yellow | 50 | 0 |
| brown | 25 | 0 |

| 슬롯 | 정상 수·점수 중앙값 | 이상 수·점수 중앙값 | 오탐 | 미탐 |
| --- | ---: | ---: | ---: | ---: |
| SLOT_01 | 45 · 0.2674 | 0 · — | 1 | 0 |
| SLOT_02 | 45 · 0.2679 | 0 · — | 1 | 0 |
| SLOT_03 | 45 · 0.2740 | 25 · 0.3547 | 1 | 0 |
| SLOT_04 | 45 · 0.2768 | 25 · 0.3590 | 1 | 0 |
| SLOT_05 | 45 · 0.2897 | 25 · 0.4772 | 0 | 0 |
| SLOT_06 | 45 · 0.2864 | 0 · — | 1 | 0 |

## test: 라벨과 슬롯

| 결함 라벨 | 이미지 수 | 미탐 |
| --- | ---: | ---: |
| yellow | 50 | 1 |
| brown | 25 | 0 |

| 슬롯 | 정상 수·점수 중앙값 | 이상 수·점수 중앙값 | 오탐 | 미탐 |
| --- | ---: | ---: | ---: | ---: |
| SLOT_01 | 45 · 0.2673 | 0 · — | 0 | 0 |
| SLOT_02 | 45 · 0.2713 | 0 · — | 0 | 0 |
| SLOT_03 | 45 · 0.2731 | 25 · 0.3589 | 0 | 0 |
| SLOT_04 | 45 · 0.2722 | 25 · 0.3645 | 0 | 1 |
| SLOT_05 | 45 · 0.2873 | 25 · 0.4796 | 1 | 0 |
| SLOT_06 | 45 · 0.2844 | 0 · — | 0 | 0 |

## 시각화

히트맵은 PatchCore 예측 점수이며 정답 마스크가 아니다. 모든 그림은 검증 히트맵 픽셀의 99백분위수를 공통 vmax로 사용하고 vmin=0, inferno 색상표, 오버레이 alpha=0.45를 사용한다.
- val/false_positive: 3장 — examples/val/false_positive/prim_rotation_rgb_0070_SLOT_06.png, examples/val/false_positive/prim_rotation_rgb_0070_SLOT_04.png, examples/val/false_positive/prim_rotation_rgb_0070_SLOT_02.png
- val/false_negative: 0장 (해당 사례 없음)
- val/true_positive: 3장 — examples/val/true_positive/synthetic_defect_rgb_0000_SLOT_05.png, examples/val/true_positive/synthetic_defect_rgb_0011_SLOT_05.png, examples/val/true_positive/synthetic_defect_rgb_0001_SLOT_05.png
- test/false_positive: 1장 — examples/test/false_positive/prim_rotation_rgb_0039_SLOT_05.png
- test/false_negative: 1장 — examples/test/false_negative/synthetic_defect_rgb_0005_SLOT_04.png
- test/true_positive: 3장 — examples/test/true_positive/synthetic_defect_rgb_0003_SLOT_05.png, examples/test/true_positive/synthetic_defect_rgb_0034_SLOT_05.png, examples/test/true_positive/synthetic_defect_rgb_0009_SLOT_05.png

## 해석 범위

검증에서 정한 한 임계값으로 최종 테스트를 평가했다. 픽셀 정확도 지표는 계산하지 않았다.
현재 이상 장면은 SLOT_03·04=yellow, SLOT_05=brown으로 고정되어 있어 슬롯 위치·배경과 색상 차이를 분리해 평가할 수 없다.
같은 원본 장면의 여러 ROI는 서로 독립 표본이 아니다. 아래 하위 그룹은 건수가 작아 비율보다 건수를 우선해 읽어야 한다.
