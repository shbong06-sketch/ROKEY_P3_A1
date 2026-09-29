# PatchCore 이미지 단위 평가

임계값: `0.40631866`. 검증 balanced accuracy 최대; 점수 `>=`이면 이상, 동점이면 높은 임계값 선택.
최종 테스트 점수는 임계값 선정에 사용하지 않았다.

| 분할 | 정상/이상 | AUROC | TP | FP | TN | FN | 정상 오탐률 | 이상 재현율 |
| --- | --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| val | 2/4 | 1.0000 | 4 | 0 | 2 | 0 | 0.0000 | 1.0000 |
| test | 2/4 | 1.0000 | 2 | 0 | 2 | 2 | 0.0000 | 0.5000 |

## 검증 점수 분포

| 라벨 | 이미지 수 | 최소 | 중앙값 | 최대 |
| --- | ---: | ---: | ---: | ---: |
| good | 2 | 0.2338 | 0.2439 | 0.2540 |
| yellow | 2 | 0.4063 | 0.4359 | 0.4654 |
| brown | 2 | 0.4938 | 0.5234 | 0.5530 |

## val: 라벨과 슬롯

| 결함 라벨 | 이미지 수 | 미탐 |
| --- | ---: | ---: |
| yellow | 2 | 0 |
| brown | 2 | 0 |

| 슬롯 | 정상 수·점수 중앙값 | 이상 수·점수 중앙값 | 오탐 | 미탐 |
| --- | ---: | ---: | ---: | ---: |
| SLOT_01 | 1 · 0.2338 | 0 · — | 0 | 0 |
| SLOT_02 | 1 · 0.2540 | 0 · — | 0 | 0 |
| SLOT_03 | 0 · — | 1 · 0.4063 | 0 | 0 |
| SLOT_04 | 0 · — | 1 · 0.4654 | 0 | 0 |
| SLOT_05 | 0 · — | 2 · 0.5234 | 0 | 0 |
| SLOT_06 | 0 · — | 0 · — | 0 | 0 |

## test: 라벨과 슬롯

| 결함 라벨 | 이미지 수 | 미탐 |
| --- | ---: | ---: |
| yellow | 2 | 2 |
| brown | 2 | 0 |

| 슬롯 | 정상 수·점수 중앙값 | 이상 수·점수 중앙값 | 오탐 | 미탐 |
| --- | ---: | ---: | ---: | ---: |
| SLOT_01 | 1 · 0.2585 | 0 · — | 0 | 0 |
| SLOT_02 | 1 · 0.2404 | 0 · — | 0 | 0 |
| SLOT_03 | 0 · — | 1 · 0.3506 | 0 | 1 |
| SLOT_04 | 0 · — | 1 · 0.3522 | 0 | 1 |
| SLOT_05 | 0 · — | 2 · 0.4872 | 0 | 0 |
| SLOT_06 | 0 · — | 0 · — | 0 | 0 |

## 시각화

히트맵은 PatchCore 예측 점수이며 정답 마스크가 아니다. 모든 그림은 검증 히트맵 픽셀의 99백분위수를 공통 vmax로 사용하고 vmin=0, inferno 색상표, 오버레이 alpha=0.45를 사용한다.
- val/false_positive: 0장 (해당 사례 없음)
- val/false_negative: 0장 (해당 사례 없음)
- val/true_positive: 2장 — examples/val/true_positive/synthetic_defect_rgb_0000_SLOT_05.png, examples/val/true_positive/synthetic_defect_rgb_0001_SLOT_05.png
- test/false_positive: 0장 (해당 사례 없음)
- test/false_negative: 2장 — examples/test/false_negative/synthetic_defect_rgb_0003_SLOT_03.png, examples/test/false_negative/synthetic_defect_rgb_0003_SLOT_04.png
- test/true_positive: 2장 — examples/test/true_positive/synthetic_defect_rgb_0003_SLOT_05.png, examples/test/true_positive/synthetic_defect_rgb_0005_SLOT_05.png

## 해석 범위

검증에서 정한 한 임계값으로 최종 테스트를 평가했다. 픽셀 정확도 지표는 계산하지 않았다.
현재 이상 장면은 SLOT_03·04=yellow, SLOT_05=brown으로 고정되어 있어 슬롯 위치·배경과 색상 차이를 분리해 평가할 수 없다.
같은 원본 장면의 여러 ROI는 서로 독립 표본이 아니다. 아래 하위 그룹은 건수가 작아 비율보다 건수를 우선해 읽어야 한다.
이번 실행은 `--limit-per-label` 표본 검사이며 전체 데이터셋 성능 추정이 아니다.
