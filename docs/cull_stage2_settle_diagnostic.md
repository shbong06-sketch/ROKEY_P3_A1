# Cull 2단계 Pallet·Romaine 초기 안정성 진단

## 1. 목적

이 문서는 고정 좌표 기반 Cull 단독 시험 중 관찰된 다음 현상의 원인을 좁히기 위한 1차 실험 결과를 기록한다.

1. 시뮬레이션 시작 직후 `Pallet_Inspect`에서 Romaine 하나가 이탈한다.
2. Pick 후 Lift 과정에서 Pallet이 끌려가며 위치가 틀어진다.

이번 실험은 첫 번째 현상만 다룬다. 로봇 모션, 마찰, 질량 및 초기 pose를 변경하지 않고 Pallet과 모든 Romaine의 자유 물리 거동을 측정했다.

---

## 2. 시험 환경

| 항목 | 값 |
|---|---|
| 시험일 | 2026-09-23 |
| 브랜치 | `feature/cull-motion` |
| 기준 커밋 | `567a73c` |
| Scene | `Collected_smartfarm_v013.usd` |
| Physics 주기 | 60 Hz |
| 측정 시간 | 300 physics steps, 5.00초 |
| 로봇 명령 | 없음 |
| 마찰 오버라이드 | 없음 |
| 질량 변경 | 없음 |
| Romaine 질량 | 각각 1.0 kg |
| Pallet 질량 | 1.0 kg |

대상 경로:

```text
/World/SmartFarm/Placed/Pallet_Inspect/Cube_011_001
/World/SmartFarm/Placed/Pallet_Inspect/root_001/Romaine_01
/World/SmartFarm/Placed/Pallet_Inspect/root_001/Romaine_02
/World/SmartFarm/Placed/Pallet_Inspect/root_001/Romaine_03
/World/SmartFarm/Placed/Pallet_Inspect/root_001/Romaine_04
/World/SmartFarm/Placed/Pallet_Inspect/root_001/Romaine_05
/World/SmartFarm/Placed/Pallet_Inspect/root_001/Romaine_06
```

---

## 3. 구현 내용

`cull_standalone.py`에 다음 진단 옵션을 추가했다.

```text
--diagnose-settle
--diagnose-steps N
```

진단 모드에서는 다음 동작을 수행한다.

1. v013 Scene을 연다.
2. RMPFlow 및 그리퍼 명령을 보내지 않는다.
3. Pallet과 Romaine 6개의 시작 world pose를 저장한다.
4. 설정한 physics step만큼 자유 물리 시뮬레이션을 진행한다.
5. 60 step마다 시작 위치 대비 이동 거리와 Z 변위를 출력한다.
6. 최종 XYZ 변위, 총 이동 거리 및 회전량을 출력한다.

실행 명령:

```bash
cd /home/rokey/ROKEY_P3_A1

PYTHONUNBUFFERED=1 /home/rokey/isaacsim/python.sh \
  cobot3_ws/isaacpjt/smart_farm/scripts/cull_standalone.py \
  --headless \
  --diagnose-settle \
  --diagnose-steps 300
```

---

## 4. 측정 결과

### 4.1 시간별 총 이동 거리

단위는 mm이며 괄호 안은 시작 위치 대비 Z 변위다.

| 대상 | 1초, 60 step | 2초, 120 step | 3초, 180 step | 4초, 240 step | 5초, 300 step |
|---|---:|---:|---:|---:|---:|
| Pallet | 0.1 (+0.0) | 0.5 (-0.0) | 0.9 (-0.0) | 1.1 (-0.0) | 1.2 (-0.0) |
| Romaine_01 | 0.4 (+0.1) | 0.8 (+0.0) | 1.3 (-0.2) | 1.1 (-0.2) | 1.2 (-0.1) |
| Romaine_02 | **146.6 (+77.0)** | **236.1 (-18.4)** | **236.1 (-18.4)** | **236.1 (-18.4)** | **236.1 (-18.4)** |
| Romaine_03 | 0.5 (+0.1) | 0.6 (+0.5) | 1.3 (+0.7) | 1.5 (+0.6) | 1.6 (+0.7) |
| Romaine_04 | 0.6 (-0.0) | 0.5 (-0.2) | 0.5 (-0.0) | 0.6 (-0.0) | 0.6 (-0.1) |
| Romaine_05 | 0.5 (+0.5) | 0.5 (+0.5) | 0.6 (+0.1) | 1.0 (+0.0) | 1.0 (+0.0) |
| Romaine_06 | 0.1 (+0.0) | 0.3 (-0.0) | 0.6 (+0.2) | 0.6 (-0.1) | 0.8 (-0.1) |

### 4.2 최종 pose 변화

| 대상 | ΔX (mm) | ΔY (mm) | ΔZ (mm) | 총 이동 (mm) | 회전 변화 (deg) |
|---|---:|---:|---:|---:|---:|
| Pallet | 1.14 | 0.25 | -0.00 | 1.17 | 0.110 |
| Romaine_01 | 1.01 | 0.60 | -0.14 | 1.18 | 0.209 |
| Romaine_02 | **54.55** | **228.98** | **-18.44** | **236.11** | **101.497** |
| Romaine_03 | 1.35 | 0.48 | 0.68 | 1.59 | 0.375 |
| Romaine_04 | 0.47 | 0.43 | -0.08 | 0.64 | 0.476 |
| Romaine_05 | 1.00 | -0.00 | 0.01 | 1.00 | 0.320 |
| Romaine_06 | 0.82 | -0.08 | -0.09 | 0.82 | 0.512 |

---

## 5. 판정

조기 이탈 대상은 `Romaine_02`로 특정됐다.

- 1초 안에 Z 방향으로 77.0 mm 상승하면서 총 146.6 mm 이동했다.
- 2초 시점에는 총 236.1 mm 이동했고 이후 같은 위치에 정착했다.
- 최종 회전 변화는 101.497°다.
- 같은 조건에서 다른 Romaine의 최종 이동은 최대 1.59 mm다.
- Pallet의 최종 이동은 1.17 mm다.

따라서 현재 결과는 Pallet 전체의 마찰 부족보다는 `Romaine_02`에 한정된 초기 배치 또는 충돌 형상 문제를 우선 의심하게 한다. 특히 시작 직후 위로 튀어 오르는 거동은 초기 collider 겹침이 PhysX에서 해소되는 과정일 가능성이 있다.

단, 이번 실험에서는 접촉점과 penetration 값을 직접 수집하지 않았으므로 collider 겹침을 원인으로 확정하지 않는다.

Pick/Lift 중 Pallet 끌림 문제는 로봇을 움직이지 않은 이번 실험의 검증 범위가 아니다.

---

## 6. 정적 검증

다음 검사를 통과했다.

```text
python3 -m py_compile: PASS
test_cull_motion.py: 6 passed
git diff --check: PASS
Isaac Sim 진단 프로세스 종료 코드: 0
```

---

## 7. 다음 단위 실험

다음 실험에서는 물리값을 변경하기 전에 `Romaine_02`만 조사한다.

1. `Romaine_02`와 Pallet 지지부의 초기 world AABB를 비교한다.
2. `Romaine_02`의 세 collision mesh와 Pallet collider의 초기 겹침 여부를 확인한다.
3. 안정적인 `Romaine_01` 또는 `Romaine_03`과 local transform 및 collider 크기를 비교한다.
4. 초기 pose 또는 collider 문제를 수정한 뒤 동일한 300-step 진단을 반복한다.
5. 시작 안정성이 확보된 이후에만 Pick/Lift 중 Pallet 끌림 진단으로 넘어간다.

마찰, 질량, Fixed Joint를 동시에 변경하면 원인을 분리할 수 없으므로 다음 실험에서도 한 항목만 변경한다.

---

## 8. 후속 실험 — Romaine_02 구조 비교

### 8.1 비교 목적

최초 실험에서 `Romaine_02`만 이탈했으므로 다음 가능성을 비교했다.

- `Romaine_02`만 다른 물리 속성을 갖는가
- `Romaine_02` collision mesh만 다른가
- 두 번째 Pallet 슬롯만 다른 AABB 겹침을 갖는가
- 이탈 결과가 반복 실행에서도 같은가

이 비교에서도 Scene, 마찰, 질량은 변경하지 않았다.

### 8.2 Rigid Body 속성 비교

`Romaine_01~06`은 모두 다음 속성이 동일했다.

| 항목 | 확인 결과 |
|---|---|
| Rigid Body | 활성 |
| Kinematic | `False` |
| 질량 | 1.0 kg |
| 초기 선속도 | `(0, 0, 0)` |
| 초기 각속도 | `(0, 0, 0)` |
| 초기 회전 | `(0, 0, 0)` |
| Scale | `(1, 1, 1)` |
| Collider 수 | 각각 3개 |

각 Romaine의 차이는 Pallet 슬롯에 맞춘 local translation뿐이다.

### 8.3 Collision mesh 비교

각 Romaine의 다음 collision mesh 데이터 해시를 비교했다.

```text
collision_seat: c1a15769ddd06a97
collision_base: 11bb4d6897c7feca
collision_body: 5f8850755a497535
```

여섯 Romaine에서 세 해시가 모두 동일했다. 따라서 `Romaine_02`만 다른 collision mesh를 사용하는 것은 아니다.

World AABB 크기도 각 슬롯에서 동일했다.

| Collider | World AABB 크기 (mm) |
|---|---:|
| `collision_seat` | 66.36 × 39.82 × 12.00 |
| `collision_base` | 57.40 × 34.44 × 27.40 |
| `collision_body` | 91.03 × 54.62 × 108.14 |

### 8.4 Pallet collider와의 AABB 비교

충돌 mesh의 purpose가 `guide`이므로 AABB 계산 시 `guide` purpose를 포함해야 했다. 이를 포함해 다시 계산한 결과, 모든 Romaine은 자신의 Pallet 구멍 collider와 동일하게 24개의 AABB 겹침 후보를 가졌다.

대표적인 `collision_seat` AABB 겹침 크기는 다음과 같으며 여섯 슬롯에서 같았다.

```text
18.83 × 11.30 × 11.70 mm
33.18 ×  5.00 × 11.70 mm
 8.33 × 19.91 × 11.70 mm
```

AABB는 mesh의 빈 공간과 실제 삼각형 표면을 구분하지 못하므로, 이 값은 실제 penetration 깊이가 아니다. 다만 `Romaine_02`만 다른 AABB 겹침을 갖는다는 가설은 지지하지 않는다.

---

## 9. 반복 실행 재현성

원본 위치에서 같은 300-step 진단을 추가로 3회 실행했다.

세 실행 모두 결과가 같았다.

| 대상 | 총 이동 (mm) | 회전 변화 (deg) |
|---|---:|---:|
| Pallet | 1.17 | 0.110 |
| Romaine_01 | 1.18 | 0.209 |
| Romaine_02 | **236.11** | **101.497** |
| Romaine_03 | 1.59 | 0.375 |
| Romaine_04 | 0.64 | 0.476 |
| Romaine_05 | 1.00 | 0.320 |
| Romaine_06 | 0.82 | 0.512 |

`Romaine_02` 이탈은 일회성 랜덤 현상이 아니라 현재 조건에서 반복 재현되는 현상이다.

---

## 10. 위치 교환 대조 실험

### 10.1 방법

원본 USD를 수정하지 않고 session layer에서 다음 local translation만 교환했다.

```text
Romaine_01: (-0.125, -0.189, 0.0136397)
             → (0.125, -0.189, 0.0136397)

Romaine_02: (0.125, -0.189, 0.0136397)
             → (-0.125, -0.189, 0.0136397)
```

실행 명령:

```bash
cd /home/rokey/ROKEY_P3_A1

PYTHONUNBUFFERED=1 /home/rokey/isaacsim/python.sh \
  cobot3_ws/isaacpjt/smart_farm/scripts/cull_standalone.py \
  --headless \
  --diagnose-settle \
  --diagnose-steps 300 \
  --diagnose-swap-01-02
```

### 10.2 결과

| 대상 | ΔX (mm) | ΔY (mm) | ΔZ (mm) | 총 이동 (mm) | 회전 변화 (deg) |
|---|---:|---:|---:|---:|---:|
| Pallet | 0.18 | 0.02 | -0.01 | 0.18 | 0.045 |
| Romaine_01 | 0.17 | 0.53 | -0.06 | 0.56 | 0.502 |
| Romaine_02 | 0.39 | 0.55 | -0.27 | 0.73 | 0.348 |
| Romaine_03 | 0.24 | 0.24 | 0.01 | 0.34 | 0.173 |
| Romaine_04 | -0.15 | -0.42 | 1.12 | 1.21 | 1.062 |
| Romaine_05 | 0.45 | -0.23 | -0.04 | 0.51 | 0.112 |
| Romaine_06 | 0.14 | -0.09 | 0.14 | 0.22 | 0.100 |

위치 교환 후에는 `Romaine_01`과 `Romaine_02` 모두 이탈하지 않았다. 전체 대상 중 최대 이동도 1.21 mm였다.

### 10.3 판정

현재 결과만으로는 다음 두 단순 가설을 확정할 수 없다.

- `Romaine_02` 강체 자체가 항상 불량이다.
- 두 번째 Pallet 슬롯에 놓인 물체는 항상 이탈한다.

두 물체의 물리 속성과 collision mesh가 동일한데 위치 교환만으로 현상이 사라졌으므로, 원래 `Romaine_02`와 두 번째 슬롯 조합의 초기 접촉 해석이 solver 순서 또는 미세한 접촉 조건에 민감한 상태일 가능성이 높다.

위치 교환은 원인을 해결한 것이 아니라 대조 실험이므로 제품 동작에 그대로 사용하지 않는다.

---

## 11. 갱신된 다음 실험

다음 단위 실험은 마찰 변경이 아니라 초기 접촉 조건 보정으로 진행한다.

1. 원래 위치의 `Romaine_02`에 작은 Z 여유만 세션에서 적용한다.
2. 다른 Romaine, 질량, 마찰은 변경하지 않는다.
3. Z 여유별 300-step 안정성을 반복 비교한다.
4. 가장 작은 안정화 값이 5회 연속 재현되는지 확인한다.
5. 안정화 후 원본 USD 수정 여부를 별도로 결정한다.

후보 Z 여유는 2 mm부터 시작하고, 실패할 때만 5 mm, 10 mm 순서로 올린다. 이 시험이 완료되기 전에는 Fixed Joint나 마찰 변경을 함께 적용하지 않는다.

---

## 12. Pick Z offset 20 mm 기준 파지 시험

### 12.1 조건

- `CullPickConfig.pick_z_offset`: `0.02 m`
- 고정 입력 좌표: `(0.0009, 0.4192, 0.1457) m`
- 실제 `PICK_DESCEND` 기준 Z: `0.1657 m`
- 그리퍼 close 목표: `1.18 rad`
- 그리퍼 drive: stiffness `100,000`, damping `1,000`, max force `10,000`
- 마찰과 질량: 기존 USD 설정 유지

실행 명령:

```bash
cd /home/rokey/ROKEY_P3_A1

PYTHONUNBUFFERED=1 /home/rokey/isaacsim/python.sh \
  cobot3_ws/isaacpjt/smart_farm/scripts/cull_standalone.py \
  --headless --max-steps 6000
```

### 12.2 결과

- `OPEN → PICK_APPROACH → PICK_DESCEND → GRASP → LIFT → HOLD` 단계는 모두 완료됐다.
- GRASP 중 손가락 관절은 저항 없이 close 목표 `1.18 rad`에 도달했다.
- 대상 `Romaine_03` 최종 상승량은 `-0.4 mm`로, 성공 기준 `50 mm`에 미달했다.
- Pallet 이동량은 `9.23 mm`, `Romaine_03` 이동량은 `10.24 mm`였다.
- 다른 로메인도 약 `8.87~12.44 mm` 이동했다.
- 최종 판정은 `RuntimeError: 물리 Pick 실패`였다.

### 12.3 판정

현재 결과는 그리퍼 drive 최대 힘 부족으로 판정할 수 없다. GRASP에서 손가락이 목표 위치까지 완전히 닫혔는데도 대상이 상승하지 않았으므로, `pick_z_offset=0.02 m`에서 유효한 파지 접촉이 형성되지 않은 것이 선행 문제다.

파지력 수치 비교는 접촉이 성립하는 Z를 먼저 확보한 뒤 수행한다. 다음 시험에서는 마찰과 질량을 그대로 유지하고 `pick_z_offset`만 낮춰 접촉 여부를 확인한다. 접촉이 확인된 동일 Z에서만 `GRIPPER_DRIVE_MAX_FORCE`를 단계별로 비교한다.

---

## 13. Romaine 평면 파지 밴드와 균일 collider 적용

### 13.1 변경 대상

원본 자산을 직접 변경했다.

```text
cobot3_ws/isaacpjt/smart_farm/scenes/Collected_smartfarm_v013/
assets/palette_tray_with_romaine/romaine_pallet_6_v005_inspect.usd
```

- 변경 전 SHA-256: `15bacfccd537f3bee8342ef52d63a571188d56e38eba0403ef34ec42baf7ad08`
- 변경 후 SHA-256: `d6bd0b1f11b38f998a903aded63c3a686e0c976fb32a8f960152003a72cff157`
- 변경 전 백업: `/tmp/romaine_pallet_6_v005_inspect.before_grip_band.usd`

### 13.2 적용 구조

`Romaine_01~06`에 동일하게 다음 구조를 적용했다.

- 기존 `collision_body`: 보존하되 collision 비활성화
- `collision_lower_body`: 기존 body 하부를 잘라 만든 convex hull
- `collision_grip`: `80 x 50 x 30 mm` 단순 Box collider
- `grip_band`: `collision_grip`과 같은 위치에 배치한 평평한 시각 형상
- 기존 Romaine별 시각 재질과 기존 PhysicsMaterial 바인딩 유지
- 질량, 마찰, Pallet collider는 변경하지 않음

하부 collider 상단과 grip box 사이에는 의도적으로 간격을 두었다. 하부 collider를 grip box까지 확장하면 `Romaine_02` 조기 이탈이 다시 발생했기 때문이다.

### 13.3 무개입 정착 검증

300 physics step, 5초 정착 결과:

| 대상 | 이동량 (mm) |
|---|---:|
| Pallet | 0.31 |
| Romaine_01 | 0.46 |
| Romaine_02 | 0.51 |
| Romaine_03 | 0.40 |
| Romaine_04 | 0.55 |
| Romaine_05 | 0.44 |
| Romaine_06 | 0.44 |

모든 대상이 0.55 mm 이내였으며 기존 `Romaine_02` 조기 이탈은 발생하지 않았다.

### 13.4 `pick_z_offset=0.02 m` Pick 검증

- `OPEN → PICK_APPROACH → PICK_DESCEND → GRASP → LIFT → HOLD` 완료
- 대상 상승량: `-0.1 mm`
- Pallet 이동량: `0.65 mm`
- `Romaine_03` 이동량: `0.70 mm`
- 비대상 Romaine 최대 이동량: `0.90 mm`
- 최종 판정: 물리 Pick 실패

형상 변경 후 주변 물체를 밀던 현상은 사라졌지만 접촉 파지는 형성되지 않았다. 실행 시 `PICK_DESCEND` TCP는 world Z 약 `0.9657 m`, 손가락 prim은 약 `0.988 m`였고 grip band 상단은 약 `0.960 m`였다. 따라서 다음 시험은 collider나 파지력을 다시 바꾸기 전에 `pick_z_offset`을 낮춰 손가락 패드와 grip band 높이를 맞추는 순서로 진행한다.
