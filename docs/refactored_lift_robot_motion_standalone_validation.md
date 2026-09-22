# 리팩터링 Lift·Robot Motion 단독 검증 가이드

## 1. 문서 목적

이 문서는 `refactor/lift-robot-motion` 브랜치에서 리팩터링한 다음 모듈을 **시스템 통합 전에 Isaac Sim에서 독립 검증**하기 위한 절차다.

- `robot_motion.py`
- `lift.py`
- `pallet_transfer.py`
- `robot_motion_standalone.py`

이 검증의 대상은 **재배랙 내부 팔레트 재배치**다.

```text
리프트 높이 정렬
→ 베이스 안정화 확인
→ 랙에서 Pick
→ 인양·인출
→ 같은 재배랙의 목적 층에 Place
→ 포크 인출
```

다음 기능은 이 문서의 검증 범위가 아니다.

- 수확 팔레트의 `TRAVEL_STOW → PRE_PICK → CARRY_ROTATE`
- `/cmd_vel` 기반 AMR 주행
- 작업 모드/주행 모드 바퀴 브레이크 전환
- 검사대 또는 컨베이어처럼 다른 작업점에 Place
- ROS 2 `TaskCommand`·`TaskResult` 연결
- Task Manager 전체 사이클

위 기능은 `feature/integration-v1`의 임시 구현을 `feature/system-integration`으로 옮기는 별도 통합 작업에서 검증한다.

---

## 2. 검증 대상 구조

| 파일 | 단독 검증에서 확인할 책임 |
|---|---|
| `lift.py` | 리프트 보정, 승강, 유지, 정지, 행정·추종·시간 초과 검사 |
| `robot_motion.py` | IK 계획, Home, Pick, Place, 팔레트 인양·미끄러짐·안착 검사 |
| `pallet_transfer.py` | 재배랙 내부 `Lift Align → Pick → Place` 순차 실행 |
| `robot_motion_standalone.py` | Scene 로딩, 객체 등록, Play/Pause/Stop 및 작업 목록 실행 |

`PalletTransferController`의 정상 상태 전이는 다음과 같다.

```text
IDLE
→ LIFT_ALIGN
→ WAIT_BASE_SETTLE
→ PICKING
→ PLACING
→ SUCCEEDED
```

실패 시에는 다음 상태로 전이해야 한다.

```text
실행 상태
→ FAILED
→ RobotMotion.cancel()
→ LiftController.stop()
```

---

## 3. 사전 조건

### 3.1 브랜치 확인

```bash
cd ~/ROKEY_P3_A1

git fetch origin --prune
git switch refactor/lift-robot-motion
git pull --ff-only origin refactor/lift-robot-motion

git branch --show-current
git status --short
```

기대 결과:

```text
refactor/lift-robot-motion
```

`git status --short`에는 의도하지 않은 변경이 없어야 한다.

### 3.2 대상 파일 확인

```bash
cd ~/ROKEY_P3_A1

test -f cobot3_ws/isaacpjt/smart_farm/scripts/robot_motion.py
test -f cobot3_ws/isaacpjt/smart_farm/scripts/lift.py
test -f cobot3_ws/isaacpjt/smart_farm/scripts/pallet_transfer.py
test -f cobot3_ws/isaacpjt/smart_farm/scripts/robot_motion_standalone.py
```

### 3.3 Scene과 M0609 파일 확인

현재 standalone은 다음 Scene을 사용한다.

```text
/home/rokey/Collected_smartfarm_v004/Collected_smartfarm_v004.usd
```

실행 PC에서 확인한다.

```bash
test -f \
  /home/rokey/Collected_smartfarm_v004/Collected_smartfarm_v004.usd
```

M0609 파일도 확인한다.

```bash
cd ~/ROKEY_P3_A1

test -f cobot3_ws/isaacpjt/M0609/doosan-robot2/urdf/m0609_isaac_sim.urdf
test -f cobot3_ws/isaacpjt/M0609/descriptor/m0609_description.yaml
```

Scene 경로가 다른 경우 검증 PC의 실제 경로에 맞춰 `SCENE_PATH`만 변경한다. 이때 Prim 경로나 waypoint 수치는 함께 변경하지 않는다.

---

## 4. 정적 검증

Isaac Sim을 실행하기 전에 문법과 잔여 API를 확인한다.

```bash
cd ~/ROKEY_P3_A1

python3 -m py_compile \
  cobot3_ws/isaacpjt/smart_farm/scripts/robot_motion.py \
  cobot3_ws/isaacpjt/smart_farm/scripts/lift.py \
  cobot3_ws/isaacpjt/smart_farm/scripts/pallet_transfer.py \
  cobot3_ws/isaacpjt/smart_farm/scripts/robot_motion_standalone.py
```

```bash
git diff --check
```

이전 API가 실행 경로에 남아 있는지 확인한다.

```bash
rg -n 'from lift import Lift\b|\.move_to\(|\.done\b' \
  cobot3_ws/isaacpjt/smart_farm/scripts
```

판정:

- `LiftController`, `start_move()`, `is_done`을 사용하는 현재 코드만 존재해야 한다.
- 과거 시험용 파일에서만 검색된다면 실제 standalone import 경로와 분리되어 있는지 확인한다.
- `robot_motion.py`를 읽는 것만으로 `SimulationApp(...)`이 생성되면 안 된다.

확인 명령:

```bash
rg -n 'SimulationApp|app\.is_running|app\.close' \
  cobot3_ws/isaacpjt/smart_farm/scripts/robot_motion.py \
  cobot3_ws/isaacpjt/smart_farm/scripts/robot_motion_standalone.py
```

`SimulationApp`과 실행 루프는 `robot_motion_standalone.py`에서만 검색되어야 한다.

---

## 5. 테스트 작업 설정 확인

`robot_motion_standalone.py`의 `TASKS`가 실제 Scene의 팔레트와 목적 층을 가리키는지 확인한다.

기본 예시:

```python
TASKS = [
    Task(
        "/World/SmartFarm/Placed/Pallet_1/Asset",
        SHELF_TOP[2],
    ),
]
```

주의사항:

1. `pallet_path`는 빈 상위 Xform이 아니라 실제 Rigid Body인 `Asset`을 가리켜야 한다.
2. 목적 슬롯이 비어 있어야 한다.
3. 팔레트 이동 순서는 빈 슬롯을 먼저 사용하는 순서여야 한다.
4. 처음에는 한 작업만 검증한다.
5. 한 작업 성공 후에 두 작업 연속 검증으로 확장한다.

---

## 6. 실행 방법

Isaac Sim 설치 경로를 환경에 맞게 지정한다.

```bash
export ISAAC_PATH="$HOME/isaacsim"
```

설치 위치가 다르면 실제 Isaac Sim 5.1.0 디렉터리를 사용한다.

```bash
cd ~/ROKEY_P3_A1

"$ISAAC_PATH/python.sh" \
  cobot3_ws/isaacpjt/smart_farm/scripts/robot_motion_standalone.py
```

창이 열린 뒤 Timeline의 **Play**를 누른다.

- Play: 실행 또는 Pause 지점에서 재개
- Pause: 현재 상태에서 일시정지
- Stop 후 Play: 처음부터 초기화하여 재실행

---

## 7. 시험 1 — Scene 초기화와 리프트 보정

### 관찰할 로그

```text
[장면] 안정될 때까지 120 스텝 기다립니다
[리프트] 기준 잡기: ...
[리프트] 만들 수 있는 베이스 높이 ...
```

### 확인 항목

- Scene이 정상적으로 열리는가
- 필요한 Prim을 모두 찾는가
- M0609 관절 인덱스를 정상적으로 읽는가
- 리프트 joint의 최소·최대 행정을 읽는가
- 물리 안정화 이후 `calibrate()`가 호출되는가
- 리프트 joint 값과 arm base 높이의 offset이 비정상적으로 변하지 않는가

### 실패 판정

- `Prim이 없습니다`
- `USD 관절을 찾지 못했습니다`
- `calibrate()를 먼저 부르세요`
- 시작 직후 리프트나 리그가 튀어 오름
- 보정 offset이 재실행마다 크게 달라짐

---

## 8. 시험 2 — 단일 재배랙 Transfer

한 개의 팔레트를 비어 있는 목적 층으로 이동한다.

### 기대 상태

```text
LIFT_ALIGN
→ WAIT_BASE_SETTLE
→ PICKING
→ PLACING
→ SUCCEEDED
```

### 기대 로봇 단계

Pick:

```text
HOME
→ ENTRY
→ READY
→ APPROACH
→ DOCK
→ PALLET_UP
→ RETRACT
```

Place:

```text
DESCEND
→ PLACE_IN
→ PALLET_DOWN
→ FORK_OUT
→ EXIT
```

### 물리 판정

- Lift가 목표 층에 맞춰 움직인 뒤 완전히 멈추는가
- Lift 이동 중 M0609가 움직이지 않는가
- M0609 동작 중 Lift 높이가 유지되는가
- READY와 APPROACH 사이에서 랙을 긁지 않는가
- DOCK 전에 팔레트를 밀지 않는가
- PALLET_UP에서 실제 팔레트 상승이 확인되는가
- RETRACT 중 팔레트가 포크에서 미끄러지지 않는가
- PALLET_DOWN에서 팔레트가 선반에 안착하는가
- FORK_OUT 중 팔레트가 포크를 따라 나오지 않는가
- 종료 후 `TransferState.SUCCEEDED`가 되는가

### 성공 로그의 핵심

```text
[인양 확인] 실제 상승량 ...
[안착 확인] 실제 하강량 ...
[DONE] 놓기·안착·포크 인출까지 확인했습니다.
[완료] 모든 작업을 마쳤습니다.
```

---

## 9. 시험 3 — Pause와 재개

각 구간에서 한 번씩 Pause 후 Play한다.

권장 시점:

1. `LIFT_ALIGN`
2. `APPROACH`
3. `RETRACT`
4. `PLACE_IN`
5. `FORK_OUT`

확인 항목:

- Pause 중 단계 index가 진행되지 않는가
- 재개 후 목표를 건너뛰지 않는가
- 재개 직후 추종 오차가 급증하지 않는가
- 팔레트 검증 기준이 초기화되지 않는가
- Lift와 Arm이 동시에 재개되지 않는가

---

## 10. 시험 4 — Stop 후 재시작

`PICKING` 또는 `PLACING` 중 Timeline을 Stop하고 다시 Play한다.

확인 항목:

- `transfer.cancel()`이 호출되는가
- M0609가 현재 위치에서 안전하게 정지하는가
- Lift가 현재 위치를 유지하는가
- 이전 `JointSequence`가 폐기되는가
- 이전 `_PalletTracker` 상태가 남지 않는가
- `task_index`가 0으로 돌아가는가
- 안정화 스텝 후 Lift가 다시 보정되는가
- 새 작업이 `LIFT_ALIGN`부터 시작하는가

주의: 이미 팔레트가 물리적으로 이동한 뒤 Stop했다면 Scene reset 후 팔레트가 원래 위치로 복원되는지도 함께 확인한다.

---

## 11. 시험 5 — 두 작업 연속 재배치

단일 작업 성공 후 실제 시나리오 순서로 두 작업을 등록한다.

빈 슬롯이 3번이라면 작업 순서는 다음이어야 한다.

```text
PALLET_002: RACK_L2 → RACK_L3
PALLET_001: RACK_L1 → RACK_L2
```

즉, 목적 슬롯을 먼저 비우는 순서로 진행한다.

`TASKS`에는 실제 Scene의 팔레트 Prim path와 슬롯 높이를 사용한다.

예시 형식:

```python
TASKS = [
    Task(PALLET_002_PRIM_PATH, SHELF_TOP[3]),
    Task(PALLET_001_PRIM_PATH, SHELF_TOP[2]),
]
```

위 상수 이름은 설명용이다. 실제 Prim path를 확인하지 않고 그대로 추가하지 않는다.

확인 항목:

- 첫 작업 종료 후 두 번째 작업이 자동 시작되는가
- 첫 작업의 `PalletTracker`가 두 번째 작업에 남지 않는가
- 두 번째 작업은 불필요하게 HOME으로 복귀하지 않는가
- `start_from_home=False` 경로에서 랙과 충돌하지 않는가
- 첫 팔레트의 최종 위치가 두 번째 작업 중 유지되는가
- 두 번째 작업 종료 후 최종 상태가 `SUCCEEDED`인가

---

## 12. 시험 6 — 안전 실패 검증

안전검사는 한 번에 하나씩 의도적으로 실패시킨다. 시험 후 설정을 반드시 원복한다.

### 12.1 잘못된 Pallet Prim

기대 결과:

```text
Prim이 없습니다: ...
```

### 12.2 목적 높이가 Lift 행정 밖

기대 결과:

```text
리프트 행정 밖입니다
```

요청 높이가 자동으로 clamp되어 작업을 계속하면 안 된다.

### 12.3 포크가 랙 내부인 상태에서 Lift 이동

기대 결과:

```text
포크가 아직 랙 안에 있습니다
```

### 12.4 도킹 위치 허용 범위 밖

기대 결과 예:

```text
팔레트와의 앞뒤 거리가 범위 밖입니다
팔레트와의 좌우 어긋남이 범위 밖입니다
차체가 팔레트를 정면으로 보고 있지 않습니다
```

### 12.5 도달할 수 없는 IK 목표

기대 결과:

```text
지금 도킹 위치에서는 목표에 닿지 않습니다
```

### 12.6 실패 후 상태

- `TransferState.FAILED`로 전이해야 한다.
- `RobotMotion.cancel()`이 호출되어야 한다.
- `LiftController.stop()`이 호출되어야 한다.
- 물리 동작이 계속 진행되면 안 된다.
- 오류 확인을 위해 World가 Pause 상태로 남아야 한다.

---

## 13. 시험 7 — 중복 실행과 API 보호

별도 시험 harness에서 동작 중 같은 객체에 다시 명령한다.

예상 동작:

```python
motion.start_pick(pallet)
motion.start_pick(pallet)
```

두 번째 호출은 다음 의미의 오류로 거절되어야 한다.

```text
팔 동작이 이미 실행 중입니다
```

Pick 없이 Place를 호출하는 경우:

```python
motion.start_place(target_height)
```

기대 결과:

```text
Place는 Pick이 끝난 CARRYING 상태에서만 시작할 수 있습니다
```

`PalletTransferController.start()`를 실행 중 다시 호출하는 경우도 `이미 실행 중`으로 거절되어야 한다.

---

## 14. 시험 결과 기록 양식

| 항목 | 결과 | 측정값·로그 | 비고 |
|---|---|---|---|
| Scene 및 Prim 확인 | PASS/FAIL |  |  |
| Lift calibration | PASS/FAIL | offset: |  |
| Lift 목표 도달 | PASS/FAIL | 오차 mm: |  |
| Base settle | PASS/FAIL | 시간 s: |  |
| HOME | PASS/FAIL | 최대 관절 오차 °: |  |
| Pick 접근 | PASS/FAIL |  | 충돌 여부 |
| Pallet lift | PASS/FAIL | 상승량 mm: |  |
| Retract | PASS/FAIL | slip mm: |  |
| Place insert | PASS/FAIL |  | 충돌 여부 |
| Pallet down | PASS/FAIL | 하강량 mm: |  |
| Fork out | PASS/FAIL | 팔레트 이동 mm: |  |
| Pause/Resume | PASS/FAIL |  |  |
| Stop/Play reset | PASS/FAIL |  |  |
| 연속 2작업 | PASS/FAIL |  |  |
| 안전 실패 처리 | PASS/FAIL | reason: |  |

---

## 15. 단독 검증 완료 조건

다음을 모두 만족하면 리팩터링 코드의 독립 검증을 완료한 것으로 본다.

- 네 Python 파일이 문법 검사를 통과한다.
- `robot_motion.py` import가 SimulationApp을 자동 생성하지 않는다.
- Lift 보정과 단독 승강이 정상이다.
- 단일 재배랙 Transfer가 `SUCCEEDED`로 끝난다.
- Pick과 Place 단계 순서가 리팩터링 전과 동일하다.
- Lift와 Arm이 동시에 움직이지 않는다.
- 인양, 미끄러짐, 안착 검사가 유지된다.
- Pause/Resume에서 단계와 검증 상태가 보존된다.
- Stop/Play에서 작업 상태와 팔레트 추적 상태가 초기화된다.
- 두 작업을 연속 실행할 수 있다.
- 안전조건 위반 시 `FAILED`로 전이하고 Arm과 Lift가 정지한다.
- Scene, Prim, waypoint 및 안전 임계값을 검증 편의를 위해 완화하지 않았다.

이 완료 조건은 **재배랙 내부 재배치 모듈의 완료 조건**이다. 수확 Pick, 운반 자세, 바퀴 모드 전환, Navigation, 검사대 Place 및 ROS 2 계약은 `feature/system-integration`에서 별도로 통합·검증한다.
