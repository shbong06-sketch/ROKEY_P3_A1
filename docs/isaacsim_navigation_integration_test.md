# Isaac Sim 연동 Navigation Executor 통합 검증 절차

> 이 문서는 `mock_sim_task_executor + 실제 Navigation Executor` 조합을 검증한다. 실제 Standalone Sim Executor까지 함께 연결하는 시험은 [Standalone 연결 테스트](./standalone_task_manager_navigation_test.md)를 따른다.

## 1. 문서 목적

이 문서는 Isaac Sim이 없는 환경에서 다음 검증을 완료한 이후 수행하는 실환경 통합 검증 절차를 정의한다.

- `TaskCommand`, `TaskResult`, `ExecutorStatus` 커스텀 메시지 빌드
- `/navigation/command`, `/navigation/result`, `/navigation/status` 타입 확인
- Navigation Executor heartbeat 확인
- 잘못된 명령 및 목적지에 대한 실패 응답 확인
- Task Manager와 Mock Executor 기반 상태 전이 확인

이번 검증의 핵심 목표는 다음과 같다.

> Task Manager가 발행한 `NAVIGATION` 명령을 실제 Navigation Executor가 받아 Isaac Sim의 Nova Carter를 이동시키고, 도착 결과를 반환하여 Task Manager가 다음 공정으로 전이하는지 확인한다.

이번 단계에서는 다음 구성으로 시험한다.

```text
task_manager
├── mock_sim_task
├── real_navigation
└── mock_inspection

Isaac Sim
└── Nova Carter
    ├── /chassis/odom 발행
    └── /cmd_vel 구독
```

---

## 2. 검증 대상

### 2.1 ROS 2 노드

| 노드 | 실제/Mock | 역할 |
|---|---|---|
| `task_manager` | 실제 | 전체 시나리오 순서 및 timeout 관리 |
| `mock_sim_task_executor` | Mock | `TRANSFER`, `PICK_HARVEST`, `PLACE_INSPECT`, `CULL`, `CONVEYOR_OUT` 처리 |
| `navigation_node` | 실제 | Navigation 명령 수신 및 주행 프로세스 실행 |
| `mock_inspection_executor` | Mock | `INSPECT` 처리 |
| `path_runner_smooth` | 실제 | `/chassis/odom` 기반 경로 추종 및 `/cmd_vel` 발행 |

### 2.2 주요 인터페이스

| 인터페이스 | 타입 | 방향 |
|---|---|---|
| `/start_cycle` | `smart_farm_interfaces/srv/StartCycle` | 사용자 → Task Manager |
| `/cycle/status` | `smart_farm_interfaces/msg/CycleStatus` | Task Manager → 외부 |
| `/navigation/command` | `smart_farm_interfaces/msg/TaskCommand` | Task Manager → Navigation |
| `/navigation/status` | `smart_farm_interfaces/msg/ExecutorStatus` | Navigation → Task Manager |
| `/navigation/result` | `smart_farm_interfaces/msg/TaskResult` | Navigation → Task Manager |
| `/chassis/odom` | `nav_msgs/msg/Odometry` | Isaac Sim → Path Runner |
| `/cmd_vel` | `geometry_msgs/msg/Twist` | Path Runner → Isaac Sim |

---

## 3. 사전 조건

### 3.1 코드 및 빌드 조건

- `feature/system-integration` 또는 동일한 통합 브랜치에 다음 패키지가 존재해야 한다.

```text
cobot3_ws/src/
├── smart_farm_interfaces
├── smart_farm_manager
└── smart_farm_navigation
```

- `navigation_node.py`는 JSON `String`이 아니라 다음 메시지를 사용해야 한다.
  - `TaskCommand`
  - `TaskResult`
  - `ExecutorStatus`
- Navigation heartbeat는 0.5초 주기로 발행되어야 한다.
- Navigation 내부 timeout은 Task Manager의 120초보다 짧아야 한다. 권장값은 110초다.
- 다음 혼합 통합 launch가 준비되어 있어야 한다.

```text
smart_farm_manager/launch/task_manager_navigation_test.launch.py
```

### 3.2 Isaac Sim Scene 조건

Isaac Sim Scene에는 최소한 다음 구성이 필요하다.

- Nova Carter가 Physics Scene 안에 배치되어 있음
- Nova Carter articulation 및 wheel joint가 정상 구성됨
- ROS 2 Bridge extension 활성화
- Differential Drive Action Graph 활성화
- `/cmd_vel`을 구독하는 ROS 2 Subscribe Twist 노드 존재
- `/chassis/odom`을 발행하는 Odometry/TF Action Graph 존재
- Timeline의 **Play** 상태에서 물리 시뮬레이션이 진행됨

> 주의: `path_runner_smooth.yaml`의 waypoint 설명은 `Collected_smartfarm_v003` 기준일 수 있다. 실제 시험 Scene이 v004이거나 Nova Carter의 시작 위치·방향이 변경되었다면 인터페이스가 정상이어도 잘못된 방향으로 이동할 수 있다.

### 3.3 실행 환경 조건

- 모든 ROS 2 터미널에서 같은 `ROS_DOMAIN_ID`를 사용해야 한다.
- Isaac Sim ROS 2 Bridge와 외부 ROS 2 환경이 모두 Jazzy 기준이어야 한다.
- Fast DDS를 사용한다면 모든 프로세스에서 `RMW_IMPLEMENTATION`을 동일하게 설정한다.

환경 확인 예시:

```bash
echo "$ROS_DISTRO"
echo "$ROS_DOMAIN_ID"
echo "$RMW_IMPLEMENTATION"
```

---

## 4. 패키지 빌드

워크스페이스에서 실행한다.

```bash
cd ~/ROKEY_P3_A1/cobot3_ws

source /opt/ros/jazzy/setup.bash

colcon build \
  --symlink-install \
  --packages-select \
  smart_farm_interfaces \
  smart_farm_navigation \
  smart_farm_manager

source install/setup.bash
```

빌드 실패가 없는지 확인한다.

```bash
colcon test-result --verbose
```

---

## 5. Isaac Sim 준비

### 5.1 Scene 실행

1. 통합 검증에 사용할 스마트팜 USD Scene을 연다.
2. ROS 2 Bridge extension이 활성화되어 있는지 확인한다.
3. Nova Carter의 다음 Action Graph를 확인한다.
   - Differential Drive
   - ROS 2 Subscribe Twist
   - Odometry/TF Publisher
4. `/cmd_vel` 및 `/chassis/odom` 토픽명이 코드와 일치하는지 확인한다.
5. Timeline의 **Play** 버튼을 누른다.
6. Nova Carter가 초기 위치에서 튀거나 기울거나 바닥을 관통하지 않는지 확인한다.

### 5.2 Isaac Sim 연결 확인

새 터미널에서 실행한다.

```bash
source /opt/ros/jazzy/setup.bash
source ~/ROKEY_P3_A1/cobot3_ws/install/setup.bash
```

토픽 존재 여부를 확인한다.

```bash
ros2 topic list | grep -E '^/chassis/odom$|^/cmd_vel$'
```

기대 결과:

```text
/chassis/odom
/cmd_vel
```

Odometry 발행 주기를 확인한다.

```bash
ros2 topic hz /chassis/odom
```

판정 기준:

- 지속적으로 주기가 출력되어야 한다.
- 경고 없이 Odometry 메시지가 계속 수신되어야 한다.
- Timeline을 Pause하면 발행이 멈추거나 시각 갱신이 중단될 수 있다.

`/cmd_vel` 연결 상태를 확인한다.

```bash
ros2 topic info /cmd_vel --verbose
```

판정 기준:

- 시험 시작 전 publisher는 0개일 수 있다.
- Isaac Sim 측 subscriber는 최소 1개 존재해야 한다.
- 실제 주행이 시작되면 `path_runner_smooth` publisher가 추가되어야 한다.

이 단계에서 `/chassis/odom` publisher 또는 `/cmd_vel` subscriber가 없다면 통합 launch를 시작하지 말고 Action Graph부터 수정한다.

---

## 6. 혼합 통합 시스템 실행

### 6.1 Launch 실행

별도 터미널에서 실행한다.

```bash
source /opt/ros/jazzy/setup.bash
source ~/ROKEY_P3_A1/cobot3_ws/install/setup.bash

ros2 launch smart_farm_manager \
  task_manager_navigation_test.launch.py
```

기대 노드:

```bash
ros2 node list
```

최소한 다음 노드가 보여야 한다.

```text
/task_manager
/mock_sim_task_executor
/navigation_node
/mock_inspection_executor
```

`path_runner_smooth`는 아직 표시되지 않는 것이 정상이다. `NAVIGATION` 명령이 도착했을 때 자식 프로세스로 실행된다.

### 6.2 PREFLIGHT 전 상태 확인

Navigation heartbeat를 확인한다.

```bash
ros2 topic echo /navigation/status
```

기대 상태:

```yaml
executor: navigation
state: READY
task_id: ''
command_id: ''
operation: NAVIGATION
phase: IDLE
```

약 0.5초마다 계속 발행되어야 한다.

토픽 타입을 다시 확인한다.

```bash
ros2 topic type /navigation/command
ros2 topic type /navigation/status
ros2 topic type /navigation/result
```

기대 결과:

```text
smart_farm_interfaces/msg/TaskCommand
smart_farm_interfaces/msg/ExecutorStatus
smart_farm_interfaces/msg/TaskResult
```

---

## 7. 관찰 터미널 구성

검증 중에는 최소 두 개의 관찰 터미널을 두는 것을 권장한다.

### 관찰 터미널 A: 전체 사이클

```bash
source /opt/ros/jazzy/setup.bash
source ~/ROKEY_P3_A1/cobot3_ws/install/setup.bash

ros2 topic echo /cycle/status
```

### 관찰 터미널 B: Navigation 인터페이스

명령 확인:

```bash
ros2 topic echo /navigation/command
```

필요하면 별도 터미널에서 결과를 확인한다.

```bash
ros2 topic echo /navigation/result
```

상태를 확인한다.

```bash
ros2 topic echo /navigation/status
```

### 선택 사항: rosbag 기록

재현이 어려운 오류를 분석하려면 다음 토픽을 기록한다.

```bash
ros2 bag record \
  /cycle/status \
  /navigation/command \
  /navigation/status \
  /navigation/result \
  /chassis/odom \
  /cmd_vel
```

---

## 8. 정상 시나리오 검증

### 8.1 사이클 시작

```bash
ros2 service call \
  /start_cycle \
  smart_farm_interfaces/srv/StartCycle \
  "{scenario_id: 'DEMO_HARVEST_01'}"
```

기대 응답:

```yaml
accepted: true
task_id: TASK-...
reason: NONE
```

### 8.2 예상 실행 순서

```text
PREFLIGHT
→ TRANSFER             mock_sim_task
→ PICK_HARVEST         mock_sim_task
→ NAVIGATION           real_navigation
→ PLACE_INSPECT        mock_sim_task
→ INSPECT              mock_inspection
→ CULL 또는 Skip       mock_sim_task
→ CONVEYOR_OUT         mock_sim_task
→ COMPLETE
```

### 8.3 Navigation 명령 확인

`/navigation/command`에서 다음 값을 확인한다.

```yaml
task_id: TASK-...
command_id: TASK-...-CMD-003
operation: NAVIGATION
destination: INSPECTION_DOCK
```

`command_id`의 순번은 시나리오 변경에 따라 달라질 수 있으므로 고정값으로 판단하지 않는다.

### 8.4 Navigation 실행 상태 확인

명령 수신 후 `/navigation/status`는 다음처럼 변해야 한다.

```yaml
executor: navigation
state: EXECUTING
task_id: TASK-...
command_id: TASK-...-CMD-...
operation: NAVIGATION
phase: DRIVING
detail: INSPECTION_DOCK
```

동시에 다음을 확인한다.

```bash
ros2 node list | grep path_runner
ros2 topic hz /cmd_vel
```

기대 결과:

- `/path_runner_smooth` 노드가 생성된다.
- `/cmd_vel`이 지속적으로 발행된다.
- Nova Carter가 waypoint 경로를 따라 이동한다.
- `/chassis/odom` 위치가 실제 이동 방향에 맞게 변한다.

### 8.5 실제 도착 확인

Nova Carter가 정지했을 때 다음을 확인한다.

- 검사 작업점 또는 설정된 `INSPECTION_DOCK` 위치에 도착함
- 차체가 최종 목표 방향으로 정렬됨
- 정지 후 `/cmd_vel`의 선속도와 각속도가 0으로 돌아옴
- 랙, 컨베이어, 벽 또는 다른 로봇과 충돌하지 않음
- 리프트 및 M0609가 주행 중 불안정하게 흔들리지 않음

### 8.6 Navigation 결과 확인

정상 결과 예시는 다음과 같다.

```yaml
task_id: TASK-...
command_id: TASK-...-CMD-...
operation: NAVIGATION
status: SUCCEEDED
phase: ARRIVED
reason: NONE
safe_to_navigate: false
reached_station: INSPECTION_DOCK
completed_units: []
defect_slots: []
unknown_slots: []
```

필수 확인 항목:

- 결과의 `task_id`가 명령과 동일함
- 결과의 `command_id`가 명령과 동일함
- `operation= NAVIGATION`
- `status=SUCCEEDED`
- `reached_station=INSPECTION_DOCK`

### 8.7 Task Manager 후속 전이 확인

Navigation 성공 결과 이후 `/cycle/status`가 다음 상태로 넘어가야 한다.

```text
NAVIGATION
→ PLACE_INSPECT
```

Mock Executor가 남은 작업을 처리한 뒤 최종 상태는 다음이어야 한다.

```yaml
state: COMPLETE
status: SUCCEEDED
reason: NONE
```

> 현재 Task Manager에는 외부 `/reset_cycle` service가 없을 수 있다. `COMPLETE` 또는 `ERROR` 후 다시 시험하려면 혼합 통합 launch를 재시작한다.

---

## 9. 성공 판정 체크리스트

다음 항목을 모두 만족해야 1단계 실제 통합 검증을 통과한 것으로 본다.

- [ ] Isaac Sim Timeline이 Play 상태다.
- [ ] `/chassis/odom`이 지속적으로 발행된다.
- [ ] `/cmd_vel`에 Isaac Sim subscriber가 존재한다.
- [ ] Navigation Executor가 0.5초 주기로 `READY` heartbeat를 발행한다.
- [ ] Task Manager PREFLIGHT가 통과한다.
- [ ] Task Manager가 `NAVIGATION` 명령을 정확한 메시지 타입으로 발행한다.
- [ ] Navigation Executor가 `EXECUTING/DRIVING` 상태로 전환된다.
- [ ] `path_runner_smooth`가 자식 프로세스로 실행된다.
- [ ] `/cmd_vel`이 발행되고 Nova Carter가 실제로 이동한다.
- [ ] `/chassis/odom` 위치가 이동에 따라 변한다.
- [ ] Nova Carter가 `INSPECTION_DOCK`에 허용 오차 내로 도착한다.
- [ ] 정지 후 `/cmd_vel`이 0으로 돌아온다.
- [ ] Navigation 결과가 원래 `task_id`, `command_id`를 유지한다.
- [ ] 결과가 `SUCCEEDED/ARRIVED/NONE`이다.
- [ ] `reached_station=INSPECTION_DOCK`이다.
- [ ] Task Manager가 `PLACE_INSPECT`로 전이한다.
- [ ] 최종 Mock 혼합 사이클이 `COMPLETE/SUCCEEDED`로 끝난다.

---

## 10. 장애 상황별 확인 방법

### 10.1 `PREFLIGHT failed: NAV_NOT_READY`

확인 순서:

```bash
ros2 node list | grep navigation
ros2 topic type /navigation/status
ros2 topic hz /navigation/status
ros2 topic echo /navigation/status --once
```

가능한 원인:

- Navigation Node가 실행되지 않음
- `ExecutorStatus`가 아닌 이전 `std_msgs/String` 빌드가 source됨
- heartbeat timer가 실행되지 않음
- `message.executor` 값이 `navigation`이 아님
- 서로 다른 `ROS_DOMAIN_ID` 사용
- 이전 빌드 환경이 source됨

조치:

```bash
source /opt/ros/jazzy/setup.bash
source ~/ROKEY_P3_A1/cobot3_ws/install/setup.bash
```

필요하면 관련 패키지를 다시 빌드한다.

### 10.2 Navigation 명령은 수신했지만 AMR이 움직이지 않음

확인 순서:

```bash
ros2 node list | grep path_runner
ros2 topic hz /cmd_vel
ros2 topic info /cmd_vel --verbose
ros2 topic hz /chassis/odom
```

가능한 원인:

- Isaac Sim Timeline이 Pause 또는 Stop 상태
- ROS 2 Subscribe Twist Action Graph 비활성화
- `/cmd_vel` 토픽 이름 불일치
- Differential Drive의 wheel joint 설정 오류
- `path_runner_smooth`가 Odometry를 기다리는 중
- `/chassis/odom` 미발행 또는 stale 상태

### 10.3 `path_runner_smooth`가 생성되지 않음

Navigation Node 로그의 실행 명령을 확인한다.

```text
ros2 launch smart_farm_navigation ...
```

확인 항목:

- `destinations.yaml`에 `INSPECTION_DOCK` 존재
- 지정된 launch 파일 존재
- 지정된 params 파일 존재
- `smart_farm_navigation` 패키지가 source됨

직접 실행해 원인을 분리한다.

```bash
ros2 launch smart_farm_navigation \
  path_smooth.launch.py \
  auto_start:=true
```

### 10.4 AMR이 반대 방향 또는 잘못된 위치로 이동함

인터페이스 문제가 아니라 좌표 및 Scene 기준 문제일 가능성이 높다.

확인 항목:

- 현재 Scene의 Nova Carter 시작 위치
- 현재 Scene의 Nova Carter 시작 yaw
- `drive_direction_sign`
- `waypoints_x`, `waypoints_y`
- `final_heading_deg`
- v003 기준 waypoint를 v004 Scene에 그대로 사용했는지 여부

먼저 시작 pose를 기록한 뒤 waypoint를 재측정한다.

### 10.5 Navigation은 도착했지만 Task Manager가 다음 단계로 가지 않음

`/navigation/result`에서 다음 값을 비교한다.

```text
task_id
command_id
operation
reached_station
```

Task Manager가 요구하는 값:

```text
operation = NAVIGATION
status = SUCCEEDED
reached_station = INSPECTION_DOCK
```

특히 `reached_station`이 비어 있으면 `DOCKING_ERROR`가 발생한다.

### 10.6 `RESULT_TIMEOUT`

확인 항목:

- Navigation 내부 timeout이 110초인지 확인
- Task Manager Navigation timeout이 120초인지 확인
- 자식 launch 프로세스가 완료 후 실제로 종료되는지 확인
- `path_runner_smooth`가 `COMPLETE` 후 종료되는지 확인
- Odometry 대기로 무한정 `WAITING`에 머무는지 확인

정상적인 timeout 순서:

```text
Navigation 내부 timeout
→ TIMEOUT TaskResult 발행
→ Task Manager가 결과 처리
```

Task Manager가 먼저 timeout 처리하는 구조가 되면 안 된다.

### 10.7 주행 종료 후 AMR이 계속 움직임

즉시 Isaac Sim을 Pause하고 다음을 확인한다.

```bash
ros2 topic echo /cmd_vel
```

확인 항목:

- Path Runner가 완료 시 zero Twist를 발행하는지
- 종료 처리 중 자식 프로세스가 강제 종료되었는지
- 다른 `/cmd_vel` publisher가 동시에 존재하는지

```bash
ros2 topic info /cmd_vel --verbose
```

publisher가 둘 이상이면 동시에 제어하는 노드를 찾아 중지한다.

---

## 11. 추가 권장 실패 검증

정상 시나리오 성공 후 다음 시험을 별도로 수행한다.

### 11.1 주행 중 Odometry 중단

1. 정상적으로 Navigation을 시작한다.
2. 주행 중 Odometry Action Graph를 비활성화하거나 Timeline을 Pause한다.
3. Path Runner가 stale Odometry를 감지하는지 확인한다.
4. Navigation이 `FAILED/NAV_FAILED` 또는 설정된 실패 결과를 반환하는지 확인한다.
5. AMR 정지 명령이 발행되는지 확인한다.

### 11.2 경로 이탈 또는 정체

1. 이동 경로에 충돌 물체를 배치하거나 Carter를 움직이지 못하게 한다.
2. `stall_timeout_s` 이후 Path Runner가 중단되는지 확인한다.
3. Navigation 결과가 `FAILED/NAV_FAILED`인지 확인한다.
4. Task Manager가 `ERROR`로 전이하는지 확인한다.

### 11.3 Navigation Node 종료

1. `NAVIGATION` 실행 중 Navigation Node를 종료한다.
2. Task Manager가 결과를 받지 못한 상태에서 command timeout으로 전이하는지 확인한다.
3. 자식 Path Runner와 `/cmd_vel` publisher가 남지 않는지 확인한다.

---

## 12. 시험 결과 기록 양식

| 항목 | 기록값 |
|---|---|
| 시험 일시 |  |
| Git 브랜치 |  |
| Git commit SHA |  |
| 사용 Scene |  |
| Isaac Sim 버전 | 5.1.0 |
| ROS 2 배포판 | Jazzy |
| 시작 위치 `(x, y, yaw)` |  |
| 목표 위치 `(x, y, yaw)` |  |
| 실제 도착 위치 `(x, y, yaw)` |  |
| 위치 오차 |  |
| 방향 오차 |  |
| Navigation 소요 시간 |  |
| 최종 Cycle 상태 |  |
| 충돌 발생 여부 |  |
| 특이 사항 |  |

### 최종 판정

```text
[ ] PASS
[ ] FAIL
```

실패한 경우 다음 정보를 함께 보관한다.

- Task Manager 로그
- Navigation Node 로그
- Path Runner 로그
- `/navigation/command` 메시지
- `/navigation/result` 메시지
- `/cycle/status` 마지막 메시지
- rosbag 경로
- 실패 시점 Isaac Sim 화면 캡처

---

## 13. 검증 종료 절차

1. `/cmd_vel`이 0인지 확인한다.
2. 혼합 통합 launch를 `Ctrl+C`로 종료한다.
3. 자식 Path Runner가 남아 있지 않은지 확인한다.

```bash
ros2 node list | grep -E 'navigation|path_runner'
```

4. rosbag을 기록했다면 recorder를 종료한다.
5. Isaac Sim Timeline을 Pause한다.
6. 다음 시험을 수행할 경우 Task Manager launch를 다시 시작한다.

---

## 14. 이번 단계에서 수정하지 않는 범위

다음 항목은 Navigation 계약 연결 검증과 분리한다.

- Task Manager FSM 재설계
- 전체 실제 `sim_task_executor` 연결
- 실제 Inspection Executor 연결
- Lift 제어 통합
- M0609 Pick/Place 통합
- 경로 최적화 알고리즘 변경
- Navigation waypoint 성능 최적화

현재 단계의 완료 기준은 **실제 Navigation Executor 한 개가 Task Manager 계약에 따라 명령을 받고, Isaac Sim AMR을 움직인 뒤, 올바른 결과를 반환하는 것**이다.
