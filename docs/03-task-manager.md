# Task Manager 통합 및 테스트 가이드

이 문서는 현재 `feature/system-integration` 브랜치의 실제 구현을 기준으로 Task Manager를 mock executor 또는 실제 executor에 연결하고 검증하는 절차를 설명한다.

관련 설계는 [시스템 아키텍처](./01-architecture.md)와 [인터페이스 설계](./02-interfaces.md)를 함께 참고한다.

실제 Isaac Sim Standalone Sim Executor와 Navigation Executor를 함께 연결하는 절차는 [Standalone 연결 테스트](./standalone_task_manager_navigation_test.md)를 따른다.

## 1. 현재 통합 범위

현재 Task Manager는 다음 ROS 2 인터페이스를 제공한다.

| 구분 | 이름 | 타입 |
| --- | --- | --- |
| 사이클 시작 | `/start_cycle` | `smart_farm_interfaces/srv/StartCycle` |
| 사이클 상태 | `/cycle/status` | `smart_farm_interfaces/msg/CycleStatus` |
| Sim 명령/결과/상태 | `/sim_task/command`, `/sim_task/result`, `/sim_task/status` | `std_msgs/msg/String` JSON |
| 주행 명령/결과/상태 | `/navigation/command`, `/navigation/result`, `/navigation/status` | `TaskCommand`, `TaskResult`, `ExecutorStatus` |
| 검사 명령/결과/상태 | `/inspection/command`, `/inspection/result`, `/inspection/status` | `TaskCommand`, `TaskResult`, `ExecutorStatus` |

Task Manager는 실제 로봇이나 Nav2 API를 직접 호출하지 않는다. 각 executor는 자신의 `command`를 구독하고, 주기적인 `status`와 명령당 하나의 terminal `result`를 발행해야
한다.

현재 제공되는 `mock_executor` 하나가 `executor` 파라미터에 따라 `sim_task`, `navigation`, `inspection` 역할 중 하나를 수행한다. 따라서 실제 executor를 붙일 때는
해당 역할의 mock만 제외하고 동일한 토픽 계약으로 실제 노드를 실행하면 된다.

## 2. 사전 준비와 빌드

저장소 루트에서 브랜치와 변경사항을 먼저 확인한다.

```bash
cd ~/ROKEY_P3_A1
git branch --show-current
git status --short
```

ROS 2 Jazzy 환경을 불러오고 인터페이스와 Task Manager를 빌드한다.

```bash
cd ~/ROKEY_P3_A1/cobot3_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install \
  --packages-select smart_farm_interfaces smart_farm_manager
source install/setup.bash
```

새 터미널을 열 때마다 ROS 2와 workspace 환경을 다시 불러와야 한다.

```bash
source /opt/ros/jazzy/setup.bash
source ~/ROKEY_P3_A1/cobot3_ws/install/setup.bash
```

빌드 후 패키지와 인터페이스가 보이는지 확인한다.

```bash
ros2 pkg prefix smart_farm_manager
ros2 interface show smart_farm_interfaces/srv/StartCycle
ros2 interface show smart_farm_interfaces/msg/TaskCommand
ros2 interface show smart_farm_interfaces/msg/TaskResult
ros2 interface show smart_farm_interfaces/msg/ExecutorStatus
ros2 interface show std_msgs/msg/String
```

## 3. 전체 mock으로 한 사이클 검증

### 3.1 노드 실행

터미널 1에서 Task Manager와 세 mock executor를 한 번에 실행한다.

```bash
ros2 launch smart_farm_manager task_manager_fake_test.launch.py
```

이 launch는 다음 네 노드를 실행한다.

- `/task_manager`
- `/mock_sim_task_executor`
- `/mock_navigation_executor`
- `/mock_inspection_executor`

### 3.2 연결 상태 확인

터미널 2에서 노드, 서비스, 토픽을 확인한다.

```bash
ros2 node list
ros2 service type /start_cycle
ros2 topic list | sort
```

세 status 토픽에서 heartbeat가 들어오는지 각각 한 건씩 확인한다.

```bash
ros2 topic echo /sim_task/status \
  std_msgs/msg/String --once
ros2 topic echo /navigation/status \
  smart_farm_interfaces/msg/ExecutorStatus --once
ros2 topic echo /inspection/status \
  smart_farm_interfaces/msg/ExecutorStatus --once
```

사이클 시작 전에는 각 메시지가 다음 조건을 만족해야 한다.

- `executor`가 토픽 역할과 정확히 일치한다.
- `state`가 `READY`이다.
- heartbeat 간격이 `executor_status_timeout_sec`보다 짧다.
- 기본 mock 주기는 0.5초이고 Task Manager의 freshness 제한은 3초이다.

### 3.3 상태 관찰과 사이클 시작

터미널 2에서 사이클 상태를 계속 관찰한다.

```bash
ros2 topic echo /cycle/status smart_farm_interfaces/msg/CycleStatus
```

터미널 3에서 사이클을 시작한다.

```bash
ros2 service call /start_cycle \
  smart_farm_interfaces/srv/StartCycle \
  "{scenario_id: DEMO_HARVEST_01}"
```

서비스의 `accepted: true`는 요청이 수락됐다는 뜻이며 전체 사이클 성공을 의미하지
않는다. 최종 성공은 `/cycle/status`에서 확인한다.

기본 mock에서는 다음 순서가 나타나야 한다.

```text
PREFLIGHT
→ TRANSFER
→ PICK_HARVEST
→ NAVIGATION
→ PLACE_INSPECT
→ INSPECT
→ CULL
→ CONVEYOR_OUT
→ COMPLETE / SUCCEEDED
```

기본 검사 mock은 `SLOT_03`, `SLOT_06`을 불량으로 반환하므로 CULL이 실행된다.
Task Manager 로그에서도 `PREFLIGHT complete`, 각 `Command published`, 마지막
`Cycle completed`를 확인한다.

## 4. 메시지 흐름 상세 확인

문제가 생겼을 때는 별도 터미널에서 담당 토픽을 직접 관찰한다.

```bash
ros2 topic echo /sim_task/command std_msgs/msg/String
ros2 topic echo /sim_task/result std_msgs/msg/String
ros2 topic echo /navigation/command smart_farm_interfaces/msg/TaskCommand
ros2 topic echo /navigation/result smart_farm_interfaces/msg/TaskResult
ros2 topic echo /inspection/command smart_farm_interfaces/msg/TaskCommand
ros2 topic echo /inspection/result smart_farm_interfaces/msg/TaskResult
```

결과를 검증할 때는 다음 규칙을 적용한다.

1. `task_id`, `command_id`, `operation`은 수신한 명령 값을 그대로 반환해야 한다.
2. Task Manager는 현재 active command와 세 값이 모두 일치하는 결과만 처리한다.
3. `status`는 `SUCCEEDED`, `FAILED`, `CANCELED`, `TIMEOUT` 중 terminal 상태로 보낸다.
4. 성공 결과에는 operation별 필수 증거가 포함돼야 한다.

| operation | 성공 결과에서 확인할 필드 |
| --- | --- |
| `TRANSFER` | `completed_units`에 `PALLET_002:RACK_L3:RACK_L2`, `PALLET_003:RACK_L4:RACK_L3` 포함 |
| `PICK_HARVEST` | `safe_to_navigate: true` |
| `NAVIGATION` | `reached_station: INSPECTION_DOCK` |
| `PLACE_INSPECT` | `status: SUCCEEDED` |
| `INSPECT` | 유효한 `defect_slots`, 비어 있는 `unknown_slots` |
| `CULL` | 요청한 모든 `target_slots`가 `completed_units`에 포함 |
| `CONVEYOR_OUT` | `status: SUCCEEDED` |

## 5. 실제 executor를 하나씩 연결하는 방법

`task_manager_fake_test.launch.py`는 세 mock을 모두 실행하므로 부분 통합 때는 사용하지
않는다. Task Manager와 필요한 mock을 각각 실행하고, 교체 대상만 실제 노드로 띄운다.

### 5.1 Task Manager 단독 실행

```bash
ros2 launch smart_farm_manager task_manager.launch.py
```

### 5.2 역할별 mock 수동 실행

아래 명령 중 실제 executor로 교체하지 않은 역할만 실행한다.

```bash
ros2 run smart_farm_manager mock_executor \
  --ros-args -r __node:=mock_sim_task_executor \
  -p executor:=sim_task

ros2 run smart_farm_manager mock_executor \
  --ros-args -r __node:=mock_navigation_executor \
  -p executor:=navigation

ros2 run smart_farm_manager mock_executor \
  --ros-args -r __node:=mock_inspection_executor \
  -p executor:=inspection
```

권장 교체 순서는 다음과 같다.

1. Task Manager + 전체 mock
2. 실제 Navigation Executor + Sim/Inspection mock
3. 실제 Sim Task Executor + Navigation/Inspection mock
4. 실제 Inspection Executor + Sim/Navigation mock
5. 세 실제 executor 전체 연결

예를 들어 Navigation Executor를 처음 통합할 때는 Task Manager, sim task mock,
inspection mock, 실제 Navigation Executor만 실행한다. 실제 노드의 실행 명령과 launch
파일은 해당 패키지에서 구현된 뒤 이 문서에 추가한다.

## 6. 실제 executor의 최소 연결 계약

각 executor를 연결하기 전에 아래 항목을 만족해야 한다.

### 6.1 공통 조건

- 자신에게 해당하는 `/.../command`를 구독한다.
- 자신의 `/.../result`와 `/.../status`를 발행한다.
- QoS는 command/result/status 모두 현재 구현과 호환되는 `RELIABLE`, `VOLATILE`,
  `KEEP_LAST`, depth 10을 사용한다. 현재 Python 노드는 `10`으로 지정해 이 기본 QoS를
  사용한다.
- 시작 준비가 끝난 뒤 1~2Hz로 `READY` heartbeat를 발행한다.
- 작업 중에도 heartbeat를 멈추지 않고 `BUSY` 또는 구현 상태값을 발행한다.
- status의 `executor`는 담당 이름과 정확히 일치해야 한다. Sim Task는 JSON의
  `executor`, Navigation·Inspection은 `ExecutorStatus.executor`를 사용한다.
- 한 번에 명령 하나만 실행하며, 완료한 `command_id`를 재수신해도 물리 동작을
  반복하지 않는다.
- 명령 수신 콜백에서 장시간 블로킹하지 않는다.

QoS 연결 결과는 다음 명령으로 확인한다.

```bash
ros2 topic info /navigation/status --verbose
ros2 topic info /navigation/command --verbose
ros2 topic info /navigation/result --verbose
```

`Publisher count`, `Subscription count`와 양쪽 endpoint의 reliability/durability를 함께
확인한다. 다른 executor도 토픽 prefix만 바꿔 같은 방법으로 검사한다. Sim Task의 세
토픽 타입은 `std_msgs/msg/String`, 나머지 executor는 해당 사용자 정의 메시지여야 한다.

### 6.2 Navigation Executor

- `operation: NAVIGATION`과 `destination: INSPECTION_DOCK`을 처리한다.
- Nav2 action server가 준비된 뒤 `READY`가 되어야 한다.
- Nav2 성공과 최종 정지/도킹을 확인한 뒤 `reached_station: INSPECTION_DOCK`을 반환한다.
- 실패, cancel, timeout을 `TaskResult.status`와 `reason`으로 변환한다.

### 6.3 Sim Task Executor

- command/result/status 토픽은 모두 `std_msgs/msg/String`이며 `String.data`를 JSON object로 파싱한다.
- command의 `task_id`, `command_id`, `operation`과 result의 `status`는 필수 문자열이다.
- 잘못된 JSON, 필수 필드 누락 및 타입 불일치 명령은 물리 동작을 시작하지 않는다.
- `TRANSFER`, `PICK_HARVEST`, `PLACE_INSPECT`, `CULL`, `CONVEYOR_OUT`을 처리한다.
- Isaac Sim 장면과 제어기가 준비된 뒤에만 `READY`를 보낸다.
- ROS 콜백에서는 명령을 저장하고, 물리 동작은 Standalone update loop에서 진행한다.
- 실패 시 `phase`, `reason`, 완료된 `completed_units`를 가능한 범위에서 기록한다.

### 6.4 Inspection Executor

- `operation: INSPECT`와 `pallet_id: PALLET_001`을 처리한다.
- 요청과 이미지 frame을 같은 검사 세션에 연결한다.
- `SLOT_01`부터 `SLOT_06` 범위의 `defect_slots`, `unknown_slots`를 반환한다.
- 미검출이나 신뢰도 부족을 임의로 PASS 처리하지 않는다.

## 7. 단계별 통합 체크포인트

| 체크포인트 | 실행 조합 | 통과 조건 |
| --- | --- | --- |
| CP-1 | Task Manager + mock 3개 | PREFLIGHT부터 COMPLETE까지 한 사이클 성공 |
| CP-2 | 실제 Navigation + mock 2개 | Nav2 goal 전송, 도착/정지 확인, 다음 PLACE 명령 발생 |
| CP-3 | 실제 Sim Task + mock Navigation/Inspection | 다섯 Sim operation의 명령·결과 및 물리 성공 확인 |
| CP-4 | 실제 Inspection + mock Sim/Navigation | 슬롯별 결과가 CULL target으로 그대로 전달됨 |
| CP-5 | 실제 executor 3개 | 전체 순서, 안전 조건, timeout, 최종 결과 확인 |

각 체크포인트에서는 최소한 다음 자료를 남긴다.

- launch 및 executor 로그
- `/cycle/status` 출력
- 세 executor의 command/result/status 출력 또는 rosbag
- Isaac Sim이 포함된 경우 시연 영상과 실패 시점의 장면 상태

필요하면 핵심 통합 토픽을 rosbag으로 기록한다.

```bash
ros2 bag record \
  /cycle/status \
  /sim_task/command /sim_task/result /sim_task/status \
  /navigation/command /navigation/result /navigation/status \
  /inspection/command /inspection/result /inspection/status
```

## 8. 실패 및 경계 조건 검증

### 8.1 PREFLIGHT 실패

Task Manager만 실행하거나 mock 하나를 제외한 상태에서 `/start_cycle`을 호출한다.

```bash
ros2 launch smart_farm_manager task_manager.launch.py
```

10초 뒤 `/cycle/status`가 `ERROR`로 전이되고, 로그에 준비되지 않은 executor 전체의
`received`, `state`, `age`, `ready`, `stale`, `detail`이 출력돼야 한다. 최종 `reason`은
현재 우선순위상 첫 미준비 executor의 `SIM_NOT_READY`, `NAV_NOT_READY`,
`INSPECTION_NOT_READY` 중 하나이다.

### 8.2 stale heartbeat

5.2절처럼 mock을 각각 별도 터미널에서 실행한다. 세 executor의 READY를 확인하고
사이클을 시작하기 전에 특정 mock을 그 터미널에서 `Ctrl-C`로 종료한다. 그 뒤
`executor_status_timeout_sec`인 3초 이상 기다리고 `/start_cycle`을 호출해 해당
executor가 `stale=true`로 진단되는지 확인한다.

주의: 현재 heartbeat freshness 검사는 PREFLIGHT에서만 수행한다. 명령 수행 중
heartbeat 손실을 별도 실패로 전환하는 기능은 아직 구현되지 않았으며 현재는 명령별
result timeout이 최종 안전망이다.

### 8.3 잘못된 result와 오래된 result

통합 executor 시험에서는 현재 command의 ID 중 하나를 의도적으로 다르게 하여 결과를
발행한다. Task Manager가 `MISMATCHED_RESULT` 로그를 남기고 현재 단계에 머무는지
확인한다. 이전 명령의 결과를 다시 발행한 경우에도 다음 단계가 바뀌면 안 된다.

수동 발행 시 먼저 command 토픽에서 현재 ID를 확인한다. 현재 active executor의 result
토픽에 아래 값을 시험 목적에 맞게 수정해 발행한다. 다음 예시는 NAVIGATION 명령이
활성 상태일 때 사용한다.

```bash
ros2 topic pub --once /navigation/result \
  smart_farm_interfaces/msg/TaskResult \
  "{task_id: WRONG_TASK, command_id: WRONG_COMMAND, operation: NAVIGATION,
    status: SUCCEEDED, phase: RESULT, reason: NONE,
    safe_to_navigate: false, reached_station: INSPECTION_DOCK,
    completed_units: [], defect_slots: [], unknown_slots: []}"
```

### 8.4 잘못된 Sim Task JSON

아래처럼 JSON 구문 오류가 있는 결과를 발행한다.

```bash
ros2 topic pub --once /sim_task/result std_msgs/msg/String \
  "{data: '{invalid-json'}"
```

Task Manager가 `Invalid sim_task result JSON ignored` 경고를 남기고 현재 active command를
유지해야 한다. `task_id` 같은 필수 필드 누락, 문자열 필드의 숫자 입력, 배열 필드에
문자열을 넣은 경우도 같은 방식으로 무시돼야 한다.

### 8.5 executor 실패

실제 executor 또는 시험용 publisher가 현재 ID와 operation은 그대로 유지하고
`status: FAILED`와 구체적인 `reason`을 반환하게 한다. Task Manager가 다음 명령을
발행하지 않아야 한다. 물리 operation 실패는 `RESET_REQUIRED`, 그 외 실패는
`FAILED`로 종료되는지 확인한다.

### 8.6 결과 timeout

executor가 command를 수신하되 result를 발행하지 않도록 만든다. 현재 timeout은
`scenario.py`의 단계별 `timeout_sec` 값이며 YAML 파라미터가 아니다. 짧은 시험값이
필요하면 별도 테스트 브랜치에서만 값을 낮추고 시험 후 복구한다.

### 8.7 CULL 생략

Inspection mock을 다음처럼 수동 실행하면 불량 슬롯이 없는 결과를 만들 수 있다.

```bash
ros2 run smart_farm_manager mock_executor \
  --ros-args -r __node:=mock_inspection_executor \
  -p executor:=inspection -p defect_slots:="[]"
```

`INSPECT` 다음에 `/sim_task/command`로 `CULL`이 발행되지 않고 바로
`CONVEYOR_OUT`이 발행되는지 확인한다. `/sim_task/command`의 `String.data` JSON에서
`operation` 값을 확인한다.

## 9. 자동 테스트

순수 Python 상태 머신 테스트는 Isaac Sim과 ROS graph 없이 실행할 수 있다.

```bash
cd ~/ROKEY_P3_A1/cobot3_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
python3 -m pytest \
  src/smart_farm_manager/test/test_state_machine.py -q
```

패키지 전체 테스트는 다음과 같이 실행한다.

```bash
colcon test --packages-select smart_farm_manager \
  --event-handlers console_direct+
colcon test-result --verbose
```

전체 테스트에는 기능 테스트 외에 copyright, flake8, pydocstyle 검사도 포함된다. 실패
시 `test_state_machine.py`의 기능 실패와 스타일 검사 실패를 구분해서 확인한다.

현재 ROS graph 전체를 자동으로 기동하는 `launch_testing` 테스트는 없다. CP-1 절차는
현재 수동 통합 시험이며, 반복 검증이 필요해지면 `/start_cycle` 호출과 terminal
`/cycle/status` 대기를 자동화하는 launch test를 추가한다.

## 10. 파라미터와 현재 제약

Task Manager 파라미터는
`cobot3_ws/src/smart_farm_manager/config/demo_harvest.yaml`에 있다.

| 파라미터 | 현재 값 | 의미 |
| --- | --- | --- |
| `use_sim_time` | `false` | 현재 mock 시험은 wall time 사용 |
| `tick_period_sec` | `0.1` | 상태 머신 처리 주기 |
| `preflight_timeout_sec` | `10.0` | READY heartbeat 대기 제한 |
| `executor_status_timeout_sec` | `3.0` | 마지막 status freshness 제한 |

실제 Isaac Sim과 연결할 때는 모든 관련 노드의 `use_sim_time` 정책을 함께 결정해야
한다. Task Manager의 timeout 자체는 simulation pause에 묶이지 않도록 monotonic wall
time을 사용한다.

현재 구현의 제약은 다음과 같다.

- 한 프로세스에서 완료 또는 실패 후 새 사이클로 돌아가는 ROS reset 인터페이스가
  없다. 반복 시험 전에는 Task Manager를 재시작한다.
- 자동 retry는 구현되어 있지 않다.
- 실행 중 heartbeat loss 감시는 아직 없다.
- 단계 timeout은 `scenario.py`에 정의돼 있고 런타임 파라미터가 아니다.
- mock은 정상 결과 중심이며 실패 주입 파라미터는 제공하지 않는다.
- mock은 작업 중 status에 `EXECUTING`을 사용한다. PREFLIGHT는 `READY`만 요구하며,
  Task Manager의 단계 전이는 status가 아니라 result를 기준으로 한다.

## 11. 통합 문제 확인 순서

사이클이 진행되지 않으면 다음 순서로 확인한다.

1. `ros2 node list`에서 Task Manager와 필요한 executor가 실행 중인지 본다.
2. `ros2 topic info <topic> --verbose`로 publisher/subscriber 수와 QoS를 확인한다.
3. 세 status의 `executor`, `state`, heartbeat 주기와 freshness를 확인한다.
4. `/cycle/status`의 `state`, `active_command_id`, `status`, `reason`을 확인한다.
5. 해당 단계 command가 발행됐는지 확인한다.
6. result의 `task_id`, `command_id`, `operation`이 command와 같은지 확인한다.
7. operation별 성공 필드가 채워졌는지 확인한다.
8. 실패 후에는 다음 단계 명령이 발행되지 않았는지 확인한다.
9. 물리 동작이 일부 수행된 실패라면 재시도하지 말고 Isaac Sim 장면과 Task Manager를
   함께 초기화한다.

