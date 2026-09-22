# Standalone Sim Executor + Task Manager + Navigation 연결 테스트

## 1. 목적과 현재 범위

실제 Isaac Sim Standalone Sim Task Executor를 Task Manager와 실제 Navigation
Executor에 연결해 다음 구간을 검증한다.

```text
TRANSFER
  Pallet_02: L3 -> L2
  Pallet_03: L4 -> L3
-> PICK_HARVEST
  Pallet_01: RACK_L1 Pick
-> CARRY_ROTATE
  Pick 완료 자세에서 joint_1만 +90 deg
-> LIFT_TO_TRAVEL
  팔 베이스 월드 높이 1.0388 m
-> NAVIGATION
  INSPECTION_DOCK 주행
```

현재 Standalone Sim Executor가 지원하는 operation은 `TRANSFER`와
`PICK_HARVEST`다. `PLACE_INSPECT`는 아직 구현되지 않았다.

이번 테스트의 판정 범위:

1. 필수: `PICK_HARVEST`가 `safe_to_navigate=true`로 끝나고 Task Manager가
   `NAVIGATION` 명령을 발행한다.
2. 선택: Nova Carter가 `INSPECTION_DOCK`까지 주행하고 Navigation 결과가
   `SUCCEEDED`로 반환된다.

Navigation 성공 뒤 발행되는 `PLACE_INSPECT`는 현재 Sim Executor에서
`INVALID_COMMAND`로 종료된다. 이는 알려진 다음 구현 항목이다.

> `task_manager_navigation_test.launch.py`는 `mock_sim_task_executor`를 함께
> 실행한다. 실제 Standalone과 명령을 중복 수신하므로 이번 테스트에서는 사용하지
> 않는다.

## 2. 실행 구성

| 구성요소 | 실행 형태 | 역할 |
| --- | --- | --- |
| `standalone_app.py` | 실제 | TRANSFER, PICK_HARVEST, 운반 준비 |
| `task_manager` | 실제 | 전체 시나리오와 명령 순서 관리 |
| `navigation_node` | 실제 | INSPECTION_DOCK 주행 |
| `mock_inspection_executor` | Mock | PREFLIGHT의 Inspection READY 충족 |

모든 프로세스는 같은 `ROS_DOMAIN_ID`와 `RMW_IMPLEMENTATION`을 사용해야 한다.

## 3. 빌드

```bash
cd ~/ROKEY_P3_A1/cobot3_ws
source /opt/ros/jazzy/setup.bash

colcon build --symlink-install \
  --packages-select \
  smart_farm_interfaces \
  smart_farm_manager \
  smart_farm_navigation

source install/setup.bash
```

새 ROS 터미널마다 다음 환경을 적용한다.

```bash
source /opt/ros/jazzy/setup.bash
source ~/ROKEY_P3_A1/cobot3_ws/install/setup.bash
```

## 4. 프로세스 실행 순서

### 4.1 실제 Standalone Sim Executor

터미널 1:

```bash
cd ~/ROKEY_P3_A1/cobot3_ws/isaacpjt/smart_farm
~/isaacsim/python.sh runtime/standalone_app.py --autoplay
```

`--autoplay`는 Scene을 열고 Timeline을 재생한 뒤 Task Manager 명령을 기다린다.
자체 TRANSFER 명령을 발행하는 `--demo`는 사용하지 않는다.

다음 로그가 나온 뒤 나머지 프로세스를 실행한다.

```text
[시작] Scene과 제어기 구성이 완료되었습니다.
[시작] ROS 2 노드 초기화가 완료되었습니다.
...
[대기] /sim_task/command ...
```

### 4.2 실제 Navigation Executor

터미널 2:

```bash
source /opt/ros/jazzy/setup.bash
source ~/ROKEY_P3_A1/cobot3_ws/install/setup.bash
ros2 launch smart_farm_navigation navigation_node.launch.py
```

### 4.3 Inspection Mock

터미널 3:

```bash
source /opt/ros/jazzy/setup.bash
source ~/ROKEY_P3_A1/cobot3_ws/install/setup.bash
ros2 run smart_farm_manager mock_executor \
  --ros-args \
  -r __node:=mock_inspection_executor \
  -p executor:=inspection
```

이번 범위에서 검사를 수행하지 않더라도 PREFLIGHT에는 세 executor의 READY가 모두
필요하므로 Inspection Mock을 실행해야 한다.

### 4.4 Task Manager

터미널 4:

```bash
source /opt/ros/jazzy/setup.bash
source ~/ROKEY_P3_A1/cobot3_ws/install/setup.bash
ros2 launch smart_farm_manager task_manager.launch.py
```

## 5. 시작 전 연결 확인

노드와 중복 실행 여부를 확인한다.

```bash
ros2 node list | sort
```

최소한 다음 노드가 하나씩 보여야 한다.

```text
/mock_inspection_executor
/navigation_node
/sim_task_executor
/task_manager
```

`/mock_sim_task_executor`가 보이면 중복 구성이므로 테스트를 시작하지 않고 해당
노드를 종료한다.

각 executor의 READY heartbeat를 한 건씩 확인한다.

```bash
ros2 topic echo /sim_task/status std_msgs/msg/String --once
ros2 topic echo /navigation/status \
  smart_farm_interfaces/msg/ExecutorStatus --once
ros2 topic echo /inspection/status \
  smart_farm_interfaces/msg/ExecutorStatus --once
```

확인 조건:

- 세 executor의 `state`가 `READY`다.
- status가 약 0.5초 주기로 계속 발행된다.
- Sim detail에 `TRANSFER/PICK_HARVEST physical profiles loaded`가 포함된다.

Navigation의 Isaac 연결도 확인한다.

```bash
ros2 topic hz /chassis/odom
ros2 topic info /cmd_vel --verbose
```

`/chassis/odom`이 지속적으로 수신되고 `/cmd_vel`에 Isaac Sim subscriber가 하나
이상 있어야 한다.

> 주의: Standalone 기본 Scene은 `Collected_smartfarm_v008`이지만 현재 `path_runner_smooth.yaml`의 경로 주석은 v004 기준이다. 실제 주행 전에 Nova Carter 시작 pose, 진행 방향과 waypoint가 v008 Scene에 맞는지 반드시 확인한다.

## 6. 관찰과 기록

```bash
ros2 topic echo /cycle/status \
  smart_farm_interfaces/msg/CycleStatus
```

```bash
ros2 topic echo /sim_task/command std_msgs/msg/String
ros2 topic echo /sim_task/result std_msgs/msg/String
```

```bash
ros2 topic echo /navigation/command \
  smart_farm_interfaces/msg/TaskCommand
ros2 topic echo /navigation/result \
  smart_farm_interfaces/msg/TaskResult
```

재현이 어려운 오류는 다음과 같이 기록한다.

```bash
ros2 bag record \
  /cycle/status \
  /sim_task/command /sim_task/status /sim_task/result \
  /navigation/command /navigation/status /navigation/result \
  /chassis/odom /cmd_vel
```

## 7. 사이클 시작

모든 executor의 READY를 확인한 뒤 한 번만 호출한다.

```bash
ros2 service call /start_cycle \
  smart_farm_interfaces/srv/StartCycle \
  "{scenario_id: 'DEMO_HARVEST_01'}"
```

`accepted: true`는 요청 수락을 뜻하며 물리 동작 전체 성공을 뜻하지 않는다.
`/cycle/status`와 각 result로 실제 결과를 판단한다.

## 8. 단계별 기대 결과

### 8.1 TRANSFER

```text
Pallet_02: 3단 -> 2단
Pallet_03: 4단 -> 3단
```

성공 결과의 `completed_units`에는 다음 값이 포함돼야 한다.

```text
PALLET_002:RACK_L3:RACK_L2
PALLET_003:RACK_L4:RACK_L3
```

### 8.2 PICK_HARVEST와 운반 준비

```text
Pallet_01 RACK_L1 Pick 및 인출
-> CARRY_ROTATE
-> LIFT_TO_TRAVEL
-> VERIFY_TRANSPORT_READY
```

Standalone 로그:

```text
[DONE] 집기·인양·인출까지 확인했습니다.
[CARRY_ROTATE] ...
[DONE] 운반 자세에 도달했습니다.
[리프트] ... -> 1.039 m
[리프트] 도착...
[브레이크] 카터 바퀴 ...개 해제
```

운반 자세는 Pick 완료 시점의 실제 관절값에서 `joint_1`만 `+90 deg`여야 한다.
나머지 다섯 관절은 Pick 완료값을 유지해야 한다. `1.0388 m`는 리프트 조인트 값이
아니라 M0609 팔 베이스의 월드 z 높이다.

`/sim_task/result`는 다음 조건을 만족해야 한다.

```json
{
  "operation": "PICK_HARVEST",
  "status": "SUCCEEDED",
  "safe_to_navigate": true
}
```

### 8.3 NAVIGATION 진입

`PICK_HARVEST` 성공 전에는 Navigation 명령이 나오면 안 된다. 성공 직후 다음 명령이
한 번 발행돼야 한다.

```yaml
operation: NAVIGATION
destination: INSPECTION_DOCK
```

여기까지 확인되면 현재 필수 연결 범위는 통과다.

### 8.4 실제 주행과 도착

Navigation 명령 뒤 다음을 확인한다.

- `/path_runner_smooth` 노드가 생성된다.
- `/cmd_vel`이 발행된다.
- 주행 중 팔과 리프트가 운반 자세를 유지한다.
- Pallet_01이 포크에서 미끄러지거나 이탈하지 않는다.
- 랙, 벽, 컨베이어와 충돌하지 않는다.
- 도착 후 `/cmd_vel`이 0으로 돌아간다.

정상 결과:

```yaml
operation: NAVIGATION
status: SUCCEEDED
reached_station: INSPECTION_DOCK
```

이후 Task Manager가 `PLACE_INSPECT`로 전이하는 것까지 확인한다. 현재 Standalone은
이 operation을 지원하지 않으므로 다음 결과는 예상된 종료다.

```text
operation=PLACE_INSPECT
status=FAILED
reason=INVALID_COMMAND
phase=COMMAND_DISPATCH
```

## 9. 실패 판정과 조치

| 증상 | 확인 사항 |
| --- | --- |
| PREFLIGHT 실패 | 세 executor의 READY와 동일한 ROS domain |
| Sim 작업이 두 번 실행됨 | `mock_sim_task_executor` 중복 실행 여부 |
| Pick 뒤 Navigation이 안 나옴 | `safe_to_navigate`와 task/command ID 일치 |
| CARRY_ROTATE 실패 | `joint_1 +90 deg`가 USD 관절 한계 안인지 |
| travel 높이 실패 | 팔 베이스 z, 목표 1.0388 m, 허용 오차 15 mm |
| 차체가 안 움직임 | 바퀴 브레이크 해제 로그와 cmd_vel subscriber |
| Navigation timeout/stall | odom, 경로 방향, waypoint와 Scene 버전 |
| PLACE_INSPECT/INVALID_COMMAND | 현재 범위의 예상 종료 |

물리 동작 중 실패하면 같은 Scene 상태에서 `/start_cycle`을 다시 호출하지 않는다.
Isaac Sim과 Task Manager를 함께 재시작해 팔레트 배치와 논리 상태를 초기화한다.

## 10. 통과 체크리스트

- [ ] 실제 Standalone만 `/sim_task/*`를 처리한다.
- [ ] 세 executor가 PREFLIGHT 전에 READY다.
- [ ] Pallet_02가 L3에서 L2로 이동한다.
- [ ] Pallet_03이 L4에서 L3로 이동한다.
- [ ] Pallet_01을 RACK_L1에서 Pick하고 랙 밖으로 인출한다.
- [ ] 운반 자세에서 joint_1만 Pick 종료값 대비 +90 deg다.
- [ ] 팔 베이스 높이가 1.0388 m에 도달한다.
- [ ] 바퀴 브레이크 해제 뒤 safe_to_navigate=true가 반환된다.
- [ ] Task Manager가 NAVIGATION/INSPECTION_DOCK을 발행한다.
- [ ] 선택: Nova Carter가 주행 중 Pallet_01을 안정적으로 유지한다.
- [ ] 선택: Navigation 결과가 reached_station=INSPECTION_DOCK이다.
