# 인터페이스 계약 — v0.2.0

이 문서는 [시스템 아키텍처](01-architecture.md)의 `DEMO_HARVEST_01` 공정이 실제로 사용하는 ROS 2 경계를 정리한다. 공정 인자·제한 시간의 기준은 [시나리오 코드](../cobot3_ws/src/smart_farm_manager/smart_farm_manager/scenario.py), 결과 검증의 기준은 [상태 머신](../cobot3_ws/src/smart_farm_manager/smart_farm_manager/state_machine.py)이다.

## 통신 경계

| 이름 | ROS 2 타입 | 송신 → 수신 | 역할 |
| --- | --- | --- | --- |
| `/start_cycle` | [`StartCycle.srv`](../cobot3_ws/src/smart_farm_interfaces/srv/StartCycle.srv) | 사용자 → Task Manager | `DEMO_HARVEST_01` 시작 요청·수락 |
| `/cycle/status` | [`CycleStatus.msg`](../cobot3_ws/src/smart_farm_interfaces/msg/CycleStatus.msg) | Task Manager → 관찰자 | 현재 공정과 최종 상태 |
| `/sim_task/command`, `/sim_task/result`, `/sim_task/status` | `std_msgs/msg/String` 안의 JSON object | Task Manager ↔ Isaac Sim | 물리 공정 명령·terminal 결과·heartbeat |
| `/navigation/command`, `/navigation/result`, `/navigation/status` | `TaskCommand`, `TaskResult`, `ExecutorStatus` | Task Manager ↔ Navigation Executor | Nav2 이동·정밀 도킹 |
| `/inspection/command`, `/inspection/result`, `/inspection/status` | `TaskCommand`, `TaskResult`, `ExecutorStatus` | Task Manager ↔ Inspection Executor | `INSPECT`·`RECHECK`와 준비 상태 |
| `/sim_task/inspection_context` | `std_msgs/msg/String` JSON | Task Manager → Isaac Sim | 최초 `INSPECT`의 ID와 팔레트 지정 |
| `/inspection/detections_2d` | `std_msgs/msg/String` JSON | Inspection Executor → Isaac Sim | 슬롯 판정과 원본 영상 픽셀 검출 |
| `/sim_task/inspection_data_status` | `std_msgs/msg/String` JSON | Isaac Sim → Task Manager | 최초 검출 데이터의 `EXPECTED`·`STORED`·`REJECTED` |
| `/rgb` | `sensor_msgs/msg/Image` | Isaac Sim → Inspection Executor | 검사 카메라 원본 영상 |
| `/clock`, `/tf`, `/tf_static`, `/chassis/odom`, `/scan`, `/cmd_vel` | 표준 ROS 2 타입 | Isaac Sim ↔ Nav2 | 시뮬레이션 시각·위치·센서·주행 |
| `navigate_to_pose`, `/feeder_dock/start`, `/feeder_dock/result` | Nav2 Action, `std_msgs/msg/String` | Navigation Executor ↔ Nav2·도킹 노드 | 접근 주행과 도킹 결과 |

Navigation Executor는 [`NavigateToPose` ActionClient](../cobot3_ws/src/smart_farm_navigation/smart_farm_navigation/navigation_node.py)를 직접 사용한다. 접근점까지 Nav2로 이동한 뒤 같은 `command_id`를 전달받은 `feeder_dock` 결과만 인정한다. Task Manager는 `/cmd_vel`을 발행하지 않는다. Vision은 [GPU 컨테이너](../compose.vision.yaml)에서 실행하며 호스트와 같은 `ROS_DOMAIN_ID`가 필요하다.

## 공통 ROS 메시지

[`TaskCommand.msg`](../cobot3_ws/src/smart_farm_interfaces/msg/TaskCommand.msg)는 `task_id`, `command_id`, `operation`, `recipe_id`, `pallet_id`, `source`, `destination`, `target_slots[]`를 담는다. [`TaskResult.msg`](../cobot3_ws/src/smart_farm_interfaces/msg/TaskResult.msg)는 `task_id`, `command_id`, `operation`, `status`, `phase`, `reason`, `safe_to_navigate`, `reached_station`, `completed_units[]`, `defect_slots[]`, `unknown_slots[]`를 담는다. 이 두 메시지는 Navigation·Inspection 경계에서 사용한다. `TaskResult.msg`에는 `pallet_id`가 없으므로 해당 명령의 `pallet_id`와 ID로 연결해 추적한다.

[`ExecutorStatus.msg`](../cobot3_ws/src/smart_farm_interfaces/msg/ExecutorStatus.msg)는 `executor`, `state`, `task_id`, `command_id`, `operation`, `phase`, `detail`의 heartbeat다. `PREFLIGHT`는 `sim_task`·`navigation`·`inspection`의 최신 `READY`를 요구한다. 진행 중 상태 알림만으로 공정을 전이하지 않는다.

`StartCycle.srv`는 `scenario_id`를 받고 `accepted`, `task_id`, `reason`을 반환한다. `accepted=true`는 요청 수락이며 사이클 성공이 아니다. 최종 결과는 `/cycle/status`의 `state`·`status`·`reason`으로 확인한다. `CycleStatus.msg`에는 이 세 필드 외에 `task_id`, `scenario_id`, `active_command_id`가 있다.

Task Manager는 로컬 시각으로 `TASK-YYYYMMDD-HHMMSS` 형식의 `task_id`를 만들고, 같은 사이클의 명령마다 `<task_id>-CMD-NNN` 형식의 `command_id`를 부여한다. 결과의 executor와 세 식별 필드(`task_id`, `command_id`, `operation`)가 현재 활성 명령과 같아야 한다. 물리 공정의 Sim JSON 결과에는 `pallet_id`도 현재 명령과 일치해야 한다. 다른 ID의 결과와 terminal 이후의 결과는 무시한다.

## Sim Task JSON

Isaac Sim 경계의 `String.data`는 UTF-8 JSON **object** 한 개다. 명령의 필수 비어 있지 않은 문자열은 `task_id`, `command_id`, `operation`이며 선택 필드는 `recipe_id`, `pallet_id`, `source`, `destination` 문자열과 `target_slots` 문자열 배열이다. 사용하지 않는 필드는 빈 문자열·빈 배열로 보낸다.

```json
{
  "task_id": "TASK-20260930-120000",
  "command_id": "TASK-20260930-120000-CMD-001",
  "operation": "TRANSFER",
  "recipe_id": "RACK_REARRANGE_01",
  "pallet_id": "",
  "source": "",
  "destination": "",
  "target_slots": []
}
```

`/sim_task/result`는 명령 ID와 `status`, `phase`, `reason`, `pallet_id`, `safe_to_navigate`, `reached_station`, `completed_units[]`, `defect_slots[]`, `unknown_slots[]`를 담는다. `/sim_task/status`는 공통 `ExecutorStatus` 필드를 JSON으로 보내며 현재 구현은 `pallet_id`도 포함한다. Task Manager는 잘못된 결과·status JSON을 경고 후 무시한다. [Sim Task Node](../cobot3_ws/isaacpjt/smart_farm/runtime/sim_task_node.py)는 잘못된 명령 JSON을 실행하지 않는다.

## 공정별 성공 계약

표의 제한 시간은 Task Manager의 **벽시계 초**다. 물리 동작의 내부 제한과 별개다. 모든 행에서 `SUCCEEDED` terminal 결과와 활성 명령 ID 일치가 선행 조건이다.

| 공정 | 실행 주체 | 명령 핵심 인자 | 추가 성공 조건 | 제한 시간 |
| --- | --- | --- | --- | ---: |
| `TRANSFER` | Sim | `recipe_id=RACK_REARRANGE_01` | `completed_units`에 PALLET_002 L3→L2와 PALLET_003 L4→L3 포함 | 400 |
| `PICK_HARVEST` | Sim | `PALLET_001`, `RACK_L1→CARRY` | `safe_to_navigate=true` | 400 |
| `NAVIGATION` | Navigation | `destination=FEEDER_DOCK` | `reached_station=FEEDER_DOCK` | 400 |
| `PLACE_INSPECT` | Sim | `PALLET_001`, `CARRY→INSPECT_STATION` | Sim의 배치 성공 결과 | 300 |
| `CONVEY_TO_INSPECT` | Sim | `PALLET_001`, `INSPECT_STATION→INSPECT_STOP` | `reached_station=INSPECT_STOP` | 600 |
| `PREPARE_INSPECT` | Sim | `PALLET_001`, `INSPECT_STOP→INSPECT_WORK_POS` | `reached_station=INSPECT_WORK_POS` | 450 |
| `MOVE_TO_INSPECT` | Sim | `PALLET_001`, `INSPECT_WORK_POS→INSPECT_CAMERA_POSE` | `reached_station=INSPECT_CAMERA_POSE` | 300 |
| `INSPECT` | Inspection | `PALLET_001` | 슬롯 ID 유효, `unknown_slots=[]`; 불량 슬롯 저장 | 300 |
| `CULL` | Sim | `PALLET_001`, 최초 불량 `target_slots` | `completed_units`가 대상 슬롯과 정확히 일치 | 900 |
| `RECHECK` | Inspection | `PALLET_001`, 제거 대상 `target_slots` | `defect_slots=[]`, `unknown_slots=[]` | 300 |
| `RELEASE_INSPECT` | Sim | `PALLET_001`, `INSPECT_WORK_POS→INSPECT_STOP` | `reached_station=INSPECT_STOP` | 450 |
| `CONVEYOR_OUT` | Sim | `PALLET_001`, `INSPECT_STOP→PACK_OUT` | `reached_station=PACK_OUT` | 600 |

`INSPECT`의 `defect_slots`가 비면 `CULL`만 생략한다. `RECHECK`와 `RELEASE_INSPECT`는 정상 경로에도 필요하다. 상태 머신은 `TRANSFER`의 두 단위 완료, `CULL`의 전체 대상 완료, 검사 슬롯, 물리 공정의 팔레트·도착지 등을 검증한다.

## 검사 데이터와 CULL 동기화

Task Manager는 최초 `INSPECT` 명령을 발행할 때 `/sim_task/inspection_context`에 `task_id`, `inspection_command_id`, `pallet_id`, `operation="INSPECT"`를 보낸다. Vision은 같은 명령에서 받은 새 `/rgb` 프레임으로 추론하고 `/inspection/detections_2d`를 terminal `/inspection/result`보다 먼저 발행한다.

검출 JSON의 필드는 다음과 같다.

| 필드 | 의미 |
| --- | --- |
| `header.stamp.sec/nanosec`, `header.frame_id` | 원본 영상의 0이 아닌 시각과 카메라 frame ID |
| `task_id`, `command_id`, `inspection_command_id`, `pallet_id`, `operation` | 현재 검사·팔레트 식별자 |
| `coordinate_frame="image_pixels"`, `image_width`, `image_height` | resize 이전 원본 영상의 픽셀 좌표계와 크기 |
| `slot_states` | `SLOT_01`~`SLOT_06` 각각의 `NORMAL`·`DEFECT`·`UNKNOWN` 또는 재검사 `REMOVED` |
| `valid_for_cull` | 최초 `INSPECT`가 추론 오류와 미판정 없이 완료됐을 때만 `true` |
| `detections[]` | `slot_id`, `class_name`, `confidence`, `center_u/v`, `bbox_x_min/y_min/x_max/y_max` |

[Sim 검출 저장소](../cobot3_ws/isaacpjt/smart_farm/runtime/inspection_detection_store.py)는 준비된 `task_id`·`pallet_id`와 검사 명령 ID, 영상 시각·크기, 여섯 슬롯 상태, 슬롯마다 하나의 검출 및 좌표 범위를 검증한다. 유효한 최초 검사 데이터가 저장되면 `/sim_task/inspection_data_status`로 `STORED`를 보낸다. 불량이 있을 때 Task Manager는 해당 `STORED`를 확인한 뒤에만 `CULL`을 보낸다. `REJECTED` 또는 대기 시간 초과는 CULL 전에 사이클 실패로 끝난다. CULL 대상은 저장된 `DEFECT` 슬롯 집합과 정확히 같아야 한다.

`RECHECK`는 명령 후 새 프레임으로 수행한다. 제거 대상 슬롯에 검출이 없으면 `REMOVED`로 기록하고, 다른 슬롯은 `NORMAL`이어야 성공한다. 이때 `valid_for_cull=false`이며 Sim은 RECHECK 검출을 최초 CULL 데이터로 저장하지 않는다. 픽셀 좌표는 CULL 대상·진단 정보이고, 실제 파지 목표는 트레이 자세와 슬롯 오프셋으로 만든다.

## 시간·실패·재시작

명령·결과·status의 기본 ROS QoS는 depth 10이며, `/rgb`는 BEST_EFFORT depth 1이다. Task Manager의 PREFLIGHT 신선도, CULL 데이터 대기, 명령 제한 시간은 `time.monotonic()` 기준이므로 시뮬레이션 Pause 중에도 지난다. Navigation Executor와 Nav2는 시뮬레이션 시각을 사용한다.

`status`의 `READY`·`BUSY`·`ERROR`는 준비·진행 관찰용이다. 공정 전이는 terminal 결과만으로 결정한다. `SUCCEEDED` 이외의 결과, 성공 필드 검증 실패, 결과 제한 시간 초과는 `ERROR`로 이어지고 다음 명령을 발행하지 않는다. 물리 공정 실패는 `/cycle/status.status=RESET_REQUIRED`가 될 수 있다. Isaac Sim의 Stop→Play 장면 재로드 후에는 Task Manager도 새 `IDLE` 상태로 시작해야 한다.
