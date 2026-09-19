# 인터페이스 설계

관련 이슈·To-do: 인터페이스 설계 (https://app.notion.com/p/3dd0f4c1b9cc80ce93cfc9bcd586ccf6?pvs=21)
기록일: 2026년 9월 16일
날짜: 2026년 9월 16일
담당자: 봉승현
마지막 수정: 2026년 9월 17일 오전 12:08
분야: IsaacSim, ROS2
분류: 설계 결정
생성일: 2026년 9월 16일 오후 7:30
작성 상태: 작성 중

# 인터페이스 설계 — 1차 MVP

> 기준 아키텍처: [시스템 아키텍쳐](https://app.notion.com/p/3dd0f4c1b9cc80a2a1a2e624fa2f7c2c?pvs=21). 외부 ROS 2의 Nav2 기반 이동과 Isaac Sim 내부 M0617 제어를 연결하는 최소 계약이다. 이름·시간 제한·메시지 형식은 단독 시험 후 확정한다.
> 

## 1. 통신 경계

- `task_manager` ↔ `navigation_node` 및 `transport_arm_node`: 작업 명령과 결과를 ROS 2 Topic으로 교환한다. 각각 한 번에 하나의 요청만 실행한다.
- `navigation_node` ↔ Nav2: `BasicNavigator`가 내부적으로 `NavigateToPose` Action을 사용한다. `navigation_node`가 직접 주행 경로나 `/mir100/cmd_vel`을 계산·발행하지 않는다.
- Nav2 ↔ Isaac Sim: Nav2의 속도 명령을 MiR100 구동에 연결하고, Isaac Sim은 Nav2에 필요한 지도·위치 추정용 로봇 TF, 오도메트리, 주행 센서와 `/clock`을 제공한다.
- `transport_arm_node` ↔ M0617: Isaac Sim 프로세스 내부에서 `motion`이 관절 Action을 직접 적용하고 현재 관절값을 읽는다. 외부 `/m0617/joint_command`, 필수 `/m0617/joint_states` 통신은 사용하지 않는다.
- 팔레트 TF·위치 Topic, DB, `conveyor_node`, M0609 검사·비전은 1차 계약에서 제외한다. **팔레트 TF 제외는 Nav2에 필요한 로봇 TF 제외와 다르다.**

## 2. 인터페이스 목록: 입력과 출력

| 구분·이름 | 송신 → 수신 | 입력 데이터 | 출력 데이터 |
| --- | --- | --- | --- |
| `/start_cycle` · Service | 실행자 → task_manager | Request: scenario_id | Response: accepted, task_id, reason; 작업 수락만 표시 |
| `/navigation/command` · Topic | task_manager → navigation_node | command_id, task_id, destination | 명령 메시지 발행; 동기 응답 없음 |
| `/navigation/result` · Topic | navigation_node → task_manager | 실행한 명령의 command_id, task_id와 Nav2 결과 | command_id, task_id, status, reason, reached_station 발행 |
| `/transport_arm/command` · Topic | task_manager → transport_arm_node | command_id, task_id, pallet_id, operation, station | 명령 메시지 발행; 동기 응답 없음 |
| `/transport_arm/result` · Topic | transport_arm_node → task_manager | 실행한 명령의 ID·작업 결과 | command_id, task_id, pallet_id, operation, status, reason 발행 |
| `NavigateToPose` · Nav2 Action | navigation_node → Nav2 | `map` 기준 PoseStamped 목표 | 목표 수락·주행 피드백·SUCCEEDED/FAILED/CANCELED 결과 |
| MiR100 주행 Topic | Nav2 ↔ Isaac Sim | Nav2 속도 명령; 장면의 센서·로봇 상태 | 속도 구동과 로봇 TF·오도메트리·LiDAR 등 발행 |
| M0617 내부 API | transport_arm_node ↔ motion·장면 | 고정 작업점 자세, 팔레트 ID, 현재 베이스 자세 | apply_action() 명령; get_joint_positions() 및 팔레트 상태 확인 |
| `/clock` · Topic | Isaac Sim → 외부 ROS 2 | 장면 시뮬레이션 시간 | rosgraph_msgs/Clock; 응답 없음 |

Topic의 출력 칸은 **해당 송신자가 발행하는 데이터**를 뜻하며 즉시 반환되는 응답이 아니다. 최초 통합은 `std_msgs/msg/String` JSON으로 작업 명령·결과를 표현한다. 필드가 확정되면 사용자 정의 메시지로 교체할 수 있다. 센서·Nav2 Topic 이름과 네임스페이스는 에셋·launch 구성에서 실제 값을 확인해 통일한다.

## 3. 공통 데이터 규칙

- `task_id`: 랙 순환 실행 1회의 ID. `task_manager`가 생성한다.
- `command_id`: 개별 이동 또는 팔 명령의 고유 ID. 한 `task_id` 안에서 요청마다 새로 생성하고 결과에 그대로 되돌린다.
- `pallet_id`: `PALLET_01`~`PALLET_04`, `PALLET_SEED`. 요청 대상의 논리 ID이며 자동 인식 결과가 아니다.
- `destination` / `station`: `RACK_L1`~`RACK_L4`, `SEED_PICKUP`, `INSPECT_ZONE`. 지도 좌표는 `navigation_node` 설정, 팔 자세·접근점은 `motion` 설정에 저장한다. 실측 좌표는 보류한다.
- `operation`: `PICK` 또는 `PLACE`. 한 명령에 한 작업만 수행한다.
- `status`: `SUCCEEDED`, `FAILED`, `CANCELED`, `TIMEOUT`. 실제 구현하지 않은 취소 기능은 발행하지 않는다.
- `reason`: `NONE`, `INVALID_COMMAND`, `BUSY`, `SIM_NOT_READY`, `NO_FEEDBACK`, `NAV_FAILED`, `MOTION_FAILED`, `TIMEOUT` 등을 시작값으로 사용한다.
- 명령 수신자는 작업 중 새 명령을 실행하지 않고 동일 `command_id`로 `FAILED/BUSY` 결과를 보낸다. 이미 완료한 `command_id`의 재전송은 재실행하지 않는다. 최초 MVP에서 재시작 후 중복 기록은 유지하지 않는다.

## 4. 작업 계층

### 4.1 전체 시작: StartCycle.srv

```
# Request
string scenario_id
---
# Response
bool accepted
string task_id
string reason
```

`RACK_CYCLE_01`만 허용한다. `accepted=true`는 작업 시작 수락을 뜻한다. 전체 성공은 별도 `task_manager` 로그·시험 기록으로 확인한다. 이미 실행 중이면 즉시 `accepted=false`, `reason=BUSY`.

### 4.2 이동 명령과 결과

```json
{"command_id":"c-01","task_id":"t-01","destination":"RACK_L2"}
```

```json
{"command_id":"c-01","task_id":"t-01","status":"SUCCEEDED","reason":"NONE","reached_station":"RACK_L2"}
```

`navigation_node`는 작업점 좌표를 `map` 기준 PoseStamped로 변환하고 `BasicNavigator.goToPose()`로 보낸다. `isTaskComplete()`, `getResult()`로 완료를 확인한다. Nav2의 `TaskResult.SUCCEEDED`만 이동 성공으로 변환한다. `last_pose` 같은 마지막 피드백 좌표는 최종 도킹 위치를 독립적으로 증명하지 않는다. 지도 기준 도킹 오차가 꼭 필요하면 `/amcl_pose` 등 지도 좌표 추정값과 목표를 비교하는 검증을 추가한다; `/odom` 좌표를 지도 목표와 직접 비교하지 않는다.

### 4.3 팔 명령과 결과

```json
{"command_id":"c-02","task_id":"t-01","pallet_id":"PALLET_02","operation":"PICK","station":"RACK_L2"}
```

```json
{"command_id":"c-02","task_id":"t-01","pallet_id":"PALLET_02","operation":"PICK","status":"SUCCEEDED","reason":"NONE"}
```

`transport_arm_node`는 Isaac Sim 내부의 `motion`을 호출한다. PICK: 접근 → 포크 삽입 → 상승·인출 → 운송 자세. PLACE: 접근 → 하강·안착 → 포크 이탈 → 안전 자세. 시뮬레이션 루프에서 명령 수신과 동작을 프레임별로 진행한다. 긴 PICK/PLACE를 ROS 수신 콜백 안에서 블로킹 호출하지 않는다. 현재 관절 위치는 내부 `get_joint_positions()`로 확인하고 필요한 경우 TCP 목표 수렴을 평가한다. 관절 동작 완료와 실제 팔레트 파지·안착은 구분한다.

### 4.4 task_manager 호출 순서

```
navigation/command(출발지) → navigation/result(SUCCEEDED)
transport_arm/command(PICK) → transport_arm/result(SUCCEEDED; 운송 자세 포함)
navigation/command(목적지) → navigation/result(SUCCEEDED)
transport_arm/command(PLACE) → transport_arm/result(SUCCEEDED)
```

이 패턴을 `PALLET_01`→`INSPECT_ZONE`, `PALLET_02` L2→L1, `PALLET_03` L3→L2, `PALLET_04` L4→L3, `PALLET_SEED`→L4에 적용한다. 각 명령의 결과 ID를 검증한 뒤 다음 단계로 간다. 실패·시간초과 후 자동 재시도하지 않는다.

## 5. Nav2 ↔ Isaac Sim 장치 계층

| 항목 | 방향·형식 | 구성·검증 |
| --- | --- | --- |
| 지도 | map_server → Nav2; `nav_msgs/OccupancyGrid` | 장면의 고정 점유 지도를 생성·저장하고 `map` 기준 작업점 좌표와 일치시킨다 |
| 위치 추정·TF | 로봇·위치 추정 → Nav2; `/tf`, `/tf_static` 등 | `map→odom→base_link` 및 센서 프레임 변환을 사용할 수 있어야 한다. 팔레트 TF는 발행하지 않는다 |
| 오도메트리 | Isaac Sim → Nav2; `nav_msgs/Odometry` | 로봇 위치·속도와 frame_id·시간을 확인한다. 실제 토픽명은 장면 구성에 맞춘다 |
| 주행 센서 | Isaac Sim → Nav2; `sensor_msgs/LaserScan` 등 | 로컬 장애물 인식에 필요한 센서와 프레임·QoS를 맞춘다 |
| 속도 명령 | Nav2 → Isaac Sim; `geometry_msgs/Twist` | Nav2가 발행하고 장면이 MiR100 구동기로 변환한다. 실제 이름은 예: `/mir100/cmd_vel`; navigation_node가 직접 발행하지 않는다 |
| 시간 | Isaac Sim → ROS 2; `/clock` | 외부 관련 노드의 `use_sim_time`과 시각을 일치시킨다 |

이 항목들은 예제의 다른 로봇에서 제공되는 기능을 MiR100 에셋에 자동 보장하지 않는다. Isaac Sim 장면 그래프, 로봇 차륜 구동, 지도·위치 추정 launch를 구성해야 한다. `BasicNavigator.setInitialPose()`는 실제 시작 위치에 맞춰 한 번 설정한다.

## 6. M0617 내부 제어와 관절 상태

- `motion`은 `controller.forward()` → `robot.apply_action()` 방식으로 목표를 매 프레임 적용하고, `robot.get_joint_positions()`와 TCP 위치로 종료를 판단한다.
- 이동 후에는 MiR100 위 M0617 베이스의 **현재** 월드 자세를 RMPflow에 갱신한다. 고정된 시작 베이스 자세를 계속 쓰지 않는다.
- 팔레트 실제 들림·지지·안착의 자동 검사가 필요하면 Isaac Sim 내부의 대상 prim 위치·물리 상태를 조회하는 로직을 별도로 구현한다. 1차에 이 검사를 구현하지 못했다면 결과는 **팔 동작 완료**만 의미하고 화면 관찰 기록을 별도로 남긴다.
- 관절 상태를 외부 시각화할 필요가 생기면 Isaac Sim `ROS2 Publish Joint State`를 추가해 `/m0617/joint_states`를 발행할 수 있다. 기본 작업 결과 판정에 필수는 아니다. 내부 `apply_action()`과 외부 관절 명령을 동시에 같은 관절에 적용하지 않는다.

## 7. 오류·제한시간·실행 준비

- Topic은 수신 준비 전 발행된 명령이 소실될 수 있다. 장면·Nav2·두 실행 노드를 먼저 띄운 뒤 `/start_cycle`을 호출한다. 첫 통합에서 노드 준비 여부 확인을 시험 항목으로 둔다.
- `navigation_node`는 ROS 명령 수신을 막지 않도록 요청을 큐에 전달하고, 주행 실행 흐름에서 BasicNavigator를 사용한다. `goToPose()`와 결과 대기를 단순 수신 콜백 안에 넣지 않는다.
- `task_manager`는 결과의 `command_id`, `task_id`와 필요시 팔레트 ID를 비교하고 제한시간을 감시한다. 실패 또는 결과 없음 시 다음 명령을 보내지 않는다. Nav2 목표가 살아 있으면 취소 요청·정지 확인을 시도한다.
- 시뮬레이션 정지 중에도 무기한 대기하지 않도록 바깥에서 측정한 실제 시간 기준 제한시간을 둔다. 값은 단독 시험으로 정한다.
- 도킹 위치 5 cm·방향 5°, 정지 0.5초, 관절 오차 0.03 rad은 **시험용 시작값**이며 Nav2 성공 판정과 별개로 검증할 때만 적용한다.

## 8. 통합 시험 및 남은 확인

1. 고정 장면, 지도·로봇 TF·오도메트리·주행 센서·`/clock`, MiR100 구동 연결을 확인한다.
2. BasicNavigator 단독으로 작업점 이동과 Nav2 성공·실패 결과를 검증한다.
3. Isaac Sim 내부 팔 제어로 M0617 PICK/PLACE와 이동 후 베이스 자세 갱신을 검증한다.
4. 네 작업 Topic의 요청·응답 및 `command_id` 짝짓기·BUSY·TIMEOUT을 확인한다.
5. `task_manager`가 수확 팔레트 인계와 랙 순환을 순차 실행하도록 연결한다.
6. 반복 시험에서 ROS 작업 완료, 물리 파지·안착, 운반 중 낙하, 컨베이어 연출을 구분해 기록한다.

**확정 전 항목:** MiR100 로봇 모델·차륜 구동과 Nav2 launch, 센서/Topic 실제 명칭, `map` 작업점 좌표, M0617 관절 이름·URDF/RMPflow 설정, 운반 안정성, 물리 성공 판정 범위와 제한시간.