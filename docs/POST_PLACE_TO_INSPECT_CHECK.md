# PLACE_INSPECT 이후 검사 준비 구간 확인 (Ubuntu)

## 이번 구현의 명령 계약

두 명령의 대상은 `PALLET_001`임. Sim Task의 명령·상태·결과는 각각 `/sim_task/command`, `/sim_task/status`, `/sim_task/result`의 `std_msgs/msg/String` JSON임. `task_id`, `command_id`, `operation`, `pallet_id`는 명령과 terminal result에서 일치해야 함. 상태 알림의 `PALLET_DETECTED`는 관찰용이며 Task Manager는 그 알림으로 전이하지 않음.

| 명령 | 시작 조건 | 물리 완료 조건 | 실패 사유 |
|---|---|---|---|
| `CONVEY_TO_INSPECT` | 같은 런타임에서 `PLACE_INSPECT` 성공, 포크 인출 완료, 팔레트 미운반, 스테이션 `IDLE`, 컨베이어·스테이션 존재. `recipe_id=CONVEY_TO_INSPECT`, `source=INSPECT_STATION`, `destination=INSPECT_STOP` | 줄기 벨트 인터록 해제 후 대상 팔레트가 컨베이어 `VISION` 영역에서 검출·고정되고, 월드 위치의 프레임당 변위가 0.002 m 이하로 물리 시간 0.5초 지속됨. 결과 `reached_station=INSPECT_STOP` | 형식 `INVALID_COMMAND`, 순서 `INVALID_STATE`, 구성 `NOT_READY`, 대상 소실 `PALLET_LOST`, 컨베이어 정체 코드, 물리 시간 120초 초과 `CONVEY_TIMEOUT`, 예외 `CONVEYOR_FAILED` |
| `PREPARE_INSPECT` | 직전 `CONVEY_TO_INSPECT` 성공, 대상 팔레트가 `VISION`에 고정, 스테이션 `IDLE`. `recipe_id=PREPARE_INSPECT`, `source=INSPECT_STOP`, `destination=INSPECT_WORK_POS` | 기존 이송 프레임의 진입·하강·밀기 후 트레이가 목표 월드 x/y 각각 0.05 m 안, 선속도 0.05 m/s 이하, 포기 자리 오차 20 mm 이하이며 대상 팔레트가 다시 고정되고 스테이션 `PREPARED`임. 결과 `reached_station=INSPECT_WORK_POS` | 형식 `INVALID_COMMAND`, 순서·물리 상태 `INVALID_STATE`, 지그 물리 오류 `JIG_FAILED`, 물리 시간 90초 초과 `PREPARE_TIMEOUT` |

Task Manager의 바깥 제한은 각각 벽시계 600초·450초임. Sim 내부 제한은 물리 스텝 시간임. 실패한 명령은 `FAILED` terminal result를 보내며 Sim Task는 `ERROR; scene reset required`가 됨. 검사 준비 중 실패·시간 초과이면 지그는 `FAILED`에 남고 자동 검사·배출을 시작하지 않음. 통합 런타임은 컨베이어 자동 배출 타이머를 쓰지 않음. 장면을 새로 시작해야 재시도 가능함.

현재 `smart_farm_interfaces/msg/TaskResult.msg`에는 `pallet_id` 필드가 없음. 새 두 명령은 `/sim_task/result` JSON 필드를 사용하며, Task Manager가 이 필드를 검사함. 별도 Inspection 노드의 ROS 메시지 계약은 이번 변경에 포함하지 않음.

## 별도 프로세스로 전체 구간 확인

같은 호스트의 ROS 터미널은 동일한 `ROS_DOMAIN_ID`를 사용하고 `/opt/ros/jazzy/setup.bash`와 `cobot3_ws/install/setup.bash`를 source함. Isaac 터미널은 시스템 ROS를 source하지 않음. v014 USD 장면 및 참조 로봇 자산, v014 지도, `best.pt`, GPU Docker 구성이 먼저 준비되어야 함. 정확한 파일과 Docker 점검은 [TRANSFER_TO_PLACE_CHECK.md](TRANSFER_TO_PLACE_CHECK.md)의 준비·기동 절차를 따름.

1. Isaac 터미널: 저장소 루트에서 `/path/to/isaacsim/python.sh cobot3_ws/isaacpjt/smart_farm/runtime/standalone_app.py --autoplay --scene cobot3_ws/isaacpjt/smart_farm/scenes/Collected_smartfarm_v014/Collected_smartfarm_v014_room_core_cabbage.usd` 실행. `--no-vision-station`을 쓰지 않음. 통합 경로는 지그의 YOLO worker를 시작하지 않으므로 Isaac Python에 `ultralytics`가 없어도 이 두 명령을 준비할 수 있음.
2. Nav2: `ros2 launch smart_farm_navigation nav2.launch.py scan_mode:=auto use_rviz:=false` 실행.
3. Navigation Executor: `ros2 launch smart_farm_navigation navigation_node.launch.py` 실행.
4. Inspection Executor: 저장소 루트에서 `sudo docker compose -f compose.vision.yaml up -d vision` 후 `sudo docker compose -f compose.vision.yaml exec vision /entrypoint.sh ros2 launch smart_farm_vision object_detection.launch.py config_file:=/config/object_detection.yaml` 실행. `/inspection/status`의 `READY`와 `/rgb` 수신을 확인함.
5. Task Manager: `ros2 launch smart_farm_manager task_manager.launch.py` 실행. 세 Executor `READY`를 확인한 다음 `ros2 service call /start_cycle smart_farm_interfaces/srv/StartCycle "{scenario_id: DEMO_HARVEST_01}"` 실행.

명령 전에 별도 터미널에서 아래 토픽을 각각 기록함. `ros2 bag record`를 쓸 경우 저장 경로를 별도로 지정함.

```bash
ros2 topic echo /sim_task/command
ros2 topic echo /sim_task/status
ros2 topic echo /sim_task/result
ros2 topic echo /cycle/status
ros2 topic echo /inspection/command
```

`PLACE_INSPECT`의 `SUCCEEDED` 뒤 `CONVEY_TO_INSPECT` 명령이 발행되어야 함. 팔레트 감지 시 `/sim_task/status`가 `CONVEY_TO_INSPECT/PALLET_DETECTED`를 보여도 `/cycle/status`는 아직 `CONVEY_TO_INSPECT`여야 함. 같은 `task_id/command_id/pallet_id`와 `reached_station=INSPECT_STOP`을 가진 `SUCCEEDED` 결과 뒤에만 `PREPARE_INSPECT` 명령이 나와야 함. 다시 `reached_station=INSPECT_WORK_POS`의 성공 결과 뒤 상태가 `INSPECT`로 바뀜. 생산 시나리오는 그 다음 기존 `/inspection/command`를 자동 발행함. 여기서는 그 검사 결과를 이 구간의 성공으로 간주하지 않음.

## Sim Task 단위 명령 시험

Task Manager를 실행하지 않고, 새 장면의 Sim Executor·Nav2·Navigation Executor만 실행함. `/sim_task/result`, `/sim_task/status`, `/navigation/result` 구독 터미널을 명령 발행 전에 열어 둠. `ros2 topic pub --once --max-wait-time-secs 15`를 사용하며, 각 단계의 실제 `SUCCEEDED` terminal result를 본 뒤 다음 명령을 보냄. 같은 `task_id=UNIT-POST-PLACE`와 서로 다른 `command_id`를 사용함. Navigation만 `/navigation/command`의 `smart_farm_interfaces/msg/TaskCommand` 타입이고, 나머지는 `/sim_task/command`의 `std_msgs/msg/String` JSON임. 먼저 아래 네 명령으로 실제 Place를 완료함.

```bash
ros2 topic pub --once --max-wait-time-secs 15 /sim_task/command std_msgs/msg/String \
  '{data: "{\"task_id\":\"UNIT-POST-PLACE\",\"command_id\":\"UNIT-POST-PLACE-CMD-001\",\"operation\":\"TRANSFER\",\"recipe_id\":\"RACK_REARRANGE_01\"}"}'

ros2 topic pub --once --max-wait-time-secs 15 /sim_task/command std_msgs/msg/String \
  '{data: "{\"task_id\":\"UNIT-POST-PLACE\",\"command_id\":\"UNIT-POST-PLACE-CMD-002\",\"operation\":\"PICK_HARVEST\",\"recipe_id\":\"HARVEST_RACK_L1\",\"pallet_id\":\"PALLET_001\",\"source\":\"RACK_L1\",\"destination\":\"CARRY\"}"}'

ros2 topic pub --once --max-wait-time-secs 15 /navigation/command smart_farm_interfaces/msg/TaskCommand \
  '{task_id: UNIT-POST-PLACE, command_id: UNIT-POST-PLACE-CMD-003, operation: NAVIGATION, destination: FEEDER_DOCK}'

ros2 topic pub --once --max-wait-time-secs 15 /sim_task/command std_msgs/msg/String \
  '{data: "{\"task_id\":\"UNIT-POST-PLACE\",\"command_id\":\"UNIT-POST-PLACE-CMD-004\",\"operation\":\"PLACE_INSPECT\",\"recipe_id\":\"PLACE_AT_INSPECTION\",\"pallet_id\":\"PALLET_001\",\"source\":\"CARRY\",\"destination\":\"INSPECT_STATION\"}"}'
```

`PLACE_INSPECT` 결과가 `SUCCEEDED`이고 포크 인출 완료를 확인한 뒤 다음 두 명령을 입력함.

```bash
ros2 topic pub --once --max-wait-time-secs 15 /sim_task/command std_msgs/msg/String \
  '{data: "{\"task_id\":\"UNIT-POST-PLACE\",\"command_id\":\"UNIT-POST-PLACE-CMD-005\",\"operation\":\"CONVEY_TO_INSPECT\",\"recipe_id\":\"CONVEY_TO_INSPECT\",\"pallet_id\":\"PALLET_001\",\"source\":\"INSPECT_STATION\",\"destination\":\"INSPECT_STOP\"}"}'

ros2 topic pub --once --max-wait-time-secs 15 /sim_task/command std_msgs/msg/String \
  '{data: "{\"task_id\":\"UNIT-POST-PLACE\",\"command_id\":\"UNIT-POST-PLACE-CMD-006\",\"operation\":\"PREPARE_INSPECT\",\"recipe_id\":\"PREPARE_INSPECT\",\"pallet_id\":\"PALLET_001\",\"source\":\"INSPECT_STOP\",\"destination\":\"INSPECT_WORK_POS\"}"}'
```

두 번째 명령은 첫 번째 명령의 성공 결과를 본 뒤에만 입력함. 음성 시험은 새 장면에서 `PREPARE_INSPECT`를 먼저 보내 `FAILED/INVALID_STATE`를 확인함. 이 경우 Sim Task가 `ERROR`로 전환되므로 정상 시험을 위해 장면을 다시 시작함. 허위 `SUCCEEDED` 결과를 발행하지 않음.

## 실패 때 확인 순서

1. `/sim_task/result`의 `task_id`, `command_id`, `pallet_id`, `status`, `reason`, `reached_station` 확인. 결과가 없으면 `/sim_task/status`의 `state/phase/detail`과 Isaac 터미널 예외 확인.
2. `CONVEY_TO_INSPECT` 지연이면 Isaac의 `[컨베이어] 벨트 감지`, `카메라 앞 정지`, 고장 로그와 `/sim_task/status`의 `zone`, `still` 확인. `PALLET_DETECTED`만으로 다음 단계가 나왔다면 Task Manager 로그의 결과 처리 경로 확인.
3. `PREPARE_INSPECT` 지연이면 Isaac의 `[솎아내기] 프레임 이송 완료`, `[검사 준비] 실패`, `station_out/station_events.json`의 `PUSH_DONE_PREPARED` 확인. 목표 x/y와 실측 x/y는 장면 월드 좌표임. `seat_error_after_push_mm`와 트레이 속도를 함께 확인.
4. Task Manager가 멈추면 `Command timeout`, `Result ignored`, `PREFLIGHT failed` 로그와 `/cycle/status` 확인. Docker Inspection `READY`가 없으면 컨테이너의 `/rgb` 수신과 ROS 도메인 설정부터 확인.

정적·단위 테스트는 실제 Isaac 물리 이동을 검증하지 않음. 이 문서의 `SUCCEEDED` result와 화면·Isaac 물리 로그가 실기동 판정 근거임.
