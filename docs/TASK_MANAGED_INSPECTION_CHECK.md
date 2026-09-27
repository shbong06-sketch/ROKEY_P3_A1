# Task Manager 통합 확인: TRANSFER → INSPECT (Ubuntu)

`DEMO_HARVEST_01`의 실제 명령과 terminal result를 확인하는 절차다. Task Manager는 `TRANSFER → PICK_HARVEST → NAVIGATION → PLACE_INSPECT → CONVEY_TO_INSPECT → PREPARE_INSPECT → MOVE_TO_INSPECT → INSPECT`를 순서대로 발행한다. 각 전이는 활성 명령과 일치하는 `SUCCEEDED` terminal result로만 진행한다. `PALLET_DETECTED` 등 상태 알림만으로 전이하지 않는다. 검사 이후의 `CULL`·`CONVEYOR_OUT`은 이 절차의 통과 범위가 아니며, 현재 Sim Executor의 물리 명령으로 구현되지 않았다. 생산 시나리오의 다음 명령이 실패하더라도 앞 단계의 결과를 위조하거나 시나리오를 `COMPLETE`로 바꾸지 않는다.

## 준비 및 외부 자산

- 모든 호스트 ROS 터미널과 Vision 컨테이너에 같은 `ROS_DOMAIN_ID`를 설정한다. 두 PC를 사용하면 DDS로 `/clock`, `/tf`, `/chassis/odom`, `/scan`, `/rgb`가 필요한 프로세스에 도달해야 한다. Isaac 터미널에는 시스템 ROS를 source하지 않는다.
- Git 외부 자산인 `cobot3_ws/isaacpjt/smart_farm/scenes/Collected_smartfarm_v014/Collected_smartfarm_v014_room_core_cabbage.usd`와 참조 로봇 자산, v014 지도, `cobot3_ws/src/smart_farm_vision/resource/best.pt`를 준비한다.
- Inspection Executor는 `compose.vision.yaml`의 별도 GPU 컨테이너에서 실행한다. 컨테이너에 `torch`, `ultralytics`, OpenCV, `cv_bridge`가 필요하다. 통합 경로의 Isaac 검사 스테이션은 자체 YOLO worker를 시작하지 않으므로 Isaac Python에 `ultralytics`를 설치할 필요가 없다. 호스트에서 Vision 노드를 직접 실행하면 호스트 Python의 `torch` 부재로 중단될 수 있다.
- Docker를 실행하는 각 호스트(VM과 RTX 5080 노트북 모두)에 NVIDIA 드라이버와 Container Toolkit이 필요하다. `could not select device driver "nvidia"`가 나오면 [NVIDIA Container Toolkit 설치 안내](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html)에 따라 해당 호스트의 Docker GPU runtime을 구성한다. `service "vision" is not running`은 보통 그 시작 실패의 후속 메시지다. 현재 Vision Dockerfile의 PyTorch/CUDA 빌드가 대상 GPU에서 동작하는지도 실제 CUDA 연산으로 확인한다.

저장소 루트가 `/path/to/ROKEY_P3_A1`인 예시다. 각 ROS 터미널에서 `/opt/ros/jazzy/setup.bash`와 빌드된 `cobot3_ws/install/setup.bash`를 source한다. 코드 변경 후에는 기존 프로세스를 종료하고 새 장면·노드로 시작한다.

```bash
cd /path/to/ROKEY_P3_A1/cobot3_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-up-to smart_farm_manager smart_farm_navigation smart_farm_vision --symlink-install
source install/setup.bash
```

Vision 코드나 설정이 바뀌었다면 저장소 루트에서 이미지를 다시 빌드하고 컨테이너를 재생성한다. `compose.vision.yaml`은 `ROS_DOMAIN_ID`를 저장소 루트의 `.env` 또는 셸 환경에서 받는다. `down -v`는 `.env`를 삭제하지 않지만, 재생성 전에 도메인 값이 호스트 ROS 터미널과 같은지 확인한다. `FASTDDS_BUILTIN_TRANSPORTS=UDPv4`도 컨테이너 환경에 적용되어야 한다.

```bash
cd /path/to/ROKEY_P3_A1
cat .env
test -f cobot3_ws/src/smart_farm_vision/resource/best.pt
sudo docker compose -f compose.vision.yaml build vision
sudo docker compose -f compose.vision.yaml up -d --force-recreate vision
sudo docker compose -f compose.vision.yaml exec vision /entrypoint.sh printenv ROS_DOMAIN_ID FASTDDS_BUILTIN_TRANSPORTS
sudo docker compose -f compose.vision.yaml exec vision /entrypoint.sh python3 -c 'import torch; print(torch.__version__, torch.version.cuda, torch.cuda.get_device_name(0)); print((torch.ones(1, device="cuda") + 1).item())'
```

`up -d`는 컨테이너의 기본 `sleep infinity`만 시작한다. 아래 Inspection launch를 별도로 실행해야 한다. RTX 5080에서 CUDA 연산이 실패하면 현재 Dockerfile의 PyTorch/CUDA 빌드를 GPU에 맞게 갱신·재빌드한 뒤 다시 확인한다.

## 별도 프로세스 기동

아래 항목을 각각 별도 터미널에서 실행한다. 2~5번 및 관찰용 터미널에는 위 ROS 환경을 source한다. `run_flow.py`, PowerShell 올인원 도구, mock executor는 이 실기동 경로에 사용하지 않는다.

1. **Isaac Sim / Sim Executor** — 시스템 ROS를 source하지 않은 터미널에서 저장소 루트로 이동한다. `--no-vision-station`은 사용하지 않는다.

   ```bash
   /path/to/isaacsim/python.sh cobot3_ws/isaacpjt/smart_farm/runtime/standalone_app.py \
     --autoplay --scene cobot3_ws/isaacpjt/smart_farm/scenes/Collected_smartfarm_v014/Collected_smartfarm_v014_room_core_cabbage.usd
   ```

   원격 화면이 필요하면 같은 명령에 `--livestream`과 접속 가능한 서버 주소를 준다.

   ```bash
   /path/to/isaacsim/python.sh cobot3_ws/isaacpjt/smart_farm/runtime/standalone_app.py \
     --livestream --autoplay \
     --scene cobot3_ws/isaacpjt/smart_farm/scenes/Collected_smartfarm_v014/Collected_smartfarm_v014_room_core_cabbage.usd \
     --/app/livestream/publicEndpointAddress=서버_IP --/app/livestream/port=49100
   ```

2. **Nav2** — `ros2 launch smart_farm_navigation nav2.launch.py scan_mode:=auto use_rviz:=false`
3. **Navigation Executor** — `ros2 launch smart_farm_navigation navigation_node.launch.py`
4. **Inspection Executor** — 저장소 루트에서 다음 명령을 실행한 터미널을 유지한다.

   ```bash
   sudo docker compose -f compose.vision.yaml exec vision \
     /entrypoint.sh ros2 launch smart_farm_vision object_detection.launch.py \
     config_file:=/config/object_detection.yaml
   ```

5. **Task Manager** — `ros2 launch smart_farm_manager task_manager.launch.py`

PREFLIGHT는 Sim Task, Navigation, Inspection의 최근 `READY` heartbeat를 모두 요구한다. Inspection은 모델 로드와 `/rgb`의 유효한 영상 수신 후 `READY`가 된다. 시작 전에 다음을 확인한다.

```bash
ros2 topic echo --once /sim_task/status
ros2 topic echo --once /navigation/status
ros2 topic echo --once /inspection/status
ros2 topic hz /rgb
ros2 action list
```

## Bag 기록과 자동 실행

별도 ROS 터미널에서 사이클 시작 **전**에 bag을 기록한다. `/tmp`는 재부팅 후 사라질 수 있으므로 저장소 아래 영구 경로를 사용한다. 출력 디렉터리가 이미 존재하면 `post_inspect_02`처럼 새 이름을 준다. `/rgb` 원본 영상은 용량이 크므로 기본 목록에서 제외한다. 영상 시각은 `/inspection/detections_2d`의 `header.stamp`와 필요 시 `/rgb` 헤더로 확인한다.

```bash
mkdir -p /path/to/ROKEY_P3_A1/results/bags
ros2 bag record -o /path/to/ROKEY_P3_A1/results/bags/post_inspect_01 \
  /cycle/status /sim_task/command /sim_task/result /sim_task/status \
  /navigation/command /navigation/result /navigation/status \
  /feeder_dock/status /feeder_dock/result \
  /sim_task/inspection_context /sim_task/inspection_data_status \
  /inspection/command /inspection/status /inspection/detections_2d /inspection/result
```

세 Executor의 `READY`를 확인한 후 다른 ROS 터미널에서 시작한다.

```bash
ros2 service call /start_cycle smart_farm_interfaces/srv/StartCycle "{scenario_id: DEMO_HARVEST_01}"
```

`task_id`는 `/start_cycle` 응답의 값이며 명령 ID는 보통 그 값에 `-CMD-001`부터 붙는다. 실제 성공 판정에는 같은 `task_id`, 해당 활성 명령과 결과의 `command_id`·`operation`, 요구되는 `pallet_id`와 물리 완료 필드를 대조한다. `/sim_task/command`, `/sim_task/result`, `/sim_task/status`는 `std_msgs/msg/String` JSON이고, Navigation·Inspection의 명령과 결과는 각각 `smart_farm_interfaces/msg/TaskCommand`, `TaskResult`다. 현재 `TaskResult.msg`에는 `pallet_id` 필드가 없으므로 Navigation·Inspection에서는 명령·결과 ID와 결과 내용을 대조한다.

| 순서 | 명령 토픽·operation | 실제 성공 결과와 다음 상태 |
|---|---|---|
| 1 | `/sim_task/command` `TRANSFER` (`CMD-001`) | `SUCCEEDED`, `completed_units`에 `PALLET_002:RACK_L3:RACK_L2`, `PALLET_003:RACK_L4:RACK_L3` → `PICK_HARVEST` |
| 2 | `/sim_task/command` `PICK_HARVEST` (`CMD-002`, `PALLET_001`) | `SUCCEEDED`, `safe_to_navigate=true` → `NAVIGATION` |
| 3 | `/navigation/command` `NAVIGATION` (`CMD-003`, `FEEDER_DOCK`) | `SUCCEEDED`, `reached_station=FEEDER_DOCK` → `PLACE_INSPECT` |
| 4 | `/sim_task/command` `PLACE_INSPECT` (`CMD-004`, `PALLET_001`) | `SUCCEEDED`, 포크 인출 완료 → `CONVEY_TO_INSPECT` |
| 5 | `/sim_task/command` `CONVEY_TO_INSPECT` (`CMD-005`, `PALLET_001`) | `SUCCEEDED`, `reached_station=INSPECT_STOP` → `PREPARE_INSPECT` |
| 6 | `/sim_task/command` `PREPARE_INSPECT` (`CMD-006`, `PALLET_001`) | `SUCCEEDED`, `reached_station=INSPECT_WORK_POS` → `MOVE_TO_INSPECT` |
| 7 | `/sim_task/command` `MOVE_TO_INSPECT` (`CMD-007`, `PALLET_001`) | 비전 M0609이 검사 카메라 자세에 도달해 안정화된 뒤 `SUCCEEDED`, `reached_station=INSPECT_CAMERA_POSE` → `INSPECT` |
| 8 | `/inspection/command` `INSPECT` (`CMD-008`, `PALLET_001`) | `/inspection/result`에서 같은 명령의 `SUCCEEDED` 또는 명시적 `FAILED`; 성공 후 생산 시나리오가 다음 명령을 발행할 수 있음 |

## 컨베이어·지그의 물리 완료 조건

| 명령 | 시작 조건 | 물리 완료 조건 | 주요 실패 사유·제한 |
|---|---|---|---|
| `CONVEY_TO_INSPECT` | 같은 런타임의 `PLACE_INSPECT` 성공, 포크 인출 완료, 팔레트 미운반, 스테이션 `IDLE`; `recipe_id=CONVEY_TO_INSPECT`, `source=INSPECT_STATION`, `destination=INSPECT_STOP` | 대상 팔레트가 컨베이어 `VISION` 영역에서 검출·고정되고 월드 위치의 프레임당 변위 ≤0.002 m가 물리 시간 0.5초 지속 | `INVALID_COMMAND`, `INVALID_STATE`, `NOT_READY`, `PALLET_LOST`, 컨베이어 정체 코드, `CONVEY_TIMEOUT`(물리 120초), `CONVEYOR_FAILED`; Task Manager 벽시계 600초 |
| `PREPARE_INSPECT` | 직전 Convey 성공, 같은 팔레트 `VISION` 고정, 스테이션 `IDLE`; `recipe_id=PREPARE_INSPECT`, `source=INSPECT_STOP`, `destination=INSPECT_WORK_POS` | 지그 진입·하강·밀기 후 트레이 월드 x/y 각각 목표 0.05 m 이내, 선속도 ≤0.05 m/s, 자리 오차 ≤20 mm, 재고정 및 스테이션 `PREPARED` | `INVALID_COMMAND`, `INVALID_STATE`, `JIG_FAILED`, `PREPARE_TIMEOUT`(물리 90초); Task Manager 벽시계 450초 |
| `MOVE_TO_INSPECT` | 같은 팔레트의 Prepare 성공, 스테이션 `PREPARED`, 팔레트 고정; `recipe_id=MOVE_TO_INSPECT`, `source=INSPECT_WORK_POS`, `destination=INSPECT_CAMERA_POSE` | 기존 올인원의 검사 자세 모션이 끝나고 20 물리 프레임 안정화 후 스테이션 `INSPECT_READY` | `INVALID_COMMAND`, `INVALID_STATE`, `INSPECT_POSE_FAILED`, `INSPECT_POSE_TIMEOUT`(물리 90초); Task Manager 벽시계 300초 |

`CONVEY_TO_INSPECT/PALLET_DETECTED` 상태가 나와도 `/cycle/status`는 Convey에 머물러야 한다. 세 물리 명령 모두 성공 terminal result가 다음 명령의 유일한 전이 조건이다. 물리 실패·시간 초과 때 임의로 배출하지 않고 `FAILED`를 반환한다. Sim Task가 `ERROR; scene reset required`가 되면 새 장면으로 재시작한다. 통합 경로의 컨베이어 자동 배출 타이머와 스테이션의 팔레트 감지 후 자동 PUSH_IN·검사 시작 경로는 사용하지 않는다.

## INSPECT 결과와 Cull용 2D 검출 데이터

| 토픽 | 타입 | 계약 |
|---|---|---|
| `/inspection/command` | `smart_farm_interfaces/msg/TaskCommand` | Task Manager가 `task_id`, `command_id`, `pallet_id`, `operation=INSPECT` 발행 |
| `/inspection/result` | `smart_farm_interfaces/msg/TaskResult` | 별도 Inspection Executor가 명령 이후 새 프레임을 검사하여 같은 명령 ID의 terminal 결과 발행; 성공 시 `defect_slots` 포함 |
| `/sim_task/inspection_context` | `std_msgs/msg/String` JSON | Task Manager가 Sim에 `task_id`, `inspection_command_id`, `pallet_id` 전달 |
| `/inspection/detections_2d` | `std_msgs/msg/String` JSON | Inspection Executor가 슬롯별 검출 데이터를 Sim에 직접 전달 |
| `/sim_task/inspection_data_status` | `std_msgs/msg/String` JSON | Sim의 `EXPECTED`, `WAITING_CONTEXT`, `STORED`, `REJECTED` 관찰용 상태; Task Manager 전이 조건이 아님 |

기존 `/inspection/detections_2d`를 재사용한다. 상위 JSON에는 `task_id`, `command_id`, 같은 값의 `inspection_command_id`, `pallet_id`, `coordinate_frame=image_pixels`, `header.frame_id`(카메라 프레임), `header.stamp.sec/nanosec`(원본 영상 시각), `image_width/height`, `slot_states`, `valid_for_cull`, `detections`가 들어간다. 각 detection에는 `slot_id`, `class_name`, `confidence`, `center_u/v`, `bbox_x_min/y_min/x_max/y_max`가 들어가며 좌표는 영상 픽셀이다. 월드 좌표·깊이는 제공하지 않는다.

| 판정 | Inspection 결과 |
|---|---|
| `lettuce_dark_green` | `NORMAL` |
| `lettuce_yellow`, `lettuce_brown` | `DEFECT`; 해당 슬롯을 `defect_slots`에 포함 |
| `SLOT_01`~`SLOT_06` 중 누락·중복 검출, ROI 충돌, 알 수 없는 클래스 | 해당 슬롯 `UNKNOWN`, `FAILED/UNKNOWN_SLOT`, `valid_for_cull=false` |
| 추론 예외 | `FAILED/INSPECTION_FAILED`, `valid_for_cull=false` |
| 명령 후 새로운 영상 시각이 오지 않음 | `FAILED/IMAGE_TIMEOUT` |
| 영상 stamp가 0 또는 `frame_id`가 비어 있음 | `FAILED/INVALID_IMAGE_METADATA` |

기존 올인원의 yellow·brown 불량 규칙에 맞춰 Vision의 이전 yellow=`UNKNOWN` 설정을 변경했다. 현재 슬롯 할당은 영상의 3열×2행 ROI 중심 기준이며, 올인원의 3D 포기 투영·다중 프레임 투표와 다르다. ROI 밖 검출은 슬롯 판정에서 제외한다. `/inspection/debug_image`에서 여섯 포기와 ROI가 맞는지 확인한다. `UNKNOWN`이면 검사 실패로 멈춘다.

Sim은 물리적으로 성공한 `PREPARE_INSPECT`의 `task_id/pallet_id`와 검사 컨텍스트의 `inspection_command_id`에 맞는 검출만 저장한다. 컨텍스트보다 검출이 먼저 오면 임시 보류 후 대조하고, 새 팔레트 준비·장면 재시작 시 이전 데이터를 지운다. 여섯 슬롯, 영상 시각, 픽셀 범위, 클래스 판정도 검증한다. `STORED`는 저장 완료만 뜻하며 Cull을 시작시키지 않는다.

검사 시에는 `PREPARE_INSPECT` 성공 → `MOVE_TO_INSPECT`의 `INSPECT_READY` 물리 상태 및 성공 결과 → 같은 식별자의 `/sim_task/inspection_context`와 `/inspection/command` → 새 영상 시각의 `/inspection/detections_2d` (`valid_for_cull=true`) → `/sim_task/inspection_data_status`의 `STORED` → `/inspection/result`의 같은 명령 terminal 결과를 확인한다. ROS 전달 순서에 따라 검출과 컨텍스트·결과의 도착 순서는 달라질 수 있으므로 각 메시지의 식별자와 시각으로 대조한다. 필요하면 `ros2 topic echo /rgb --field header`로 영상 시각을 관찰한다.

## Task Manager 없이 Sim Task 명령 시험

Task Manager를 **실행하지 않고** 새 장면의 Sim Executor, Nav2, Navigation Executor를 실행한다. `/sim_task/status`, `/sim_task/result`, `/navigation/result`를 먼저 구독하고, 각 명령의 실제 `SUCCEEDED` terminal result를 본 다음 명령을 보낸다. 같은 `task_id`를 쓰고 명령마다 고유 `command_id`를 사용한다. Navigation만 `TaskCommand`이며 나머지는 String/JSON이다. Inspection Executor는 이 일곱 명령의 물리 시험에는 필수가 아니다.

```bash
ros2 topic pub --once --max-wait-time-secs 15 /sim_task/command std_msgs/msg/String \
  '{data: "{\"task_id\":\"UNIT-POST-PLACE\",\"command_id\":\"UNIT-POST-PLACE-CMD-001\",\"operation\":\"TRANSFER\",\"recipe_id\":\"RACK_REARRANGE_01\"}"}'

ros2 topic pub --once --max-wait-time-secs 15 /sim_task/command std_msgs/msg/String \
  '{data: "{\"task_id\":\"UNIT-POST-PLACE\",\"command_id\":\"UNIT-POST-PLACE-CMD-002\",\"operation\":\"PICK_HARVEST\",\"recipe_id\":\"HARVEST_RACK_L1\",\"pallet_id\":\"PALLET_001\",\"source\":\"RACK_L1\",\"destination\":\"CARRY\"}"}'

ros2 topic pub --once --max-wait-time-secs 15 /navigation/command smart_farm_interfaces/msg/TaskCommand \
  '{task_id: UNIT-POST-PLACE, command_id: UNIT-POST-PLACE-CMD-003, operation: NAVIGATION, destination: FEEDER_DOCK}'

ros2 topic pub --once --max-wait-time-secs 15 /sim_task/command std_msgs/msg/String \
  '{data: "{\"task_id\":\"UNIT-POST-PLACE\",\"command_id\":\"UNIT-POST-PLACE-CMD-004\",\"operation\":\"PLACE_INSPECT\",\"recipe_id\":\"PLACE_AT_INSPECTION\",\"pallet_id\":\"PALLET_001\",\"source\":\"CARRY\",\"destination\":\"INSPECT_STATION\"}"}'

ros2 topic pub --once --max-wait-time-secs 15 /sim_task/command std_msgs/msg/String \
  '{data: "{\"task_id\":\"UNIT-POST-PLACE\",\"command_id\":\"UNIT-POST-PLACE-CMD-005\",\"operation\":\"CONVEY_TO_INSPECT\",\"recipe_id\":\"CONVEY_TO_INSPECT\",\"pallet_id\":\"PALLET_001\",\"source\":\"INSPECT_STATION\",\"destination\":\"INSPECT_STOP\"}"}'

ros2 topic pub --once --max-wait-time-secs 15 /sim_task/command std_msgs/msg/String \
  '{data: "{\"task_id\":\"UNIT-POST-PLACE\",\"command_id\":\"UNIT-POST-PLACE-CMD-006\",\"operation\":\"PREPARE_INSPECT\",\"recipe_id\":\"PREPARE_INSPECT\",\"pallet_id\":\"PALLET_001\",\"source\":\"INSPECT_STOP\",\"destination\":\"INSPECT_WORK_POS\"}"}'

ros2 topic pub --once --max-wait-time-secs 15 /sim_task/command std_msgs/msg/String \
  '{data: "{\"task_id\":\"UNIT-POST-PLACE\",\"command_id\":\"UNIT-POST-PLACE-CMD-007\",\"operation\":\"MOVE_TO_INSPECT\",\"recipe_id\":\"MOVE_TO_INSPECT\",\"pallet_id\":\"PALLET_001\",\"source\":\"INSPECT_WORK_POS\",\"destination\":\"INSPECT_CAMERA_POSE\"}"}'
```

마지막 세 명령은 각각 직전 실제 성공 결과를 확인한 뒤 입력한다. 음성 시험은 새 장면에서 `PREPARE_INSPECT`를 먼저 보내 `FAILED/INVALID_STATE`를 확인할 수 있다. 이 경우 정상 시험 전 장면을 다시 시작한다. 허위 성공 결과는 발행하지 않는다.

## 실패 시 확인 순서

1. `/cycle/status`가 `PREFLIGHT`에서 멈추면 Task Manager의 `PREFLIGHT failed`와 세 Executor의 `/.../status`를 본다. Inspection이 `WAITING_IMAGE`면 호스트와 컨테이너의 `/rgb` 수신, `ROS_DOMAIN_ID`, `FASTDDS_BUILTIN_TRANSPORTS`를 확인한다. 모델 오류는 Vision 터미널과 `/models/best.pt` 마운트를 본다.
2. Sim 명령·결과가 없거나 실패하면 `/sim_task/command`, `/sim_task/status`의 `phase/detail`, `/sim_task/result`의 `task_id/command_id/pallet_id/status/reason/reached_station`, Isaac의 `Sim command queued`, `Sim result published`, 물리 예외 로그를 본다. `PICK_HARVEST`의 Task Manager 벽시계 제한은 400초다. 제한 뒤 늦게 온 결과는 `Result ignored: no active command`로 기록되며 그 사이클은 복구되지 않는다.
3. Navigation 실패는 Navigation의 `goToPose`·Nav2 액션 서버 로그, `/feeder_dock/status`·`/feeder_dock/result`, `/clock`, `/tf`, `/chassis/odom`, `/scan`을 본다.
4. Place가 멈추면 Isaac의 `WAIT_BASE_SETTLED`, `ARM_PLACE`, 포크 인출 및 TurnTable 로그를 본다. `[대기] 팔 베이스 정지를 기다리는 중`은 BaseWatcher의 주기적 안내이므로 실제 대기 여부는 `/sim_task/status.phase`로 확인한다. Place의 해당 대기는 물리 시간 15초 제한이 있고 Transfer/Pick의 `WAIT_BASE_SETTLE`에는 내부 제한이 없다.
5. Convey가 지연되면 Isaac의 벨트 감지·카메라 앞 정지·정체 로그와 `/sim_task/status`의 `zone/still`을 본다. Prepare는 지그의 프레임 이송·밀기 실패 로그, `station_out/station_events.json`의 `PUSH_DONE_PREPARED`, 월드 x/y·`seat_error_after_push_mm`·트레이 속도를 본다. Move는 Isaac의 `[비전] 검사 자세로 이동`, `INSPECT_POSE_READY` 이벤트, `/sim_task/status`의 `MOVE_TO_INSPECT/MOVING`, terminal result를 본다. `PALLET_DETECTED` 직후 다음 명령이 나왔다면 Task Manager의 결과 처리 경로를 점검한다.
6. Inspect 실패는 Vision의 `IMAGE_TIMEOUT`, `INSPECTION_FAILED`, `UNKNOWN_SLOT`, `INVALID_IMAGE_METADATA`, `/inspection/debug_image`, `/inspection/detections_2d`를 본다. Sim의 `Inspection detections rejected`와 `/sim_task/inspection_data_status`, Task Manager의 `Result ignored`·`Command timeout`을 대조한다.

bag의 terminal result와 Isaac 물리 화면·로그가 실기동 판정 근거다. 정적·단위 검증만으로 도킹, 이송, 검사 영상의 성공을 주장하지 않는다.
