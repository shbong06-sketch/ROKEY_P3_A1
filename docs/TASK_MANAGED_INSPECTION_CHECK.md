# Task Manager 전체 공정 확인: TRANSFER → COMPLETE (Ubuntu)

`DEMO_HARVEST_01`의 실제 명령과 terminal result를 확인하는 절차다. Task Manager는 `TRANSFER → PICK_HARVEST → NAVIGATION → PLACE_INSPECT → CONVEY_TO_INSPECT → PREPARE_INSPECT → MOVE_TO_INSPECT → INSPECT → (불량이 있으면 CULL) → RECHECK → RELEASE_INSPECT → CONVEYOR_OUT`을 순서대로 발행한다. 각 전이는 활성 명령과 일치하는 `SUCCEEDED` terminal result로만 진행한다. `PALLET_DETECTED` 등 상태 알림만으로 전이하지 않는다.

## 준비 및 외부 자산

- 모든 호스트 ROS 터미널과 Vision 컨테이너에 같은 `ROS_DOMAIN_ID`를 설정한다. 두 PC를 사용하면 DDS로 `/clock`, `/tf`, `/chassis/odom`, `/scan`, `/rgb`가 필요한 프로세스에 도달해야 한다. Isaac 터미널에는 시스템 ROS를 source하지 않는다.
- Git 외부 장면 `cobot3_ws/isaacpjt/smart_farm/scenes/Collected_smartfarm_v014/Collected_smartfarm_v014_room_core_cabbage.usd`와 장면이 참조하는 로봇·재질 자산을 준비한다. 현재 브랜치의 `cobot3_ws/src/smart_farm_navigation/maps/Collected_smartfarm_v014.yaml`·PNG와 `cobot3_ws/src/smart_farm_vision/resource/best.pt`는 Git에 포함되어 있으나, 대상 PC의 설치 overlay와 Vision 이미지에 반영됐는지 확인한다.
- Inspection Executor는 `compose.vision.yaml`의 별도 GPU 컨테이너에서 실행한다. 컨테이너에 `torch`, `ultralytics`, OpenCV, `cv_bridge`가 필요하다. 통합 경로의 Isaac 검사 스테이션은 자체 YOLO worker를 시작하지 않으므로 Isaac Python에 `ultralytics`를 설치할 필요가 없다. 호스트에서 Vision 노드를 직접 실행하면 호스트 Python의 `torch` 부재로 중단될 수 있다.
- Docker를 실행하는 각 호스트(VM과 RTX 5080 노트북 모두)에 NVIDIA 드라이버와 Container Toolkit이 필요하다. `could not select device driver "nvidia"`가 나오면 [NVIDIA Container Toolkit 설치 안내](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html)에 따라 해당 호스트의 Docker GPU runtime을 구성한다. `service "vision" is not running`은 보통 그 시작 실패의 후속 메시지다. 현재 Vision Dockerfile의 PyTorch/CUDA 빌드가 대상 GPU에서 동작하는지도 실제 CUDA 연산으로 확인한다.

아래 명령은 저장소가 `~/ROKEY_P3_A1`, Isaac Sim이 `~/isaacsim`에 설치된 Ubuntu 호스트를 기준으로 한다. 경로가 다르면 해당 두 경로만 바꾼다. 모든 ROS 터미널에서 **이번에 빌드한 동일한 overlay**를 source한다. 저장소 루트의 `install/`과 `cobot3_ws/install/`을 혼용하면 이전 실행 코드가 로드될 수 있다. 코드 변경 후에는 기존 프로세스를 종료하고 새 장면·노드로 시작한다.

```bash
cd ~/ROKEY_P3_A1/cobot3_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-up-to smart_farm_manager smart_farm_navigation smart_farm_vision --symlink-install
source install/setup.bash
ros2 pkg prefix smart_farm_manager
ros2 pkg prefix smart_farm_navigation
```

Vision 코드나 설정이 바뀌었다면 저장소 루트에서 이미지를 다시 빌드하고 컨테이너를 재생성한다. `compose.vision.yaml`은 `ROS_DOMAIN_ID`를 저장소 루트의 `.env` 또는 셸 환경에서 받는다. `down -v`는 `.env`를 삭제하지 않지만, 재생성 전에 도메인 값이 호스트 ROS 터미널과 같은지 확인한다. `FASTDDS_BUILTIN_TRANSPORTS=UDPv4`도 컨테이너 환경에 적용되어야 한다.

```bash
cd ~/ROKEY_P3_A1
cat .env
test -f cobot3_ws/src/smart_farm_vision/resource/best.pt
sudo docker compose -f compose.vision.yaml build vision
sudo docker compose -f compose.vision.yaml up -d --force-recreate vision
sudo docker compose -f compose.vision.yaml exec vision /entrypoint.sh printenv ROS_DOMAIN_ID FASTDDS_BUILTIN_TRANSPORTS
sudo docker compose -f compose.vision.yaml exec vision /entrypoint.sh python3 -c 'import torch; print(torch.__version__, torch.version.cuda, torch.cuda.get_device_name(0)); print((torch.ones(1, device="cuda") + 1).item())'
```

`up -d`는 컨테이너의 기본 `sleep infinity`만 시작한다. 아래 Inspection launch를 별도로 실행해야 한다. RTX 5080에서 CUDA 연산이 실패하면 현재 Dockerfile의 PyTorch/CUDA 빌드를 GPU에 맞게 갱신·재빌드한 뒤 다시 확인한다.

## 별도 프로세스 기동

아래 항목을 각각 별도 터미널에서 실행한다. 2~5번 및 관찰용 터미널에서는 매번 `source /opt/ros/jazzy/setup.bash`와 `source ~/ROKEY_P3_A1/cobot3_ws/install/setup.bash`를 실행하고 동일한 `ROS_DOMAIN_ID`를 설정한다. `run_flow.py`, PowerShell 올인원 도구, mock executor는 이 실기동 경로에 사용하지 않는다. Nav2가 `active`가 되지 않았는데 Navigation Executor가 `READY`라고 표시될 수 있으므로, 5번 이전에 lifecycle 상태를 따로 확인한다.

1. **Isaac Sim / Sim Executor** — 시스템 ROS를 source하지 않은 터미널에서 저장소 루트로 이동한다. `--no-vision-station`은 사용하지 않는다.

   ```bash
   cd ~/ROKEY_P3_A1
   ~/isaacsim/python.sh cobot3_ws/isaacpjt/smart_farm/runtime/standalone_app.py \
     --autoplay --scene cobot3_ws/isaacpjt/smart_farm/scenes/Collected_smartfarm_v014/Collected_smartfarm_v014_room_core_cabbage.usd
   ```

   원격 화면이 필요하면 같은 명령에 `--livestream`과 접속 가능한 서버 주소를 준다.

   ```bash
   cd ~/ROKEY_P3_A1
   ~/isaacsim/python.sh cobot3_ws/isaacpjt/smart_farm/runtime/standalone_app.py \
     --livestream --autoplay \
     --scene cobot3_ws/isaacpjt/smart_farm/scenes/Collected_smartfarm_v014/Collected_smartfarm_v014_room_core_cabbage.usd \
     --/app/livestream/publicEndpointAddress=서버_IP --/app/livestream/port=49100
   ```

2. **Nav2** — `ros2 launch smart_farm_navigation nav2.launch.py scan_mode:=auto use_rviz:=false record:=true`. `record:=true`는 Nav2의 `/clock`, TF, 지도, costmap, 계획과 주행 토픽을 별도 bag에 남긴다. 설치된 launch의 기본값은 `false`이다.
3. **Navigation Executor** — `ros2 launch smart_farm_navigation navigation_node.launch.py`
4. **Inspection Executor** — 저장소 루트에서 다음 명령을 실행한 터미널을 유지한다.

   ```bash
   cd ~/ROKEY_P3_A1
   sudo docker compose -f compose.vision.yaml exec vision \
     /entrypoint.sh ros2 launch smart_farm_vision object_detection.launch.py \
     config_file:=/config/object_detection.yaml
   ```

5. **Task Manager** — `ros2 launch smart_farm_manager task_manager.launch.py`

Isaac 화면에서 **Stop → Play**를 누르면 `standalone_app.py`는 같은 프로세스에서 원본 USD를 다시 열고 로봇·팔레트·포기·검사 지그를 초기 배치로 재구성한다. Stop 중 `/sim_task/status`는 `STARTING/SCENE_RESET`, Play 뒤 초기화가 끝나면 `READY/IDLE`이어야 한다. 터미널의 `[장면] Stop → Play: 원본 USD를 다시 엽니다.`와 `[READY]`를 확인한다. 이 기능은 Sim Executor의 장면 복원이며, `COMPLETE/ERROR`인 Task Manager와 Nav2의 위치 추정까지 초기화하지는 않는다. 전체 공정을 다시 시작할 때는 해당 ROS 프로세스도 새 사이클용으로 재시작한다.

화면 없이 이 경로만 시험하려면 Isaac 터미널에서 `--verify-stop-play`를 붙여 실행한다. 이 옵션은 `PALLET_001`을 잠시 옮기고 Stop 상태를 120 Kit update 동안 유지한 뒤 Play하여 원래 월드 위치와 리프트 보정을 확인하고 종료한다. 스트리밍 구성까지 확인할 때는 `--livestream --verify-stop-play`를 함께 준다. 성공 로그는 `[RESET_CHECK] PASS`이다. 검증 옵션이므로 실제 시연 명령에는 붙이지 않는다.

PREFLIGHT는 Sim Task, Navigation, Inspection의 최근 `READY` heartbeat를 모두 요구한다. Inspection은 모델 로드와 `/rgb`의 유효한 영상 수신 후 `READY`가 된다. 시작 전에 다음을 확인한다.

```bash
ros2 topic echo --once /sim_task/status
ros2 topic echo --once /navigation/status
ros2 topic echo --once /inspection/status
ros2 topic hz /rgb
ros2 action list
ros2 lifecycle get /map_server
ros2 lifecycle get /amcl
ros2 lifecycle get /planner_server
ros2 lifecycle get /bt_navigator
```

네 lifecycle 노드는 모두 `active`여야 한다. `/map_server`가 지도 파일을 읽었다는 로그만으로는 `/map` 발행이 확인되지 않는다. 06·07번 실행처럼 AMCL 전이가 멈추면 costmap에 `no map received`가 반복되고 `bt_navigator`가 목표를 거부할 수 있으므로, 이 상태에서는 `/start_cycle`을 호출하지 않는다.

## Bag 기록과 자동 실행

별도 ROS 터미널에서 사이클 시작 **전**에 bag을 기록한다. `/tmp`는 재부팅 후 사라질 수 있으므로 저장소 아래 영구 경로를 사용한다. 실행마다 `full_cycle_01`, `full_cycle_02`처럼 새 이름을 준다. `/inspection/debug_image`는 RECHECK의 ROI·잔존 불량을 확인하기 위해 포함한다. `/rgb` 원본 영상은 용량이 크므로 기본 목록에서 제외한다. 원본이 필요한 실행에만 별도 기록한다.

```bash
mkdir -p ~/ROKEY_P3_A1/results/bags
ros2 bag record -o ~/ROKEY_P3_A1/results/bags/full_cycle_01 \
  /cycle/status /sim_task/command /sim_task/result /sim_task/status \
  /navigation/command /navigation/result /navigation/status \
  /feeder_dock/status /feeder_dock/result \
  /sim_task/inspection_context /sim_task/inspection_data_status \
  /inspection/command /inspection/status /inspection/detections_2d /inspection/result \
  /inspection/debug_image /clock
```

세 Executor의 `READY`를 확인한 후 다른 ROS 터미널에서 시작한다.

```bash
ros2 service call /start_cycle smart_farm_interfaces/srv/StartCycle "{scenario_id: DEMO_HARVEST_01}"
```

`/start_cycle`은 실행당 **한 번만** 호출한다. 실패 후 같은 장면에서 재호출하지 않는다. 기록을 종료한 뒤 `ros2 bag info ~/ROKEY_P3_A1/results/bags/full_cycle_01`로 실제 저장 토픽과 건수를 확인한다. Nav2의 보조 bag은 `~/.ros/smart_farm_navigation/bags/nav2_<시각>/`에 저장된다.

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
| 8 | `/inspection/command` `INSPECT` (`CMD-008`, `PALLET_001`) | `SUCCEEDED`이면 불량 슬롯을 저장. 불량이 있으면 `CULL`, 없으면 `RECHECK`로 전이 |
| 9 | `/sim_task/command` `CULL` (`CMD-009`, 불량 슬롯이 있을 때만) | 실제 배출과 검사 자세 복귀를 확인한 `SUCCEEDED` → `RECHECK`; 일부라도 실패하면 `FAILED` |
| 10 | `/inspection/command` `RECHECK` (`CMD-010`, 불량이 없어서 CULL을 건너뛰면 `CMD-009`) | 명령 후 새 프레임에서 잔존 불량·UNKNOWN이 없는 `SUCCEEDED` → `RELEASE_INSPECT` |
| 11 | `/sim_task/command` `RELEASE_INSPECT` (앞 단계 다음 ID) | 트레이가 벨트 줄로 복귀하고 지그가 후퇴한 `SUCCEEDED`, `reached_station=INSPECT_STOP` → `CONVEYOR_OUT`; 벨트 배출은 아직 시작하지 않음 |
| 12 | `/sim_task/command` `CONVEYOR_OUT` (앞 단계 다음 ID) | `inspection_done()` 후 컨베이어 배출 구역 `GONE` 도착을 확인한 `SUCCEEDED`, `reached_station=PACK_OUT` → `COMPLETE` |

## 세 경로의 합격 기준

| 경로 | bag에서 확인할 명령·결과 | 물리 화면과 이벤트에서 확인할 사항 |
|---|---|---|
| 정상 판정 | `INSPECT SUCCEEDED`와 빈 `defect_slots` → `CULL` 명령 없음 → `RECHECK`의 빈 `target_slots`와 `SUCCEEDED` → `RELEASE_INSPECT SUCCEEDED` → `CONVEYOR_OUT SUCCEEDED` → `/cycle/status`의 `COMPLETE/SUCCEEDED` | 여섯 칸 정상 판정, 지그가 트레이를 검사 벨트로 되돌린 뒤 팔레트가 배출 구역을 통과. `PUSH_BACK_DONE_HELD`가 배출 시작보다 먼저 기록됨 |
| 불량 판정 | `INSPECT SUCCEEDED`의 불량 슬롯 = `CULL.target_slots` = 실제 배출 확인된 `CULL.completed_units`; 이어 `RECHECK SUCCEEDED`의 제거 슬롯은 `REMOVED`, 나머지는 `NORMAL`; 이후 `RELEASE_INSPECT` → `CONVEYOR_OUT` → `COMPLETE/SUCCEEDED` | 각 대상 포기가 상자 안으로 들어가고 지그가 검사 자세로 돌아옴. `CULL_DONE_*`, `PUSH_BACK_DONE_HELD`, `CONVEYOR_OUT_DONE` 순서 확인 |
| 실패 | 활성 명령의 `FAILED`·`TIMEOUT` 또는 CULL 데이터의 `CULL_COORDINATE_MISSING` 등 검증 실패 → `/cycle/status`의 `ERROR` (`FAILED` 또는 `RESET_REQUIRED`) | ERROR 시점 뒤 Task Manager가 다음 `/.../command`를 발행하지 않음. 자동 지그 복귀·배출도 시작되지 않음 |

정상 경로도 재검사를 거친다. `RECHECK`는 정상 슬롯 전체를 새 프레임에서 다시 확인하므로, 정상 판정이라는 이유만으로 복귀·배출로 건너뛰지 않는다. 09번 실행처럼 CULL의 물리 성공과 RECHECK 영상 판정이 다르면 `ERROR`가 맞다. `UNKNOWN_SLOT`이 하나라도 있으면 잔존 불량과 함께 기록하되 최종 사유는 `UNKNOWN_SLOT`이다.

### 식별자와 전이 추적

1. `/start_cycle` 응답의 `task_id`를 하나 기록한다. `/cycle/status.task_id` 및 모든 명령·결과의 `task_id`가 이 값인지 확인한다.
2. 각 `/.../command`의 `command_id`, `operation`, `pallet_id`를 적고 같은 executor의 `/.../result`와 대조한다. 성공 전이에는 활성 명령과 같은 `task_id`·`command_id`·`operation`의 terminal `SUCCEEDED`가 필요하다. Sim 명령의 팔레트 작업은 JSON 결과의 `pallet_id`도 같아야 한다. `TRANSFER`와 `NAVIGATION` 명령의 `pallet_id`는 비어 있다.
3. Navigation·Inspection의 `TaskResult.msg`에는 `pallet_id` 필드가 없다. 이 둘은 같은 ID의 명령에 기록된 `pallet_id`를 조인해 추적한다. `INSPECT`와 `RECHECK`의 명령 `pallet_id`는 `PALLET_001`; CULL용 `/inspection/detections_2d`에는 별도의 `pallet_id`와 최초 검사 `inspection_command_id`가 있다.
4. `/cycle/status.active_command_id`가 현재 명령 ID인지 확인한다. `SUCCEEDED` 뒤 다음 명령으로 바뀌어야 하며, `ERROR` 뒤에는 빈 값이어야 한다. 상태 알림(`/.../status`)이나 검출 데이터만으로 단계가 바뀌면 안 된다.
5. 실패 사유는 `/.../result.reason`과 마지막 `/cycle/status.reason`을 함께 남긴다. Task Manager의 벽시계 제한은 자체 `RESULT_TIMEOUT`으로 끝나며 executor에서 뒤늦게 온 결과는 현재 사이클을 되살리지 않는다. 이 제한은 Isaac의 활성 물리 명령을 취소하지 않으므로, timeout이 발생했다면 장면을 정지·초기화한 뒤 새 실행을 시작한다.

bag 기록이 끝난 뒤 `ros2 bag info`로 위 7개 명령·결과 토픽과 `/cycle/status`의 건수를 확인한다. 필요하면 별도 터미널에서 `ros2 topic echo /cycle/status smart_farm_interfaces/msg/CycleStatus`, `ros2 topic echo /inspection/result smart_farm_interfaces/msg/TaskResult`, `ros2 topic echo /sim_task/result std_msgs/msg/String`을 실행해 terminal 결과를 바로 관찰한다. `ros2 bag play`는 실제 Executor가 실행 중인 ROS 도메인에서 사용하지 않는다. 저장된 명령이 다시 물리 실행될 수 있다.

## 컨베이어·지그의 물리 완료 조건

| 명령 | 시작 조건 | 물리 완료 조건 | 주요 실패 사유·제한 |
|---|---|---|---|
| `CONVEY_TO_INSPECT` | 같은 런타임의 `PLACE_INSPECT` 성공, 포크 인출 완료, 팔레트 미운반, 스테이션 `IDLE`; `recipe_id=CONVEY_TO_INSPECT`, `source=INSPECT_STATION`, `destination=INSPECT_STOP` | 대상 팔레트가 컨베이어 `VISION` 영역에서 검출·고정되고 월드 위치의 프레임당 변위 ≤0.002 m가 물리 시간 0.5초 지속 | `INVALID_COMMAND`, `INVALID_STATE`, `NOT_READY`, `PALLET_LOST`, 컨베이어 정체 코드, `CONVEY_TIMEOUT`(물리 120초), `CONVEYOR_FAILED`; Task Manager 벽시계 600초 |
| `PREPARE_INSPECT` | 직전 Convey 성공, 같은 팔레트 `VISION` 고정, 스테이션 `IDLE`; `recipe_id=PREPARE_INSPECT`, `source=INSPECT_STOP`, `destination=INSPECT_WORK_POS` | 지그 진입·하강·밀기 후 트레이 월드 x/y 각각 목표 0.05 m 이내, 선속도 ≤0.05 m/s, 자리 오차 ≤20 mm, 스테이션 `PREPARED`. 기존 올인원처럼 트레이·포기의 강체 물리를 켠 채 유지 | `INVALID_COMMAND`, `INVALID_STATE`, `JIG_FAILED`, `PREPARE_TIMEOUT`(물리 90초); Task Manager 벽시계 450초 |
| `MOVE_TO_INSPECT` | 같은 팔레트의 Prepare 성공, 스테이션 `PREPARED`, 팔레트 고정; `recipe_id=MOVE_TO_INSPECT`, `source=INSPECT_WORK_POS`, `destination=INSPECT_CAMERA_POSE` | 기존 올인원의 검사 자세 모션이 끝나고 20 물리 프레임 안정화 후 스테이션 `INSPECT_READY` | `INVALID_COMMAND`, `INVALID_STATE`, `INSPECT_POSE_FAILED`, `INSPECT_POSE_TIMEOUT`(물리 90초); Task Manager 벽시계 300초 |

`CONVEY_TO_INSPECT/PALLET_DETECTED` 상태가 나와도 `/cycle/status`는 Convey에 머물러야 한다. 세 물리 명령 모두 성공 terminal result가 다음 명령의 유일한 전이 조건이다. 물리 실패·시간 초과 때 임의로 배출하지 않고 `FAILED`를 반환한다. Sim Task가 `ERROR; scene reset required`가 되면 새 장면으로 재시작한다. 통합 경로의 컨베이어 자동 배출 타이머와 스테이션의 팔레트 감지 후 자동 PUSH_IN·검사 시작 경로는 사용하지 않는다.

`conveyor.is_locked()`는 검사 정지 때 세운 플래그로, 지그 진입을 위해 강체 물리를 다시 켜도 `True`로 남는다. 따라서 이 값만으로 물리 고정 상태를 판정할 수 없다. `PREPARE_INSPECT`에서는 지그가 밀기 전에 트레이·포기의 강체 물리를 다시 켜고, 완료 뒤에도 재잠그지 않는다. 검사와 CULL 중 포기는 그리퍼 접촉으로 실제로 움직일 수 있다.

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

## CULL의 좌표·완료 계약

`INSPECT`가 `SUCCEEDED`이고 `defect_slots`가 비어 있지 않을 때, Task Manager는 같은 검사 ID의 `/sim_task/inspection_data_status=STORED`까지 확인한 뒤 `CULL`을 발행한다. `STORED`만으로 공정이 전이되지 않는다. 저장 거부 또는 벽시계 15초 내 미수신은 `INSPECTION_DATA_REJECTED`·`INSPECTION_DATA_TIMEOUT`으로 CULL 발행 전에 실패한다. `target_slots`는 그 불량 슬롯 전체이며, `recipe_id=CULL_DEFECT_SLOTS`, `pallet_id=PALLET_001`, `source=INSPECT_STATION`, `destination=INSPECT_STATION`이다. 불량이 없으면 CULL 없이 `RECHECK`로 진행한다.

Sim은 저장된 `/inspection/detections_2d`의 `task_id`, `pallet_id`, `inspection_command_id`, 여섯 슬롯 판정과 CULL의 `target_slots`를 대조한다. 대상은 판정된 불량 슬롯 집합과 정확히 같아야 하고, 각 슬롯에 유효한 검출이 있어야 한다. 불일치나 데이터 누락은 물리 동작 전에 `INSPECTION_ID_MISMATCH`, `INSPECTION_DATA_MISSING`, `CULL_TARGET_MISMATCH`, `CULL_COORDINATE_MISSING`으로 실패한다.

첫 통합의 파지 목표 **XY는 픽셀 좌표가 아니다**. 검사 준비 때 보관한 각 포기 위치의 트레이 로컬 오프셋을 현재 트레이 월드 자세에 적용한 다음 비전 M0609 base 좌표로 바꾼다. 목표 높이는 트레이 평면의 Z + `AIM_ABOVE_TRAY`(0.090 m)를 기준으로 계획하고, 실제 파지는 `GRIP_ABOVE_TRAY`(0.078 m) 높이를 사용한다. 기존 올인원의 `cull_motion.py` 계획과 RMPflow 실행기를 그대로 사용한다. 영상에는 depth가 없으므로 검출 픽셀 중심으로 3D 파지점을 직접 만들지 않는다. 진단용 `vision_to_pick_xy_mm`만 USD 카메라의 focal length·horizontal aperture, 현재 카메라 월드 자세, 트레이 위의 가정 평면(Z + 0.090 m)으로 환산해 기하 목표와 비교한다. 원본 영상 시각의 카메라 자세를 별도로 저장하지 않으므로 이 수치는 파지 목표나 정밀 보정값으로 사용하지 않는다.

각 대상 포기는 `LIFT` 뒤 실제 상승량이 최소 50 mm인지 확인하고, `RELEASE` 뒤 45 물리 프레임을 기다려 머리 중심이 배출 상자의 3D 경계 안에 있는지 확인한다. 이 확인을 통과한 슬롯만 `completed_units`에 추가한다. 리프트 뒤 포기가 따라오지 않으면 `PICK_FAILED`, 상자 안에 내려놓지 못하면 `DROP_FAILED`, 다른 물리 오류는 `CULL_FAILED`, 물리 180초 제한은 `CULL_TIMEOUT`으로 확인된 슬롯만 포함한 `FAILED` 결과를 보내고 자동 재검사·배출은 시작하지 않는다. Task Manager의 CULL 벽시계 제한은 900초다.

Ubuntu 재시험에서는 새 장면과 다시 빌드한 세 ROS 패키지·Vision 컨테이너로 위 별도 프로세스를 시작하고 bag을 먼저 기록한다. `/inspection/result`의 최초 `defect_slots`와 `/sim_task/command`의 `CULL.target_slots`가 같은지, `/sim_task/status`가 `CULL/CULL`·`CULL/HOME`을 거쳐 결과를 내는지, `/sim_task/result.completed_units`가 실제 상자 안에 들어간 슬롯만 담는지 확인한다. Isaac 로그의 `[솎아내기] SLOT_XX 버림 성공/실패`, `[CULL:검증] 대상 상승량`, `station_out/station_events.json`의 `CULL_DONE_*`도 대조한다.

`RECHECK`는 최초 INSPECT 이후 별도 `/inspection/command`이며 명령 수신 뒤의 새 `/rgb` 영상 시각만 사용한다. `target_slots`는 CULL로 제거한 슬롯이다. 해당 칸에 검출이 없으면 `REMOVED` 정상 상태이며, 다른 칸은 `dark_green=NORMAL`이어야 한다. 제거한 칸에 불량이 남으면 `FAILED/DEFECT_REMAINS`, 빈칸이어야 할 곳에서 정상 포기가 검출되거나 다른 칸이 누락·중복되면 `FAILED/UNKNOWN_SLOT`, 영상이 없으면 `FAILED/IMAGE_TIMEOUT`이다. 정상 판정으로 CULL을 건너뛰면 `target_slots=[]`이고 여섯 칸 모두 NORMAL이어야 한다. RECHECK의 `/inspection/detections_2d`는 `valid_for_cull=false`이며 Sim의 최초 검사 데이터 저장소에 다시 저장하지 않는다.

`RELEASE_INSPECT`는 `recipe_id=RELEASE_INSPECT`, `source=INSPECT_WORK_POS`, `destination=INSPECT_STOP`이다. 지그가 트레이를 벨트 줄로 되돌리고 프레임이 올라간 뒤 트레이 월드 x/y가 목표에서 각각 0.05 m 이내, 선속도가 0.05 m/s 이하이고 스테이션 `RELEASED`일 때 `SUCCEEDED/reached_station=INSPECT_STOP`을 반환한다. 물리 제한 90초는 `RELEASE_TIMEOUT`, 복귀 검증 실패는 `RELEASE_FAILED`이다. 이 단계에서는 `inspection_done()`을 호출하지 않는다. `CONVEYOR_OUT`은 `recipe_id=CONVEY_TO_PACK_OUT`, `source=INSPECT_STOP`, `destination=PACK_OUT`이며 RELEASE 성공 후에만 `inspection_done()`을 호출한다. 컨베이어의 실제 `GONE` 구역 도착을 확인하면 `SUCCEEDED/reached_station=PACK_OUT`, 물리 제한 120초는 `CONVEYOR_TIMEOUT`, 구역 이탈은 `PALLET_LOST`, 정체는 컨베이어 고장 코드로 `FAILED`가 된다. 실패·시간 초과 시 가로줄기 롤러를 정지한다. 각 명령의 Task Manager 벽시계 제한은 순서대로 300초, 450초, 600초이다.

Ubuntu에서는 `/inspection/result`의 RECHECK `command_id`와 새 `/inspection/detections_2d.header.stamp`, `REMOVED`/`NORMAL` 상태를 먼저 확인한다. 다음 `/sim_task/result`의 RELEASE 성공과 `station_out/station_events.json`의 `PUSH_BACK_DONE_HELD`를 확인하고, 그 이전에 `/sim_task/command`의 CONVEYOR_OUT 또는 컨베이어 출발이 없어야 한다. 마지막으로 CONVEYOR_OUT 결과의 `reached_station=PACK_OUT`과 `/cycle/status`의 `COMPLETE/SUCCEEDED`를 확인한다. RECHECK가 실패했거나 RELEASE가 실패·시간 초과되면 뒤따르는 명령이 없어야 한다.

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

## development 대비 PR 검토 지점

- Task Manager는 `INSPECT → (불량이면 CULL) → RECHECK → RELEASE_INSPECT → CONVEYOR_OUT → COMPLETE`를 수행한다. `CULL`을 건너뛰어도 `RECHECK`와 물리 복귀·배출을 수행한다. 이전 `CONVEYOR_OUT` 직접 전이와 충돌하는 테스트·문서를 확인한다.
- Nav2는 새 `nav2.launch.py`, v014 지도, `stations.yaml`, `destinations_nav2.yaml`을 사용한다. PR에서 지도와 장면의 원점·축·도킹 좌표, AMCL 초기 자세, `/clock`·TF·스캔, map server와 navigator의 `active` 상태를 확인한다. 지도 파일 로드 로그만으로 활성화를 판정하지 않는다.
- v014 검사 장면 본체는 Git에 없고, `development` 대비 로봇·포크·랙 자산에는 삭제·이동과 팔레트 트레이 USD 수정이 있다. PR 검토 PC에서 USD 참조가 모두 열리는지, 비전 M0609·포크·그리퍼와 트레이 강체가 통합 장면에 일치하는지 확인한다. 작은 `smart_farm_nav2_01*.usd` 파일이 v014 검사 장면을 대체하지는 않는다.
- Vision 패키지·Docker 이미지·`best.pt`·`object_detection.yaml`은 `development` 대비 새 구성이다. 클래스 판정(`dark_green=NORMAL`, `yellow/brown=DEFECT`), 여섯 ROI, CUDA 장치, `/rgb`, 모델 버전과 컨테이너 코드가 같아야 한다. 호스트 소스 변경 후에는 Docker 이미지를 재빌드해야 한다.
- Sim Executor는 관리형 검사 스테이션에서 감지 후 자동 PUSH_IN·검사·배출을 시작하지 않는다. `RELEASE_INSPECT` 성공 전에는 `conveyor.inspection_done()` 호출이 없고, `CONVEYOR_OUT` 명령 후에만 배출한다. 물리 실패·시간 초과 시 팔레트를 임의로 배출하지 않는지 확인한다.

bag의 terminal result와 Isaac 물리 화면·로그가 실기동 판정 근거다. 정적·단위 검증만으로 도킹, 이송, 검사 영상의 성공을 주장하지 않는다.
