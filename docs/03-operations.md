# 통합 시연 운영 가이드 — v0.2.0

이 문서는 v015 장면에서 `DEMO_HARVEST_01` 전체 공정을 실행하고 결과를 확인하는 절차다. 구성과 책임은 [아키텍처](01-architecture.md), 메시지와 단계별 성공 계약은 [인터페이스](02-interfaces.md)를 따른다. 명령·제한 시간의 기준은 [시나리오 코드](../cobot3_ws/src/smart_farm_manager/smart_farm_manager/scenario.py)다.

아래 명령은 저장소가 `~/ROKEY_P3_A1`, Isaac Sim이 `~/isaacsim`에 설치된 Ubuntu 환경을 예로 든다. 경로와 `ROS_DOMAIN_ID`는 실행 환경에 맞게 바꾼다. Isaac과 ROS 노드를 다른 PC에서 실행해도 되지만, 필요한 ROS 2 토픽이 같은 DDS 도메인에서 통신해야 한다.

## 1. 자산과 환경 준비

- ROS 2 Jazzy, Nav2, `colcon`, Isaac Sim 5.1 Standalone, Docker Compose, NVIDIA GPU·드라이버·Container Toolkit을 준비한다. Inspection Executor는 [GPU 컨테이너](../compose.vision.yaml)에서 실행한다.
- Git 외부 v015 장면과 참조 자산을 [장면 자산 문서](04-assets.md)에 따라 `cobot3_ws/isaacpjt/smart_farm/scenes/Collected_smartfarm_v015/`에 풀어 `Collected_smartfarm_v015.usd`가 존재하도록 한다. [Nav2 지도](../cobot3_ws/src/smart_farm_navigation/maps/Collected_smartfarm_v015.yaml)도 v015를 사용한다. `standalone_app.py`에는 다른 장면으로 대체 실행되지 않도록 `--scene`을 명시한다.
- [YOLO 모델](../cobot3_ws/src/smart_farm_vision/resource/best.pt)과 [Vision 설정](../cobot3_ws/src/smart_farm_vision/config/object_detection.yaml)을 확인한다. PatchCore는 이 통합 공정에서 사용하지 않는다.
- 호스트 ROS 터미널과 Vision 컨테이너의 `ROS_DOMAIN_ID`를 동일하게 설정한다. Vision Compose는 셸 환경 또는 저장소 루트 `.env`의 값을 읽는다. 다른 PC를 쓰면 `/clock`, `/tf`, `/chassis/odom`, `/scan`, `/rgb`가 해당 노드에 도달해야 한다.
- Isaac 터미널에는 시스템 ROS 2 환경을 source하지 않는다. 나머지 ROS 터미널에서는 **같은** `cobot3_ws/install` overlay를 사용하고 저장소 루트의 별도 `install`과 혼용하지 않는다.

저장소 루트에서 자산을 확인하고 ROS 패키지를 빌드한다.

```bash
cd ~/ROKEY_P3_A1
test -f cobot3_ws/isaacpjt/smart_farm/scenes/Collected_smartfarm_v015/Collected_smartfarm_v015.usd
test -f cobot3_ws/src/smart_farm_navigation/maps/Collected_smartfarm_v015.yaml
test -f cobot3_ws/src/smart_farm_vision/resource/best.pt
cd cobot3_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-up-to smart_farm_manager smart_farm_navigation --symlink-install
source install/setup.bash
```

Vision 이미지에는 자체 ROS 2 overlay가 들어 있다. Vision 코드·설정이 바뀌었으면 저장소 루트에서 이미지를 다시 빌드한다. `up -d`는 대기 컨테이너만 시작하며 검사 노드는 아래 4번 절차에서 별도로 실행한다.

```bash
cd ~/ROKEY_P3_A1
export ROS_DOMAIN_ID=0  # 다른 ROS 터미널·PC와 같은 값 사용
docker compose -f compose.vision.yaml build vision
docker compose -f compose.vision.yaml up -d --force-recreate vision
docker compose -f compose.vision.yaml ps
```

## 2. 프로세스 기동

아래 순서대로 각각 별도 터미널에서 실행한다. 2~5번과 관찰 터미널에는 먼저 다음 환경을 적용한다. 1번 Isaac 터미널은 동일한 `ROS_DOMAIN_ID`만 설정하고 시스템 ROS를 source하지 않는다.

```bash
export ROS_DOMAIN_ID=0  # 모든 프로세스에서 같은 값 사용
source /opt/ros/jazzy/setup.bash
source ~/ROKEY_P3_A1/cobot3_ws/install/setup.bash
```

1. **Isaac Sim / Sim Task** — 저장소 루트에서 v015 장면을 지정한다.

   ```bash
   cd ~/ROKEY_P3_A1
   export ROS_DOMAIN_ID=0
   ~/isaacsim/python.sh cobot3_ws/isaacpjt/smart_farm/runtime/standalone_app.py \
     --autoplay \
     --scene cobot3_ws/isaacpjt/smart_farm/scenes/Collected_smartfarm_v015/Collected_smartfarm_v015.usd
   ```

2. **Nav2와 도킹 노드** — `ros2 launch smart_farm_navigation nav2.launch.py scan_mode:=auto use_rviz:=false`를 실행한다. 이 launch의 기본 지도는 v015다. `record:=true`를 추가하면 Nav2 진단 bag을 별도로 기록한다.
3. **Navigation Executor** — `ros2 launch smart_farm_navigation navigation_node.launch.py`를 실행한다.
4. **Inspection Executor** — 저장소 루트에서 아래 명령을 실행하고 터미널을 유지한다.

   ```bash
   cd ~/ROKEY_P3_A1
   docker compose -f compose.vision.yaml exec vision \
     /entrypoint.sh ros2 launch smart_farm_vision object_detection.launch.py \
     config_file:=/config/object_detection.yaml
   ```

5. **Task Manager** — `ros2 launch smart_farm_manager task_manager.launch.py`를 실행한다.

## 3. 시작 전 확인과 사이클 실행

Task Manager의 `PREFLIGHT`는 Sim Task, Navigation, Inspection의 최근 `READY` heartbeat를 요구한다. Inspection은 모델 로드와 유효한 `/rgb` 프레임 수신 후 `READY`가 된다. Navigation의 `READY`만으로 Nav2 활성화까지 보장되지 않으므로, 시작 전에 다음을 확인한다.

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

네 Nav2 lifecycle 노드가 모두 `active`이고, 위 상태 토픽 세 개가 `READY`인지 확인한다. 실행 기록이 필요하면 [5절의 bag 기록](#5-기록과-판정)을 먼저 시작한다. 사이클 상태를 관찰할 터미널을 열고, 다른 터미널에서 시작 서비스를 **한 번** 호출한다.

```bash
ros2 topic echo /cycle/status smart_farm_interfaces/msg/CycleStatus
```

```bash
ros2 service call /start_cycle smart_farm_interfaces/srv/StartCycle \
  "{scenario_id: DEMO_HARVEST_01}"
```

응답의 `accepted: true`는 시작 요청 수락만 뜻한다. 최종 성공은 `/cycle/status`의 `state: COMPLETE`, `status: SUCCEEDED`로 확인한다. 응답의 `task_id`를 기록해 해당 실행의 명령·결과를 추적한다.

## 4. 단계별 확인

Task Manager는 `PREFLIGHT` 뒤 아래 순서로 명령을 보낸다. 활성 명령과 같은 `task_id`·`command_id`·`operation`의 `SUCCEEDED` terminal 결과와 필수 성공 필드를 확인해야 다음 단계로 간다. 물리 공정의 Sim JSON 결과는 `pallet_id`도 대조한다. 단계별 필드와 제한 시간은 [인터페이스 계약](02-interfaces.md#공정별-성공-계약)을 참조한다.

| 순서 | 명령·실행 주체 | 운영 중 확인할 결과 |
| ---: | --- | --- |
| 1 | `TRANSFER` · Sim | PALLET_002 L3→L2, PALLET_003 L4→L3 완료 |
| 2 | `PICK_HARVEST` · Sim | PALLET_001 운반 준비, `safe_to_navigate=true` |
| 3 | `NAVIGATION` · Navigation | Nav2 접근과 도킹 후 `reached_station=FEEDER_DOCK` |
| 4 | `PLACE_INSPECT` · Sim | PALLET_001을 검사 컨베이어에 배치 |
| 5 | `CONVEY_TO_INSPECT` · Sim | `INSPECT_STOP`에서 정지 |
| 6 | `PREPARE_INSPECT` · Sim | 지그가 `INSPECT_WORK_POS`에 배치 |
| 7 | `MOVE_TO_INSPECT` · Sim | M0609이 `INSPECT_CAMERA_POSE`에 도달 |
| 8 | `INSPECT` · Inspection | 새 `/rgb` 프레임으로 여섯 슬롯 판정, 불량 슬롯·2D 검출 발행 |
| 9 | `CULL` · Sim | 불량이 있을 때만 대상 슬롯 배출과 전체 `completed_units` 확인 |
| 10 | `RECHECK` · Inspection | 새 프레임에서 제거 슬롯은 `REMOVED`, 나머지는 `NORMAL` |
| 11 | `RELEASE_INSPECT` · Sim | 지그 복귀, `reached_station=INSPECT_STOP` |
| 12 | `CONVEYOR_OUT` · Sim | 배출 완료, `reached_station=PACK_OUT` → `COMPLETE/SUCCEEDED` |

`INSPECT`에서 불량이 없으면 9번 `CULL`만 생략한다. 이때도 `RECHECK → RELEASE_INSPECT → CONVEYOR_OUT`을 수행하므로 이후의 `command_id` 번호는 한 칸 당겨진다. 불량이 있으면 Task Manager가 최초 검출의 `/sim_task/inspection_data_status=STORED`를 확인한 뒤에만 CULL을 보낸다. `STORED` 상태 알림이나 중간 상태 알림만으로 공정이 전이되지는 않는다.

검사 오류를 조사할 때는 `/inspection/detections_2d`의 `inspection_command_id`·`pallet_id`·`slot_states`와 `/sim_task/inspection_data_status`를 대조한다. CULL의 `target_slots`는 최초 검사에서 나온 불량 슬롯 집합과 같아야 한다. 픽셀 좌표는 검출·진단용이고 물리 파지 목표는 트레이 자세와 슬롯 오프셋을 사용한다. 재검사의 검출은 최초 CULL 데이터로 다시 저장하지 않는다.

## 5. 기록과 판정

재현 가능한 실행 기록이 필요하면 **시작 서비스 호출 전** 별도 터미널에서 bag을 기록한다. 출력 디렉터리는 실행마다 새 이름으로 바꾼다. `/rgb`는 용량이 커서 기본 목록에서 제외하고 필요할 때만 추가한다.

```bash
cd ~/ROKEY_P3_A1
mkdir -p results/bags
ros2 bag record -o results/bags/full_cycle_01 \
  /cycle/status /sim_task/command /sim_task/result /sim_task/status \
  /navigation/command /navigation/result /navigation/status \
  /feeder_dock/status /feeder_dock/result \
  /sim_task/inspection_context /sim_task/inspection_data_status \
  /inspection/command /inspection/status /inspection/detections_2d /inspection/result \
  /inspection/debug_image /clock
```

- **정상 판정:** `INSPECT`의 `defect_slots=[]`, CULL 명령 없음, `RECHECK`의 빈 `target_slots`와 성공, `RELEASE_INSPECT`·`CONVEYOR_OUT` 성공, 최종 `COMPLETE/SUCCEEDED`.
- **불량 판정:** `INSPECT.defect_slots` = `CULL.target_slots` = `CULL.completed_units`, 최초 검사 데이터 `STORED`, 새 영상의 `RECHECK` 성공, 복귀·배출 성공, 최종 `COMPLETE/SUCCEEDED`. 배출 대상이 실제 상자에 들어갔는지도 Isaac 화면·로그로 확인한다.
- **실패 판정:** executor의 실패·제한 시간 초과·결과 계약 불일치·검사 데이터 거부 뒤에는 `/cycle/status`가 `ERROR`이고 다음 명령이 없어야 한다. 물리 공정 실패는 `status: RESET_REQUIRED`가 될 수 있다.

기록 후 `ros2 bag info results/bags/full_cycle_01`로 토픽과 건수를 확인한다. 저장한 bag을 실제 executor가 실행 중인 ROS 도메인에서 재생하면 명령이 다시 물리 실행될 수 있으므로 별도 도메인에서 분석한다.

## 6. 장애 확인과 재시작

| 증상 | 먼저 확인할 곳 |
| --- | --- |
| `PREFLIGHT` 실패 | 세 `/.../status`의 `READY`·최근 heartbeat, Vision 모델 로드와 `/rgb`, `ROS_DOMAIN_ID` |
| Nav2가 목표를 거부하거나 AMR이 움직이지 않음 | `/clock`, `/tf`, `/chassis/odom`, `/scan`, Nav2 lifecycle, v015 지도와 장면, `/feeder_dock/result` |
| Sim 공정 지연·실패 | `/sim_task/status`의 `phase`·`detail`, `/sim_task/result.reason`, Isaac 물리 로그 |
| `INSPECT`·`RECHECK` 실패 | Vision 로그, `/inspection/debug_image`, 새 영상 시각, 여섯 슬롯 판정과 `UNKNOWN` 여부 |
| `CULL`이 시작되지 않음 | 최초 검사 결과와 `/sim_task/inspection_data_status`의 `STORED`·`REJECTED`, 검사 ID·팔레트 일치 여부 |
| Task Manager가 `ERROR` | `/cycle/status.reason`, 활성 명령의 terminal 결과, 제한 시간과 실제 물리 상태 |

Task Manager의 제한 시간은 벽시계 기준이며 Isaac Pause 중에도 흐른다. 제한 시간이 지났다고 Isaac의 물리 명령이 취소되는 것은 아니다. 실패나 재시작 후에는 Isaac Sim의 **Stop → Play**로 원본 v015 장면을 다시 열고, Task Manager를 재시작해 새 `IDLE` 상태를 만든 다음 Nav2·Navigation·Inspection 상태도 새 실행 기준으로 확인한다. 같은 Task Manager 인스턴스의 `COMPLETE`·`ERROR` 상태에서 `/start_cycle`을 다시 호출해 새 사이클을 시작할 수 없다.
