# guidance2_22차 — 주행·도킹·Place 전 구간 (고피에서 지시, 내피에서 Nav2)

- 작성일: 2026-09-23, 브랜치 `feature/Inspection-Place-nav2`. 도킹 거리를 팔 작업 범위에 맞추고 결과를 놓치지 않게 고친 판이며 21차는 `guidance/past/` 로 옮김.
- **PC 분담**: Nav2 쪽(Nav2·RViz2·정밀 도킹·주행 노드)만 내피(`lwh19180`)에서 돌리고, 나머지(Isaac, 명령 발행, 나중의 Task Manager)는 모두 고피(`IsaacSim03`)에서 돌린다.
- 그래서 **고피에도 워크스페이스를 한 번 빌드해야 한다.** 명령에 쓰는 `smart_farm_interfaces` 메시지가 거기 있어야 하기 때문이다. 18차 실측이 막힌 지점이 이것이다.
- `ROS_DOMAIN_ID` 는 두 PC 모두 **101**. 비전 컨테이너를 띄운다면 그것도 101 이다.
- 장면 `Collected_smartfarm_v011.usd`, 지도는 패키지에 설치된 `Collected_smartfarm_v011.yaml` 을 자동으로 쓴다.
- 아래 순서대로 위에서 아래로 실행한다. 블록마다 환경 줄이 들어 있으므로 그대로 복사해 붙이면 된다.

## 0. 이번 판에서 알아 둘 것 (읽기만)

| 항목 | 내용 |
|---|---|
| 고피 빌드 | 명령을 보내는 터미널에는 `smart_farm_interfaces` 가 필요하다. 고피에서 1회 빌드하면 그 뒤로는 `source` 만 하면 된다 |
| Isaac 터미널 주의 | **Isaac 을 띄우는 터미널에서는 워크스페이스를 `source` 하지 않는다.** 팀 앱이 Isaac 번들 ROS 라이브러리를 먼저 쓰도록 되어 있어 시스템 ROS 와 섞이면 죽는다. 빌드와 명령 발행은 Isaac 과 다른 터미널에서 한다 |
| 주행 실행 방식 | `navigation_node` 가 NavigateToPose 액션을 직접 보내고, 도착하면 명령 식별자를 실어 정밀 도킹을 시작시킨다. 같은 식별자로 돌아온 결과만 인정한다 |
| 도킹 시작 | 자동 시작은 꺼져 있다. 명령으로만 시작한다. RViz2 클릭으로 손 시험할 때만 `dock_auto:=true` 로 띄운다 |
| 도킹 거리 | 면에서 **0.85 m**. 팀 코드가 요구하는 팔 작업 범위(팔 밑동에서 대상까지 0.89~1.05 m) 한가운데인 0.96 m 가 된다. 이전 0.75 m 는 0.86 m 로 그 범위보다 가까웠다 |
| 제한 시간 | NAVIGATION 단계 400 s, 주행 노드 600 s, 도킹 240 s |
| 발행 명령 | 모두 `--once --max-wait-time-secs 15`. 15 초 안에 구독자를 못 찾으면 무한 대기 대신 오류로 끝난다 |
| Place 직전 검증 | 팀 앱이 차체 정지를 확인한 뒤 브레이크를 걸고, 실제 정지 위치와 place 대상까지의 거리를 로그로 남긴다. 판정은 하지 않는다 |

### 21차 실측에서 확인한 것 (읽기만)

| 관찰 | 해석 | 이번 조치 |
|---|---|---|
| 도킹 지점이 TurnTable 에 지나치게 가깝고 place 여유가 있어 보임 | 팀 `robot_motion` 이 요구하는 팔 작업 범위는 팔 밑동에서 대상까지 0.89~1.05 m 다. 도킹 0.75 m 에서는 0.86 m 로 **범위보다 가까웠다** | 도킹 거리를 0.85 m 로 늘렸다. 팔 밑동~대상 0.96 m 로 범위 한가운데가 되고, 차체 뒤끝과 면 사이 간격도 0.14 m 에서 0.24 m 로 넓어진다 |
| Place 지시 후 아무 반응 없음 | 결과 토픽을 명령 **뒤에** 구독하면 이미 발행된 결과를 놓친다. 결과가 실패였어도 화면에 안 나온다 | 결과 구독 터미널을 따로 두어 실측 내내 켜 둔다(7절). 명령 터미널은 발행만 한다 |
| 팔이 움직이지 않음 | 차체 정지 확인이 통과하지 못하면 팔을 움직이지 않는데, 즉시 실패로 끝나 이유가 보이지 않았다 | 최대 15 초까지 기다리며 진행 상황을 Isaac 터미널에 남기도록 바꿨다. 끝내 안 멈추면 이유를 담아 실패한다 |

### 19차 실측에서 막힌 곳 (읽기만)

주행과 도킹은 성공했다(`face_dist=0.758 m, yaw_err=0.31도, lat=-0.086 m`). Place 명령을 보내자마자 Isaac 이 종료되었고 원인은 다음과 같다.

| 증상 | 원인 | 이번 조치 |
|---|---|---|
| `stdbuf: failed to run command 'isaac_python'` | `stdbuf` 처럼 외부 명령을 앞에 붙이면 셸 별칭이 풀리지 않는다. 2절 블록대로 `isaac_python` 을 첫 낱말로 두고 `PYTHONUNBUFFERED=1` 만 쓴다 |
| Place 지시 직후 Isaac 종료 | Place 직전 도킹 위치를 기록하는 코드가 그 파일에 없는 함수(`robot_motion.prim_world_pose`)와 import 하지 않은 `numpy` 를 불렀다. 예외가 팀 앱의 최상위까지 올라가 `app.close()` 로 이어졌다 | 표준 API(`robot.get_world_pose()`)와 표준 연산으로 바꿨다. 같은 유형이 더 없는지 바뀐 파일 전체를 정적 검사로 확인했다 |
| 오류 내용이 로그에 없음 | `tee` 로 넘길 때 Python 출력이 블록 단위로 모였다가 나가므로, 갑자기 죽으면 마지막 묶음이 통째로 사라진다 | Isaac 실행 앞에 `PYTHONUNBUFFERED=1` 을 두어 한 줄씩 바로 기록되게 했다 |
| 팔 베이스 대기가 181초까지 늘어남 | 팔레트 이송 제어기가 쓰는 정지 감시기를 main loop 에서 또 갱신해 시간이 두 배로 흘렀다 | Place 검사용 감시기를 따로 두어 서로 간섭하지 않게 했다 |
| GPU 원인 여부 확인 불가 | 감시 수단이 없었다 | 고피에 GPU 사용량 기록 터미널을 하나 추가했다(3절) |

## 1. 고피 터미널 1 — 워크스페이스 빌드 (처음 한 번만)

Isaac 을 띄우지 않은 새 터미널에서 실행한다.

```bash
export ROS_DOMAIN_ID=101
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE=/home/rokey/.ros/fastdds_whitelist.xml
source /opt/ros/jazzy/setup.bash
cd /home/rokey/ROKEY_P3_A1/cobot3_ws
colcon build --packages-select smart_farm_interfaces smart_farm_manager 2>&1 | tee /home/rokey/ROKEY_P3_A1/cobot3_ws/src/smart_farm_navigation/results/build_gopi_$(date +%Y%m%d_%H%M).txt
source /home/rokey/ROKEY_P3_A1/cobot3_ws/install/setup.bash
ros2 interface show smart_farm_interfaces/msg/TaskCommand
```

기대: `Summary: 2 packages finished` 뒤에 `string task_id` 로 시작하는 필드 목록이 나온다. 이 터미널은 8절에서 다시 쓴다.

**빌드는 한 번이면 되지만 `source` 는 새 터미널을 열 때마다 해야 한다.** 그래서 8절의 모든 명령 블록에 환경 줄과 `source` 가 들어 있다. 블록을 통째로 붙여 쓴다.

- `colcon: command not found` 가 나오면 고피에 colcon 이 없는 것이다. 그때는 5절을 내피에서 실행하고 그 사실을 보고한다.
- 빌드 산출물(`build/`, `install/`, `log/`)은 git 에 올라가지 않는다.

## 2. 고피 터미널 2 — Isaac Sim + 팀 통합 앱

**이 터미널에서는 워크스페이스를 `source` 하지 않는다.** 튜터 지시대로 환경을 준비한 뒤:

```bash
export ROS_DOMAIN_ID=101
export PYTHONUNBUFFERED=1
isaac_python /home/rokey/ROKEY_P3_A1/cobot3_ws/isaacpjt/smart_farm/runtime/standalone_app.py --autoplay 2>&1 | tee /home/rokey/ROKEY_P3_A1/cobot3_ws/src/smart_farm_navigation/results/isaac_$(date +%Y%m%d_%H%M).txt
```

기대: `[라이다] 3D 라이다 fullScan=True (1개 helper)` → `[READY] Collected_smartfarm_v011 scene ready; TRANSFER/PICK_HARVEST/PLACE_INSPECT physical profiles loaded` → `[대기] /sim_task/command의 String/JSON 명령을 기다립니다`.

- `PYTHONUNBUFFERED=1` 은 출력을 한 줄씩 바로 기록하기 위한 것이다. 이것이 없으면 갑자기 종료됐을 때 마지막 출력이 사라져 원인을 볼 수 없다. 환경 변수라서 `isaac_python` 을 그대로 쓸 수 있다.
- 그래도 마지막 출력이 사라지면 `isaac_python` 뒤에 `-u` 를 붙여 본다. 예: `isaac_python -u /home/rokey/...standalone_app.py --autoplay`.
- Isaac 의 Stop 버튼은 누르지 않는다(팀 앱이 종료 처리에서 죽는다). 끝낼 때는 Ctrl+C 를 쓴다.
- Isaac 을 다시 실행하면 5·6 절의 내피 터미널도 모두 다시 실행한다. 시뮬레이션 시각이 0 으로 돌아가기 때문이다.

## 3. 고피 터미널 3 — GPU 사용량 감시 (실측 내내 켜 둔다)

Isaac 을 띄운 뒤 다른 터미널에서 실행한다. 종료 원인이 메모리 부족인지 가려내기 위한 것이다.

```bash
nvidia-smi --query-gpu=timestamp,name,utilization.gpu,memory.used,memory.total,temperature.gpu --format=csv -l 2 2>&1 | tee /home/rokey/ROKEY_P3_A1/cobot3_ws/src/smart_farm_navigation/results/gpu_$(date +%Y%m%d_%H%M).csv
```

2 초마다 한 줄씩 쌓인다. 실측이 끝날 때까지 켜 두고 Ctrl+C 로 끝낸다.

- `memory.used` 가 `memory.total` 에 가깝게 붙으면 메모리 부족이다. 그때는 이 파일과 Isaac 로그를 함께 보고한다.
- 어느 프로세스가 얼마나 쓰는지 보려면 다른 터미널에서 아래를 쓴다.

```bash
nvidia-smi --query-compute-apps=timestamp,pid,process_name,used_memory --format=csv -l 5 2>&1 | tee /home/rokey/ROKEY_P3_A1/cobot3_ws/src/smart_farm_navigation/results/gpuapps_$(date +%Y%m%d_%H%M).csv
```

- `nvidia-smi: command not found` 가 나오면 그 PC 에 NVIDIA 드라이버 도구가 없는 것이다. 그 사실만 알려 주면 된다.

## 4. 내피 터미널 1 — 빌드와 연결 점검

```bash
export ROS_DOMAIN_ID=101
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE=/home/rokey/.ros/fastdds_whitelist.xml
source /opt/ros/jazzy/setup.bash
cd /home/rokey/ROKEY_P3_A1/cobot3_ws
colcon build --packages-select smart_farm_interfaces smart_farm_navigation smart_farm_manager
source /home/rokey/ROKEY_P3_A1/cobot3_ws/install/setup.bash
ros2 run smart_farm_navigation nav2_link_check 2>&1 | tee /home/rokey/ROKEY_P3_A1/cobot3_ws/src/smart_farm_navigation/results/link_$(date +%Y%m%d_%H%M).txt
```

기대: `/clock`·`/tf`·`/chassis/odom` 이 15 Hz 이상, `/front_3d_lidar/lidar_points` 가 3 Hz 이상, `[6] self returns inside the rig box: N of M points/scan`, 마지막 줄 `RESULT OK - nav2.launch.py scan_mode:=cloud`.

- 전부 0 Hz 면 두 PC 의 도메인이 다르거나 화이트리스트에 상대 IP 가 없다.
- `N` 과 `highest z` 를 적어 둔다. 실측 기준값은 35~66점, 0.55 m 다.

## 5. 내피 터미널 2 — Nav2 · RViz2 · 정밀 도킹 · 기록

```bash
export ROS_DOMAIN_ID=101
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE=/home/rokey/.ros/fastdds_whitelist.xml
source /opt/ros/jazzy/setup.bash
source /home/rokey/ROKEY_P3_A1/cobot3_ws/install/setup.bash
ros2 launch smart_farm_navigation nav2.launch.py record:=true 2>&1 | tee /home/rokey/ROKEY_P3_A1/cobot3_ws/src/smart_farm_navigation/results/nav2_$(date +%Y%m%d_%H%M).txt
```

기대: `rosbag -> ~/.ros/smart_farm_navigation/bags/nav2_…` → `scan_mode auto -> cloud` → `AMCL initial pose (-0.421, 1.006, 90.0deg)` → `Managed nodes are active` → `[feeder_dock]: [IDLE] waiting`.

- `record:=true` 는 이번 실측에 필요하다. 팔 자세 허용 범위를 여기서 나온 기록으로 정한다.
- RViz2 에서 카터가 Rack_1 과 Rack_4 사이 통로에 있고 스캔 점이 양쪽 랙 면에 붙어 있으면 그대로 둔다. `2D Pose Estimate` 는 찍지 않는다.

## 6. 내피 터미널 3 — 주행 노드

```bash
export ROS_DOMAIN_ID=101
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE=/home/rokey/.ros/fastdds_whitelist.xml
source /opt/ros/jazzy/setup.bash
source /home/rokey/ROKEY_P3_A1/cobot3_ws/install/setup.bash
ros2 launch smart_farm_navigation navigation_node.launch.py 2>&1 | tee /home/rokey/ROKEY_P3_A1/cobot3_ws/src/smart_farm_navigation/results/navnode_$(date +%Y%m%d_%H%M).txt
```

기대: `navigation_node ready; destinations: ['FEEDER_DOCK', 'RACK_DOCK', 'CORRIDOR_EXIT']`.

## 7. 고피 터미널 4·5 — 결과 구독 (명령을 보내기 전에 먼저 띄운다)

결과 토픽은 보관되지 않으므로, 명령을 보낸 뒤에 구독하면 이미 지나간 결과를 못 본다. 두 터미널을 먼저 띄워 실측이 끝날 때까지 켜 둔다.

고피 터미널 4 (Isaac 작업 결과):

```bash
export ROS_DOMAIN_ID=101
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE=/home/rokey/.ros/fastdds_whitelist.xml
source /opt/ros/jazzy/setup.bash
source /home/rokey/ROKEY_P3_A1/cobot3_ws/install/setup.bash
ros2 topic echo /sim_task/result std_msgs/msg/String 2>&1 | tee /home/rokey/ROKEY_P3_A1/cobot3_ws/src/smart_farm_navigation/results/simresult_$(date +%Y%m%d_%H%M).txt
```

고피 터미널 5 (주행 결과):

```bash
export ROS_DOMAIN_ID=101
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE=/home/rokey/.ros/fastdds_whitelist.xml
source /opt/ros/jazzy/setup.bash
source /home/rokey/ROKEY_P3_A1/cobot3_ws/install/setup.bash
ros2 topic echo /navigation/result smart_farm_interfaces/msg/TaskResult 2>&1 | tee /home/rokey/ROKEY_P3_A1/cobot3_ws/src/smart_farm_navigation/results/navresult_$(date +%Y%m%d_%H%M).txt
```

두 터미널 모두 아무것도 찍히지 않은 채 대기하는 것이 정상이다. 명령을 보내면 그때 결과가 나타난다.

## 8. 고피 터미널 1 — 명령 발행 (1절 터미널을 그대로 쓴다)

먼저 워크스페이스를 읽고 메시지 타입이 보이는지 확인한다.

```bash
export ROS_DOMAIN_ID=101
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE=/home/rokey/.ros/fastdds_whitelist.xml
source /opt/ros/jazzy/setup.bash
source /home/rokey/ROKEY_P3_A1/cobot3_ws/install/setup.bash
hostname
ros2 interface show smart_farm_interfaces/msg/TaskCommand
```

`IsaacSim03` 과 `string task_id` 로 시작하는 필드 목록이 함께 나와야 한다. `No such file or directory` 나 `Unknown package` 가 나오면 1절 빌드가 끝나지 않은 것이므로 1절을 다시 실행한다.

이어서 팔레트를 집는다.

```bash
export ROS_DOMAIN_ID=101
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE=/home/rokey/.ros/fastdds_whitelist.xml
source /opt/ros/jazzy/setup.bash
source /home/rokey/ROKEY_P3_A1/cobot3_ws/install/setup.bash
ros2 topic pub --once --max-wait-time-secs 15 /sim_task/command std_msgs/msg/String '{data: "{\"task_id\": \"TASK-20260923-001\", \"command_id\": \"TASK-20260923-001-CMD-001\", \"operation\": \"PICK_HARVEST\", \"recipe_id\": \"HARVEST_RACK_L1\", \"pallet_id\": \"PALLET_001\", \"source\": \"RACK_L1\", \"destination\": \"CARRY\"}"}' 2>&1 | tee -a /home/rokey/ROKEY_P3_A1/cobot3_ws/src/smart_farm_navigation/results/cmd_$(date +%Y%m%d)_gopi.txt
```

**고피 터미널 4** 에 `"status": "SUCCEEDED"` 와 `"safe_to_navigate": true` 가 나오면 주행을 지시한다.

```bash
export ROS_DOMAIN_ID=101
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE=/home/rokey/.ros/fastdds_whitelist.xml
source /opt/ros/jazzy/setup.bash
source /home/rokey/ROKEY_P3_A1/cobot3_ws/install/setup.bash
ros2 topic pub --once --max-wait-time-secs 15 /navigation/command smart_farm_interfaces/msg/TaskCommand "{task_id: 'TASK-20260923-001', command_id: 'TASK-20260923-001-CMD-002', operation: 'NAVIGATION', destination: 'FEEDER_DOCK'}" 2>&1 | tee -a /home/rokey/ROKEY_P3_A1/cobot3_ws/src/smart_farm_navigation/results/cmd_$(date +%Y%m%d)_gopi.txt
```

6절 내피 터미널 3 의 기대 출력은 다음과 같다.

```
goToPose FEEDER_APPROACH: -2.19, -1.55
정밀 도킹 시작 요청: run_id=TASK-20260923-001-CMD-002
도킹 결과 SUCCEEDED: face_dist=0.7x yaw_err=-0.x lat=0.0x
result SUCCEEDED/NONE for TASK-20260923-001-CMD-002 (phase ARRIVED)
```

**고피 터미널 5** 에 `status: SUCCEEDED`, `reason: NONE`, `reached_station: FEEDER_DOCK` 이 나와야 한다.

마지막으로 팔레트를 내려놓는다.

```bash
export ROS_DOMAIN_ID=101
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE=/home/rokey/.ros/fastdds_whitelist.xml
source /opt/ros/jazzy/setup.bash
source /home/rokey/ROKEY_P3_A1/cobot3_ws/install/setup.bash
ros2 topic pub --once --max-wait-time-secs 15 /sim_task/command std_msgs/msg/String '{data: "{\"task_id\": \"TASK-20260923-001\", \"command_id\": \"TASK-20260923-001-CMD-003\", \"operation\": \"PLACE_INSPECT\", \"recipe_id\": \"PLACE_AT_INSPECTION\", \"pallet_id\": \"PALLET_001\", \"source\": \"CARRY\", \"destination\": \"INSPECT_STATION\"}"}' 2>&1 | tee -a /home/rokey/ROKEY_P3_A1/cobot3_ws/src/smart_farm_navigation/results/cmd_$(date +%Y%m%d)_gopi.txt
```

이때 2절 고피 터미널(Isaac)에 `[Place] 차체 정지 확인` 과 `[도킹] 카터 본체 world (…), place 대상까지 x … y … 직선 … m` 이 찍힌다. **이 줄을 그대로 옮겨 적어 주면 팔 자세 허용 범위를 정할 수 있다.**

## 9. 문제가 생겼을 때

| 증상 | 조치 |
|---|---|
| `install/setup.bash: No such file or directory` (고피) | 1절 빌드를 하지 않았다. 1절을 먼저 실행한다 |
| `Unknown package 'smart_farm_interfaces'` 또는 `The passed message type is invalid` | 그 터미널에서 `source .../install/setup.bash` 를 하지 않았다. 8절 블록을 줄 일부만 붙이지 말고 통째로 붙인다. 그래도 안 되면 1절 빌드부터 다시 한다 |
| 발행 명령이 15 초 뒤 오류로 끝남 | 구독자를 못 찾았다. `/navigation/command` 면 6절 터미널이 떠 있는지, `/sim_task/command` 면 2절 Isaac 이 `[대기]` 상태인지 확인한다. 두 PC 의 도메인이 같은지도 본다 |
| `stdbuf: failed to run command 'isaac_python'` | `stdbuf` 처럼 외부 명령을 앞에 붙이면 셸 별칭이 풀리지 않는다. 2절 블록대로 `isaac_python` 을 첫 낱말로 두고 `PYTHONUNBUFFERED=1` 만 쓴다 |
| Place 지시 직후 Isaac 종료 | 19차와 같은 증상이면 2절 로그 끝에 traceback 이 남는다(이번 판부터 유실되지 않는다). 그 몇 줄과 `gpu_*.csv` 를 함께 보고한다 |
| Isaac 터미널에서 rclpy 오류로 죽음 | 그 터미널에서 워크스페이스를 `source` 했을 가능성이 크다. 새 터미널에서 워크스페이스 없이 2절만 실행한다 |
| `navigate_to_pose 액션 서버가 없습니다` | 5절 Nav2 가 아직 안 떴다. `Managed nodes are active` 를 본 뒤 명령을 보낸다 |
| 접근 지점에 멈춘 뒤 도킹이 시작되지 않음 | 5절 터미널에 `FEEDER_APPROACH 부근에 서 있으나 자동 시작이 꺼져 있습니다` 경고가 뜬다. 8절 주행 명령으로 시작한다 |
| Place 명령 뒤 Isaac 에 `[Place] 차체 정지를 확인합니다` 만 반복 | 차체가 계속 미세하게 움직이고 있다. 15 초 뒤 이유를 담아 실패한다. 주행 결과를 받은 뒤 2~3 초 기다렸다 다시 보낸다 |
| 명령을 보냈는데 결과가 안 보임 | 7절 결과 터미널을 명령보다 **먼저** 띄웠는지 확인한다. 나중에 띄우면 이미 지나간 결과를 못 본다 |
| 팔이 닿지 않음(역기구학 실패) | 도킹 거리를 조정한다. 5절을 `ros2 launch smart_farm_navigation nav2.launch.py record:=true dock_auto:=false` 로 띄운 뒤, 내피의 다른 터미널에서 `ros2 run smart_farm_navigation feeder_dock --ros-args -p use_sim_time:=true -p standoff_m:=0.90` 처럼 바꿔 실행한다. 팔 작업 범위는 팔 밑동에서 대상까지 0.89~1.05 m 이고, 그 거리는 `0.11 + standoff_m` 이다. 즉 `standoff_m` 은 0.78~0.94 사이여야 한다 |
| 주행 중 시간 초과 | 실시간 배율이 0.2 아래로 떨어진 경우다. 고피에서 다른 GPU 작업을 함께 돌리고 있는지 확인한다 |
| RViz2 클릭으로 손 시험하고 싶음 | 5절을 `dock_auto:=true` 로 띄우고 6·8 절을 생략한다. `FEEDER_APPROACH` 화살표를 Nav2 Goal 로 한 번 클릭하면 도착 후 자동으로 도킹한다 |

## 10. 결과 보고

`results/` 의 `build_gopi_*`, `isaac_*`, `link_*`, `nav2_*`, `navnode_*`, `cmd_*_gopi.txt`, `simresult_*`, `navresult_*`, `gpu_*.csv` 와 bag 디렉터리 이름(`~/.ros/smart_farm_navigation/bags/`), 그리고 8절 마지막의 `[도킹]` 줄을 함께 남긴다.

## 11. 파일과 역할

| 파일 | 실행 PC | 역할 |
|---|---|---|
| `isaacpjt/smart_farm/runtime/standalone_app.py` | 고피 | 장면 실행, 팔레트 집기·내려놓기, 라이다 fullScan, Place 전 차체 정지 확인과 도킹 위치 기록 |
| `launch/nav2.launch.py` | 내피 | Nav2 · RViz2 · `/scan` 생성 · 정밀 도킹 노드 · 작업점 마커 · 기록 |
| `launch/navigation_node.launch.py` | 내피 | `/navigation/command` 를 받아 NavigateToPose 와 정밀 도킹을 구동 |
| `smart_farm_navigation/feeder_dock.py` | 내피 | 라이다로 TurnTable 앞면을 보며 후진 도킹 |
| `smart_farm_navigation/cloud_self_filter.py` | 내피 | 결합카터 자기 반사 제거와 점군 합치기 |
| `smart_farm_navigation/nav2_link_check.py` | 내피 | 두 PC 사이 토픽 도달과 자기 반사 점검 |
| `smart_farm_navigation/stations.py` | 내피 | 작업점 읽기와 목표 자세 생성 |
| `config/stations.yaml`, `config/destinations_nav2.yaml` | 내피 | 작업점과 목적지 매핑 |
| `smart_farm_interfaces` | 고피·내피 | 명령·결과·상태 메시지. 명령을 보내는 PC 에 반드시 빌드되어 있어야 한다 |
