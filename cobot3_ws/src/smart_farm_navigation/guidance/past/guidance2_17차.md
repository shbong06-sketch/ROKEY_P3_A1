# guidance2_17차 — Task Manager 명령으로 카터 자율주행·정밀 도킹 (통합 브랜치, 고피1)

- 작성일: 2026-09-23, 브랜치 `feature/Inspection-Place-nav2`. 16차는 `guidance/past/` 로 옮김.
- 목표: Task Manager 가 `/navigation/command` 로 `NAVIGATION`/`FEEDER_DOCK` 을 보내면, 내피의 `navigation_node` 가 Nav2 로 접근 작업점까지 가고 이어서 정밀 도킹까지 마친 뒤 `/navigation/result` 를 돌려준다. RViz2 클릭은 쓰지 않는다.
- 실행 PC: 고피1(Isaac), 내피(Nav2·주행 노드). **`ROS_DOMAIN_ID` 는 101**. 비전 컨테이너도 같은 값으로 띄운다.
- 장면 `Collected_smartfarm_v011.usd`, 지도는 패키지에 설치된 `Collected_smartfarm_v011.yaml` 을 자동으로 쓴다.
- 각 터미널 블록은 그대로 복사해 붙이면 된다. 블록마다 환경 5줄이 들어 있다.

## 0. 이번에 바뀐 것 (읽기만)

| 항목 | 내용 |
|---|---|
| 주행 실행 방식 | `navigation_node` 가 자식 프로세스를 띄우지 않고 NavigateToPose 액션을 직접 보낸다. 취소가 실제로 동작하고, 결과에 단계와 도킹 오차가 담긴다 |
| 도킹 시작 | `navigation_node` 가 `/feeder_dock/start` 에 명령 식별자를 실어 보내고, 같은 식별자로 돌아온 결과만 인정한다. 자동 시작은 꺼져 있다(RViz2 수동 절차는 `dock_auto:=true`) |
| 도킹 거리 | 면에서 0.75 m. 팔 밑동에서 place 대상까지 직선 0.90 m 로 M0609 도달 한계와 같다. 팔이 닿지 않으면 6절을 본다 |
| 자기 반사 상자 | `self_box_x/y/z` 두 원소 배열로 통일. x 는 −0.60~0.60 |
| 제한 시간 | NAVIGATION 단계 400 s, 주행 노드 600 s, 도킹 240 s. 속도 0.3 m/s 와 실시간 배율 0.3 이 겹쳐 편도가 벽시계 3 분을 넘길 수 있다 |
| Place 직전 검증 | 팀 앱이 차체 정지를 확인한 뒤 브레이크를 걸고, 실제 정지 위치와 place 대상까지의 거리를 로그로 남긴다. 판정은 하지 않는다 |
| 목적지 파일 | 기본값이 Nav2 목적지로 바뀌었다. 1차 시연용 설정과 launch 는 `config/past/`, `launch/past/` 로 내렸다 |

## 1. 고피1 터미널 1 (Isaac Sim + 팀 통합 앱)

튜터 지시대로 환경을 준비한 뒤:

```bash
export ROS_DOMAIN_ID=101
isaac_python /home/rokey/ROKEY_P3_A1/cobot3_ws/isaacpjt/smart_farm/runtime/standalone_app.py --autoplay 2>&1 | tee /home/rokey/ROKEY_P3_A1/cobot3_ws/src/smart_farm_navigation/results/isaac_$(date +%Y%m%d_%H%M).txt
```

기대: `[라이다] 3D 라이다 fullScan=True (1개 helper)` → `[READY] Collected_smartfarm_v011 scene ready; TRANSFER/PICK_HARVEST/PLACE_INSPECT physical profiles loaded` → `[대기] /sim_task/command의 String/JSON 명령을 기다립니다`.

- Isaac 의 Stop 버튼은 누르지 않는다(팀 앱이 종료 처리에서 죽는다). 끝낼 때는 Ctrl+C 를 쓴다.
- Isaac 을 다시 실행하면 3절의 터미널도 모두 다시 실행한다. 시뮬레이션 시각이 0 으로 돌아가기 때문이다.

## 2. 내피 터미널 2 (빌드와 연결 점검)

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

- `PARTIAL SLICES` 가 보이면 고피의 fullScan 설정이 빠진 것이다. 1절 출력을 확인한다. 그대로 진행해도 동작은 한다.
- `N` 과 `highest z` 를 적어 둔다. 실측 기준값은 66점, 0.53 m 다.

## 3. 내피 터미널 3 (Nav2 + RViz2 + 도킹 노드 + 기록)

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

## 4. 내피 터미널 4 (주행 노드)

```bash
export ROS_DOMAIN_ID=101
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE=/home/rokey/.ros/fastdds_whitelist.xml
source /opt/ros/jazzy/setup.bash
source /home/rokey/ROKEY_P3_A1/cobot3_ws/install/setup.bash
ros2 launch smart_farm_navigation navigation_node.launch.py 2>&1 | tee /home/rokey/ROKEY_P3_A1/cobot3_ws/src/smart_farm_navigation/results/navnode_$(date +%Y%m%d_%H%M).txt
```

기대: `navigation_node ready; destinations: ['FEEDER_DOCK', 'RACK_DOCK', 'CORRIDOR_EXIT']`.

## 5. 내피 터미널 5 (명령 발행과 결과 확인)

먼저 팔레트를 집는다.

```bash
export ROS_DOMAIN_ID=101
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE=/home/rokey/.ros/fastdds_whitelist.xml
source /opt/ros/jazzy/setup.bash
source /home/rokey/ROKEY_P3_A1/cobot3_ws/install/setup.bash
ros2 topic pub -t 3 -r 1 /sim_task/command std_msgs/msg/String '{data: "{\"task_id\": \"TASK-20260923-001\", \"command_id\": \"TASK-20260923-001-CMD-001\", \"operation\": \"PICK_HARVEST\", \"recipe_id\": \"HARVEST_RACK_L1\", \"pallet_id\": \"PALLET_001\", \"source\": \"RACK_L1\", \"destination\": \"CARRY\"}"}'
ros2 topic echo /sim_task/result std_msgs/msg/String --once
```

`"status": "SUCCEEDED"` 와 `"safe_to_navigate": true` 를 확인한 뒤 주행을 지시한다.

```bash
export ROS_DOMAIN_ID=101
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE=/home/rokey/.ros/fastdds_whitelist.xml
source /opt/ros/jazzy/setup.bash
source /home/rokey/ROKEY_P3_A1/cobot3_ws/install/setup.bash
ros2 topic pub -t 3 -r 1 /navigation/command smart_farm_interfaces/msg/TaskCommand "{task_id: 'TASK-20260923-001', command_id: 'TASK-20260923-001-CMD-002', operation: 'NAVIGATION', destination: 'FEEDER_DOCK'}"
ros2 topic echo /navigation/result smart_farm_interfaces/msg/TaskResult --once
```

터미널 4 의 기대 출력은 다음과 같다.

```
goToPose FEEDER_APPROACH: -2.19, -1.55
실행 중인 명령 재수신, 무시: TASK-20260923-001-CMD-002     <- -t 3 으로 세 번 보내서 정상이다
정밀 도킹 시작 요청: run_id=TASK-20260923-001-CMD-002
도킹 결과 SUCCEEDED: face_dist=0.7x yaw_err=-0.x lat=0.0x
result SUCCEEDED/NONE for TASK-20260923-001-CMD-002 (phase ARRIVED)
```

터미널 5 의 결과는 `status: SUCCEEDED`, `reason: NONE`, `reached_station: FEEDER_DOCK` 이어야 한다.

마지막으로 팔레트를 내려놓는다.

```bash
export ROS_DOMAIN_ID=101
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE=/home/rokey/.ros/fastdds_whitelist.xml
source /opt/ros/jazzy/setup.bash
source /home/rokey/ROKEY_P3_A1/cobot3_ws/install/setup.bash
ros2 topic pub -t 3 -r 1 /sim_task/command std_msgs/msg/String '{data: "{\"task_id\": \"TASK-20260923-001\", \"command_id\": \"TASK-20260923-001-CMD-003\", \"operation\": \"PLACE_INSPECT\", \"recipe_id\": \"PLACE_AT_INSPECTION\", \"pallet_id\": \"PALLET_001\", \"source\": \"CARRY\", \"destination\": \"INSPECT_STATION\"}"}'
ros2 topic echo /sim_task/result std_msgs/msg/String --once
```

고피1 터미널 1 에 `[도킹] 카터 본체 world (…), place 대상까지 x … y … 직선 … m` 이 찍힌다. **이 줄을 그대로 옮겨 적어 주면 팔 자세 허용 범위를 정할 수 있다.**

## 6. 문제가 생겼을 때

| 증상 | 조치 |
|---|---|
| 터미널 4 에 `navigate_to_pose 액션 서버가 없습니다` | 터미널 3 의 Nav2 가 아직 안 떴다. `Managed nodes are active` 를 본 뒤 명령을 보낸다 |
| `Nav2 가 목표를 거부했습니다` | 시뮬레이션 시각 설정 문제다. 터미널 4 를 반드시 launch 로 띄웠는지 확인한다 |
| 도킹이 시작되지 않음 | 터미널 3 의 `[feeder_dock]` 줄을 본다. 면을 못 찾으면 이유가 함께 찍힌다. 카터 뒤가 TurnTable 을 대략 향하고 있어야 한다 |
| 도킹은 끝났는데 팔이 닿지 않음(역기구학 실패) | 도킹 거리를 줄인다. 터미널 3 을 `ros2 launch smart_farm_navigation nav2.launch.py record:=true dock_auto:=false` 로 띄운 뒤, 별도 터미널에서 `ros2 run smart_farm_navigation feeder_dock --ros-args -p use_sim_time:=true -p standoff_m:=0.70` 으로 바꿔 실행한다. 이때 `self_box_x` 도 `[-0.55, 0.60]` 으로 함께 내려야 한다 |
| `PLACE_INSPECT` 가 `BASE_NOT_SETTLED` 로 실패 | 도킹 직후 카터가 아직 흔들리고 있다. 결과를 받은 뒤 2~3 초 기다렸다 명령을 보낸다 |
| 주행 중 시간 초과 | 실시간 배율이 0.2 아래로 떨어진 경우다. 고피에서 다른 GPU 작업(비전 컨테이너 등)을 함께 돌리고 있는지 확인한다 |
| RViz2 로 손 시험하고 싶음 | 터미널 3 을 `ros2 launch smart_farm_navigation nav2.launch.py record:=true dock_auto:=true` 로 띄우고 터미널 4·5 를 생략한다. `FEEDER_APPROACH` 화살표를 Nav2 Goal 로 한 번 클릭하면 도착 후 자동으로 도킹한다 |
| Isaac 을 다시 실행함 | 터미널 3·4 를 모두 다시 실행한다 |

## 7. 결과 보고

`results/` 의 `isaac_*`, `link_*`, `nav2_*`, `navnode_*` 와 bag 디렉터리 이름(`~/.ros/smart_farm_navigation/bags/`), 그리고 5절 마지막의 `[도킹]` 줄을 함께 남긴다.

## 8. 이번에 만지거나 만든 파일

| 파일 | 역할 | 이번 변경 |
|---|---|---|
| `smart_farm_navigation/navigation_node.py` | 주행 명령 수신·실행 (팀 파일) | 자식 프로세스 제거, NavigateToPose 액션 직접 사용, 도킹 시작·결과 대조, 기본 목적지와 제한 시간 변경, 같은 명령 재전송 무시 |
| `smart_farm_navigation/feeder_dock.py` | 라이다 면 검출 정밀 도킹 | 도킹 거리 0.75, 자동 시작 기본 꺼짐, 시작 신호에 실행 식별자, 종료 처리 |
| `smart_farm_navigation/cloud_self_filter.py` | 자기 반사 제거와 점군 합치기 | 파라미터 이름을 `self_box_x/y/z` 두 원소 배열로 통일, 종료 처리 |
| `smart_farm_navigation/stations.py` | 작업점 읽기와 목표 자세 생성 | 새로 만듦. 주행 노드와 손 시험 도구가 공용 |
| `smart_farm_navigation/nav2_link_check.py` | 토픽 도달과 자기 반사 점검 | 상자 값을 노드와 동기화 |
| `smart_farm_navigation/go_to_station.py` | 손 시험용 명령줄 도구 | 공용 모듈 사용, 기본 작업점 변경 |
| `launch/nav2.launch.py` | Nav2·RViz2·도킹·기록 | 자기 반사 상자 이름·값, 도킹 자동 시작 기본 꺼짐 |
| `launch/navigation_node.launch.py` | 주행 노드 | 시뮬레이션 시각 설정, Nav2 목적지·작업점 파일 지정, 제한 시간 |
| `config/stations.yaml` | 작업점 | `FEEDER_DOCK` 좌표를 도킹 거리 0.75 결과로 수정, `INSPECTION_DOCK` 제거 |
| `config/destinations_nav2.yaml` | 목적지 → 작업점 | `approach` 항목 추가, `INSPECTION_DOCK` 제거 |
| `isaacpjt/smart_farm/runtime/standalone_app.py` | 팀 통합 앱 | Place 전 차체 정지 확인과 도킹 위치 기록 |
| `smart_farm_manager/scenario.py` | 공정 시나리오 (팀 파일) | NAVIGATION 단계 제한 시간 400 s |
| `config/past/`, `launch/past/` | 1차 시연 설정·launch | 사용하지 않아 이동 |
