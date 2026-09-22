# guidance2_14차 — 결합카터 Nav2 실측 3종 (기본 / 운반 자세 / 팔레트 파지) — Collected_smartfarm_v011.usd

- 작성일: 2026-09-22, 브랜치: `feature/navigation2`. 13차는 이 문서에 흡수되어 `guidance/past/` 로 옮김 (13차 단독 실측은 불필요. 이유는 8절).
- 목적: 튜터 지시 3회 실측. 결합카터(LiftRig = 카터 + 리프트 + M0609)가 (1) 기본 자세, (2) 팔레트 없이 운반 자세, (3) Pallet_01 을 실제 파지한 상태로 각각 `RACK_DOCK → FEEDER_DOCK` 를 Nav2 로 주행하고, 라이다가 리프트·팔·팔레트를 장애물로 보는지 확인함.
- 기준 장면: `/home/rokey/ROKEY_P3_A1/cobot3_ws/isaacpjt/smart_farm/scenes/Collected_smartfarm_v011/Collected_smartfarm_v011.usd` (2026-09-22 갱신본. `ROS_Clock` 그래프가 들어 있어 `/clock` 은 장면이 직접 냄)
- 기준 지도: `/home/rokey/ROKEY_P3_A1/cobot3_ws/isaacpjt/smart_farm/maps/Collected_smartfarm_v011.yaml` (v011 에서 생성. TurnTable·검수실 벽 포함)
- 고피는 Isaac Sim 만, 내피는 Nav2·RViz2·주행 지시. 두 PC 모두 `ROS_DOMAIN_ID=101`.
- **필수 조건(튜터)**: 세 시험 모두 결합카터의 장면 초기 위치 (−0.42, 1.01, yaw 90°) 에서 출발하며, 출발부터 도착까지 **Nav2 만으로** 자율주행함. 사람이 옮기거나 스크립트가 따로 후진을 지시하지 않음. 이를 위해 경로 계획기를 Smac Hybrid-A*(Reeds-Shepp) 로 바꿔 통로 후진 탈출과 Feeder 후진 도킹까지 `NavigateToPose` 한 번으로 계획하게 하였음 (0절 첫 줄).

## 0. 이번에 바뀐 것 (사용자 3.1~3.4 반영)

| 항목 | 내용 |
|---|---|
| **Nav2 만으로 출발~도착** | 계획기를 NavFn 에서 `nav2_smac_planner::SmacPlannerHybrid`(Reeds-Shepp, 후진 허용, `reverse_penalty 1.0`) 로 바꾸고 RPP 를 `allow_reversing: true` 로 둠. `go_to_station` 은 기본(`pure_nav2:=true`)에서 초기 위치에서 `FEEDER_DOCK` 목표 하나만 보내며, 통로 후진 탈출과 Feeder 후진 도킹은 Nav2 의 경로 자체에 들어 있음. 내피 모의에서는 출발부터 도킹까지 전 구간을 후진으로 한 번에 갔음(왕복 4회 모두 성공, 편도 9~21 s). 이전 방식(스크립트가 BackUp 을 먼저 지시)은 `-p pure_nav2:=false` 로 남겨 둠 |
| Feeder 진입은 **후진** | 리프트 때문에 팔(M0609, 카터 뒤쪽)이 Feeder 를 봐야 함. 도킹 자세: base_link (−2.19, −2.72), yaw 90°(앞이 북쪽 = 팔이 TurnTable 을 봄), 차체 뒤끝이 TurnTable 북쪽 끝(y −3.60)에서 0.27 m, 팔 밑동은 0.46 m. Hybrid-A* 가 이 자세로 끝나는 후진 경로를 냄. 복귀는 앞(북쪽)으로 출발함 |
| 3D 라이다 10 Hz | `launch_scene.py` 가 실행 중에 `ROS2RtxLidarHelper` 의 `fullScan` 을 켜서 렌더 프레임마다(20~60 Hz) 가 아니라 센서 회전 1바퀴(10 Hz)마다 발행하게 함. 팀 `standalone_app.py` 로 띄우는 시험 3 에서는 이 설정이 없어 20 Hz 이상으로 옴(RViz2 는 그래도 아래 설정으로 버팀) |
| RViz2 부하 | 전용 설정 `rviz/nav2_smartfarm.rviz` (지도·/scan·경로·발자국·local costmap 만, 점군·카메라·TF 없음) 를 기본으로 씀 |
| 라이다 자기 반사 | `cloud_self_filter` 가 3D 점군에서 결합카터 자체 부피(base_link 기준 x −1.0~0.3, y ±0.45, z −0.2~2.6, 들고 있는 팔레트 포함)를 지운 뒤 `/scan` 을 만듦. `nav2_link_check` 가 "리그 상자 안 점 개수" 를 세어 주므로 시험 1·2·3 의 비교 지표로 씀 |
| 2D 라이다 | 기대하지 않음. `nav2.launch.py` 는 2D 스캔이 없으면 자동으로 3D 점군 경로를 씀 |
| 지도 경로 | ADR_nav2 1.2 유연화에 따라 `nav2.launch.py map:=…` 로 바꿀 수 있음. 기본은 v011 |
| development 병합 | `navigation_node` 는 팀 버전(`smart_farm_interfaces/TaskCommand`)을 그대로 쓰고, Nav2 모드(`station` 키) 만 덧붙임. `path_runner*.yaml` 등 /cmd_vel 쪽은 팀 값 그대로 둠 |
| Nav2 잔 수정 | BT 의 액션 응답 대기 20 ms → 200 ms (부하 시 goal 이 즉시 실패하던 것), local costmap 수신 후 출발. (`pure_nav2:=false` 일 때만 쓰는 BackUp 은 3회 재시도) |

## 1. 세 시험의 차이와 판정 기준

| 시험 | 고피 실행 방법 | 팔·리프트 상태 | 보는 것 |
|---|---|---|---|
| 1 기본 | `launch_scene.py` | 장면 저장 상태 그대로 | 기준값: 리그 상자 안 점 개수, 주행 성공 여부, 소요 시간 |
| 2 운반 자세 | `launch_scene.py --pose carry` | `config/arm_poses.yaml` 의 근사 운반 자세 + 리프트 0.30 m, 팔레트 없음 | 자세만으로 점 개수가 느는지 (팔·포크가 라이다 시야에 드는지) |
| 3 실제 파지 | 팀 `standalone_app.py --autoplay` + `PICK_HARVEST` 명령 | 팀 로직으로 Pallet_01 파지 → 운반 회전 → travel 높이 | 팔레트·엽채류가 점으로 잡히는지, `cloud_self_filter` 가 다 지우는지, 주행 성공 여부 |

판정:
- `nav2_link_check` 의 `[6] self returns inside the rig box` 값을 세 시험에서 기록함. 0 이 아니어도 `cloud_self_filter` 가 지우므로 주행에는 영향 없어야 함. **상자 밖**(x < −1.0 뒤쪽이나 y 폭 ±0.45 밖)으로 팔레트가 나가면 `/scan` 에 남아 Nav2 가 발밑 장애물로 봄 → 그때는 5절 처리.
- 주행 판정: `RESULT SUCCEEDED for FEEDER_DOCK`, `recoveries 0`, 출발 후 30 s 이상 정지 없음, 도킹 후 Isaac 화면에서 차체 뒤끝과 TurnTable 사이 약 0.25 m. 출발 위치는 세 시험 모두 장면 초기 위치이며 손대지 않음(시험 3 의 파지 동작은 제자리에서 이루어짐).
- 세 시험 모두 같은 bag 세션에 기록됨(6절). 시험마다 터미널 3 을 다시 띄우면 bag 도 시험별로 나뉨.

## 2. 고피 — 터미널 1 (Isaac Sim 만)

터미널 환경은 튜터 지시대로 준비함. 시험별로 아래 한 줄만 다름.

**시험 1 (기본)**
```bash
isaac_python /home/rokey/ROKEY_P3_A1/cobot3_ws/src/smart_farm_navigation/scripts/launch_scene.py
```

**시험 2 (운반 자세, 팔레트 없음)**
```bash
isaac_python /home/rokey/ROKEY_P3_A1/cobot3_ws/src/smart_farm_navigation/scripts/launch_scene.py --pose carry
```
- 기대 출력에 `pose applied: joint_1=270.00, … lift_prismatic_joint=0.30` 이 추가됨. 자세가 시험 3 의 실제 운반 자세와 눈에 띄게 다르면 `config/arm_poses.yaml` 의 `carry` 값을 팀 값으로 바꿔 다시 실행함(재빌드 불필요, 스크립트가 파일을 직접 읽음).

**시험 3 (실제 파지, 팀 통합 standalone)**
```bash
isaac_python /home/rokey/ROKEY_P3_A1/cobot3_ws/isaacpjt/smart_farm/runtime/standalone_app.py --autoplay
```
- `READY` 가 뜬 뒤 내피 터미널 4 에서 4절의 `PICK_HARVEST` 명령을 보냄. 팀 앱이 Pallet_01 을 집고 운반 회전·리프트 상승까지 마치면 `/sim_task/result` 에 `status: SUCCEEDED`, `safe_to_navigate: true` 가 오고 바퀴 브레이크가 풀림. 그 다음에 Nav2 주행을 지시함.
- 이 앱은 Isaac 내부에서 rclpy 를 쓰므로 고피 셸의 `ROS_DOMAIN_ID` 가 101 이어야 함(`.bashrc` 기본값).

공통 기대 출력(시험 1·2): `opening …/Collected_smartfarm_v011.usd` → `robot prim …/nova_carter_ROS: x=-0.42 y=1.01 … yaw=90.0deg` → `/clock: scene already has a ROS2PublishClock node` → `3D lidar fullScan=True on 1 helper node(s)` → `PLAY`.
- Isaac 을 다시 Play 하거나 다시 실행하면 내피 터미널 3(Nav2)도 반드시 다시 실행함(시뮬레이션 시계가 0 으로 돌아감).
- Isaac 창은 최소화하지 않음.

## 3. 내피 — 터미널 2 (빌드, 연결 점검) / 터미널 3 (Nav2 + RViz2)

터미널 2 (한 번만 빌드, 시험마다 점검):
```bash
export ROS_DOMAIN_ID=101
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE=/home/rokey/.ros/fastdds_whitelist.xml
source /opt/ros/jazzy/setup.bash
cd /home/rokey/ROKEY_P3_A1/cobot3_ws
colcon build --packages-select smart_farm_interfaces smart_farm_navigation
source /home/rokey/ROKEY_P3_A1/cobot3_ws/install/setup.bash
ros2 run smart_farm_navigation nav2_link_check 2>&1 | tee /home/rokey/ROKEY_P3_A1/cobot3_ws/src/smart_farm_navigation/results/link_$(date +%Y%m%d_%H%M).txt
```
기대: `/clock`·`/tf`·`/chassis/odom` 20 Hz 이상, `/front_3d_lidar/lidar_points` 약 10 Hz(시험 3 은 20 Hz 이상), `[6] self returns inside the rig box: N of M points/scan (rear a, mid b, front c; highest z …)`, 마지막 `RESULT OK - nav2.launch.py scan_mode:=cloud`. **N 과 highest z 를 시험별로 적어 둠.**

터미널 3 (같은 `export` 3줄 + `source` 2줄 먼저, 시험마다 새로 실행):
```bash
ros2 launch smart_farm_navigation nav2.launch.py 2>&1 | tee /home/rokey/ROKEY_P3_A1/cobot3_ws/src/smart_farm_navigation/results/nav2_$(date +%Y%m%d_%H%M).txt
```
기대: `[nav2.launch] rosbag -> …/results/bags/nav2_…`, `scan_mode auto -> cloud`, `cloud_self_filter: 10.0 Hz, M points/scan, N self points removed/scan` (5초마다), `Managed nodes are active`. RViz2 에서 카터가 Rack_1·Rack_4 사이에 있고 스캔 점이 랙·TurnTable 가장자리에 붙어야 함.

## 4. 내피 — 터미널 4 (주행 지시)

같은 5줄 환경을 먼저 실행함.

**시험 3 에서만, 주행 전에** Pallet_01 파지 명령 (팀 앱의 `/sim_task/command`):
```bash
ros2 topic pub -t 3 -r 1 /sim_task/command smart_farm_interfaces/msg/TaskCommand "{task_id: 'TASK-20260922-003', command_id: 'TASK-20260922-003-CMD-001', operation: 'PICK_HARVEST', recipe_id: 'HARVEST_RACK_L1', pallet_id: 'PALLET_001', source: 'RACK_L1', destination: 'CARRY'}"
```
```bash
ros2 topic echo /sim_task/result smart_farm_interfaces/msg/TaskResult --once
```
`status: SUCCEEDED`, `safe_to_navigate: true` 를 확인한 뒤 아래로 진행함. 실패(`reason`)가 나오면 그 출력을 `errored/` 에 두고 시험 3 은 중단함(팀 앱 쪽 문제).

**주행 (세 시험 공통, 방법 A)**
```bash
ros2 run smart_farm_navigation go_to_station --ros-args -p station:=FEEDER_DOCK 2>&1 | tee /home/rokey/ROKEY_P3_A1/cobot3_ws/src/smart_farm_navigation/results/goto_$(date +%Y%m%d_%H%M).txt
```
기대 출력과 화면:
```
local costmap available
pure_nav2: no scripted BackUp/via; one NavigateToPose from the current pose (planner must handle reversing)
goToPose FEEDER_DOCK: (-2.19, -2.72, 90.0deg)
  remaining … m, elapsed …s, recoveries 0      <- 통로에서 뒤로 빠져나와 그대로 후진으로 Feeder 앞까지 감 (모의에서는 전 구간 후진)
RESULT SUCCEEDED for FEEDER_DOCK after …s     <- 팔 쪽이 TurnTable 을 보며 정지
```
- RViz2 의 `Path` 가 출발 직후 통로 남쪽으로 이어지고, 카터가 제자리 회전 없이 뒤로 움직이면 정상임. 통로 안에서 제자리 회전을 시작하면 즉시 Ctrl+C (5절).
- 돌아오기: `station:=RACK_DOCK` (앞으로 출발해 통로로 직진 진입). 시험 3 은 팔레트를 든 채 복귀함.
- 예전 방식(스크립트가 BackUp 을 먼저 지시)과 비교하려면 `-p pure_nav2:=false` 를 붙임.

**방법 B (통합 확인용, 선택)**: 터미널 4 에서 `ros2 launch smart_farm_navigation navigation_node.launch.py` 를 띄우고 다른 터미널에서
```bash
ros2 topic pub -t 3 -r 1 /navigation/command smart_farm_interfaces/msg/TaskCommand "{task_id: 'TASK-20260922-001', command_id: 'TASK-20260922-001-CMD-001', operation: 'NAVIGATION', destination: 'FEEDER_DOCK'}"
```
```bash
ros2 topic echo /navigation/result smart_farm_interfaces/msg/TaskResult --once
```
`-t 3 -r 1`(3번 발행)인 이유: bag 기록기도 이 토픽을 구독하므로 `--once` 는 기록기에만 닿고 노드가 놓칠 수 있음. 같은 `command_id` 는 노드가 한 번만 실행함.

## 5. 문제가 생겼을 때

| 증상 | 조치 |
|---|---|
| `[6] self returns` 는 0 인데 RViz2 `/scan` 에 카터 바로 옆 점이 남음 | 팔레트가 상자 밖으로 나감. `nav2.launch.py` 를 `ros2 launch smart_farm_navigation nav2.launch.py` 대신 아래처럼 상자를 키워 실행: `ros2 run smart_farm_navigation cloud_self_filter --ros-args -p box_x:="[-1.3, 0.3]" -p box_y:="[-0.6, 0.6]"` 를 따로 띄우고 launch 는 `scan_mode:=cloud` 로 유지. 값은 `link_*.txt` 의 rear/mid/front 분포와 highest z 로 정함 |
| `RegulatedPurePursuitController detected collision ahead!` 반복 | 위와 같은 원인이거나 도킹 지점이 costmap 안. RViz2 local costmap 캡처를 `errored/` 에 |
| 통로 안에서 제자리 회전을 시작함 | Hybrid-A* 가 후진 경로를 못 만들고 다른 계획기처럼 행동한 것. `nav2_*.txt` 의 `planner_server` 줄을 `errored/` 에. 임시로 `-p pure_nav2:=false`(스크립트 BackUp) 로 시험을 이어감 |
| `RESULT FAILED … after 0s` 또는 `planner … failed to create a plan` | 시작·도착 자세가 costmap 에서 막힘(팔레트 점이 남았거나 도킹 지점이 너무 가까움). `FEEDER_DOCK` y 를 −2.60 으로 올려 재빌드 |
| 도킹 직전 앞뒤로 왔다 갔다 함 | 경로의 방향 전환(cusp)에서 RPP 가 흔들리는 것. `nav2_params.yaml` 의 `reverse_penalty` 를 1.0 그대로 두고 `change_penalty` 를 0.5 로 올려 방향 전환 횟수를 줄임 |
| 후진 탈출 뒤 30 s 이상 정지 | 13차 수정 대상(회전 정체). 재현되면 `nav2_*.txt`·`goto_*.txt` 를 `errored/` 로 |
| link_check 전부 0 Hz | 도메인(101)·화이트리스트(10.10.0.3) 확인 |
| `timestamps differ on … seconds` / `Goal failed` 즉시 | Isaac 재시작 후 Nav2 미재시작. 터미널 3 재실행 |
| 시험 3 에서 `/sim_task/result` 가 안 옴 | 팀 앱 터미널의 출력을 `errored/` 에. 고피 `ROS_DOMAIN_ID` 확인 |

## 6. rosbag 기록 (자동)

터미널 3 의 `nav2.launch.py` 가 뜨는 순간부터 Ctrl+C 까지 한 세션이 `/home/rokey/ROKEY_P3_A1/cobot3_ws/src/smart_farm_navigation/results/bags/nav2_YYYYmmdd_HHMM/` 에 mcap 으로 기록됨(22개 토픽, 실시간 1분당 약 12 MB, git 제외). 시험 1·2·3 마다 터미널 3 을 다시 띄우므로 bag 도 3개가 됨. 점군까지 담으려면 `record_cloud:=true`(1분당 약 80 MB 추가), 끄려면 `record:=false`.

확인·재생:
```bash
ros2 bag info /home/rokey/ROKEY_P3_A1/cobot3_ws/src/smart_farm_navigation/results/bags/nav2_YYYYmmdd_HHMM
```
```bash
ros2 bag play --clock /home/rokey/ROKEY_P3_A1/cobot3_ws/src/smart_farm_navigation/results/bags/nav2_YYYYmmdd_HHMM
```
재생 중 `rviz2 -d /home/rokey/ROKEY_P3_A1/cobot3_ws/src/smart_farm_navigation/rviz/nav2_smartfarm.rviz` 로 보면 Isaac 없이 되돌려 볼 수 있음. 보고 시 `results/*.txt` 와 bag 디렉터리 이름을 함께 적음.

## 7. 파일 지도

| 파일 | 실행 위치 | 역할 |
|---|---|---|
| `scripts/launch_scene.py` | 고피 | v011 열기, `/clock` 없으면 추가, 3D 라이다 10 Hz, `--pose`/`--arm-joints`/`--lift` 자세 프리셋, Play |
| `config/arm_poses.yaml` | 고피(스크립트가 읽음) | 시험 2 의 `carry` 근사 자세 |
| `launch/nav2.launch.py` | 내피 | Nav2 + RViz2(`rviz/nav2_smartfarm.rviz`) + `/scan` 생성 + rosbag |
| `smart_farm_navigation/cloud_self_filter.py` | 내피 | 결합카터 자체 부피 점 제거 |
| `smart_farm_navigation/nav2_link_check.py` | 내피 | 토픽 도달 + 리그 상자 안 점 개수 |
| `smart_farm_navigation/go_to_station.py` | 내피 | 기본 `pure_nav2`: 목표 하나. `pure_nav2:=false`: 후진 탈출 → 경유점 → 후진 도킹 스크립트 |
| `config/stations.yaml` | 내피 | `FEEDER_APPROACH`/`FEEDER_DOCK`(후진 진입), 후진 탈출 구역 |
| `config/nav2_params.yaml` | 내피 | Smac Hybrid-A*(후진 계획) + RPP `allow_reversing`, 13차 수정, BT 응답 대기 200 ms |
| `launch/navigation_node.launch.py`, `config/destinations_nav2.yaml` | 내피 | 방법 B (팀 `TaskCommand` 노드의 Nav2 모드) |
| `scripts/make_map_from_usd.py` | 내피 | USD → 지도 |

## 8. 13차와의 관계, 내피 모의 결과, 한계

- 13차의 목적(회전 정체·도착 허용치·감속·rosbag 검증)은 시험 1 이 그대로 포함하므로 13차를 따로 수행할 필요 없음. 13차와 다른 점은 Feeder 진입 방향(후진)뿐임.
- 내피 모의(지도 기반 가상 로봇, 3D 점군 + 리프트·팔레트 자기 반사 100점 모사, odom 각속도 0), Nav2 만으로: 초기 위치 → `FEEDER_DOCK` → `RACK_DOCK` 왕복 4회 모두 `SUCCEEDED`, recovery 0, 편도 9~21 s(배율 1). 도킹 위치 오차 0.07 m, 8°. 출발부터 도킹까지 한 경로로 전 구간 후진했음(RPP 가 방향 전환 없이 따라감). 방법 B(`TaskCommand`) 도 `SUCCEEDED`. 자기 반사 100점은 전부 제거됨.
- 전역 costmap 의 inflation 을 0.70 으로 올리면 Hybrid-A* 의 "Inflation layer … not set sufficiently" 경고는 사라지지만 통로 안 복귀가 99 s 로 늘어 0.45 로 되돌림. 이 경고는 무시해도 됨(발자국 검사를 정확한 방식으로 할 뿐임).
- 시험 2 의 `carry` 자세는 근사값임(팀 PICK_HARVEST 의 최종 관절값은 IK 결과라 상수가 없음). 시험 3 화면의 자세와 다르면 `config/arm_poses.yaml` 을 고쳐 시험 2 를 다시 함.
- 시험 3 은 팀 앱이 Isaac 을 소유하므로 3D 라이다 10 Hz 설정이 적용되지 않음. 필요하면 `standalone_app.py` 에 `launch_scene.py` 의 fullScan 6줄을 옮기면 됨(팀 파일이라 이쪽에서 수정하지 않았음).
- XT-32 의 수직 시야는 +15°/−16° 이고 라이다 높이는 0.53 m 임. 운반 높이(베이스 1.04 m 이상)의 팔레트는 라이다에서 1.1 m 이상 떨어져야 시야에 들어오므로, 실제로는 리프트 기둥이 주된 자기 반사이고 팔레트는 거의 안 보일 것으로 예상함. 시험 결과로 확인함.
