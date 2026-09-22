# guidance2_15차 — 결합카터 Nav2 실측 3종 재실측 (위에서 아래로 그대로 실행)

- 작성일: 2026-09-22, 브랜치 `feature/navigation2`. 14차 실측 실패(`errored/error_260922_1342.md`, bag `results/bags/nav2_20260922_1335`) 분석·수정 반영판. 14차는 `guidance/past/` 로 옮김.
- 세 시험 모두 결합카터의 장면 초기 위치에서 **Nav2 만으로** `FEEDER_DOCK` 까지 감. 목표 지정은 RViz2 의 **Nav2 Goal** 클릭으로 함(터미널 4 없음). 편도 1회이며 복귀는 하지 않음(도착 후 팔레트 Place 는 팀 절차).
- RViz2 지도에 작업점이 화살표·이름으로 표시됨(`FEEDER_APPROACH` 파랑, `FEEDER_DOCK` 초록, `RACK_DOCK` 주황). 그 화살표 위에 같은 방향(북쪽, +y)으로 클릭하면 됨.
- 장면 `/home/rokey/ROKEY_P3_A1/cobot3_ws/isaacpjt/smart_farm/scenes/Collected_smartfarm_v011/Collected_smartfarm_v011.usd`, 지도 `/home/rokey/ROKEY_P3_A1/cobot3_ws/isaacpjt/smart_farm/maps/Collected_smartfarm_v011.yaml`.
- 각 터미널 블록은 그 자체로 완결됨(환경 5줄 포함). 블록을 통째로 복사해 붙이면 됨.

## 0. 14차에서 일어난 일과 고친 것 (읽기만)

| 시험 | 실측에서 본 것 | 원인 (bag 분석) | 조치 |
|---|---|---|---|
| 1 | 출발 직후 `collision ahead` 반복, AMCL 위치가 실제(−0.42, 1.01)와 달리 (−0.14, 0.53)로 잡힘, 2D Pose Estimate 를 3번 다시 찍음. 도킹 지점에 33° 돌아간 채 멈춤 | (1) `launch_scene.py` 는 M0609 관절 드라이브를 잡아 주지 않아 팔이 1분 안에 중력으로 처지며 라이다 시야로 들어옴(자기 반사 114 → 313점/스캔). 처진 팔이 자기 반사 제거 상자(±0.45 m) 밖으로 나가 local costmap 에 카터 자리 자체가 장애물로 찍힘 → `collision ahead`. 같은 점들이 AMCL 도 흔듦. (2) 곡선 경로 끝에서 멈추면 RPP 는 후진 허용 상태에서 제자리 회전을 못 해 방향이 남음 | (1) 팀 `robot_motion.setup_arm_drives` 와 같은 값(강성 1e8)으로 관절을 고정, 제거 상자를 x −1.1~0.6, y ±0.6 으로 확대. (2) `FEEDER_APPROACH` 를 경유해 마지막 1.05 m 가 직선 후진이 되게 하고 도킹 y 를 −2.60 으로(뒤끝 여유 0.39 m), 도착 방향 허용 0.5 rad |
| 2 | `--pose carry` 실행 후 PLAY 직후 Isaac 이 종료 | 관절을 움직이던 articulation API 가 이 실행 방식(World 없음)에서 동작하지 않음 | USD DriveAPI 목표값으로 바꿈(팀 코드와 같은 방식). 실패해도 종료되지 않고 원인을 출력함 |
| 3 | 팀 앱에 `TaskCommand` 타입으로 보냈으나 반응 없음 → Stop 후 팀 앱 crash | 팀 앱의 `/sim_task/command` 는 `std_msgs/String` 에 JSON 을 담는 방식임(이전 가이던스 오류). crash 는 Stop 시 팀 앱 쪽 버그(`lift.stop` 에서 NoneType) | 명령을 String/JSON 으로 정정(5절). Isaac 의 Stop 버튼은 누르지 않음 |
| 속도 | — | — | 최고 0.5 → 0.6 m/s, 가감속은 1.0 → 0.8 m/s² 로 오히려 낮춤(팔레트·엽채류 유지). 요청대로 살짝만 |
| 목표 지정 | — | — | `go_to_station` 대신 RViz2 Nav2 Goal 클릭. `nav2.launch.py` 가 `stations.yaml` 의 작업점을 `/stations_markers`(MarkerArray) 로 발행해 RViz2 에 표시하고, `/goal_pose` 도 bag 에 기록함 |

내피 모의(3D 점군 + 처진 팔·팔레트 반사 모사) 왕복 결과는 8절.

## 1. 시험 1 — 기본 상태

### 1-1. 고피 터미널 1
튜터 지시대로 환경을 준비한 뒤:
```bash
isaac_python /home/rokey/ROKEY_P3_A1/cobot3_ws/src/smart_farm_navigation/scripts/launch_scene.py
```
기대: `robot prim … x=-0.421 y=1.006 … yaw=90.0deg` → `M0609 joint drives held (6 joints …)` → `/clock: scene already has a ROS2PublishClock node` → `3D lidar fullScan=True …` → `PLAY`. 이후 Isaac 창을 최소화하지 않고 Stop 도 누르지 않음.

### 1-2. 내피 터미널 2 (빌드 + 연결 점검)
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
기대: `/clock`·`/tf`·`/chassis/odom` 15 Hz 이상, `/front_3d_lidar/lidar_points` 약 3~10 Hz, `[6] self returns inside the rig box: N of M points/scan (…; highest z …)`, `RESULT OK - … scan_mode:=cloud`. **N 과 highest z 를 적어 둠.** 1분 뒤 한 번 더 실행해 N 이 늘지 않는지 봄(늘면 팔이 여전히 처지는 것 → 6절).

### 1-3. 내피 터미널 3 (Nav2 + RViz2 + bag 기록)
```bash
export ROS_DOMAIN_ID=101
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE=/home/rokey/.ros/fastdds_whitelist.xml
source /opt/ros/jazzy/setup.bash
source /home/rokey/ROKEY_P3_A1/cobot3_ws/install/setup.bash
ros2 launch smart_farm_navigation nav2.launch.py 2>&1 | tee /home/rokey/ROKEY_P3_A1/cobot3_ws/src/smart_farm_navigation/results/nav2_$(date +%Y%m%d_%H%M).txt
```
기대: `rosbag -> …/results/bags/nav2_…`, `scan_mode auto -> cloud`, `AMCL initial pose (-0.421, 1.006, 90.0deg)`, `cloud_self_filter: … self points removed/scan`, `Managed nodes are active`.
- RViz2 에서 카터가 Rack_1·Rack_4 사이에 있고 스캔 점(빨강)이 양쪽 랙 면에 붙어 있으면 그대로 둠. **2D Pose Estimate 는 찍지 않음**(초기 위치는 launch 가 넣었음). 위치가 명백히 다른 곳에 있을 때만 6절.
- Isaac 을 다시 실행했으면 이 터미널도 반드시 다시 실행함.

### 1-4. RViz2 에서 목표 클릭 (터미널 없음)
RViz2 위쪽 도구 모음의 **Nav2 Goal** 을 누른 뒤, 지도 위 작업점 화살표 위에서 마우스를 누른 채 **북쪽(+y, 화면 위)** 으로 끌어 놓음. 클릭 위치가 화살표 머리에서 0.2 m 안, 방향이 북쪽 ±15° 안이면 됨.

**방법 A (권장, 두 번 클릭)**: 파란 `FEEDER_APPROACH` 화살표를 먼저 클릭 → 카터가 통로에서 뒤로 빠져나와 Feeder 북쪽 1.95 m 에 멈추면(터미널 3 에 `Goal succeeded`) → 초록 `FEEDER_DOCK` 화살표를 클릭 → 직선으로 1.05 m 후진해 멈춤. 마지막 구간이 직선이라 도착 방향이 맞음.

**방법 B (한 번 클릭)**: 초록 `FEEDER_DOCK` 만 클릭. 한 번에 가지만 곡선 끝에서 멈춰 방향이 20~30° 틀어질 수 있음(RPP 는 후진 허용 시 제자리 회전을 못 함). 방향이 중요하지 않을 때만.

기대(터미널 3 로그):
```
[bt_navigator]: Begin navigating from current location (-0.42, 1.01) to (-2.19, -1.55)   <- 제자리 회전 없이 뒤로 출발
[bt_navigator]: Goal succeeded
[bt_navigator]: Begin navigating from current location (-2.1x, -1.5x) to (-2.19, -2.60)
[bt_navigator]: Goal succeeded                                                             <- 팔(뒤) 쪽이 TurnTable 을 봄, 뒤끝 여유 약 0.4 m
```
- 통로 안에서 제자리 회전을 시작하면 RViz2 도구 모음의 Nav2 Goal 을 다른 곳에 찍지 말고 터미널 3 을 Ctrl+C (6절).
- 같은 목표를 연달아 두 번 클릭하면 `Received goal preemption request` 후 다시 시작함. 문제 없음.
- 기록: 시험 번호, `link` 의 N/highest z, `Goal succeeded` 여부와 소요 시간(터미널 3 의 시각 차), 도킹 후 Isaac 화면의 차체 뒤끝~TurnTable 거리와 방향.

### 1-5. 정리
터미널 4 완료 → 터미널 3 Ctrl+C(bag 닫힘) → 고피 터미널 1 Ctrl+C. 카터를 초기 위치로 되돌릴 필요 없음(다음 시험은 Isaac 을 새로 열어 초기 위치에서 시작함).

## 2. 시험 2 — 운반 자세(팔레트 없음)

### 2-1. 고피 터미널 1
```bash
isaac_python /home/rokey/ROKEY_P3_A1/cobot3_ws/src/smart_farm_navigation/scripts/launch_scene.py --pose carry
```
기대: 1-1 의 출력 + `pose ramp start: joint_1 180.00->270.00, … lift 0.00->0.24` → 약 8 초 동안 팔이 서서히 90° 돌고 리프트가 올라감 → `pose targets applied: …`.
- 자세 값은 팀 `standalone_app.py` 기준임: HOME [180,0,0,0,0,0] 에서 `CARRY_ROTATE_DEG` 만큼 joint_1 +90°, 리프트는 `TRAVEL_BASE_HEIGHT` 1.0388 m(조인트 0.243 m). 집기 자세의 나머지 관절값은 IK 결과라 상수가 없어 0 으로 둠. `config/arm_poses.yaml` 에서 바꿀 수 있음(재빌드 불필요).
- 이전에는 목표를 한 번에 주어 팔이 강성 1e8 로 순간 이동하며 리프트와 부딪혀 Isaac 이 종료된 것으로 봄. 이제는 8 초에 걸쳐 움직임.
- **또 종료되면**: 이번부터 실행 기록이 `/home/rokey/ROKEY_P3_A1/cobot3_ws/src/smart_farm_navigation/results/launch_scene_YYYYmmdd_HHMM.log` 에 남고(네이티브 crash 도 스택이 기록됨), 이 파일을 `errored/` 로 옮겨 주면 됨. 시험 2 는 건너뛰고 시험 3 으로 진행함.

### 2-2 ~ 2-5
1-2, 1-3(블록 복사), 1-4(RViz2 클릭), 1-5 와 동일. 기록 항목 동일.

## 3. 시험 3 — Pallet_01 실제 파지(팀 통합 standalone)

### 3-1. 고피 터미널 1
```bash
isaac_python /home/rokey/ROKEY_P3_A1/cobot3_ws/isaacpjt/smart_farm/runtime/standalone_app.py --autoplay
```
기대: `[READY] Collected_smartfarm_v011 scene ready …` 와 `[대기] /sim_task/command의 String/JSON 명령을 기다립니다`. 고피 셸의 `ROS_DOMAIN_ID` 가 101 이어야 함. **Isaac 의 Stop 버튼을 누르면 팀 앱이 crash 하므로 누르지 않음**(종료는 Ctrl+C).
- 이 앱은 3D 라이다 10 Hz 설정이 없어 점군이 20 Hz 이상으로 옴. 그대로 진행함.

### 3-2. 내피 터미널 2
1-2 블록 그대로 실행.

### 3-3. 내피 터미널 3
1-3 블록 그대로 실행.

### 3-4. 내피 터미널 4 — 파지 명령 (주행은 RViz2 클릭)
```bash
export ROS_DOMAIN_ID=101
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE=/home/rokey/.ros/fastdds_whitelist.xml
source /opt/ros/jazzy/setup.bash
source /home/rokey/ROKEY_P3_A1/cobot3_ws/install/setup.bash
ros2 topic pub -t 3 -r 1 /sim_task/command std_msgs/msg/String '{data: "{\"task_id\": \"TASK-20260922-003\", \"command_id\": \"TASK-20260922-003-CMD-001\", \"operation\": \"PICK_HARVEST\", \"recipe_id\": \"HARVEST_RACK_L1\", \"pallet_id\": \"PALLET_001\", \"source\": \"RACK_L1\", \"destination\": \"CARRY\"}"}'
ros2 topic echo /sim_task/result std_msgs/msg/String --once
```
기대: 고피 창에서 팔이 Pallet_01 을 집어 올리고 돌린 뒤 리프트가 올라감(1~2분). echo 에 `"status": "SUCCEEDED"`, `"safe_to_navigate": true` 가 나오면 계속. `"FAILED"` 면 그 JSON 과 고피 터미널 출력을 `errored/` 에 두고 시험 3 중단.

이어서 같은 터미널에서 팔레트를 든 상태의 자기 반사를 한 번 더 잼:
```bash
ros2 run smart_farm_navigation nav2_link_check 2>&1 | tee /home/rokey/ROKEY_P3_A1/cobot3_ws/src/smart_farm_navigation/results/link_$(date +%Y%m%d_%H%M).txt
```
그 다음 1-4 대로 RViz2 에서 클릭함(방법 A). link 의 N/highest z 는 시험 1·2 와 비교해 적어 둠.

### 3-5. 정리
1-5 와 동일. 팀 앱은 Ctrl+C 로 종료.

## 4. 결과 보고
`results/` 의 `link_*`, `nav2_*` 와 `results/bags/nav2_*` 디렉터리 이름(클릭한 `/goal_pose` 도 bag 에 들어 있음), 시험별 N/highest z, `Goal succeeded` 여부, 도킹 거리·방향을 커밋·푸시하면 됨(bag 은 git 제외이므로 이름만).

## 5. 팀 통합용 지시 경로 (이번 실측에서는 안 해도 됨)
`go_to_station` 은 이제 이 용도로만 씀. `/navigation/command` 로 지시하면 `navigation_node` 가 `go_to_station` 을 실행함(FEEDER_APPROACH 경유 후 도킹, 방법 A 와 같은 경로). 터미널 4 에서 `ros2 launch smart_farm_navigation navigation_node.launch.py` 를 띄우고 다른 터미널(환경 5줄 후)에서:
```bash
ros2 topic pub -t 3 -r 1 /navigation/command smart_farm_interfaces/msg/TaskCommand "{task_id: 'TASK-20260922-001', command_id: 'TASK-20260922-001-CMD-001', operation: 'NAVIGATION', destination: 'FEEDER_DOCK'}"
ros2 topic echo /navigation/result smart_farm_interfaces/msg/TaskResult --once
```
(`/navigation/command` 는 `TaskCommand` 타입, 팀 앱의 `/sim_task/command` 는 `String` JSON 임. 서로 다름.)

## 6. 문제가 생겼을 때

| 증상 | 조치 |
|---|---|
| link 의 N 이 1분 사이 크게 늚 | 팔이 처지는 것. `nav2_*.txt` 의 `cloud_self_filter` 줄과 link 두 장을 `errored/` 에. 시험은 계속 진행 가능(상자가 넓어져 대부분 제거됨) |
| RViz2 에서 카터가 실제와 다른 곳에 있음 | `2D Pose Estimate` 로 주황 `RACK_DOCK` 화살표 위에 북쪽(+y)으로 한 번만 찍음. 반복해서 찍지 않음 |
| 통로 안에서 제자리 회전 시작 | 터미널 3 Ctrl+C. `nav2_*.txt` 의 `planner_server` 줄을 `errored/` 에. 임시로 5절의 `go_to_station --ros-args -p station:=FEEDER_DOCK -p pure_nav2:=false`(스크립트 후진) 로 시험을 이어감 |
| `collision ahead` 반복 | RViz2 의 local costmap 에 카터 자리 안에 검은 칸이 있으면 자기 반사가 남은 것. bag 이름을 알려주면 상자 값을 다시 정함 |
| 도킹 후 방향이 15° 이상 틀어짐 | 방법 B 였으면 방법 A 로. 방법 A 에서도 그러면 `config/stations.yaml` 의 `FEEDER_APPROACH` y 를 −1.35 로(직선 구간 1.25 m) 늘리고 재빌드 |
| `timestamps differ …` / `Goal failed` 즉시 | Isaac 재실행 후 터미널 3 미재실행. 터미널 3 다시 |
| link 전부 0 Hz | `ROS_DOMAIN_ID` 101, `hostname -I` 의 IP 가 양쪽 whitelist 에 있는지 |

## 7. 파일 지도
| 파일 | 실행 | 역할 |
|---|---|---|
| `scripts/launch_scene.py` | 고피 | v011 열기, M0609 관절 고정, 3D 라이다 10 Hz, `--pose` 자세, Play |
| `config/arm_poses.yaml` | 고피 | 시험 2 `carry` 근사 자세 |
| `launch/nav2.launch.py` | 내피 | Nav2 + RViz2 + `/scan` + 작업점 마커(`station_markers`) + rosbag(`/goal_pose` 포함) |
| `config/nav2_params.yaml` | 내피 | Hybrid-A*(후진), RPP 0.6 m/s, 도착 허용 0.25 m / 0.5 rad |
| `config/stations.yaml` | 내피 | `FEEDER_APPROACH` → `FEEDER_DOCK`(y −2.60) |
| `smart_farm_navigation/cloud_self_filter.py` | 내피 | 자기 반사 제거 상자 x −1.1~0.6, y ±0.6 |
| `smart_farm_navigation/nav2_link_check.py` | 내피 | 토픽 도달 + 상자 안 점 개수 |
| `smart_farm_navigation/go_to_station.py` | 내피 | 팀 통합용(5절). 실측에서는 안 씀 |
| `smart_farm_navigation/station_markers.py` | 내피 | 작업점 MarkerArray 발행 |

## 8. 내피 모의 결과와 한계
- 처진 팔·팔레트 반사를 상자 밖 30점 포함 166점으로 모사한 조건에서, RViz2 클릭과 같은 `/goal_pose` 발행으로 초기 위치에서 출발: 방법 A(두 번) 도킹 (−2.13, −2.56) 방향 92°, 방법 B(한 번) 도킹 (−2.13, −2.57) 방향 62°. 둘 다 `Goal succeeded`, 편도 약 15~25 s(배율 1). 팀 통합 경로(`TaskCommand` → `go_to_station`)도 `reached_station: FEEDER_DOCK`.
- 같은 클릭을 30 s 안에 다시 하면 이전 목표가 끝나기 전이라 preemption 이 걸림. 앞 목표의 `Goal succeeded` 를 보고 다음을 클릭함.
- 도킹 방향 오차 7~11° 는 RPP 가 후진 시 제자리 회전을 못 해서 생김. M0609 Place 가 이 오차를 못 받으면 6절의 직선 구간 연장으로 줄일 수 있음.
- 팀 앱(`standalone_app.py`) 은 이쪽에서 수정하지 않음. Stop 시 crash 와 라이다 10 Hz 미적용은 팀에 전달할 사항임.
