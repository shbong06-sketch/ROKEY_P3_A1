# [11차] smartfarm_v1.usd Nav2 + RViz2 연동 및 INSPECT_ZONE 자율주행 (터미널 4개)

## 목적
- 10차(path_runner, /cmd_vel 직접 제어)와 별개의 두 번째 트랙. Isaac Sim의 carter를 Nav2 스택(map_server·AMCL·planner·controller)과 RViz2에 연결하고, RViz2의 Nav2 Goal 또는 `go_to_station` 노드로 목적지까지 자율주행함.
- 내피에서 Nav2 Jazzy 전체 스택을 합성 로봇(odom·/clock·LiDAR 점군)으로 시험하여 INSPECT_ZONE 도달까지 확인하였음. 고피에서는 Isaac Sim 실제 센서로 같은 절차를 수행함.

## 구성 요소 (모두 커밋됨)
| 파일 | 역할 |
| --- | --- |
| maps/smartfarm_v1.pgm, .yaml | 장면 기하에서 생성한 점유 지도(랙 4개, 컨베이어). 해상도 0.05 m, 원점 (−8, −8), 16 × 20 m |
| config/nav2_params.yaml | 단일 로봇용 Nav2 파라미터. /scan(3D LiDAR 변환), odom 토픽 /chassis/odom, 통로 폭 1.44 m에 맞춰 속도 0.5 m/s·inflation 0.55 m |
| launch/nav2.launch.py | map_server + AMCL + Nav2 + RViz2 + pointcloud_to_laserscan(/front_3d_lidar/lidar_points → /scan) 일괄 실행. AMCL 초기 자세는 results/robot_spawn.yaml에서 자동 반영 |
| scripts/launch_scene.py | 장면을 열 때 이름에 carter/nova가 든 prim의 월드 자세를 results/robot_spawn.yaml에 기록 |
| config/stations.yaml, go_to_station | 작업점 이름 → map 좌표. `go_to_station -p station:=INSPECT_ZONE` 으로 BasicNavigator 주행 |

## 사전 조건
- feature/navigation 최신 상태. Isaac Sim GUI가 떠 있지 않음. 터미널 4개.
- 장면에 Nova Carter가 `/World/…carter…` 또는 `…nova…` 이름으로 들어 있어야 함(커밋된 USD에는 아직 없음). 없으면 3단계에서 초기 자세를 인자로 직접 줌.
- 10차(path_runner)와 동시에 실행하지 않음. 둘 다 /cmd_vel을 발행함.
- 고정 경로
  - 워크스페이스: `/home/rokey/ROKEY_P3_A1/cobot3_ws`
  - 장면: `/home/rokey/ROKEY_P3_A1/cobot3_ws/isaacpjt/smart_farm/scenes/smartfarm_v1/smartfarm_v1.usd`
  - 결과: `/home/rokey/ROKEY_P3_A1/cobot3_ws/src/smart_farm_navigation/results`

## [터미널 1] Isaac Sim 실행
```bash
ros_set
isaac_ros
bash /home/rokey/ROKEY_P3_A1/cobot3_ws/src/smart_farm_navigation/scripts/env_check.sh
isaac_python /home/rokey/ROKEY_P3_A1/cobot3_ws/src/smart_farm_navigation/scripts/launch_scene.py /home/rokey/ROKEY_P3_A1/cobot3_ws/isaacpjt/smart_farm/scenes/smartfarm_v1/smartfarm_v1.usd
```
- 기대: env_check에 Nav2 패키지 6개 "설치됨". 출력에 `[launch_scene] robot prim /World/…: x=… y=… yaw=…deg` 와 `spawn pose written to …/results/robot_spawn.yaml`, 이어서 `[launch_scene] PLAY`.
- `WARNING: no prim with 'carter'/'nova'` 가 나오면 로봇 prim 이름을 확인하고, 3단계에서 `initial_x:= initial_y:= initial_yaw_deg:=` 를 직접 줌(값은 Isaac Sim Stage에서 로봇 prim의 Translate·Orient).

## [터미널 2] 빌드 및 장면 통신 점검
```bash
ros_set
isaac_ros
bash /home/rokey/ROKEY_P3_A1/cobot3_ws/src/smart_farm_navigation/scripts/env_check.sh
cd /home/rokey/ROKEY_P3_A1/cobot3_ws
rm -rf build/smart_farm_navigation install/smart_farm_navigation
colcon build --packages-select smart_farm_navigation
source install/setup.bash
ros2 run smart_farm_navigation scene_check 2>&1 | tee -a /home/rokey/ROKEY_P3_A1/cobot3_ws/src/smart_farm_navigation/results/check_$(date +%Y%m%d_%H%M).txt
```
- 기대: `[1b] /clock publishers=1`, RESULT: OK. /clock이 0이면 장면에 ROS_Clock 그래프가 없는 것이므로 Nav2를 띄우지 않음(use_sim_time이 멈춤).

## [터미널 3] Nav2 + RViz2 실행
```bash
ros_set
isaac_ros
source /home/rokey/ROKEY_P3_A1/cobot3_ws/install/setup.bash
ros2 launch smart_farm_navigation nav2.launch.py 2>&1 | tee -a /home/rokey/ROKEY_P3_A1/cobot3_ws/src/smart_farm_navigation/results/nav2_$(date +%Y%m%d_%H%M).txt
```
- 초기 자세를 직접 줄 때: `ros2 launch smart_farm_navigation nav2.launch.py initial_x:=0.0 initial_y:=0.0 initial_yaw_deg:=90.0`
- 첫 줄에 `[nav2.launch] AMCL initial pose (x, y, yaw) from …` 가 찍힘. 어디서 왔는지(launch arguments / robot_spawn.yaml / stations.yaml) 확인.
- RViz2 확인 항목(배운 순서대로): 지도(랙 4개·컨베이어 블록) → LaserScan 빨간 점이 랙 위치와 겹침 → Navigation2 패널 Navigation·Localization active → Global/Local costmap 표시 → 로봇 위치가 지도 위 실제 위치와 일치. 로봇 아이콘이 랙 사이(통로) 안에 있어야 함.
- 상태 확인 명령(터미널 2에서):
```bash
ros2 lifecycle get /bt_navigator
ros2 topic echo /amcl_pose --once --field pose.pose
```
- 로봇 위치가 틀리면 RViz2 상단 `2D Pose Estimate` 로 실제 위치·방향을 찍어 줌.

## [터미널 4] 자율주행
방법 A. RViz2 상단 `Nav2 Goal` 로 지도 흰 영역을 클릭·드래그.

방법 B. 작업점 이름으로 주행:
```bash
ros_set
isaac_ros
source /home/rokey/ROKEY_P3_A1/cobot3_ws/install/setup.bash
ros2 run smart_farm_navigation go_to_station --ros-args -p station:=INSPECT_ZONE 2>&1 | tee -a /home/rokey/ROKEY_P3_A1/cobot3_ws/src/smart_farm_navigation/results/goto_$(date +%Y%m%d_%H%M).txt
```
- 기대: `initial pose set …` → `Nav2 active` → `goToPose INSPECT_ZONE: (-2.44, 5.40, 90.0deg)` → 남은 거리 감소 → `RESULT SUCCEEDED`. 로봇은 통로를 +y로 빠져나와 왼쪽으로 돌아 컨베이어 앞 0.9 m에서 +y를 보고 정지함.
- 작업점 목록과 좌표는 `config/stations.yaml` (CORRIDOR_EXIT, INSPECT_ZONE, RACK_FRONT). 좌표를 바꾸면 `colcon build` 후 적용.
- Nav2가 로봇을 base_link +x 방향으로 전진시키므로, 7·8차에서 "보이는 정면"이라 판단한 쪽과 반대 끝이 앞서 갈 수 있음. 동작에는 문제 없음.

## 중단 조건
- 터미널 3에서 어떤 노드든 `[ERROR]`로 종료되면 Ctrl+C 후 로그 파일을 커밋·푸시함. 다시 띄우기 전 터미널 1의 Isaac Sim도 재시작함(초기 자세 복원).
- RViz2에서 로봇이 지도 밖이나 랙 위에 그려지면 초기 자세가 틀린 것임. `2D Pose Estimate`로 바로잡거나 3단계를 인자와 함께 다시 실행.

## 기록
- results/의 check_, nav2_, goto_ 로그를 커밋·푸시함. 최신 smartfarm_v1.usd(carter·큐브 포함)도 함께 커밋함(.gitignore 예외 추가됨).

## 다음 단계(카터2 시나리오)
- 카터2: 맵 바깥에서 등장 → RACK_FRONT → 통로 안 목표 랙 앞 저속 정지 → M0609 pick & place. 다중 로봇은 수업 자료의 node_namespace 방식(carter1/carter2 + params 2벌)을 적용함. 이번 11차가 고피에서 통과하면 착수함.
