# guidance2_16차 — 팔레트를 든 카터를 FEEDER_DOCK 에 정확한 위치·자세로 도킹 (라이다 면 검출 기반)

- 작성일: 2026-09-22, 브랜치 `feature/navigation2`. 15차는 `guidance/past/`.
- 목표: 시험 3 상태(Pallet_01 파지)의 카터를 RViz2 에서 `FEEDER_APPROACH` 에 Nav2 Goal 로 보낸 뒤, 정밀 도킹 노드가 자동으로 `FEEDER_DOCK` (−2.19, −2.60), 카터 뒤(팔 쪽)가 TurnTable 앞면을 정면으로 보는 자세(yaw 90°)에 세움.
- 원리: 도킹 구간은 AMCL·Nav2 를 쓰지 않음. `feeder_dock` 노드가 `/scan`(자기 반사 제거된 3D 라이다 단면)에서 TurnTable 북쪽 앞면(길이 1.15 m 직선)을 RANSAC 으로 찾아, 그 면의 가운데 법선 위 1.00 m 지점을 목표로 삼고 `/cmd_vel` 로 (1) 제자리 회전 → (2) 후진 → (3) 면과 직각 맞추기 → (4) 3 cm 이내 거리 미세 조정을 함. 지도 오차·AMCL 방향 흔들림과 무관함.
- 내피 모의: 도킹 결과 위치 오차 3 cm, 방향 1° (`face_dist 1.027 m, yaw_err 0.3°, lat 0.004 m`).

## 0. 18:26 실측에서 도킹이 시작되지 않은 원인 (bag `nav2_20260922_1826` 분석)
- 팀 앱(`standalone_app.py`)으로 띄운 Isaac 은 3D 라이다를 **프레임마다 60° 조각(약 6,900점, 20 Hz)** 으로 발행함(`fullScan` 꺼짐). `launch_scene.py` 는 한 바퀴(41,000점, 10 Hz)로 발행함.
- 조각 하나로 만든 `/scan` 은 723개 방향 중 121개만 값이 있고 나머지는 비어 있음. 그래서 (1) AMCL 이 한 방향 조각으로 위치를 맞춰 도착 방향이 흔들렸고(110° 등), (2) `feeder_dock` 은 뒤쪽 점이 0개라 TurnTable 면을 못 찾아 시작하지 않았음. 카터 위 기물이나 팔레트 때문이 아님(자기 반사 0점).
- 조치 두 가지: (a) `cloud_self_filter` 가 조각을 0.35 s 동안 모아 한 바퀴로 합친 뒤 `/scan` 을 만듦(Isaac 설정과 무관하게 동작). (b) 팀 앱 `standalone_app.py` 의 `open_scene()` 에 `fullScan=True` 6줄을 추가함(세션에만 적용, USD 불변). 팀 파일이므로 병합 시 사라지면 (a) 만으로도 됨.
- `nav2_link_check` 가 `[5] … points/msg` 뒤에 `PARTIAL SLICES` 를 표시하면 (b) 가 빠진 것임. 진행은 가능함.
- `feeder_dock` 은 대기 중 5 s 마다 `idle: amcl dist … , nav2 idle …, face …` 를 찍으므로 왜 시작하지 않는지 터미널 3 에서 바로 볼 수 있음.
- 내피 모의(60° 조각 20 Hz 로 흉내)에서 합치기만으로 도킹 성공: `face_dist 1.024 m, yaw_err 0.06°, lat −0.007 m`.
- Isaac 을 다시 실행하면 터미널 3 도 다시 실행해야 함(18:26 로그 끝의 TF 오류가 그 경우임).

**19:06 실측 (fullScan 적용 후)**: 라이다는 정상(41,000점)이었고 `feeder_dock` 이 도착 2 s 뒤 면을 찾아(`d=1.92 yaw=+12.7 len=1.06`) 시작했으나 6 s 뒤 `FACE_NOT_FOUND` 로 실패, 수동 재시작도 후진 도중 같은 이유로 실패함. bag 재생 결과 검출은 모든 스캔에서 성공했음. 원인은 시간 기준임: 노드가 벽시계로 "0.6 s 안에 새 스캔" 을 요구했는데 고피 실시간 배율 0.33 에서는 스캔이 벽시계 0.9 s 간격으로 와서 늘 "오래된 값" 으로 취급되어 정지·실패함. 시뮬레이션 시계(`use_sim_time`) 기준으로 바꾸고 신선 기준 1.0 s, 미검출 실패 8 s 로 고침. 내피 모의(배율 0.33 재현)에서 도킹 성공: `face_dist 1.028 m, yaw_err 0.18°, lat 0.012 m`. 절차 변경 없음(내피 재빌드만).

## 1. 고피 터미널 1 (팀 통합 standalone, 시험 3 과 동일)
```bash
isaac_python /home/rokey/ROKEY_P3_A1/cobot3_ws/isaacpjt/smart_farm/runtime/standalone_app.py --autoplay
```
`[READY] …` 와 `[대기] /sim_task/command …` 확인. Isaac 의 Stop 은 누르지 않음(팀 앱 crash).

## 2. 내피 터미널 2 (빌드 + 연결 점검)
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
`RESULT OK - … scan_mode:=cloud` 확인.

## 3. 내피 터미널 3 (Nav2 + RViz2 + 도킹 노드 + bag)
```bash
export ROS_DOMAIN_ID=101
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE=/home/rokey/.ros/fastdds_whitelist.xml
source /opt/ros/jazzy/setup.bash
source /home/rokey/ROKEY_P3_A1/cobot3_ws/install/setup.bash
ros2 launch smart_farm_navigation nav2.launch.py 2>&1 | tee /home/rokey/ROKEY_P3_A1/cobot3_ws/src/smart_farm_navigation/results/nav2_$(date +%Y%m%d_%H%M).txt
```
`Managed nodes are active` 와 `[feeder_dock]: [IDLE] waiting (auto: arms within 0.6 m of FEEDER_APPROACH)` 확인. RViz2 에 작업점 화살표가 보임. 2D Pose Estimate 는 찍지 않음.

## 4. 내피 터미널 4 (파지 명령)
```bash
export ROS_DOMAIN_ID=101
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE=/home/rokey/.ros/fastdds_whitelist.xml
source /opt/ros/jazzy/setup.bash
source /home/rokey/ROKEY_P3_A1/cobot3_ws/install/setup.bash
ros2 topic pub -t 3 -r 1 /sim_task/command std_msgs/msg/String '{data: "{\"task_id\": \"TASK-20260922-003\", \"command_id\": \"TASK-20260922-003-CMD-001\", \"operation\": \"PICK_HARVEST\", \"recipe_id\": \"HARVEST_RACK_L1\", \"pallet_id\": \"PALLET_001\", \"source\": \"RACK_L1\", \"destination\": \"CARRY\"}"}'
ros2 topic echo /sim_task/result std_msgs/msg/String --once
```
`"status": "SUCCEEDED"`, `"safe_to_navigate": true` 확인 후 다음으로.

## 5. RViz2 에서 한 번만 클릭
Nav2 Goal 도구로 **파란 `FEEDER_APPROACH` 화살표** 위에서 북쪽(+y)으로 끌어 놓음. `FEEDER_DOCK` 은 클릭하지 않음.

기대(터미널 3):
```
[bt_navigator]: Begin navigating from current location (-0.42, 1.01) to (-2.19, -1.55)
[bt_navigator]: Goal succeeded                                   <- 도착 방향은 틀어져 있어도 됨
[feeder_dock]: [ALIGN_TO_GOAL] started (auto: near FEEDER_APPROACH and Nav2 idle)   <- 도착 후 약 3 s 뒤 자동 시작
[feeder_dock]: [REVERSE] G behind at 1.1 m; face d=2.0 yaw=+11 lat=-0.2
[feeder_dock]: [SQUARE] reached G (...)
[feeder_dock]: [CREEP] square (yaw +1.3); d=1.055 target 1.00
[feeder_dock]: [DONE] {"status": "SUCCEEDED", "face_dist_m": 1.0x, "yaw_err_deg": 0.x, "lat_m": 0.0x}
```
`face_dist_m` 은 base_link 와 TurnTable 앞면의 거리(목표 1.00 ± 0.03 m), `yaw_err_deg` 는 뒤가 면과 직각에서 벗어난 각(목표 ±1.5°), `lat_m` 은 면 가운데에서의 좌우 어긋남임. 이 세 값을 결과로 기록함.

같은 터미널 4 에서 결과를 다시 보려면:
```bash
ros2 topic echo /feeder_dock/result std_msgs/msg/String --once
```

## 6. 문제가 생겼을 때
| 증상 | 조치 |
|---|---|
| Nav2 도착 후 `feeder_dock` 이 시작하지 않음 | AMCL 위치가 `FEEDER_APPROACH` 에서 0.6 m 밖. 터미널 4 에서 수동 시작: `ros2 topic pub --once /feeder_dock/start std_msgs/msg/Empty '{}'` |
| `[FAILED] … FACE_NOT_FOUND` | 뒤쪽 0.5~3.4 m 안에 길이 0.6~1.6 m 직선이 안 보임. RViz2 `/scan` 에서 TurnTable 앞면 점이 보이는지 확인. 카터 뒤가 TurnTable 을 대략(±60°) 향하도록 Nav2 Goal 을 다시 찍은 뒤 수동 시작 |
| `PHASE_TIMEOUT_*` / `TIMEOUT` | 45 s 안에 단계가 안 끝남. `/cmd_vel` 이 다른 노드와 겹치는지(`ros2 topic info /cmd_vel`), 바퀴 브레이크가 풀렸는지(`safe_to_navigate`) 확인 |
| 도킹 위치가 너무 가깝거나 멂 | `nav2.launch.py` 에 인자 없이 `ros2 run smart_farm_navigation feeder_dock --ros-args -p standoff_m:=1.10` 처럼 따로 띄워 재시험 (launch 의 것은 `dock_auto:=false` 로 끔) |
| Isaac 재실행 | 터미널 3 도 재실행 |

## 7. 파일
| 파일 | 역할 |
|---|---|
| `smart_farm_navigation/feeder_dock.py` | 라이다 면 검출 정밀 도킹 노드. 파라미터: `standoff_m`, `arm_radius_m`, 속도·허용치 |
| `launch/nav2.launch.py` | `dock_auto:=true`(기본) 로 `feeder_dock` 자동 실행, bag 에 `/feeder_dock/*` 포함 |
| `smart_farm_navigation/cloud_self_filter.py` | 자기 반사 상자 뒤끝을 −0.85 m 로 줄임(1.0 m 뒤의 TurnTable 면이 지워지지 않게) |
| `scripts/launch_scene.py` | 15차 시험 2 종료 원인(`_Tee.fileno`) 수정 |

## 8. 검토 결과: docs/Nova Carter Autonomous Docking System.md 의 방안
- opennav_docking + FoundationPose: 카메라 딥러닝 인식 파이프라인과 도킹 서버 설정이 필요해 한 시간 안에 검증 불가. opennav_docking 을 외부 인식 없이 쓰면 결국 AMCL 자세로 스테이징을 잡아 같은 오차가 남음. 이번 노드는 그 문서의 "AprilTag/라이다 ICP 도 같은 PoseStamped 로 넣으면 된다" 는 자리에 라이다 직선 검출을 넣은 것과 같은 구조이며, 나중에 opennav_docking 의 `external_detection_pose` 로 옮길 수 있음.
- 2D 라이다: 고피 실측 4회 모두 `/front_2d_lidar/scan` 0 Hz 였고, 2D 라이다 높이(0.42 m)에서는 리프트 기둥이 뒤쪽 시야를 가림. 3D 라이다 단면(`/scan`)이 이미 10 Hz 로 나오고 TurnTable 면(높이 1.17 m)이 잘 잡히므로 그것을 씀.
