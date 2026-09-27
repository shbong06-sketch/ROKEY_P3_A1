# [guidance1 12차] 1차 시스템 통합 리허설 — Collected_smartfarm_v003.usd + /cmd_vel 주행 (2026-09-20 갱신)

설계 문서: `docs/integration_plan.md`. 이 판은 v003 장면과 팀장 스크립트(run_world.py, rig_mode.py) 기준으로 갱신함.

## 0. 현재 상태 요약
| 항목 | 상태 |
| --- | --- |
| 메인 장면 | `isaacpjt/smart_farm/scenes/Collected_smartfarm_v003/Collected_smartfarm_v003.usd`. 로봇은 LiftRig 1대(Nova Carter + 리프트 + M0609), namespace 없음. carter yaw 는 통로 탈출 방향으로 회전됨(팀장 수정, git 반영 대기) |
| 시뮬레이션 standalone | 팀장 `run_world.py`(bridge 활성화 + 바닥 보정 + 바퀴 자동 브레이크). 기본 월드 이름이 `World0_carter_tuned.usd` 라 `--world Collected_smartfarm_v003.usd` 를 줘야 함. 팔 동작(robot_motion.py)은 팀장이 여기에 결합 예정 |
| 주행 | `path_runner_smooth`(곡선) 또는 `path_runner`(직선·제자리회전). 경유지는 v003 기하로 갱신됨 |
| /clock | v003 에는 ROS_Clock 그래프가 없음. /cmd_vel 주행에는 영향 없음 |
| carter2 | v003 에 두 번째 로봇이 없어 `carter2_dock.launch.py` 는 보류 |

## 1. 사전 점검 (터미널 1개)
```bash
ros_set
cd /home/rokey/ROKEY_P3_A1/cobot3_ws
rm -rf build/smart_farm_navigation install/smart_farm_navigation
colcon build --packages-select smart_farm_navigation
source install/setup.bash
ros2 pkg executables smart_farm_navigation
```
- 기대: path_runner, path_runner_smooth, scene_check.

## 2. 통합 실행 (터미널 3개, 모두 `ros_set`만)

### [터미널 1] 시뮬레이션 본체 (팀장 스크립트)
```bash
ros_set
cd /home/rokey/ROKEY_P3_A1/cobot3_ws/isaacpjt/smart_farm/scenes/Collected_smartfarm_v003
isaac_python run_world.py --world Collected_smartfarm_v003.usd
```
- Isaac Sim 창에서 Play. 콘솔에 `[리그] .../nova_carter_ROS`, `[리그 모드] 작업 (바퀴 브레이크 ON)` 이 보이면 정상. /cmd_vel 이 오면 자동으로 주행 모드로 바뀜.
- 주행 단독 시험만 할 때는 대신 `isaac_python /home/rokey/ROKEY_P3_A1/cobot3_ws/src/smart_farm_navigation/scripts/launch_scene.py <장면 경로>` 를 써도 됨(브레이크 없음).

### [터미널 2] 통신 점검
```bash
ros_set
source /home/rokey/ROKEY_P3_A1/cobot3_ws/install/setup.bash
ros2 run smart_farm_navigation scene_check 2>&1 | tee -a /home/rokey/ROKEY_P3_A1/cobot3_ws/src/smart_farm_navigation/results/check_$(date +%Y%m%d_%H%M).txt
```
- 기대: /cmd_vel subscribers=1, odom → base_link, RESULT: OK. `[1b] /clock publishers=0` 은 v003 에서 정상.

### [터미널 3] carter → INSPECTION_DOCK
```bash
ros_set
source /home/rokey/ROKEY_P3_A1/cobot3_ws/install/setup.bash
ros2 launch smart_farm_navigation path_smooth.launch.py auto_start:=true 2>&1 | tee -a /home/rokey/ROKEY_P3_A1/cobot3_ws/src/smart_farm_navigation/results/carter1_$(date +%Y%m%d_%H%M).txt
```
- 동작: 통로를 정면(탈출 방향)으로 1.5 m 직진 → 곡선으로 0.45 m 오른쪽 축에 정렬 → 직진 접근 → 컨베이어 앞 0.3 m, 검수 M0609 정면에서 정지. 정지 방향 = 탈출 방향.
- 기대 로그: `path: ... curve from (1.50,0.00) to approach (4.10,-0.45), final (4.90,-0.45)` → `Phase TRACK` → `approach point reached` → `final point reached` → `Path COMPLETE`.
- 첫 실행 전 확인: 터미널 1 콘솔 또는 Stage 에서 carter 의 탈출 방향이 남쪽(−y, 컨베이어 쪽)인지. 북쪽이면 `config/path_runner_smooth.yaml` 의 Y 부호를 뒤집고 X 를 다시 잼.
- 정지 후 1 s 뒤 rig_mode 가 바퀴 브레이크를 걸어 팔 작업 중 밀리지 않음.

## 3. 팔 작업 연계 (팀장 결합 후)
- robot_motion.py 방식이면 도킹 창(BASE_X/Y_WINDOW, 월드 좌표)을 INSPECTION_DOCK 위치에 맞추고, carter 정지 확인(0.5 s) 후 계획이 시작됨. 창을 벗어나면 콘솔의 `범위 밖입니다` 값만큼 경유지를 보정해 재실행.

## 4. 기록
- results/ 의 check_, carter1_ 로그를 커밋·푸시함. yaw 를 돌린 v003 장면도 함께 커밋함.
