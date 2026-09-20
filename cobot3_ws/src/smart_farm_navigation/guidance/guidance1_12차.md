# [guidance1 12차] 1차 시스템 통합 리허설 — robot_motion.py + carter1/carter2 /cmd_vel 주행

설계 문서: `docs/integration_plan.md`. 장면 `Collected_smartfarm_v001.usd` 확정 후 수행함. 확정 전에는 1절만으로 준비 상태를 점검함.

## 0. 파일 위치 정리
| 질문 | 답 |
| --- | --- |
| 주행 시험용 Isaac standalone | `/home/rokey/ROKEY_P3_A1/cobot3_ws/src/smart_farm_navigation/scripts/launch_scene.py` |
| 주행 로직(ROS 노드) | `smart_farm_navigation/path_runner.py`, `path_runner_smooth.py` |
| 팔 standalone(팀) | `/home/rokey/ROKEY_P3_A1/cobot3_ws/isaacpjt/smart_farm/scripts/robot_motion.py` |
| 통합 시 장면을 여는 standalone | robot_motion.py 하나 (launch_scene.py 는 쓰지 않음) |

## 1. 사전 점검 (장면 확정 전에도 가능)
```bash
ros_set
cd /home/rokey/ROKEY_P3_A1/cobot3_ws
rm -rf build/smart_farm_navigation install/smart_farm_navigation
colcon build --packages-select smart_farm_navigation
source install/setup.bash
ros2 pkg executables smart_farm_navigation
ros2 launch smart_farm_navigation carter2_dock.launch.py --show-args
```
- 기대: executables 5개(escape_controller, go_to_station, path_runner, path_runner_smooth, scene_check). launch 인자 목록 출력.

## 2. robot_motion.py 수정 (팀장과 함께, docs/integration_plan.md 2절)
- SimulationApp 직후 `enable_extension("isaacsim.ros2.bridge")` + `app.update()`.
- SCENE_PATH → Collected_smartfarm_v001.usd, ROBOT_PATH → carter2 의 M0609 prim, physics_prim_path 확인.

## 3. 통합 실행 (터미널 4개, 모두 `ros_set`만)

### [터미널 1] 시뮬레이션 본체
```bash
ros_set
isaac_python /home/rokey/ROKEY_P3_A1/cobot3_ws/isaacpjt/smart_farm/scripts/robot_motion.py
```
- Isaac Sim 창에서 Play. 콘솔에 `[대기] AMR 정지를 기다리는 중` 이 주기적으로 찍히면 정상.

### [터미널 2] 통신 점검
```bash
ros_set
source /home/rokey/ROKEY_P3_A1/cobot3_ws/install/setup.bash
ros2 topic list | grep -E "carter[12]" | sort
ros2 run smart_farm_navigation scene_check --ros-args -p cmd_vel_topic:=/carter2/cmd_vel -p odom_topic:=/carter2/chassis/odom 2>&1 | tee -a /home/rokey/ROKEY_P3_A1/cobot3_ws/src/smart_farm_navigation/results/check_$(date +%Y%m%d_%H%M).txt
```
- 기대: `/carter1/cmd_vel`, `/carter1/chassis/odom`, `/carter2/cmd_vel`, `/carter2/chassis/odom` 이 보이고 scene_check RESULT: OK. (namespace 를 안 쓴 단일 로봇이면 토픽에 접두어가 없음. 그 경우 4·5단계의 topic 인자를 생략.)

### [터미널 3] carter1 → INSPECTION_DOCK(검수 도킹점)
```bash
ros_set
source /home/rokey/ROKEY_P3_A1/cobot3_ws/install/setup.bash
ros2 launch smart_farm_navigation path_smooth.launch.py auto_start:=true 2>&1 | tee -a /home/rokey/ROKEY_P3_A1/cobot3_ws/src/smart_farm_navigation/results/carter1_$(date +%Y%m%d_%H%M).txt
```
- namespace 사용 시 launch 대신: `ros2 run smart_farm_navigation path_runner_smooth --ros-args --params-file /home/rokey/ROKEY_P3_A1/cobot3_ws/install/smart_farm_navigation/share/smart_farm_navigation/config/path_runner_smooth.yaml -p auto_start:=true -p cmd_vel_topic:=/carter1/cmd_vel -p odom_topic:=/carter1/chassis/odom`
- 기대: `Path COMPLETE`. carter1 이 통로를 완전히 벗어난 뒤 4단계 진행.

### [터미널 4] carter2 → RACK_DOCK(랙 도킹점) 저속 정지 → pick & place
```bash
ros_set
source /home/rokey/ROKEY_P3_A1/cobot3_ws/install/setup.bash
ros2 launch smart_farm_navigation carter2_dock.launch.py auto_start:=true 2>&1 | tee -a /home/rokey/ROKEY_P3_A1/cobot3_ws/src/smart_farm_navigation/results/carter2_$(date +%Y%m%d_%H%M).txt
```
- 기대: `waypoint 1 reached` → `Path COMPLETE` → 터미널 1 콘솔에 `[도킹 확인] x=… 범위 안` → `── 계획 ──` → `[인양 확인]`, `[안착 확인]`, `[DONE]`.
- 터미널 1에 `도킹 … 범위 밖입니다` 가 나오면 그 x·y·yaw 값을 기록함. carter2_dock.yaml 의 waypoints 를 그만큼 보정해 재실행함(Isaac Sim Stop → Play 후).

## 4. 기록
- results/ 의 check_, carter1_, carter2_ 로그와 터미널 1 콘솔 출력(복사 가능 범위)을 커밋·푸시함.
- carter2_dock.yaml 의 waypoints 최종값과 robot_motion.py 수정본도 함께 커밋함.
