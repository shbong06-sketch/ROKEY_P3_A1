# [10차] smartfarm_v1.usd 통로 탈출 → 좌회전 → 컨베이어 앞 정지 경로 실행 가이드

## 목적
- 새 메인 장면 `smartfarm_v1.usd`에서 "전진만으로 통로 탈출 → 좌회전 → 큐브(컨베이어) 코앞 정지(정지 방향 = 통로 진행 방향)"를 수행함.
- 후진·원호 단계는 없어졌으므로 새 노드 `path_runner`가 담당함. 경유지 3개를 순서대로 "제자리 회전 → 직진(중앙선 유지) → 정지"로 잇고 마지막에 방향을 맞춤. 모든 단계는 시간이 아니라 odom 진행량으로 끝나고 4 s 정체 시 중단함.
- `escape_controller`(이전 장면용)는 그대로 남겨 두었음.

## 사전 조건
- 이 문서를 읽는 시점에 feature/navigation 최신 상태임.
- Isaac Sim GUI가 떠 있지 않아야 함. 터미널 3개를 씀.
- 고정 경로
  - 워크스페이스: `/home/rokey/ROKEY_P3_A1/cobot3_ws`
  - 장면: `/home/rokey/ROKEY_P3_A1/cobot3_ws/isaacpjt/smart_farm/scenes/smartfarm_v1/smartfarm_v1.usd`
  - 경로 설정: `/home/rokey/ROKEY_P3_A1/cobot3_ws/src/smart_farm_navigation/config/path_runner.yaml`
  - 결과: `/home/rokey/ROKEY_P3_A1/cobot3_ws/src/smart_farm_navigation/results`

## 내피에서 확인한 장면 값과 가정
| 항목 | 값 |
| --- | --- |
| 랙 4개 | x = ±1.0, 각 0.56 × 1.0 m, y −0.55 ~ +1.48 → 통로 안쪽 폭 1.44 m |
| 컨베이어(1 m 정육면체) | 중심 (−2.44, 6.82), 앞면 y = 6.32 |
| /clock | ROS_Clock 그래프가 장면에 있어 이번부터 발행됨 |

- **가정**: 로봇이 원점 부근, 보이는 정면이 +y(통로 방향)인 상태로 시작함. 커밋된 USD에는 carter와 큐브가 들어 있지 않아 이 가정을 확인하지 못했음. 목표 큐브는 컨베이어 정육면체로 해석하였음.
- 경유지는 "시작 기준 좌표계"(X = 시작 시 보이는 정면, Y = 왼쪽)로 적음. 로봇 시작 위치가 달라도 로봇 기준 상대 이동은 같음.

| 순서 | 경유지 (X, Y) m | 의미 |
| --- | --- | --- |
| 1 | (3.0, 0.0) | 통로를 완전히 빠져나온 지점 |
| 2 | (3.0, 2.44) | 좌회전 후 컨베이어 정면 라인까지 이동 |
| 3 | (5.4, 2.44) | 우회전 후 컨베이어 앞면 0.3 m 앞에서 정지 |
| 끝 | 방향 0° | 시작(통로 진행) 방향과 동일 |

값을 바꿀 때는 `path_runner.yaml`의 `waypoints_x`, `waypoints_y`만 고침. 빌드 없이 쓰려면 3단계 명령에 `params_file:=/home/rokey/ROKEY_P3_A1/cobot3_ws/src/smart_farm_navigation/config/path_runner.yaml`을 붙임.

## [터미널 1] Isaac Sim 실행
```bash
ros_set
isaac_ros
bash /home/rokey/ROKEY_P3_A1/cobot3_ws/src/smart_farm_navigation/scripts/env_check.sh
isaac_python /home/rokey/ROKEY_P3_A1/cobot3_ws/src/smart_farm_navigation/scripts/launch_scene.py /home/rokey/ROKEY_P3_A1/cobot3_ws/isaacpjt/smart_farm/scenes/smartfarm_v1/smartfarm_v1.usd
```
- 기대: env_check OK 또는 WARN, Isaac Sim 창에 랙·컨베이어·로봇이 보이고 "[launch_scene] PLAY" 출력.
- 대체: `isaac` 실행 → File > Open으로 위 장면 → Play.

## [터미널 2] 빌드 및 장면 통신 점검
```bash
ros_set
isaac_ros
bash /home/rokey/ROKEY_P3_A1/cobot3_ws/src/smart_farm_navigation/scripts/env_check.sh
cd /home/rokey/ROKEY_P3_A1/cobot3_ws
rm -rf build/smart_farm_navigation install/smart_farm_navigation
colcon build --packages-select smart_farm_navigation
source install/setup.bash
ros2 pkg executables smart_farm_navigation
ros2 run smart_farm_navigation scene_check 2>&1 | tee -a /home/rokey/ROKEY_P3_A1/cobot3_ws/src/smart_farm_navigation/results/check_$(date +%Y%m%d_%H%M).txt
```
- 기대: executables 3줄(escape_controller, path_runner, scene_check). scene_check에 `[1b] /clock publishers=1`이 새로 보이고 마지막 줄 RESULT: OK.
- 로그의 `[2] /chassis/odom ... yaw`가 0° 근처인지 기록해 둠(시작 방향 확인용).

## [터미널 3] 경로 실행
```bash
ros_set
isaac_ros
bash /home/rokey/ROKEY_P3_A1/cobot3_ws/src/smart_farm_navigation/scripts/env_check.sh
source /home/rokey/ROKEY_P3_A1/cobot3_ws/install/setup.bash
ros2 launch smart_farm_navigation path.launch.py auto_start:=true 2>&1 | tee -a /home/rokey/ROKEY_P3_A1/cobot3_ws/src/smart_farm_navigation/results/path_$(date +%Y%m%d_%H%M).txt
```
- 기대 로그 순서: `Start ... waypoints(start frame)=(3.00,0.00), (3.00,2.44), (5.40,2.44)` → segment 1 TURN → DRIVE → `waypoint 1 reached` → segment 2 … → `waypoint 3 reached` → FINAL_TURN → `Path COMPLETE ... heading error ±2deg 이내`.
- 뷰포트 기대: 통로를 곧게 빠져나온 뒤 제자리 좌회전, 왼쪽으로 이동, 제자리 우회전, 컨베이어 앞에서 정지. 정지 방향은 통로를 나올 때 방향과 같음.
- 재실행 전에는 터미널 1의 Isaac Sim을 Ctrl+C로 닫고 다시 띄워 초기 자세로 되돌림.

## 중단 조건
- rc가 0이 아니면 추가 조작 없이 종료함. 로그를 커밋·푸시하면 내피에서 분석함.
- 로그의 `Path ABORTED ... cross-track ... exceeds` 는 통로 안에서 0.6 m 이상 치우친 것이므로 시작 위치가 통로 중앙이 아닌 경우임.

## 기록과 확인 요청
- results/의 check_*.txt, path_*.txt를 커밋·푸시함.
- **장면 파일 동기화**: `isaacpjt/smart_farm/.gitignore`의 `scenes/smartfarm_v1/*` 때문에 `smartfarm_v1.usd` 변경분이 커밋되지 않는 상태였음. 이번 커밋에서 `!scenes/smartfarm_v1/smartfarm_v1.usd`를 추가했으므로, carter와 큐브가 들어간 최신 장면을 저장한 뒤 `git add`하면 추적됨. 최신 장면이 올라오면 경유지 값을 실제 좌표로 다시 맞추겠음.
- 확인해 주면 좋은 것: 로봇이 실제로 원점 부근에서 +y를 보고 시작하는지, 목표 큐브가 컨베이어 정육면체가 맞는지.
