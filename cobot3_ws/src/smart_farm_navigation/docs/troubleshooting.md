# 트러블슈팅 정리 (Nova Carter /cmd_vel 주행 트랙)

- 기준 시점: 2026-09-21
- 범위: Nova Carter 를 Isaac Sim 5.1 + ROS 2 Jazzy 에서 `/cmd_vel` 로 구동하기 시작한 이후. 그 이전(MiR100 검토기 포함)은 제외함.
- 기준 장면: `/home/rokey/ROKEY_P3_A1/cobot3_ws/isaacpjt/smart_farm/scenes/Collected_smartfarm_v004/Collected_smartfarm_v004.usd`
- 원문 로그: `/home/rokey/ROKEY_P3_A1/cobot3_ws/src/smart_farm_navigation/errored/`, `results/`

---

## 1. 실행 환경·통신 (ROS 2 Jazzy ↔ Isaac Sim)

### 1-1. `ros2 run` 이 패키지를 찾지 못함 / 경로 변수 비어 있음
- 증상: `Package 'smart_farm_navigation' not found`, `bash: cd: /cobot3_ws: No such file or directory` (errored/error_260918_2101, 2123)
- 원인: 새 터미널에서 `install/setup.bash` 를 소싱하지 않았거나, 쉘 변수(`$PROJECT_ROOT`)가 비어 있어 경로가 `/cobot3_ws` 로 잘려 나감.
- 조치: 터미널마다 아래 순서를 고정함. 절차서에는 쉘 변수 대신 절대 경로만 적기로 함.
  ```bash
  source /opt/ros/jazzy/setup.bash
  source /home/rokey/ROKEY_P3_A1/cobot3_ws/install/setup.bash
  ```

### 1-2. `--params-file` 인자가 통째로 한 문자열로 전달됨
- 증상: `Couldn't parse params file: '--params-file /home/rokey/cobot3_ws/src/.../escape_controller.yaml'` (errored/error_260918_2034)
- 원인: 따옴표로 `"--params-file 경로"` 를 한 덩어리로 묶어 넘김. rcl 은 이를 옵션 이름으로 해석하지 못함.
- 조치: 옵션과 값을 분리하고, 이후에는 파라미터 지정을 launch 파일(`params_file:=`) 로 옮겨 오타 여지를 없앰.

### 1-3. Isaac Sim 브리지 라이브러리와 Jazzy 가 한 터미널에 섞임
- 증상: `ImportError: cannot import name 'RCLError' from 'rclpy.exceptions'` (errored/error_260918_2101). 노드가 rclpy 를 import 하는 단계에서 즉시 종료.
- 원인: `isaac_ros` 함수로 `LD_LIBRARY_PATH` 에 `isaacsim/exts/isaacsim.ros2.bridge/jazzy/lib` 를 추가한 터미널에서 ROS 노드를 실행함. 브리지 동봉 `.so` 가 `/opt/ros/jazzy` 의 라이브러리보다 먼저 잡히면서 rclpy 내부 심볼이 어긋남.
- 확인법: 노드 실행 후 `cat /proc/<PID>/maps | grep -c isaacsim.ros2.bridge` 가 0 이어야 함.
- 조치: ROS 노드 터미널은 `ros_set` 만 사용하고 `isaac_ros` 는 쓰지 않음. `isaac_ros` 는 Isaac Sim 이 브리지를 로드하지 못할 때만 Isaac 터미널에서 사용함. 확인용 `scripts/env_check.sh` 를 둠.

### 1-4. 두 PC 사이에서 토픽이 보이지 않음
- 증상: 같은 `ROS_DOMAIN_ID` 인데 `ros2 topic list` 에 상대 PC 토픽이 없음.
- 원인 후보 3가지를 순서대로 확인함.
  1. `~/.bashrc` 의 `ROS_DOMAIN_ID` 가 PC 마다 다름(101 vs 103). 터미널마다 `export ROS_DOMAIN_ID=101`.
  2. `FASTRTPS_DEFAULT_PROFILES_FILE` 의 화이트리스트(`~/.ros/fastdds_whitelist.xml`)에 상대 PC IP 대역이 없음. 실제 NIC IP(`ip -4 addr`)를 화이트리스트에 추가.
  3. `RMW_IMPLEMENTATION` 불일치. 양쪽 모두 `rmw_fastrtps_cpp`.
- 조치: 위 3개 + Jazzy 소싱 + 워크스페이스 소싱, 총 5줄을 모든 터미널의 고정 전제 조건으로 정함.

### 1-5. `/clock` 이 없는 장면
- 증상: `Collected_smartfarm_v002` 이후 장면에는 `ROS_Clock` Action Graph 가 없어 `/clock` 이 발행되지 않음. `use_sim_time: true` 로 띄운 노드는 시간이 0 에서 멈춘 채 타임아웃이 걸리지 않거나 TF 가 만료됨.
- 조치: `/cmd_vel` 주행 노드는 wall clock 기준으로 동작하도록 두고(`use_sim_time` 사용 안 함), `scene_check` 에서 `/clock` 발행자 수를 표시만 하도록 함. Nav2 를 붙일 때는 장면에 `ROS_Clock` 그래프를 추가해야 함.

---

## 2. 빌드·패키지 구성 (colcon / launch / 파라미터)

### 2-1. `--symlink-install` 에서 `File exists` 로 빌드 실패
- 증상: `error: [Errno 17] File exists: '/home/rokey/ROKEY_P3_A1/cobot3_ws/build/.../resource/smart_farm_navigation' -> '/home/rokey/ROKEY_P3_A1/cobot3_ws/install/...'` (errored/error_260919_0937)
- 원인: 같은 워크스페이스를 `/home/rokey/cobot3_ws`(심볼릭 링크)와 `/home/rokey/ROKEY_P3_A1/cobot3_ws`(실경로) 두 경로에서 번갈아 빌드함. `build/` 에 남은 심볼릭 링크 경로 문자열이 달라 재생성에 실패.
- 조치: `build/ install/ log/` 를 지우고 실경로 한 곳에서만 빌드. `--symlink-install` 은 쓰지 않음.
  ```bash
  cd /home/rokey/ROKEY_P3_A1/cobot3_ws
  rm -rf build/smart_farm_navigation install/smart_farm_navigation
  colcon build --packages-select smart_farm_navigation
  ```

### 2-2. 네임스페이스를 붙인 노드가 YAML 파라미터를 무시함
- 증상: `/carter2` 네임스페이스로 띄운 `path_runner` 가 YAML 값 대신 코드 기본값으로 동작함.
- 원인: YAML 최상위 키가 `path_runner:` 로 되어 있으면 노드의 완전 이름 `/carter2/path_runner` 와 일치하지 않아 적용되지 않음.
- 조치: 네임스페이스가 붙는 설정 파일은 최상위 키를 `/**:` 로 둠 (`config/carter2_dock.yaml`).

### 2-3. `pkill -f` 가 자기 자신을 종료시킴
- 증상: 정리용 스크립트에서 `pkill -f path_runner` 를 실행하면 스크립트 쉘 자체가 함께 죽음(종료 코드 144).
- 원인: `pkill -f` 는 명령행 문자열 전체를 검색하므로, 그 문자열이 들어 있는 호출 쉘도 대상에 포함됨.
- 조치: 패턴을 `pkill -f "[p]ath_runner"` 처럼 대괄호로 쓰거나, launch 가 만든 PID/프로세스 그룹을 기록해 두고 그 번호로 종료함.

### 2-4. USD 참조 경로 경고
- 증상: 빌드·실행 시 `Unresolved reference prim path .../SubUSDs/onrobot_rg2_physics.usd@</visuals/world>` 경고 다수 (errored/error_260919_0937 상단).
- 원인: 장면을 Collect 로 묶을 때 그리퍼 서브 USD 의 내부 참조가 함께 복사되지 않음. 주행에는 영향 없고 그리퍼 시각 메시만 빠짐.
- 조치: 주행 트랙에서는 무시. 팀 장면 담당이 Collect 시 `onrobot_rg2_physics.usd` 를 포함해 다시 내보내야 함.

---

## 3. 주행 동작 (경로 추종 · 정지 판정)

### 3-1. 직진 중 벽 쪽으로 서서히 쏠림
- 증상: 통로(폭 1.72 m)를 지나는 동안 좌·우 랙 쪽으로 밀려 스치거나 멈춤.
- 원인: 초기 버전은 "목표 방향각만 유지" 하는 방식이라, 출발 시 생긴 횡오차가 누적되어도 복귀하지 않음.
- 조치: 출발점~목표점을 잇는 직선을 기준선으로 두고 횡오차(cross-track)를 되돌리는 항을 추가함. 관련 파라미터는 `cross_track_gain`, `cross_track_max_angle_rad`, `max_cross_track_m`(초과 시 중단).

### 3-2. 컨베이어 앞에서 뒤쪽이 컨베이어를 훑음
- 증상: 도착점에서 최종 회전을 하면 차체 뒤쪽이 컨베이어 가장자리를 스침.
- 원인: 도착점을 컨베이어에 너무 붙여 잡고, 회전 반경(차체 길이 약 0.9 m)을 고려하지 않음.
- 조치: 도착점 앞 `approach_length_m`(0.8 m) 지점에 먼저 도달한 뒤 직진으로 접근하고, 도착점 자체를 컨베이어 북쪽 가장자리(y = -4.55)에서 0.4~0.5 m 떨어뜨림. 곡선 구간은 `lookahead_m` 0.5 의 pure pursuit 로 바꿔 급회전을 없앰.

### 3-3. 회전·직진 중 정지해 있는데 종료되지 않음
- 증상: 랙 모서리에 걸린 상태에서 바퀴만 돌고 노드가 끝나지 않음.
- 원인: 거리 기준 완료 판정만 있고, 진행이 없을 때의 판정이 없음.
- 조치: `stall_timeout_s`(4 s) 동안 `stall_min_progress`(0.01 m) 미만 진행이면 ABORTED 로 종료하고 종료 코드 2 를 반환. 각 단계에는 `phase_timeout_factor`, `phase_timeout_min_s` 로 상한 시간을 둠.

### 3-4. `scene_check` 가 `/cmd_vel` 구독자를 0 으로 보고함
- 증상: Isaac Sim 이 Play 중인데도 점검 결과가 FAIL.
- 원인: 노드 시작 직후 DDS 디스커버리가 끝나기 전에 구독자 수를 셈.
- 조치: 3 초 spin 후 집계하도록 수정.

### 3-5. Isaac Sim 쪽 standalone 두 개가 같은 장면을 잡음
- 증상: 팀 스크립트(`run_world.py`)와 별도 실행 스크립트를 동시에 띄우면 `/cmd_vel` 이 두 곳에서 처리되거나 두 번째가 장면을 열지 못함.
- 원인: 한 시뮬레이션은 한 `SimulationApp` 만 소유할 수 있음.
- 조치: Isaac Sim 은 `run_world.py --world Collected_smartfarm_v004.usd` 하나만 실행하고, 주행·지시 수신 노드는 모두 외부 PC 의 ROS 노드로 분리함. Isaac 내부에서는 rclpy 를 import 하지 않음(Python 3.11 vs Jazzy rclpy 3.12).

---

## 4. 추가: 좌표계 혼동 (Collected_smartfarm_v004.usd 기준)

전제: 장면은 현장에서 계속 바뀌고 있어 이 문서를 쓴 시점의 v004 파일이 최신본과 다를 수 있음. 아래 수치 중 "현장 확인" 표시는 최신 장면에서 다시 재야 하는 값임.

### 4-1. 무엇이 헷갈렸는가
세 가지 좌표계가 동시에 등장하는데, YAML 의 값이 어느 좌표계인지가 섞였음.

| 좌표계 | 기준 | 어디에 쓰이는가 |
|---|---|---|
| 월드 (USD) | 장면 원점 | 랙·컨베이어·검수 영역 위치, 로봇 배치(`LiftRig` 의 translate / rotate) |
| `base_link` | 로봇 섀시. `+x` 가 섀시 앞 | `/chassis/odom` 의 x, y, yaw. `/cmd_vel` 의 `linear.x` 부호 |
| 시작 기준 (YAML) | 출발 순간의 **주행 방향**을 X, 그 왼쪽을 Y | `waypoints_x`, `waypoints_y`, `final_heading_deg` |

- `waypoints_*` 는 월드 좌표가 아님. 로봇이 장면 어디에 있든 "지금 위치에서 주행 방향으로 X, 왼쪽으로 Y" 임. 장면이 바뀌어 로봇 배치가 달라져도 YAML 의 X, Y 는 로봇을 따라감.
- `drive_direction_sign` 은 "주행 방향 = `base_link` 의 +x(1.0) 인지 −x(−1.0) 인지"만 정함. 이 값이 바뀌어도 `waypoints_y` 의 부호는 바뀌지 않음. Y 는 항상 주행 방향의 왼쪽이기 때문임.

### 4-2. v004 에서 바뀐 것
- **전진 방향**: v004 에서 카터가 배치된 채로 바라보는 방향이 그대로 전진해야 하는 올바른 방향임. 따라서 장면을 손댈 일은 없고, `drive_direction_sign` 은 "그 방향으로 전진" 이 되는 값이어야 함. 확인 방법은 4-4 참고.
- **목적지**: `smart_farm_nav2_01_test.usd` 에서는 컨베이어 대신 놓은 cube 앞이 목적지였음. v004 는 실제 컨베이어 벨트 모양의 prim 이 목적지이고, 멈춰야 하는 검수 영역도 위치가 바뀜.
- **꺾이는 방향**: "전진 → 곡선 → 전진" 은 같으나, `smart_farm_nav2_01_test.usd` 에서는 화면상 **오른쪽**으로 갔고 v004 에서는 화면상 **왼쪽**으로 감. 즉 곡선 이후 도착점이 주행 방향의 왼쪽에 있으므로 `waypoints_y` 의 마지막 값 부호가 **양수(+)** 로 바뀌어야 함. 이전 값 `-0.45` 를 그대로 두면 오른쪽으로 벗어남.

### 4-3. 혼동이 생긴 지점
- 화면에서 보이는 "오른쪽/왼쪽" 은 카메라 시점에 따라 달라지고, YAML 의 Y 는 **로봇 주행 방향 기준 왼쪽** 임. 카메라가 로봇 뒤에서 진행 방향을 보고 있을 때만 둘이 일치함.
- 장면이 바뀌면 (1) 시작 위치, (2) 전진 방향, (3) 목적지 위치가 모두 바뀌므로, `waypoints_x` 의 길이와 `waypoints_y` 의 부호·크기를 매번 다시 재야 함. 이전 장면 값이 남아 있으면 "코드는 정상인데 엉뚱한 곳으로 간다" 로 보임.

### 4-4. v004 확정 값 (`config/path_runner_smooth.yaml`, 2026-09-21)
1차 시연 후 확정한 값임. 검수 영역은 컨베이어 그룹의 왼쪽 끝 prim(`/World/SmartFarm/Placed/Conveyor/Seg_6`)의 가운데이고, 도착 시 컨베이어 벨트가 뻗은 방향(world X)과 수직으로 서서 정면으로 바라봄.

| 항목 | 값 | 근거 |
|---|---|---|
| 카터 시작 | world (−0.40, 1.20), yaw +90° | `LiftRig/Asset/nova_carter_ROS` |
| 주행 정면 | world −Y | 카터가 바라보는 방향 그대로 전진. `drive_direction_sign −1.0` (= `base_link` −x) |
| Seg_6 | pivot (2.328, −5.121), 길이 2.0 m → 가운데 x 3.328 | `integration_v1.py` 와 동일 값 |
| 정차점 | world (3.328, −3.621) | pivot y + standoff 1.5 m |
| `waypoints_x` | `[1.5, 4.821]` | 4.821 = 1.20 − (−3.621) |
| `waypoints_y` | `[0.0, 3.728]` | 3.728 = 3.328 − (−0.40). 양수 = 왼쪽(world +X) |
| `final_heading_deg` | 0.0 | 주행 정면(−Y)이 곧 컨베이어를 정면으로 보는 방향. 회전 없음 |
| `approach_length_m` | 0.8 | 유지 |

같은 정차점을 `integration_v1.py` 가 도착 판정(`CONVEYOR_LATERAL_OFFSET_X_M`, `CONVEYOR_STANDOFF_M`)에 쓰므로 두 파일의 값은 항상 함께 바꿔야 함. 수정 후 `colcon build --packages-select smart_farm_navigation`.

### 4-5. 방향이 맞는지 5초 안에 확인하는 방법
- 시작 로그 `heading ≈ ±180°` 이면 주행 방향 = `base_link` −x (`sign −1.0`, v004 정상). `≈ 0°` 이면 반대로 가는 것임.
- 카터가 바라보는 쪽으로 움직이는지, 곡선에서 화면상 왼쪽으로 꺾이는지 두 가지만 봄.
  - 반대로 전진·후진 → `drive_direction_sign` 만 바꿈.
  - 오른쪽으로 꺾임 → `waypoints_y[1]` 부호만 바꿈.
  - 두 개를 동시에 바꾸지 않음. 하나씩 바꾸고 재빌드 후 다시 봄.


---

## 5. Nav2 + 정밀 도킹 트랙 (feature/navigation2, 2026-09-21 ~ 23)

| 증상 | 원인 | 조치 |
|---|---|---|
| 후진 탈출 뒤 카터가 2~3분 정지 후 출발 | RPP 회전 명령이 odom 각속도 기준 가속 제한(2.0)에 걸려 0.1 rad/s 로 기어감. 진행 판정기가 회전을 무시해 15 s 마다 중단 | `max_angular_accel` 20, `PoseProgressChecker` |
| 도착 0.16 m 앞에서 흔들림 | goal 허용 0.15 m | 0.25 m |
| 통로 안 제자리 회전 → 랙 충돌 | NavFn 이 자세를 무시 | Smac Hybrid-A*(Reeds-Shepp, 후진) + RPP `allow_reversing` |
| 출발 직후 `collision ahead`, AMCL 0.5 m 오차 | `launch_scene.py` 가 M0609 드라이브를 잡지 않아 팔이 처져 라이다에 잡힘 | 관절 드라이브 고정(강성 1e8), 자기 반사 상자 확대 |
| 곡선 경로 끝에서 도착 방향 30° 오차 | RPP 는 후진 허용 시 제자리 회전 불가 | 마지막 구간 직선(경유점) → 최종적으로 `feeder_dock` 으로 대체 |
| 팀 앱으로 띄우면 AMCL 방향 흔들림, 도킹 시작 안 됨 | 3D 라이다가 프레임마다 60° 조각(6,900점)만 발행 → `/scan` 한 방향만 유효 | 팀 앱에 `fullScan=True`, `cloud_self_filter` 점군 합침 |
| 도킹 시작 후 6 s 만에 `FACE_NOT_FOUND` | 벽시계 기준 신선도 0.6 s. 실시간 배율 0.33 에서 스캔이 0.9 s 간격 | 시뮬레이션 시계 기준, 신선 2.5 s |
| 정렬 후 전진 안 함 | fullScan 이어도 일부 스캔에서 뒤쪽 30~60° 섹터가 비어 옴 | 최근 0.25 s 점군 항상 합침 |
| `launch_scene.py --pose` 로 Isaac 즉시 종료 | (1) 관절 목표를 한 번에 주어 팔이 리프트와 충돌, (2) stdout 가로채기에 `fileno` 누락 | 8 s 램프 + DriveAPI 목표, `_Tee.fileno()` |
| 시험 3 파지 명령 무반응 | 팀 앱 `/sim_task/command` 는 `std_msgs/String` JSON | 명령 형식 정정 |
| `ros2 topic pub --once` 를 노드가 놓침 | bag 기록기도 같은 토픽을 구독해 첫 매칭에서 발행 종료 | `-t 3 -r 1` |
| Isaac 재실행 후 `Goal failed`/TF 오류 | 시뮬레이션 시계가 0 으로 돌아감 | Nav2 도 재실행 |
| `Timed out waiting for action server to acknowledge` | BT 응답 대기 20 ms | 200 ms |

**미해결(2026-09-22 22시 녹화 중 1회)**: 도킹 후진 중 방향이 틀어지며 앞으로 나감. 같은 조건 3회 중 1회. 가설: 후진 중 면 검출이 한 스캔에서 다른 직선(TurnTable 옆면 등)에 붙어 목표점 G 가 튀었을 가능성. 대책 후보: 직전 면과 거리 0.4 m·각 20° 이상 다른 검출을 버리는 연속성 검사(`feeder_dock.detect_face` 뒤 한 줄). 재현 bag(`results/bags/nav2_20260922_2134` 또는 `2142`)으로 `detect_face` 를 재생해 확인할 것.

### 5-1. 통합 작업 중 찾은 것 (2026-09-23)

| 증상 | 원인 | 조치 |
|---|---|---|
| 자기 반사 상자 설정이 먹지 않음 | launch 가 `self_box_x_m` 같은 스칼라 이름을 넘겼으나 노드는 `box_x` 두 원소 배열을 선언 | 이름을 `self_box_x/y/z` 두 원소 배열로 통일 |
| 같은 명령을 여러 번 보내면 결과가 BUSY 로 굳음 | 실행 중 같은 `command_id` 가 다시 오면 FAILED/BUSY 로 답하고 그것을 캐시함. 뒤이어 온 진짜 결과가 가려짐 | 같은 명령의 재전송은 무시하고, BUSY 는 최종 결과가 아니므로 캐시하지 않음 |
| 주행 노드가 목표를 보내고도 결과를 못 받음 | `BasicNavigator` 를 타이머 콜백 안에서 쓰면 같은 실행기를 재진입해 깨짐 | NavigateToPose 액션 클라이언트를 노드가 직접 쓰고, 완료는 콜백과 타이머로 확인 |
| 도킹 노드가 종료 시 오류 코드로 죽음 | 종료 신호로 컨텍스트가 먼저 닫히며 나는 `RCLError` 를 잡지 않음 | 종료 중 예외는 무시하고, 컨텍스트가 살아 있을 때만 예외를 드러냄 |
| 주행이 정상인데 시간 초과로 실패 | 속도를 0.3 m/s 로 낮춘 것과 실시간 배율 0.3 이 겹쳐 편도가 벽시계 3 분을 넘김. 제한이 120 s/110 s 였음 | 단계 제한 400 s, 노드 기본 600 s |
| 목적지를 못 찾아 잘못된 명령으로 실패 | 노드 기본 목적지 파일이 1차 시연용이라 `FEEDER_DOCK` 이 없음 | 기본값을 Nav2 목적지 파일로 바꾸고 1차 시연 설정·launch 는 `past/` 로 |
| 도킹 결과를 시작 전에 성공으로 읽음 | 결과 토픽이 래치되어 과거 결과가 새 구독자에게 바로 전달됨 | 시작 요청에 실행 식별자를 담고, 같은 식별자로 돌아온 결과만 인정 |

### 5-2. 통합 실측에서 더 찾은 것 (2026-09-23 저녁)

| 증상 | 원인 | 조치 |
|---|---|---|
| Place 지시 후 팔이 움직이지 않고 결과도 안 보임 | 결과 토픽을 명령 뒤에 구독하면 이미 발행된 결과를 놓친다. 또 차체 정지 확인이 통과하지 못하면 즉시 실패로 끝나 이유가 남지 않았다 | 결과 구독 터미널을 명령보다 먼저 띄우고, 정지 확인을 최대 15 초 대기로 바꿔 진행 상황을 남기게 함 |
| 도킹 지점이 TurnTable 에 지나치게 가까움 | 팀 `robot_motion.BASE_TO_PALLET_X` 가 팔 밑동에서 대상까지 0.89~1.05 m 를 요구하는데, 도킹 0.75 m 에서는 0.86 m 로 범위보다 가까웠다 | 도킹 거리를 0.85 m 로(대상까지 0.96 m). `self_box_x[0]` 도 −0.65 로 함께 이동 |
| `stdbuf` 로 Isaac 실행 실패 | `isaac_python` 은 셸 별칭이라 외부 명령 뒤에 오면 풀리지 않는다 | `isaac_python` 을 첫 낱말로 두고 `PYTHONUNBUFFERED=1` 로 무버퍼 출력 |

### 5-3. 도킹 중 컨베이어에 걸림 (2026-09-23 22차 실측)

| 증상 | 원인 | 조치 |
|---|---|---|
| 후진 1 m 동안 방향이 25도 틀어짐 | 목표점만 겨누는 조향이라 목표점이 가까워질수록 각도 민감도가 커져 발산했다 | 거리에 따라 목표점 추종과 직각 맞추기를 섞고 조향 상한을 0.20 rad/s 로 제한 |
| 도킹 지점에서 제자리 회전 중 멈춤 | 팔이 든 팔레트가 차체 밖으로 나와 있어 컨베이어에 걸렸다. 자기 반사 제거 때문에 라이다로도 미리 알 수 없다 | 도킹 지점에서 회전하지 않음. 방향이 틀어지면 0.6 m 물러나 다시 맞추고 재진입(최대 2회) |
| 걸린 뒤 30초 동안 계속 명령을 냄 | 멈춤 감지가 없었다 | odom 으로 3초간 이동이 없으면 `STALLED_*` 로 즉시 중단 |

### 5-4. 후진 시작 0.6초 만에 멈춤으로 오판 (2026-09-23 23차 실측)

| 증상 | 원인 | 조치 |
|---|---|---|
| Nav2 주행은 성공했는데 도킹이 후진하자마자 `STALLED_REVERSE` 로 끝남 | 5-3 에서 넣은 멈춤 감지가 위치 변화만 봤다. 그 앞 단계인 제자리 회전은 위치가 거의 그대로여서 회전 7초 동안 멈춤 타이머가 계속 흘렀고, 후진 0.6초 만에 3초 기준을 넘었다 | 방향 변화도 이동으로 인정(`stall_turn_rad` 0.02). 단계가 바뀌면 멈춤 기준점을 새로 잡아 타이머를 처음부터 다시 셈 |

기록에 남은 값: `t=223.8 ALIGN_TO_GOAL` → `t=230.9 REVERSE` → `t=231.5 FAILED`. 이 사이 `/cmd_vel` 은 `v=-0.150` 을 정상으로 내고 있었고 대상 면도 1.996 m 로 멀쩡했다. 즉 멈춘 것이 아니라 멈췄다고 판정한 것이다.

Isaac 을 며칠 못 쓰므로 합성 로봇으로 네 가지를 미리 확인했다.

| 시험 | 조건 | 결과 |
|---|---|---|
| 단위 | 제자리 회전 / 후진 / 미세 이동 / 완전 정지 / 회전 후 걸림 | 앞 세 가지는 판정 없음, 정지는 3.1초, 회전 후 걸림은 7.0초 |
| 통합 | 명령 → Nav2 → 도킹 전체 | 성공. 면까지 0.876 m, 방향 오차 1.33도, 좌우 −3 mm |
| 도킹 단독 | 접근 자세에서 도킹만 | 성공. 방향 오차 −0.39도, 좌우 3 mm |
| 끼임 | 후진 도중 차체를 물리적으로 고착 | 고착 시점부터 정확히 3.0초 뒤 `STALLED_REVERSE` |

앞뒤 용어에 대해: 도킹 구간의 `-v` 는 `base_link` 의 −x 방향이며, 리프트가 달린 쪽으로 정면 진입하면 자체 충돌이 나기 때문에 의도적으로 택한 진입 방향이다. 장면에서 눈으로 본 앞뒤와 좌표계의 앞뒤를 초기에 반대로 잡았던 적이 있으므로(4절), 기록을 읽을 때는 항상 좌표계 기준으로 읽는다.

### 5-5. 후진 도착 방향 21도 틀어짐 + Isaac 종료 (2026-09-23 24차 실측, 2026-09-24 분석)

Nav2 주행은 성공, 5-4 의 멈춤 오판도 사라졌다. 그 뒤 두 가지가 겹쳐 실패했다.

| 증상 | 원인 | 조치 |
|---|---|---|
| 제자리 정렬을 마친 순간부터 차체가 15도 더 돌아감 | 카터(LiftRig+팔레트)의 각속도는 명령을 즉시 따르지 않는다. 0.35 rad/s 를 명령해도 3 s 걸려 0.26 까지 오르고, 명령을 끊어도 초당 0.14 rad/s 씩만 준다(Isaac DifferentialController 가속 제한). 정렬 완료를 선언한 순간 차체는 아직 0.24 rad/s 로 돌고 있었다 | 정렬 종료 조건에 "실제 각속도 < 0.03 rad/s" 를 넣고, 조향 명령은 관성으로 더 돌 각도(`wz²/2a`)와 측정 지연(0.5 s)을 뺀 오차로 계산한다(`settled_err`). 시작 전 `SETTLE` 단계에서 Nav2 감속 명령이 끝나고 차체가 멈출 때까지 기다린다(ADR 2.2) |
| 후진 1 m 동안 방향이 반대쪽으로 21도 넘어감 | 위 15도 때문에 후진 0.6 m 만에 도킹 선에서 옆으로 0.16 m 벗어났다. 목표점 G 를 "겨누는" 조향은 남은 거리 0.5 m 에서 그 횡 오차를 갚으려고 20도 이상 틀고, 굼뜬 차체는 G 에 닿기 전에 되돌리지 못한다. bag 의 `/scan` 을 노드와 같은 `detect_face` 로 다시 돌려 그때 제어기가 본 값(G 방위 −17~−24도가 5 s 지속)으로 확인했다 | 조향 목표를 "점" 이 아니라 "면 가운데를 지나는 법선(도킹 선)" 으로 바꿨다. 횡 오차를 갚기 위해 법선에서 벗어나는 방향을 최대 10도로 묶고, 마지막 0.4 m 는 직각만 맞춘다. 후진 0.10 m/s, 조향 상한 0.12 rad/s. 제자리 정렬은 G 가 아니라 더 먼 면 가운데 C 를 겨눈다(같은 횡 오차에 각도가 작다). 도착 시 횡 오차가 0.06 m 를 넘으면 물러나 다시 한다(최대 2회). 물러나는 거리는 0.6 → 0.9 m: ROS 회귀에서 0.6 m 로는 재시도마다 횡 오차가 15% 씩만 줄어 0.07 m 가 남았고, 0.9 m 에서는 모의 90 케이스 전부 0.06 m 안 |
| 정렬 재시도 중 Isaac 이 종료됨 | `RuntimeError: HOLD: 운반 중 팔레트가 30.1 mm 미끄러졌습니다`. 팀 `robot_motion.check` 가 운반 중 팔레트 상대 이동 30 mm 를 넘으면 예외를 던지는데, 앱 main loop 의 `hold()` 호출은 그 예외를 잡지 않아 앱 전체가 죽었다. 반복된 도킹 회전·후진이 팔레트를 밀어낸 것 | `standalone_app.py` main loop 에서 `motion.hold()` 의 `RuntimeError` 를 잡아 한 번만 기록하고 계속 돈다. 관절 목표는 물리 드라이브에 남아 있어 팔은 그대로 유지된다. 다음 PLACE 명령에서 같은 검사가 `MOTION_FAILED` 로 정상 보고한다. 도킹 쪽도 각속도·속도를 낮춰 팔레트가 덜 밀리게 했다 |
| 22차에는 제자리 회전이 거의 안 됨 (0.35 명령에 0.04) | 같은 가속 제한 + 캐스터 마찰. 실측마다 편차가 크다 | 제자리 정렬은 4도까지만 맞추고 나머지는 후진하며 맞춘다(이동 중 조향은 훨씬 잘 듣는다). 제자리 회전이 아예 안 되면 오차 15도 안에서는 그대로 후진으로 넘어간다 |
| `navigation_node` 도킹 제한 시간이 벽시계 | `time.monotonic()` 으로 재고 있었다(ADR 2.2 위반). 재시도가 붙으면 도킹이 sim 70 s = 벽시계 210 s 라 240 s 한계에 닿는다 | 노드 시계(sim)로 바꾸고 300 s / 150 s(sim) 로 조정 |

Isaac 을 며칠 못 쓰므로 검증은 다음 순서로 했다. **모의 통과는 실측 통과가 아니다(ADR 2.7).**

진단의 근거는 모의가 아니라 **기록된 `/scan` 을 노드와 같은 검출기에 다시 넣은 것**이다. 같은 절차를
`sim_test/replay_bag_dock.py` 로 남겨 두었으므로 누구나 다시 볼 수 있다.

```bash
python3 sim_test/replay_bag_dock.py ~/.ros/smart_farm_navigation/bags/nav2_20260923_2154 --from 295 --to 306
```

| bag 시각 | 면까지 | 방향 오차 | 횡 | G 방위 | 실제 cmd_w | odom 각속도 |
|---|---|---|---|---|---|---|
| 295.1 `[REVERSE]` 시작 | 1.99 | +0.1° | −0.01 | +0.3° | +0.116 | **+0.227** |
| 295.7 | 1.99 | −7.4° | +0.27 | −7.9° | −0.085 | +0.114 |
| 296.1 | 1.99 | −8.2° | +0.34 | −11.2° | −0.165 | +0.144 |
| 297~302 | 1.9→1.2 | −15°→+9° | +0.60→0.02 | **−17~−24° 가 5 초 지속** | **−0.200 고정** | −0.10~−0.14 |
| 303.5 | 1.01 | **+20.7°** | −0.28 | −5.9° | −0.036 | −0.045 |

읽는 법: 센서는 멀쩡했다(면을 계속 1.25 m 길이로 잡았고 점 수도 79→139 로 늘었다). `REVERSE` 를
선언한 순간 차체가 아직 0.227 rad/s 로 돌고 있었고, 그 관성이 1 초 만에 방향 −8°·횡 +0.34 m 를 만들었다.
그 뒤 G 방위가 −20° 근처로 5 초간 머무르는데, 목표점을 겨누는 옛 조향은 그동안 상한 −0.200 을 계속
내보냈고, 굼뜬 차체는 도착 전에 되돌리지 못해 반대쪽 +20.7° 로 넘어갔다.

| 단계 | 내용 | 결과 |
|---|---|---|
| 오프라인 격자 | `sim_test/dock_sim.py`: bag 에서 읽은 차체 특성(각속도 1차 지연 1~3 s, 가속 한계 0.05~0.1 rad/s², 제자리 회전 효율 12~75%, 회전 중심 뒤 0.25 m, 측정 지연 0.3~0.5 s, 잡음 1°, 시작 시 Nav2 잔류 속도)으로 시작 자세 45종 × 차체 5종 = 225 케이스 | 방향 3°·거리 ±0.05 m·충돌 없음·횡 0.06 m **225/225**, 최대 횡 오차 0.059 m |
| 이전 이득 비교 | 같은 격자를 `--old`(관성·지연 보정 없음, 정렬 1.5°, 후진 조향 상한 0.20, 후진 0.15 m/s)로 | 방향·거리·충돌 225/225 이지만 횡은 **206/225**, 최대 0.127 m. 조향 법칙 자체는 코드에 하나뿐이라 이 비교는 이득 차이만 본 것이다 |
| ROS 회귀 | `sim_test/dock_regression.py`: 현장과 같은 launch 로 Nav2·도킹을 올리고 합성 로봇(같은 차체 모델)으로 10 시나리오 | **10/10 통과** (`results/regression_20260924_sim.txt`). 좌우 0.2 m 어긋난 두 건은 재시도 1회로 횡 0.046·0.021 m |

남은 한계: 횡 오차는 재시도 2회를 써서 0.06 m 안으로 넣는 구조라 시간이 더 든다(모의 최대 sim 75 s).
팔 쪽 횡 허용치가 정해지면 `lat_tol_m`·`max_retry` 로 맞춘다. 근본 해결은 Isaac 쪽 DifferentialController
의 각가속 제한을 푸는 것(팀 에셋)이다.
