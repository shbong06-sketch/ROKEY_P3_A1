# [guidance1 11차] smartfarm_v1.usd 곡선 주행(path_runner_smooth) + ros_set 단독 운용 실측

feature/navigation(/cmd_vel 모션 제어) 트랙의 가이드임. Nav2 트랙은 feature/navigation2의 guidance2_12차부터 이어짐.

## 이번 변경 사항
- **전진 방향 부호**: 11차 관찰(팀원 모두 "후진으로 보임")에 따라 path_runner·path_runner_smooth의 `drive_direction_sign`을 `1.0`으로 바꿨음. 이제 Nav2와 같은 규칙(base_link +x = 전방)임. 장면에서 carter의 초기 yaw를 180도 돌려, base_link +x가 통로 탈출 방향(+y)을 보게 해 둘 것. 경유지는 시작 기준 좌표계라 값은 그대로임.
- **곡선 주행 노드 path_runner_smooth 신설**: 통로는 직선으로 나가고, 탈출점부터 도착 축 위의 접근점(도착점 0.8 m 앞)까지 하나의 곡선(3차 베지어, pure pursuit)으로 이동한 뒤, 도착 축을 따라 직진 접근하여 정지함. 내피 시험 결과 도착 오차 5 cm, 방향 오차 1.7도.
- **launch_scene.py**: `ISAAC_EXPERIENCE=full`을 앞에 붙이면 GUI(`isaac`)와 같은 확장 세트로 뜸(아래 3절).

## 고정 경로
- 워크스페이스 `/home/rokey/ROKEY_P3_A1/cobot3_ws`
- 장면 `/home/rokey/ROKEY_P3_A1/cobot3_ws/isaacpjt/smart_farm/scenes/smartfarm_v1/smartfarm_v1.usd`
- 설정 `/home/rokey/ROKEY_P3_A1/cobot3_ws/src/smart_farm_navigation/config/path_runner_smooth.yaml`
- 결과 `/home/rokey/ROKEY_P3_A1/cobot3_ws/src/smart_farm_navigation/results`

---

## 1. 곡선 주행 실행 (터미널 3개)

### [터미널 1] Isaac Sim
```bash
ros_set
bash /home/rokey/ROKEY_P3_A1/cobot3_ws/src/smart_farm_navigation/scripts/env_check.sh
isaac_python /home/rokey/ROKEY_P3_A1/cobot3_ws/src/smart_farm_navigation/scripts/launch_scene.py /home/rokey/ROKEY_P3_A1/cobot3_ws/isaacpjt/smart_farm/scenes/smartfarm_v1/smartfarm_v1.usd
```
(2절의 실측 B에서는 `ros_set` 대신 `isaac_ros`를 씀.)

### [터미널 2] 빌드·점검
```bash
ros_set
cd /home/rokey/ROKEY_P3_A1/cobot3_ws
rm -rf build/smart_farm_navigation install/smart_farm_navigation
colcon build --packages-select smart_farm_navigation
source install/setup.bash
ros2 run smart_farm_navigation scene_check 2>&1 | tee -a /home/rokey/ROKEY_P3_A1/cobot3_ws/src/smart_farm_navigation/results/check_$(date +%Y%m%d_%H%M).txt
```

### [터미널 3] 곡선 주행
```bash
ros_set
source /home/rokey/ROKEY_P3_A1/cobot3_ws/install/setup.bash
ros2 launch smart_farm_navigation path_smooth.launch.py auto_start:=true 2>&1 | tee -a /home/rokey/ROKEY_P3_A1/cobot3_ws/src/smart_farm_navigation/results/smooth_$(date +%Y%m%d_%H%M).txt
```
- 기대 로그: `path: N points, curve from (2.30,0.00) to approach (4.60,2.44), final (5.40,2.44)` → `Phase TRACK` → `approach point reached` → `Phase APPROACH` → `final point reached ... cross-track ±0.05` → `Path COMPLETE ... heading error ±2deg`.
- 뷰포트 기대: 통로를 직진으로 나온 뒤 멈춤 없이 왼쪽으로 휘어 컨베이어 앞 축에 올라탄 다음 짧게 직진해 정지. 정면이 앞서 가야 함(뒤로 가면 초기 yaw 180도 회전이 안 된 것).
- 직선·제자리회전 방식이 필요하면 `path.launch.py`(path_runner)를 그대로 쓰면 됨.

---

## 2. ros_set 단독 운용 검증 실측

목적: 모든 터미널에서 `ros_set`만 써도 Isaac Sim bridge와 외부 노드가 같은 ROS 2(Jazzy) 라이브러리로 동작함을 사실로 확인함. A와 B를 각각 한 번씩 수행하고 결과를 이 파일 하단에 붙임.

### 실측 A — 모든 터미널 `ros_set`만 (권장안)
1. 터미널 1~3을 1절 그대로 수행함(터미널 1에 `ros_set`만).
2. Isaac Sim이 PLAY 상태일 때 터미널 2에서 아래를 실행하고 출력 전체를 기록함.
```bash
ls /home/rokey/isaacsim/exts/isaacsim.ros2.bridge/
PID=$(pgrep -f "launch_scene.py" | head -1); echo "PID=$PID"
grep -oE "/[^ ]*/lib(rcl|rmw|rosidl|fastrtps|fastcdr|rclpy)[^ ]*\.so[^ ]*" /proc/$PID/maps | sed 's#/lib[^/]*\.so.*##' | sort | uniq -c
ros2 topic list | wc -l
```
- 판정: `uniq -c` 결과의 디렉터리가 `/opt/ros/jazzy/lib`만이면 Isaac Sim이 시스템 Jazzy를 쓰는 것임. `isaacsim.ros2.bridge/humble` 이 함께 나오면 두 배포판이 섞인 것임.
- scene_check RESULT: OK, 곡선 주행 COMPLETE 이면 A 통과.

### 실측 B — 수업 방식 (터미널 1 `isaac_ros`만, 터미널 2·3 `ros_set`)
1. 터미널 1을 닫고 새로 열어 `ros_set` 없이 아래만 실행함.
```bash
isaac_ros
bash /home/rokey/ROKEY_P3_A1/cobot3_ws/src/smart_farm_navigation/scripts/env_check.sh
isaac_python /home/rokey/ROKEY_P3_A1/cobot3_ws/src/smart_farm_navigation/scripts/launch_scene.py /home/rokey/ROKEY_P3_A1/cobot3_ws/isaacpjt/smart_farm/scenes/smartfarm_v1/smartfarm_v1.usd
```
2. 실측 A의 2번 명령을 그대로 실행하여 기록함. 터미널 3의 곡선 주행도 한 번 더 실행함.

### 판정 규칙
| 결과 | 의미 | 결정 |
| --- | --- | --- |
| A: `/opt/ros/jazzy/lib`만, OK, COMPLETE | 시스템 Jazzy 단독으로 정상 | 모든 터미널 `ros_set`만 사용 |
| B: humble 디렉터리가 보이는데도 동작 | 내장 humble bridge + 외부 Jazzy 노드가 통신은 됨 | 동작은 하나 배포판 혼용이므로 A를 표준으로 |
| A에서 topic이 안 보임 | 시스템 Jazzy 라이브러리로 bridge가 못 뜸 | 터미널 1 Isaac Sim 콘솔의 `ros2 bridge` 관련 줄을 기록해 올릴 것 |

---

## 3. standalone(isaac_python)과 GUI(isaac)의 확장 차이 확인
1절과 동일하되 터미널 1을 아래로 바꾸면 GUI와 같은 확장 세트로 뜸.
```bash
ros_set
ISAAC_EXPERIENCE=full isaac_python /home/rokey/ROKEY_P3_A1/cobot3_ws/src/smart_farm_navigation/scripts/launch_scene.py /home/rokey/ROKEY_P3_A1/cobot3_ws/isaacpjt/smart_farm/scenes/smartfarm_v1/smartfarm_v1.usd
```
- 확인: Window > Examples, Tools > Robotics(Occupancy Map) 메뉴가 보이는지. `ls /home/rokey/isaacsim/apps/`로 사용 가능한 .kit 목록도 기록함.
- 로딩 시간과 GPU 사용은 GUI와 같아짐. ROS 통신·주행에는 차이가 없어야 함.

## 기록
- results/의 check_, smooth_ 로그와 2·3절 출력을 커밋·푸시함. 최신 smartfarm_v1.usd(회전한 carter, 옮긴 컨베이어)도 커밋함.
