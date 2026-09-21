# [guidance1 13차] 운반 지시 수신 — navigation_node (내피 ↔ 고피, ROS_DOMAIN_ID 101)

## 구성
- 고피: Isaac Sim + 팀장 통합 스크립트가 `/navigation/command`로 운반 지시를 보냄.
- 내피: `navigation_node`가 지시를 받아 곡선 주행(path_runner_smooth)을 실행하고 `/navigation/result`로 결과를 돌려줌. 주행 명령(/cmd_vel)과 odom은 네트워크를 건너 고피의 Isaac Sim과 주고받음.
- 내피 통합 시험 결과: 지시 발행 → 주행 → `SUCCEEDED / reached_station=INSPECTION_DOCK` 결과 수신 확인.

## 팀장에게 전달할 계약 (std_msgs/String, UTF-8 JSON)
| 토픽 | 방향 | 내용 |
| --- | --- | --- |
| `/navigation/command` | 고피 → 내피 | `{"command_id": "TASK-20260921-001-CMD-003", "task_id": "TASK-20260921-001", "operation": "NAVIGATION", "destination": "INSPECTION_DOCK"}` |
| `/navigation/result` | 내피 → 고피 | `{"command_id", "task_id", "operation": "NAVIGATION", "status": "SUCCEEDED"｜"FAILED"｜"TIMEOUT", "phase", "reason", "safe_to_navigate": false, "reached_station": "INSPECTION_DOCK"}` |
| `/navigation/status` | 내피 → 고피 | `{"executor": "navigation", "state": "READY"｜"BUSY"｜"ERROR", ...}` (TRANSIENT_LOCAL, 마지막 값 유지) |
- destination 은 `config/destinations.yaml` 의 이름(INSPECTION_DOCK, RACK_DOCK)만 받음. 같은 command_id 재수신 시 재실행하지 않고 이전 결과를 재발행함. 실행 중 새 명령은 `FAILED/BUSY`.
- 고피에서 보내는 파이썬 예시(외부 ROS 2, Jazzy 소싱된 터미널):
```python
import json, rclpy
from std_msgs.msg import String
rclpy.init(); n = rclpy.create_node("transport_cmd")
pub = n.create_publisher(String, "/navigation/command", 10)
sub = n.create_subscription(String, "/navigation/result", lambda m: print("RESULT", m.data), 10)
import time; time.sleep(1.0)
pub.publish(String(data=json.dumps({"command_id": "TASK-20260921-001-CMD-003", "task_id": "TASK-20260921-001",
                                    "operation": "NAVIGATION", "destination": "INSPECTION_DOCK"})))
rclpy.spin(n)
```
- smart_farm_interfaces 에 TaskCommand/TaskResult 메시지가 생기면 navigation_node 의 파서만 바꾸면 됨. 필드 이름은 이미 인터페이스 설계와 같음.

## 내피에서 할 일 (순서대로)
### 1. 네트워크 조건 (가장 먼저 확인)
- 내피 `~/.bashrc` 는 `ROS_DOMAIN_ID=103` 이고, `~/.ros/fastdds_whitelist.xml` 은 127.0.0.1 과 10.10.0.1~4 만 허용함. 현재 무선 IP 는 172.16.0.4 라서 이 상태로는 고피와 통신되지 않음.
- 두 가지 중 하나를 택함.
  - (a) 고피와 유선 10.10.0.x 대역으로 연결(팀 화이트리스트 설계대로). 내피·고피 IP 가 각자의 whitelist 에 있어야 함.
  - (b) 같은 무선망을 쓰면 양쪽 `fastdds_whitelist.xml` 의 `<interfaceWhiteList>` 에 서로의 실제 IP 를 추가함. 파일을 바꾸면 터미널을 새로 열어야 적용됨.
- 내피에서 실행하는 모든 터미널에 아래를 먼저 씀(bashrc 의 103 을 덮어씀).
```bash
export ROS_DOMAIN_ID=101
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE=$HOME/.ros/fastdds_whitelist.xml
source /opt/ros/jazzy/setup.bash
```
- 통신 확인: 고피 Isaac Sim 이 Play 인 상태에서 내피에서 `ros2 topic list` 에 `/cmd_vel`, `/chassis/odom` 이 보여야 함. 안 보이면 whitelist·IP 문제임(`ip -4 addr` 로 IP 확인).

### 2. 빌드
```bash
cd /home/rokey/ROKEY_P3_A1/cobot3_ws
rm -rf build/smart_farm_navigation install/smart_farm_navigation
colcon build --packages-select smart_farm_navigation
source install/setup.bash
ros2 pkg executables smart_farm_navigation
```
- 기대: navigation_node, path_runner, path_runner_smooth, scene_check.

### 3. 내피에서 노드 실행 (통합 시)
```bash
ros2 launch smart_farm_navigation navigation_node.launch.py 2>&1 | tee -a /home/rokey/ROKEY_P3_A1/cobot3_ws/src/smart_farm_navigation/results/navnode_$(date +%Y%m%d_%H%M).txt
```
- 기대: `navigation_node ready; destinations: ['INSPECTION_DOCK', 'RACK_DOCK']`. 이후 고피가 명령을 보내면 `command ... -> ros2 launch ...`, 주행 로그, `result SUCCEEDED/NONE ...` 이 이어짐.
- 팀장 스크립트 없이 손으로 시험할 때(다른 내피 터미널, 1절 환경 설정 후):
```bash
ros2 topic pub --once /navigation/command std_msgs/msg/String "{data: '{\"command_id\": \"TEST-CMD-001\", \"task_id\": \"TEST-001\", \"operation\": \"NAVIGATION\", \"destination\": \"INSPECTION_DOCK\"}'}"
ros2 topic echo /navigation/result
```

### 3-1. 설정을 고쳤을 때 (drive_direction_sign, 경유지 등)
- launch 는 `install/` 아래 복사본 YAML 을 읽으므로 `config/*.yaml` 을 고친 뒤에는 2절의 재빌드가 필요함. 재빌드 없이 바로 쓰려면 `config/destinations.yaml` 대신 노드에 소스 경로를 주면 됨:
```bash
ros2 run smart_farm_navigation navigation_node --ros-args -p destinations_file:=/home/rokey/ROKEY_P3_A1/cobot3_ws/src/smart_farm_navigation/config/destinations.yaml
```
  단, destinations.yaml 의 params 는 install 경로 기준이므로 이 방법으로도 path_runner_smooth.yaml 은 재빌드가 필요함. 결론: **YAML 수정 후에는 항상 재빌드**.
- 후진 판별: 정지 후 `ros2 topic echo /chassis/odom --once --field pose.pose.position` 의 x 가 양수면 명령대로 +x 로 간 것이라 장면의 carter yaw 가 반대인 것이고, 음수면 sign 설정 문제임. sign 을 바꿔도 변화가 없었다면 재빌드가 안 된 것임.

### 4. 확인 항목
- 고피 장면의 carter 시작 방향이 통로 탈출 방향(정면)인지. 경유지는 `config/path_runner_smooth.yaml`(1.5 m 직진 후 곡선, 왼쪽 3.728 m·앞 4.821 m 지점 = Conveyor/Seg_6 가운데 앞 정지, 값 근거는 docs/troubleshooting.md 4-4).
- 주행 결과 로그(results/navnode_*.txt)를 커밋·푸시함.

## 알려진 제한
- 두 PC 사이 무선 지연으로 odom 이 끊기면 노드가 `no fresh Odometry` 로 중단함(1 s). 유선이 안전함.
- Isaac Sim 이 Play 가 아니면 odom 이 없어 주행이 시작되지 않고 30 s 뒤 중단됨.
