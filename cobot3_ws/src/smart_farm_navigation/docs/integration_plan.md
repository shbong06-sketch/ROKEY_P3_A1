# 1차 시스템 통합 계획 — /cmd_vel 주행 + M0609 pick & place (Isaac Sim standalone)

기준일 2026-09-20. 대상 장면 `Collected_smartfarm_v001.usd`(미완성, 확정 후 git 반영 예정). 시방서: `isaacpjt/smart_farm/scripts/robot_motion.py`.
상위 설계 기준: `docs/reference/02_notion/01_시스템아키텍쳐`(2026-09-19 23:32)와 `02_인터페이스설계`(23:49). 1차 시연의 carter 이동은 그 문서의 Nav2 대신 /cmd_vel path runner 로 수행하되, 명명(RACK_DOCK, INSPECTION_DOCK, TASK-YYYYMMDD-NNN)과 command/result 계약은 그대로 따른다.

## 1. 구성 요소와 실행 주체

| 구성 | 실행 위치 | 파일 | 역할 |
| --- | --- | --- | --- |
| 시뮬레이션 본체 | Isaac Sim standalone (`isaac_python`) 1개 프로세스 | 팀 `robot_motion.py` (+ 아래 3줄 수정) | 장면 열기, 물리 진행, M0609 pick & place, AMR 정지 감시 |
| carter1 주행 | 외부 ROS 2 노드 | `path_runner_smooth` (`path_smooth.launch.py`) | 통로 탈출 → 곡선 → INSPECTION_DOCK 정지 |
| carter2 주행 | 외부 ROS 2 노드 | `path_runner` (`carter2_dock.launch.py`) | 맵 밖에서 통로 진입 → RACK_DOCK 도킹 창 안 저속 정지 |
| Isaac ↔ ROS 연결 | 장면의 Action Graph | Nova_Carter_ROS 그래프 ×2 (node_namespace carter1/carter2) | /cmd_vel 구독, /chassis/odom·/tf 발행 |
| 점검 | 외부 ROS 2 노드 | `scene_check`, `env_check.sh` | 통신·환경 확인 |

- standalone은 **하나만** 띄움. 통합에서는 팀의 robot_motion.py가 장면을 열므로 `launch_scene.py`는 쓰지 않음. launch_scene.py는 주행 단독 시험용 standalone임.
- Isaac Sim 안에서는 ROS(rclpy)를 쓰지 않음. 팔과 주행의 동기화는 "AMR이 도킹 창 안에 0.5 s 이상 정지" 라는 물리 상태로 하며, 이는 robot_motion.py의 BaseWatcher·check_base_pose가 이미 하는 일임.

## 2. robot_motion.py 에 필요한 최소 수정 (팀장 파일, 3곳)

```python
# (1) SimulationApp 생성 직후 — ROS2 bridge 확장을 켜야 장면의 Action Graph 가 /cmd_vel 을 받음
app = SimulationApp({"headless": False})
from isaacsim.core.utils.extensions import enable_extension
enable_extension("isaacsim.ros2.bridge")
app.update()

# (2) 장면 경로
SCENE_PATH = Path(__file__).resolve().parent.parent / "scenes/smartfarm_v1/Collected_smartfarm_v001.usd"   # 확정 경로로

# (3) 팔 prim 경로 = carter2 에 결합된 M0609 (예: /World/carter2/m0609_with_fork). 장면 확정 후 기입
ROBOT_PATH = "/World/..."
```
- `physics_prim_path="/physicsScene"` 이 장면과 맞는지 확인(smartfarm_v1.usd 는 `/World/PhysicsScene`).
- 실행 터미널은 `ros_set`만 소싱(guidance1 11차 실측 A 기준). bridge는 시스템 Jazzy 라이브러리로 동작함.
- BASE_X_WINDOW/BASE_Y_WINDOW/BASE_YAW_LIMIT_DEG 는 월드 좌표의 도킹 창임. carter2 경로의 최종 도착점을 이 창의 중심에 맞춤.

## 3. 통합 시퀀스

1. 터미널 1: `ros_set` → `isaac_python robot_motion.py` → 장면 로드, Play. robot_motion.py는 "AMR 정지 대기" 상태로 진입함.
2. 터미널 2: `ros_set` → 빌드 → `scene_check`(carter1·carter2 토픽 확인: `/carter1/chassis/odom`, `/carter2/cmd_vel` 등).
3. 터미널 3: carter1 주행 `path_smooth.launch.py auto_start:=true` (cmd_vel_topic:=/carter1/cmd_vel odom_topic:=/carter1/chassis/odom) → 검수 위치 정지.
4. 터미널 4: carter2 주행 `carter2_dock.launch.py auto_start:=true` → 도킹 창 안 저속 정지 → robot_motion.py 가 정지 감지 → 계획 → pick & place 실행.
5. 로그: results/ 에 tee 로 남기고 커밋.

carter1·carter2를 동시에 움직여도 되나, 처음 리허설은 순서대로(3 → 4) 진행함.

## 4. 통합 전 확인 목록
- [ ] 장면에 carter prim 2개, 각 Nova_Carter_ROS 그래프의 node_namespace = carter1 / carter2 (수업 자료 "Multi-Robot — Nova Carter2 추가" 절차).
- [ ] `ros2 topic list` 에 `/carter1/cmd_vel`, `/carter1/chassis/odom`, `/carter2/...` 가 모두 보임.
- [ ] carter2 시작 위치에서 도킹 창 중심까지의 직선 거리·방향 → `config/carter2_dock.yaml` waypoints.
- [ ] carter1 시작 위치·통로 탈출점·컨베이어 앞 → `config/path_runner_smooth.yaml` waypoints.
- [ ] robot_motion.py 의 ROBOT_PATH, TASKS(pallet prim 경로, 층), 도킹 창 값이 새 장면과 일치.
- [ ] 두 carter 모두 base_link +x 가 진행 방향을 향하도록 배치(11차 관찰 반영).

## 5. 알려진 위험
- odom 누적 오차: carter2 가 맵 밖에서 긴 거리를 오면 도킹 창(±0.15 m)을 벗어날 수 있음. 도킹 직전 직선 구간을 1 m 이상 두고 저속(0.05~0.1 m/s)으로 진입함. 벗어나면 robot_motion.py 가 "다시 도킹하세요"로 멈추므로, carter2 노드를 짧은 보정 경유지로 재실행하면 됨.
- 두 로봇이 같은 통로를 쓰면 충돌 위험: carter1이 통로를 완전히 빠져나간 뒤 carter2를 출발시킴.
- 팀 스크립트가 World.step 을 돌리므로 렌더 부하가 커지면 실시간 배율이 더 떨어질 수 있음. path_runner 는 odom 기반이라 완주에는 영향 없으나 시연 시간이 늘어남.

## 6. 상위 계약과의 연결 (다음 단계)
- 인터페이스 설계의 Navigation Node 계약(`/navigation/command` → 주행 → `/navigation/result`, `/navigation/status`)을 /cmd_vel 기반으로 구현한 `navigation_node`를 둘 예정임. 내부에서 path_runner 로직을 호출하고, destination(INSPECTION_DOCK, RACK_DOCK)을 `config/stations.yaml`(시작 기준 경유지 묶음)로 변환함.
- 전제: `smart_farm_interfaces`에 TaskCommand, TaskResult, ExecutorStatus 메시지가 추가되어야 함(인터페이스 설계 5절). 메시지가 생기기 전까지는 launch 파일 직접 실행으로 시연함.
