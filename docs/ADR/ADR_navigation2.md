
# ADR_navigiation2

- **"/home/rokey/ROKEY_P3_A1/docs/ADR/ADR_navigation2.md"(이하 "ADR_nav2")는 "/home/rokey/ROKEY_P3_A1" git 디렉토리에 대해서 브랜치가 feature/navigation2 및 feature/Inspection-Place-nav2일 때, 당신에게 주어질 규칙, 사고규칙, 판단규칙 등을 명시하고 정의하는 문서임**
- **ADR_nav2의 규칙은 ADR_basic.md의 규칙보다 우선되지 않음.**
- **ADR_nav2의 내용을 수정하는 경우는 1) 프롬프터인 내가 그러한 행위를 직접 당신에게 지시했을 때, 2) 본 ADR_nav2의 내용이 수정되지 않고서는 내가 당신에게 시킨 업무를 해낼 수 없을 때 나한테 해당 안건을 보고하고 그것을 승인까지 받았을 때만으로 한정함**

# Agent Role & Environment Context

- **당신은 Isaac Sim과 ROS2, Nav2(Navigation2), RViz2 운용 및 로봇 자율주행 시스템 구현을 마스터한 프로로서, 내가 진행 중인 프로젝트의 개발자 포지션을 맡아 이끌어 줄 것.**

---

## 1.

1.1. **메인 장면 standalone USD파일**:
    1.1.1. 경로: "/home/rokey/ROKEY_P3_A1/cobot3_ws/isaacpjt/smart_farm/scenes/Collected_smartfarm_v###/"의 Collected_smartfarm_v###.usd의 형태로 들어가있음. 최신화될 때마다 버전값이 유동적으로 바뀔 수 있음.
1.2. **메인 씬의 map 및 yaml 설정값**: 
    1.2.1. 경로: "/home/rokey/ROKEY_P3_A1/cobot3_ws/isaacpjt/smart_farm/maps/"에서 Collected_smartfarm_v###의 이름으로 png 혹은 yaml파일로 존재함.
    1.2.2. scenes 디렉터리의 최신 버전과 ADR 의 버전이 다르면 scenes 의 최신 버전을 우선하고, 당신이 ADR 1.1·1.2 의 버전 문구 갱신을 사용자에게 요청함.
1.3. **Nova_Carter 지칭**:
    1.3.1. 나는 Nova Carter를 프롬프트상에서 카터라고 부르겠으며, 카터가 2대 이상 존재할 경우 구현된 순서대로 카터1, 카터2, ... 카터n과 같이 지칭함. 만약 USD 상에서 지정된 namespace가 있다면 나에게 일러줄 것.
    1.3.2. m0609와 lift가 카터에 결합된 상태인 LiftRig prim 또한 카터라고 지칭함.
    1.3.3. 결합되지 않은 카터를 콕 집어 말할때는 '그냥 카터'라고 지칭함.
1.4. **작업점 이름**: FEEDER_APPROACH(−2.19, −1.55, 90°) = Nav2 Goal 클릭 지점, FEEDER_DOCK = TurnTable 앞면(world y −3.60, x −2.76~−1.61, 높이 1.17 m) 기준 `standoff_m` 만큼 앞. 값은 `config/stations.yaml` 이 출처이며 장면이 바뀌면 그 파일을 먼저 고침.

---
## 2. 당신(사용자인 내가 아니고, ADR을 열람하는 당신)의 입장에서 명시한 ADR

2.1. **환경 상수 (글로벌)**
- `ROS_DOMAIN_ID` 는 그때 사용하는 고피에 맞춘다(고피1 101, 고피2 102). 고피는 유동적으로 바뀌므로 값을 고정해 두지 말고, 작업 시작 시 현재 쓰는 고피를 확인해 내피의 모든 터미널과 비전 컨테이너에 같은 값을 적용한다. 모든 내피 터미널 블록은 아래 5줄로 시작하며, 가이던스에서 "위와 같은 5줄" 같은 참조로 줄이지 않음(아래 번호는 예시이며 가이던스에는 그때의 실제 값을 적는다).
  export ROS_DOMAIN_ID=102
  export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
  export FASTRTPS_DEFAULT_PROFILES_FILE=/home/rokey/.ros/fastdds_whitelist.xml
  source /opt/ros/jazzy/setup.bash
  source /home/rokey/ROKEY_P3_A1/cobot3_ws/install/setup.bash
- 내피 `~/.bashrc`는 도메인 103이라 터미널마다 export 필요. 내피 유선 IP 10.10.0.3, 고피1(기존 고피) 유선 IP 10.10.0.2, 고피2 유선 IP 10.10.0.1임.
- 내피에서 `git pull`은 내가 하지 않음. push 거부 시 사용자 pull 대기.

2.2. **장면·지도·좌표 사실 (내가 되풀이해서 헷갈린 것)**
- 카터 앞 = base_link +x = 구동륜 쪽. 뒤 = −x = 캐스터·리프트·M0609. 초기 배치 yaw 90°(앞 북쪽, 뒤가 통로 출구). 이 배치는 팀 P&P 때문에 바꾸지 않음.
- map = Isaac world. 지도 origin (−4.525, −10.025), 0.05 m/px. 장면이 바뀌면 `make_map_from_usd.py`로 지도 재생성 후 ADR 1.2 갱신.
- XT-32 위치 base_link (−0.232, 0, 0.526). 이 값은 `feeder_dock`, `cloud_self_filter`, `nav2_link_check` 세 곳이 공유함.
- 2D 라이다 토픽은 센서가 존재하고 채널도 생성되나 일부 채널은 msg를 발행하지 않음. 2D 라이다 두 채널 모두 발행 안 됨(실측 4회). 3D 점군 → /scan 변환만 사용.
- TurnTable 앞면: world x −2.76~−1.61, y −3.60(북쪽 끝), 높이 1.17 m. 도킹 목표는 이 면 기준 `standoff_m` 0.85 (2026-09-24 `ceeafd5` 반영. 팀 `robot_motion` 의 팔 작업 범위 0.89~1.05 m 의 한가운데인 0.96 m 가 되는 값. 이전 값 0.90 은 그 범위보다 멀었음).
- 시간 판단은 `/clock`(`use_sim_time: true`) 기준으로만 함. `time.monotonic()`·`time.time()` 으로 타임아웃·신선도를 판단하지 않음. 이유: Isaac 실시간 배율이 0.3 전후라 벽시계 임계값은 3배 빨리 걸림.
- 3D 라이다는 `fullScan=True` 여야 한 바퀴(약 41,000점, 시뮬 10 Hz)로 옴. 꺼져 있으면 프레임마다 60° 조각(약 6,900점)만 와서 AMCL 과 도킹이 깨짐. 팀 앱과 `launch_scene.py` 둘 다 이 설정을 넣으며, fullScan 이어도 섹터가 빠진 스캔이 섞이므로 `cloud_self_filter` 가 0.25 s 점군을 합침.
- `/cmd_vel` 발행자는 Nav2(collision_monitor)와 `feeder_dock` 둘임. `feeder_dock` 은 Nav2 가 2 s 이상 조용할 때만 시작함. 세 번째 발행자를 추가하지 않음.

2.3. **실행 방식 규칙**
- 고피는 Isaac만, 내피가 Nav2·RViz2·도킹. 통합 실측은 팀 `standalone_app.py --autoplay`로, 단위 시험은 `launch_scene.py`로.
- Isaac 을 다시 실행하면 내피 Nav2(터미널 3)도 다시 실행함(시뮬 시계가 0 으로 돌아가 TF·센서 시각이 어긋남). Isaac 의 Stop 버튼은 누르지 않음: 팀 앱 `lift.py` 의 `stop()` 이 물리 뷰 소멸 뒤 `NoneType` 으로 crash 함(팀 수정 대기).
- 목표 지정은 RViz2 Nav2 Goal 클릭(FEEDER_APPROACH 한 번). `go_to_station`은 팀 통합(`/navigation/command`)용으로만.
- `/sim_task/command`는 `std_msgs/String` JSON, `/navigation/command`는 `smart_farm_interfaces/TaskCommand`. 혼동 금지.
- `ros2 topic pub`은 `-t 3 -r 1`(bag 기록기가 같은 토픽을 구독하므로 `--once`는 놓칠 수 있음).
- YAML·코드 수정 후 내피 `colcon build` 필수(launch는 install 복사본을 읽음). 사용자가 열람할 가이던스에도 강조해서 알릴 것.
- "Nav2 만으로" 의 정의: 출발 지시는 RViz2 Nav2 Goal 클릭 하나. 통로 탈출·도킹 진입을 위한 스크립트 후진(`feeder_dock`, `go_to_station -p pure_nav2:=false` 의 `reverse_out_zones`)은 허용되며 필요 시 추가 작성함.
- 실측 절차의 표준 순서: 고피 터미널 1(Isaac) → 내피 터미널 2(빌드·`nav2_link_check`) → 내피 터미널 3(`nav2.launch.py`) → 내피 터미널 4(파지 명령) → RViz2 클릭 → 결과 확인(`/feeder_dock/result`, 터미널 3 의 `[DONE]`).

2.4. **병합·파일 소유 규칙**
- development 병합 시 충돌은 pull된 파일이 이김. 그 뒤 내 추가분(navigation_node의 `station` 분기 / setup.py 진입점 6개: scan_sanitizer, cloud_self_filter, go_to_station, nav2_link_check, station_markers, feeder_dock / package.xml 추가 의존성: sensor_msgs, sensor_msgs_py, rosgraph_msgs, tf2_ros_py, visualization_msgs, nav2_bringup, nav2_simple_commander, pointcloud_to_laserscan)을 다시 붙여야 함. 이 목록을 ADR에 적어 두면 병합 후 점검표가 됨.
- 팀 파일 중 내가 손댄 곳은 `standalone_app.py`의 `open_scene()` fullScan 6줄뿐. 그 외 팀 파일은 수정하지 않음.
- 미사용 파일은 각 모듈의 `past/`로. 저장소 트리 재구성은 하지 않음.
- `results/bags/`, `media_log/`는 git 제외. 대용량 USD 에셋도 제외.

2.5. **문서 규칙**
- 가이던스는 `guidance2_<n>차.md`, 답변마다 차수 증가, 이전 차수는 내용이 현 단계 작업에 유의미하지 않다면 `guidance/past/`로 이동시킬 것. 위에서 아래로 실행만 하면 되는 순서형, 몇 번째 터미널에 기입할 커맨드인지 명시, 각 터미널 커맨드 세션마다 실행 후 터미널 log를 'results/'에 저장하는 커맨드는 tee로 구성중, RViz2 클릭처럼 터미널이 없는 단계는 bag이 기록, 터미널 블록마다 환경 5줄 포함, "위와 같은 5줄" 같은 참조형 구성으로 작성은 금지, 선택 항목은 선택이라고 표시, 작업한 파일에 대해 파일명-역할-요약 설명을 마지막 챕터쯤에 테이블로 기입할 것.
- 사용자의 실측 피드백은 `errored/`에, 로그는 `results/`에. 트러블슈팅은 `errored/` 파일과 최신 bag를 먼저 읽고 시작.
- 아키텍처·트러블슈팅 문서는 `docs/architecture.md`는 갱신, `docs/troubleshooting.md`은 항목 추가(양 문서 모두 새 파일로 만들지 않음).

2.6. **현재 "keep" 상태의 기준선 (바꾸려면 보고 후)**
- planner Smac Hybrid-A*(Reeds-Shepp, 후진), RPP 0.6 m/s `allow_reversing`, goal 허용 0.25 m/0.5 rad, `max_angular_accel` 20, PoseProgressChecker, BT 응답 200 ms.
- `cloud_self_filter` 상자 x −0.65~0.60, y ±0.6, 합치기 0.25 s (이전 값 −0.85~0.6. `standoff_m` 을 0.85 로 줄이면서 함께 옮김). `box_x[0]`은 `standoff_m`보다 앞에 있어야 함.
- `feeder_dock` standoff 0.85, 자동 시작 반경 0.6 m, 면 길이 0.6~1.6 m, 법선 ±60°, 신선도 2.5 s.
- 미해결 1건: 후진 중 방향이 틀어져 전진한 사례(3회 중 1회). 연속성 검사 후보는 적용 전.
- `/scan` 파이프라인: 3D 점군 → `cloud_self_filter`(상자 제거, 0.25 s 합침) → `pointcloud_to_laserscan`(높이 −0.35~1.5 m, `range_min` 0.3, 0.5°) → `/scan`. 2D 라이다는 쓰지 않음.
- `standoff_m`(0.85) 과 `box_x[0]`(−0.65) 은 짝임. 도킹 거리를 줄이면 상자 뒤끝도 그보다 앞(값이 더 큼)에 두어야 면이 지워지지 않음. 차체 뒤끝은 base_link −0.607 이므로 `standoff_m` 은 0.75 아래로 두지 않음.
- 속도 상한은 팔레트·엽채류의 위치값을 유지하는 선이 기준임.

2.7. **내가 할루시네이션을 겪기 쉬운 지점 (명시적으로 금지·확인 문구로)**
- 도킹 구간의 `feeder_dock`은 사용자가 승인한 예외임을 명기.
- 내피 모의 성공을 실측 성공으로 쓰지 말 것. 실측 여부는 `results/`·`errored/`로만 판단.
- 장면 버전은 ADR 혹은 "/home/rokey/ROKEY_P3_A1/cobot3_ws/isaacpjt/smart_farm/scenes"의 최신 경로만 신뢰. 내 메모리의 v004/v008 수치는 과거 값.
- 팀 앱의 관절값·리프트 높이(HOME [180,0,0,0,0,0], CARRY joint_1 +90, TRAVEL_BASE_HEIGHT 1.0388, 리프트 0 = 베이스 0.796)는 팀 코드가 출처. 바뀌면 `arm_poses.yaml`도 갱신.
- 화면상 좌우와 로봇 기준 좌우를 섞지 말 것(트러블슈팅 4절 참고).
- 카터가 "가만히 있다" 는 보고는 정지가 아닐 수 있음(12차: 0.1 rad/s 회전, 16차: 면 미검출 대기). 원인 판단 전에 `feeder_dock` 의 `idle:` 줄과 `/cmd_vel` 값을 확인함.
- `ros2 topic pub --once` 가 노드에 닿았다고 가정하지 않음. 노드 로그에 `command …` 줄이 있는지로 판단함.
- "잘 작동했다" 는 사용자 보고와 bag 의 결과가 다를 수 있음. 결과 판정은 `/feeder_dock/result` 와 최종 world 좌표로 함.
- 이전 차수 가이던스의 수치(도킹 y −2.60, 1.00 m 등)를 현재 값으로 착각하지 않음. 현재 값은 `config/stations.yaml` 과 `feeder_dock` 기본값이 출처임.