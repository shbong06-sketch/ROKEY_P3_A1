
# ADR_navigiation2

- **"/home/rokey/ROKEY_P3_A1/docs/ADR/ADR_navigation2.md"(이하 "ADR_nav2")는 "/home/rokey/ROKEY_P3_A1" git 디렉토리에 대해서 브랜치가 feature/navigation2 및 feature/Inspection-Place-nav2일 때, 당신에게 주어질 규칙, 사고규칙, 판단규칙 등을 명시하고 정의하는 문서임**
- **ADR_nav2의 규칙은 ADR_basic.md의 규칙보다 우선되지 않음.**
- **ADR_nav2의 내용을 수정하는 경우는 1) 프롬프터인 내가 그러한 행위를 직접 당신에게 지시했을 때, 2) 본 ADR_nav2의 내용이 수정되지 않고서는 내가 당신에게 시킨 업무를 해낼 수 없을 때 나한테 해당 안건을 보고하고 그것을 승인까지 받았을 때만으로 한정함**

# Agent Role & Environment Context

- **당신은 Isaac Sim과 ROS2, Nav2(Navigation2), RViz2 운용 및 로봇 자율주행 시스템 구현을 마스터한 프로로서, 내가 진행 중인 프로젝트의 개발자 포지션을 맡아 이끌어 줄 것.**

---

## 1.

1.1. **메인 장면 standalone USD파일**:
    1.1.1. 경로: "/home/rokey/ROKEY_P3_A1/cobot3_ws/isaacpjt/smart_farm/scenes/Collected_smartfarm_v###/" 안에 들어가있음. 최신화될 때마다 버전값이 유동적으로 바뀔 수 있음.
    1.1.2. **파일명은 `Collected_smartfarm_v###.usd` 로 고정되지 않음.** 한 폴더에 여러 USD 가 함께 들어오며 실제로 열 파일은 그중 하나임 (2026-09-25 예: 폴더 `Collected_smartfarm_v014/` 안에 기반이 되는 `Collected_smartfarm_v013_room_core.usd` 와 실제로 여는 `Collected_smartfarm_v014_room_core_cabbage.usd` 가 같이 있음). 폴더에 딸려 오는 `README_사용법.md` 가 어느 파일을 여는지 밝히므로 그것을 먼저 읽음.
1.2. **메인 씬의 map 및 yaml 설정값**: 
    1.2.1. 경로: "/home/rokey/ROKEY_P3_A1/cobot3_ws/isaacpjt/smart_farm/maps/"에서 Collected_smartfarm_v###의 이름으로 png 혹은 yaml파일로 존재함. **현행은 `Collected_smartfarm_v014.yaml`** 이며 `nav2.launch.py` 의 `DEFAULT_MAP`, `sim_test/fake_robot.py` 의 `DEFAULT_MAP` 이 그것을 가리킴(2026-09-25). v011 지도는 되돌릴 수 있도록 남겨 둠.
    1.2.2. scenes 디렉터리의 최신 버전과 ADR 의 버전이 다르면 scenes 의 최신 버전을 우선하고, 당신이 ADR 1.1·1.2 의 버전 문구 갱신을 사용자에게 요청함.
1.3. **Nova_Carter 지칭**:
    1.3.1. 나는 Nova Carter를 프롬프트상에서 카터라고 부르겠으며, 카터가 2대 이상 존재할 경우 구현된 순서대로 카터1, 카터2, ... 카터n과 같이 지칭함. 만약 USD 상에서 지정된 namespace가 있다면 나에게 일러줄 것.
    1.3.2. m0609와 lift가 카터에 결합된 상태인 LiftRig prim 또한 카터라고 지칭함.
    1.3.3. 결합되지 않은 카터를 콕 집어 말할때는 '그냥 카터'라고 지칭함.
1.4. **작업점 이름**: FEEDER_APPROACH(world x −2.19, y −1.55, yaw 90°) = Nav2 Goal 클릭 지점, FEEDER_DOCK = TurnTable 앞면 기준 `standoff_m` 만큼 앞(어림값 world x −2.19, y −2.73, yaw 90°). 면은 world x −2.76~−1.61, 높이 1.17 m 이고 **y 는 USD 기준 −3.60, 라이다 검출 면 −3.645 를 구분함**(2.2 참조). `standoff_m` 은 검출 면 기준이며 현행 **0.92**(2026-09-25). 값은 `config/stations.yaml` 과 `feeder_dock.py` 의 `DockParams` 가 출처이며 장면이 바뀌면 그 파일을 먼저 고침.

---
## 2. 당신(사용자인 내가 아니고, ADR을 열람하는 당신)의 입장에서 명시한 ADR

2.1. **환경 상수 (글로벌)**
- `ROS_DOMAIN_ID` 는 그때 사용하는 고피에 맞춘다(고피1 101, 고피2 102). 고피는 유동적으로 바뀌므로 값을 고정해 두지 말고, 작업 시작 시 현재 쓰는 고피를 확인해 내피의 모든 터미널과 비전 컨테이너에 같은 값을 적용한다. 모든 내피 터미널 블록은 아래 5줄로 시작하며, 가이던스에서 "위와 같은 5줄" 같은 참조로 줄이지 않음(아래 번호는 예시이며 가이던스에는 그때의 실제 값을 적는다).
  export ROS_DOMAIN_ID=102
  export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
  export FASTRTPS_DEFAULT_PROFILES_FILE=/home/rokey/.ros/fastdds_whitelist.xml
  source /opt/ros/jazzy/setup.bash
  source /home/rokey/ROKEY_P3_A1/cobot3_ws/install/setup.bash
- 고피3(임시 기기, ADR_basic §1)에서는 위 5줄 중 `FASTRTPS_DEFAULT_PROFILES_FILE` 을 **빼고 4줄만** 쓴다. 화이트리스트가 교육장 랜선 IP 전용이라 VM 안에서는 통신이 끊긴다.
- 내피 `~/.bashrc`는 도메인 103이라 터미널마다 export 필요. 내피 유선 IP 10.10.0.3, 고피1(기존 고피) 유선 IP 10.10.0.2, 고피2 유선 IP 10.10.0.1임.
- 내피에서 `git pull`은 내가 하지 않음. push 거부 시 사용자 pull 대기.

2.2. **장면·지도·좌표 사실 (내가 되풀이해서 헷갈린 것)**
- 카터 앞 = base_link +x = 구동륜 쪽. 뒤 = −x = 캐스터·리프트·M0609. 초기 배치 yaw 90°(앞 북쪽, 뒤가 통로 출구). 이 배치는 팀 P&P 때문에 바꾸지 않음.
- map = Isaac world. 지도 origin (−4.525, −10.025), 0.05 m/px. 장면이 바뀌면 `make_map_from_usd.py`로 지도 재생성 후 ADR 1.2 갱신.
- XT-32 위치 base_link (−0.232, 0, 0.526). 이 값은 `feeder_dock`, `cloud_self_filter`, `nav2_link_check` 세 곳이 공유함.
- 2D 라이다 토픽은 센서가 존재하고 채널도 생성되나 일부 채널은 msg를 발행하지 않음. 2D 라이다 두 채널 모두 발행 안 됨(실측 4회). 3D 점군 → /scan 변환만 사용.
- TurnTable 앞면: world x −2.76~−1.61, 높이 1.17 m. **면의 y 는 두 가지를 구분함**: USD 기준 북단은 y −3.60 이나, **라이다(RANSAC)가 실제로 검출해 도킹 기준으로 삼는 면은 y −3.645** 임(실측 2건: `results/isaac_20260923_2032.txt:704` + `nav2_20260923_2033.txt:433` → −3.644, `isaac_20260923_2125.txt:722` + `nav2_20260923_2126.txt:435` → −3.651). `standoff_m` 은 **검출 면 기준**임.
- 도킹 목표는 검출 면 기준 `standoff_m` **0.92** 임 (2026-09-25 확정, 커밋 `a95ba6c`). 출처는 **팀이 `feature/cabbage-place-fix` 에서 올인원을 끝까지 돌려 확인한 값**임(`scenes/Collected_smartfarm_v014/allinone_debug_and_changes_2026-09-25.md` 의 P3). 0.85 에서는 팔 베이스 ~ 놓을 자리가 0.72 m 라 `DESCEND_5` 가 관절 20.7°(한계 20°)로 실패했음.
  - **환산식으로 정하지 않음.** 팀이 같은 수정에서 `turntable_place_pose()` 의 놓을 자리 정의 자체를 바꿨으므로(방향을 팔 베이스 쪽으로, 쿼터니언 정규화, 벨트 윗면 +25.9 mm, 포크판 35 mm 밖) 옛 대상 기준의 환산 `standoff_m + 0.06` 은 더 이상 그 기하를 설명하지 않음. **0.92 는 팀의 place 수정과 한 쌍이며 둘을 따로 떼어 쓰지 않음.**
  - 철회한 값 둘: 0.85(2026-09-23~24)는 근거 `0.11 + standoff` 가 면을 y −3.60 으로 가정해 0.05 m 틀렸음. 0.90(2026-09-25 오전, 커밋 `0cec681`)은 실측 재환산이었으나 기준으로 삼은 place 대상이 팀 수정으로 옮겨져 근거가 소멸함. 두 값 모두 되돌리지 않음.
  - 참고: `BASE_TO_PALLET_X`(0.89~1.05)는 `robot_motion.check_base_pose` 가 `start_pick` 에서만 호출하므로 **feeder place 에는 적용되지 않음**(`robot_motion.py:883` vs `:981`). place 의 실제 관문은 역기구학과 관절 연속성(한계 20°)임.
- 시간 판단은 `/clock`(`use_sim_time: true`) 기준으로만 함. `time.monotonic()`·`time.time()` 으로 타임아웃·신선도를 판단하지 않음. 이유: Isaac 실시간 배율이 0.3 전후라 벽시계 임계값은 3배 빨리 걸림.
- 3D 라이다는 `fullScan=True` 여야 한 바퀴(약 41,000점, 시뮬 10 Hz)로 옴. 꺼져 있으면 프레임마다 60° 조각(약 6,900점)만 와서 AMCL 과 도킹이 깨짐. 팀 앱과 `launch_scene.py` 둘 다 이 설정을 넣으며, fullScan 이어도 섹터가 빠진 스캔이 섞이므로 `cloud_self_filter` 가 0.25 s 점군을 합침.
- `/cmd_vel` 발행자는 Nav2(collision_monitor)와 `feeder_dock` 둘임. `feeder_dock` 은 Nav2 가 2 s 이상 조용할 때만 시작함. 세 번째 발행자를 추가하지 않음.

2.3. **실행 방식 규칙**
- 고피는 Isaac만, 내피가 Nav2·RViz2·도킹. 통합 실측은 팀 `standalone_app.py --autoplay`로, 단위 시험은 `launch_scene.py`로.
- Isaac 을 다시 실행하면 내피 Nav2(터미널 3)도 다시 실행함(시뮬 시계가 0 으로 돌아가 TF·센서 시각이 어긋남). Isaac 의 Stop 버튼은 누르지 않음: 팀 앱 `lift.py` 의 `stop()` 이 물리 뷰 소멸 뒤 `NoneType` 으로 crash 함(팀 수정 대기).
- 목표 지정은 RViz2 Nav2 Goal 클릭(FEEDER_APPROACH 한 번). `go_to_station`은 팀 통합(`/navigation/command`)용으로만.
- `/sim_task/command`는 `std_msgs/String` JSON, `/navigation/command`는 `smart_farm_interfaces/TaskCommand`. 혼동 금지.
- `ros2 topic pub` 은 **`--once --max-wait-time-secs 15`** 를 씀(2026-09-25 확정). 15 초 안에 구독자를 못 찾으면 무한 대기 대신 오류로 끝나므로 명령이 닿지 않은 것을 바로 알 수 있음. 옛 규칙 `-t 3 -r 1` 은 같은 명령이 3 번 들어가 결과 판정이 흐려져 철회함. 명령이 닿았는지는 노드 로그의 `command …` 줄로 확인하고, 결과를 놓치지 않으려면 **결과 구독 터미널(`/sim_task/result`, `/navigation/result`)을 명령보다 먼저 띄움**.
- YAML·코드 수정 후 내피 `colcon build` 필수(launch는 install 복사본을 읽음). 사용자가 열람할 가이던스에도 강조해서 알릴 것.
- "Nav2 만으로" 의 정의: 출발 지시는 RViz2 Nav2 Goal 클릭 하나. 통로 탈출·도킹 진입을 위한 스크립트 후진(`feeder_dock`, `go_to_station -p pure_nav2:=false` 의 `reverse_out_zones`)은 허용되며 필요 시 추가 작성함.
- 실측 절차의 표준 순서: 고피 터미널 1(Isaac) → 내피 터미널 2(빌드·`nav2_link_check`) → 내피 터미널 3(`nav2.launch.py`) → 내피 터미널 4(파지 명령) → RViz2 클릭 → 결과 확인(`/feeder_dock/result`, 터미널 3 의 `[DONE]`).

2.4. **병합·파일 소유 규칙**
- development 병합 시 충돌은 pull된 파일이 이김. 그 뒤 내 추가분(navigation_node의 `station` 분기 / setup.py 진입점 6개: scan_sanitizer, cloud_self_filter, go_to_station, nav2_link_check, station_markers, feeder_dock / package.xml 추가 의존성: sensor_msgs, sensor_msgs_py, rosgraph_msgs, tf2_ros_py, visualization_msgs, nav2_bringup, nav2_simple_commander, pointcloud_to_laserscan)을 다시 붙여야 함. 이 목록을 ADR에 적어 두면 병합 후 점검표가 됨.
- 팀 파일 현황(2026-09-25 갱신): `runtime/standalone_app.py` 와 `scripts/{conveyor, conveyor_rollers, cull_motion, human_crossing, inspection_cull_station}.py` 는 커밋 `a95ba6c` 에서 **팀 브랜치 `origin/feature/cabbage-place-fix` 의 것을 그대로 반입**함. 내가 넣은 것은 그 파일 안의 `[navigation 2026-09-23]` 표시가 붙은 부분(`report_dock_pose`, Place 전 차체 정지 확인, main loop 의 `hold()` 예외 처리)뿐이며 팀 브랜치가 `ceeafd5` 에서 갈라져 그것을 이미 담고 있어 충돌 없이 들어왔음. `open_scene()` 의 fullScan 6줄은 팀이 `1e7fbc7` 로 정리해 현재는 팀 코드임. 자세한 것은 `docs/standalone_app_changes.md`. 그 외 팀 파일은 수정하지 않음.
- 미사용 파일은 각 모듈의 `past/`로. 저장소 트리 재구성은 하지 않음.
- `results/bags/`, `media_log/`는 git 제외. 대용량 USD 에셋도 제외.

2.5. **문서 규칙**
- 가이던스는 `guidance2_<n>차.md`, 답변마다 차수 증가, 이전 차수는 내용이 현 단계 작업에 유의미하지 않다면 `guidance/past/`로 이동시킬 것. 위에서 아래로 실행만 하면 되는 순서형, 몇 번째 터미널에 기입할 커맨드인지 명시, 각 터미널 커맨드 세션마다 실행 후 터미널 log를 'results/'에 저장하는 커맨드는 tee로 구성중, RViz2 클릭처럼 터미널이 없는 단계는 bag이 기록, 터미널 블록마다 환경 5줄 포함, "위와 같은 5줄" 같은 참조형 구성으로 작성은 금지, 선택 항목은 선택이라고 표시, 작업한 파일에 대해 파일명-역할-요약 설명을 마지막 챕터쯤에 테이블로 기입할 것.
- 사용자의 실측 피드백은 `errored/`에, 로그는 `results/`에. 트러블슈팅은 `errored/` 파일과 최신 bag를 먼저 읽고 시작.
- 아키텍처·트러블슈팅 문서는 `docs/architecture.md`는 갱신, `docs/troubleshooting.md`은 항목 추가(양 문서 모두 새 파일로 만들지 않음).

2.6. **현재 "keep" 상태의 기준선 (바꾸려면 보고 후)**
- planner Smac Hybrid-A*(Reeds-Shepp, 후진), RPP `desired_linear_vel` **0.3 m/s** (`config/nav2_params.yaml:162`. 옛 표기 0.6 은 철회) `allow_reversing`, goal 허용 0.25 m/0.5 rad, `max_angular_accel` 20, PoseProgressChecker, BT 응답 200 ms.
- `cloud_self_filter` 상자 x −0.65~0.60, y ±0.6, 합치기 0.25 s (이전 값 −0.85~0.6. 차체 뒤끝 −0.607 을 덮으면서 검출 면 −0.90 은 남김). `box_x[0]`은 `standoff_m`보다 앞에 있어야 함.
- `feeder_dock` standoff **0.92**, 자동 시작 반경 0.6 m, 면 길이 0.6~1.6 m, 법선 ±60°, 신선도 2.5 s, 도착 허용 방향 3°·횡 0.06 m, 재시도 2회(물러나는 거리 0.90 m).
- 미해결 1건 (2026-09-25 갱신): `standoff_m` 0.92 에서 **횡 오차 여유가 부족함.** 오프라인 격자 225 케이스 중 1 케이스가 허용 0.06 m 를 넘음(최대 0.063 m, 시작 world (−2.39, −1.85) yaw 70°, 제자리 회전이 명령의 12% 만 나오는 차체. `results/docksim_20260925_standoff092.txt:52`). 도킹 노드 자신은 `SUCCEEDED` 로 보고했으므로 노드가 재는 횡과 실제 횡이 3 mm 어긋난 것임. 0.90 일 때는 225/225(최대 0.059 m)였고, 도킹 거리가 멀수록 횡 오차를 갚을 후진 거리가 줄어드는 경향임. **팔 쪽 실제 횡 허용치를 받기 전에는 이득을 고치지 않음**(실측 전에 바꾸면 ROS 회귀 10/10 의 근거가 무효가 됨).
- 옛 미해결 항목(후진 중 방향이 틀어져 전진한 사례, 3회 중 1회)은 `ceeafd5` 의 `SETTLE` 단계와 면 법선 추종으로 해소된 것으로 보이나 **실측으로는 미확인**임.
- `/scan` 파이프라인: 3D 점군 → `cloud_self_filter`(상자 제거, 0.25 s 합침) → `pointcloud_to_laserscan`(높이 −0.35~1.5 m, `range_min` 0.3, 0.5°) → `/scan`. 2D 라이다는 쓰지 않음.
- `standoff_m`(0.92) 과 `box_x[0]`(−0.65) 은 짝임. 도킹 거리를 줄이면 상자 뒤끝도 그보다 앞(값이 더 큼)에 두어야 면이 지워지지 않음. 차체 뒤끝은 base_link −0.607 이므로 `standoff_m` 은 0.75 아래로 두지 않음.
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