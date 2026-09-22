# guidance2_13차 — Collected_smartfarm_v011.usd 에서 Nav2 로 Feeder 입구까지 자율주행 (다중 PC)

- 작성일: 2026-09-22, 브랜치: `feature/navigation2`. 12차 고피 실측 피드백(`errored/error_260921_2100.md`) 반영판.
- 목적: 고피는 Isaac Sim 만 실행하고, 내피에서 Nav2·RViz2 를 실행하여 카터를 랙 통로(RACK_DOCK) → Feeder 입구(FEEDER_DOCK) 로 보냄. 복귀(FEEDER_DOCK → RACK_DOCK)도 같은 명령으로 함.
- 기준 장면: `/home/rokey/ROKEY_P3_A1/cobot3_ws/isaacpjt/smart_farm/scenes/Collected_smartfarm_v011/Collected_smartfarm_v011.usd`
- 기준 지도: `/home/rokey/ROKEY_P3_A1/cobot3_ws/isaacpjt/smart_farm/maps/Collected_smartfarm_v011.yaml` (이번에 v011 에서 새로 생성. v005 지도는 TurnTable 이 기울어져 그려져 있어 새 목적지와 어긋남)
- 이 문서만 보고 실행할 수 있게 썼음. 12차 문서는 `guidance/past/` 로 옮김.

## 0. 12차 실측에서 확인된 것과 고친 것

| 실측에서 본 현상 | 원인 | 이번 조치 |
|---|---|---|
| 통로 후진 탈출 뒤 카터가 한참 가만히 있다가(1차 180 s, 2차 150 s) 스스로 방향을 틀고 출발 | 정지가 아니라 **아주 느린 제자리 회전**이었음. 로그에 `Failed to make progress` 가 15 s 마다 반복됨. 컨트롤러(RPP)가 회전 속도를 "지금 측정된 각속도 + 가속 한계 2.0 rad/s² × 0.05 s" 로 제한하는데, Isaac 이 보내는 odom 각속도가 이 계산에 맞지 않아 매 주기 0.1 rad/s 이하로만 명령됨. 진행 판정기(SimpleProgressChecker)는 회전을 진행으로 안 쳐서 15 s 마다 중단·재시작 | `max_angular_accel` 20.0 (사실상 제한 해제. 실제 가속 제한은 Isaac 의 DifferentialController 가 함), 진행 판정기를 회전도 인정하는 `PoseProgressChecker` 로 교체. 내피 모의(odom 각속도 0 으로 고정)에서 재현 후 해소 확인 |
| 도착 0.16 m 앞에서 멈춘 채 recovery 10회 | 도착 허용 반경이 0.15 m 라 0.16 m 에서 수렴하지 못하고 흔들림 | `xy_goal_tolerance` 0.25 m, `yaw_goal_tolerance` 0.15 rad. 팔 배치 정밀도는 M0609 쪽 보정에 맡김 |
| 전체적으로 느림 | 장애물·곡선 근접 감속(regulated scaling)과 도착 0.6 m 전 감속 | regulated scaling 끔, 도착 감속 구간 0.3 m, 충돌 감시(collision_monitor) 감속 시작 1.2 s → 0.8 s 전. 최고 속도 0.5 m/s 는 유지 |
| costmap 범위 | 요청대로 | local costmap 5 m → 4 m |
| 방법 B 에서 `TIMEOUT` | navigation_node 의 제한 180 s 가 벽시계 기준인데 Isaac 실시간 배율이 0.4 라 왕복에 3~4 분 걸림 | 제한 600 s |
| `ros2 launch navigation_node` 를 Ctrl+C 하면 traceback | rclpy 를 두 번 shutdown | 수정 |
| 2차 세션 첫 Nav2 실행에서 `timestamps differ on 480 s` 후 `Goal failed` | Isaac 을 다시 Play 했는데 Nav2 를 그대로 둠. Isaac 이 다시 시작되면 시뮬레이션 시계가 0 으로 돌아가므로 Nav2 도 다시 띄워야 함 | 절차에 명시 (3절) |
| 카터 앞뒤 | **구동륜(큰 바퀴) 쪽이 앞 = base_link +x** 로 확정. 시작 배치는 카터의 **뒤(캐스터·M0609 쪽)가 통로 출구를 향함** (랙 P&P 를 위한 팀 배치라 바꾸지 않음) | 통로 탈출은 Nav2 의 경로 계획이 아니라 `go_to_station` 이 먼저 **후진(BackUp)** 명령을 내려 뒤로 직진 탈출한 뒤에 Nav2 목표를 보냄. 통로 안에서는 항상 실행되며 제자리 회전은 하지 않음. 12차 실측 두 번 모두 이 후진은 정상이었음 (`Backing up 2.199 m … backup completed successfully`) |
| 최종 목적지 변경 | Feeder 에 붙은 돌출부(TurnTable)의 입구 | 새 작업점 `FEEDER_DOCK` (4절) |

**질문에 대한 답 — "Nav2 가 통로에서 안정적으로 후진해 주는가"**: Nav2 의 경로 추종(RPP)은 후진 계획을 세우지 않고 제자리 회전으로 방향을 잡음(`allow_reversing: false`). 통로 안에서 회전하면 리그 뒤쪽 0.66 m 가 랙·팔레트에 닿으므로, 후진은 Nav2 에 맡기지 않고 `go_to_station` 이 `config/stations.yaml` 의 `reverse_out_zones` 에 카터가 있으면 **무조건 먼저 후진**시키도록 되어 있음. 후진 거리·속도·방향은 그 파일에 적혀 있고, 뒤쪽이 출구 방향에서 20° 이상 틀어져 있으면 후진하지 않고 경고만 냄. Feeder 입구 앞에서도 같은 규칙(FEEDER_APRON 구역, 북쪽으로 1.2 m 후진)이 적용됨.

## 1. 새 목적지 FEEDER_DOCK

- v011 에서 Feeder 는 `/World/SmartFarm/Placed/Conveyor/Feeder` (x −2.71~−1.63, y −7.18~−6.12) 이고, 거기서 북쪽으로 튀어나온 것이 `Conveyor/TurnTable` (x −2.76~−1.61, y −6.32~−3.60) 임. **TurnTable 의 북쪽 끝(y −3.60)을 입구로 보고**, 카터가 그 앞 0.55 m 에서 **앞(+x, 구동륜 쪽)이 입구(남쪽)를 보도록** 정함.
- world (−2.19, −3.05), yaw −90°. 입구 가장자리에서 base_link 까지 0.55 m.
- **팔 도달 거리 확인 필요**: M0609 는 카터 뒤쪽(base_link 기준 x −0.42)에 있음. 카터 앞이 입구를 보면 팔 밑동은 입구에서 약 0.97 m 떨어짐. 1차 시연에서는 반대로 팔 쪽이 컨베이어를 보고 0.5 m 거리에서 Place 했음. 팔이 안 닿으면 `config/stations.yaml` 의 `FEEDER_DOCK` 을 `yaw_deg: 90.0`(팔 쪽이 입구를 봄) 으로 한 줄만 바꾸면 됨. 이 경우 후진 탈출 구역 `FEEDER_APRON` 의 `exit_heading_deg` 도 `-90.0` 으로 같이 바꿔야 함.
- 다른 지점을 원하면 같은 파일의 x, y, yaw_deg 만 바꾸고 재빌드(`colcon build`) 함.

## 2. 고피 — 터미널 1 (Isaac Sim 만)

터미널 환경은 튜터 지시대로 준비함. 그 다음 한 줄:

```bash
isaac_python /home/rokey/ROKEY_P3_A1/cobot3_ws/src/smart_farm_navigation/scripts/launch_scene.py
```

기대 출력: `opening …/Collected_smartfarm_v011.usd` → `robot prim …/nova_carter_ROS: x=-0.42 y=1.01 … yaw=90.0deg` → `/clock: added runtime graph` → `PLAY`.

- Isaac 을 **다시 Play 하거나 다시 실행하면 3절의 Nav2 도 반드시 다시 실행**해야 함 (시뮬레이션 시계가 0 으로 돌아가 Nav2 가 센서를 무시함).
- Isaac 창을 최소화하지 않음 (렌더링이 멈추면 라이다가 안 나옴).

## 3. 내피 — 터미널 2 (빌드, 연결 점검) / 터미널 3 (Nav2 + RViz2)

터미널 2:

```bash
export ROS_DOMAIN_ID=101
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE=/home/rokey/.ros/fastdds_whitelist.xml
source /opt/ros/jazzy/setup.bash
cd /home/rokey/ROKEY_P3_A1/cobot3_ws
colcon build --packages-select smart_farm_navigation
source /home/rokey/ROKEY_P3_A1/cobot3_ws/install/setup.bash
ros2 run smart_farm_navigation nav2_link_check 2>&1 | tee /home/rokey/ROKEY_P3_A1/cobot3_ws/src/smart_farm_navigation/results/link_$(date +%Y%m%d_%H%M).txt
```

기대: `/clock`, `/tf`, `/chassis/odom` 이 20 Hz 이상, `/front_3d_lidar/lidar_points` 20 Hz 이상, 마지막 줄 `RESULT OK - nav2.launch.py scan_mode:=cloud`. (12차 실측에서 2D 스캔은 오지 않았고 3D 점군 1.2~1.3 MB/s 로 동작했음. 유선 연결 유지.)

터미널 3 (같은 `export` 3줄 + `source` 2줄 먼저):

```bash
ros2 launch smart_farm_navigation nav2.launch.py 2>&1 | tee /home/rokey/ROKEY_P3_A1/cobot3_ws/src/smart_farm_navigation/results/nav2_$(date +%Y%m%d_%H%M).txt
```

기대: `[nav2.launch] scan_mode auto -> cloud; AMCL initial pose (-0.421, 1.006, 90.0deg) …; map …/Collected_smartfarm_v011.yaml`, 이어서 `Managed nodes are active`. RViz2 지도에서 카터가 Rack_1·Rack_4 사이 통로에 있고, 스캔 점이 랙·컨베이어·TurnTable 가장자리에 붙어 있어야 함.

## 4. 내피 — 터미널 4 (주행 지시)

같은 5줄을 먼저 실행한 뒤 아래 **한 가지만** 실행하면 됨. 두 방식은 같은 주행을 다른 입구로 시키는 것이라 둘 다 할 필요 없음.

- **방법 A (권장, 이번 실측용)**: 터미널에서 목적지 이름을 직접 주는 방식. 결과가 그 터미널에 바로 찍힘.
- **방법 B**: 팀 통합 스크립트가 쓰게 될 방식. `/navigation/command` 토픽에 JSON 을 보내면 `navigation_node` 가 방법 A 와 똑같은 주행을 대신 실행하고 `/navigation/result` 로 결과를 보냄. 통합 시점에 한 번만 확인하면 되는 항목이며, 이번 실측에서는 생략해도 됨.

**방법 A**

```bash
ros2 run smart_farm_navigation go_to_station --ros-args -p station:=FEEDER_DOCK 2>&1 | tee /home/rokey/ROKEY_P3_A1/cobot3_ws/src/smart_farm_navigation/results/goto_$(date +%Y%m%d_%H%M).txt
```

기대 출력과 화면:

```
in zone RACK1_CORRIDOR at (-0.42, 1.01, 90deg): BackUp 2.21 m at 0.25 m/s before navigating   <- 카터가 뒤로 직진해 통로를 빠져나옴
BackUp done
goToPose FEEDER_DOCK: (-2.19, -3.05, -90.0deg)     <- 곧바로 제자리 회전(약 4 s) 후 출발. 멈춰 있는 구간이 없어야 함
  remaining … m, elapsed …s, recoveries 0
RESULT SUCCEEDED for FEEDER_DOCK after …s          <- TurnTable 북쪽 끝 앞 0.55 m, 앞이 남쪽
```

돌아오기: `station:=RACK_DOCK`. Feeder 앞에서 북쪽으로 1.2 m 후진 → CORRIDOR_EXIT 에서 방향을 맞춤 → 통로로 직진 진입.

- 실시간 배율 0.4 기준으로 편도 1~2 분 걸림. `recoveries` 가 0 이 아니거나 60 s 넘게 `remaining` 이 안 줄면 그 로그를 `errored/` 에 둠.
- 도착 위치 확인: 1절 값과 Isaac 화면의 실제 정지 위치를 비교해 알려주면 됨 (허용 오차 0.25 m).

**방법 B** (통합 확인 때만)

```bash
ros2 launch smart_farm_navigation navigation_node.launch.py
```

다른 터미널(같은 5줄 후):

```bash
ros2 topic pub --once /navigation/command std_msgs/msg/String '{data: "{\"command_id\": \"TASK-20260922-001-CMD-001\", \"task_id\": \"TASK-20260922-001\", \"operation\": \"NAVIGATION\", \"destination\": \"FEEDER_DOCK\"}"}'
ros2 topic echo /navigation/result --once --full-length
```

`status` 가 `SUCCEEDED`, `reached_station` 이 `FEEDER_DOCK` 이면 됨.

## 5. 문제가 생겼을 때

| 증상 | 조치 |
|---|---|
| link_check `/clock 0 Hz` | Isaac 을 2절 방법으로 다시 실행 |
| link_check 전부 0 Hz | `echo $ROS_DOMAIN_ID` 가 101 인지, `hostname -I` 의 IP(10.10.0.3)가 양쪽 `~/.ros/fastdds_whitelist.xml` 에 있는지 |
| Nav2 로그 `timestamps differ on … seconds` / `Goal failed` 즉시 | Isaac 이 재시작된 뒤 Nav2 를 안 띄운 것. 터미널 3 을 Ctrl+C 후 다시 실행 |
| 후진 탈출 뒤 30 s 이상 정지 | 이번 수정의 대상. 재현되면 `results/nav2_*.txt` 와 `goto_*.txt` 를 `errored/` 로 옮겨 둠 |
| `in zone … rear points NNdeg away from the exit; not backing up` | 카터 뒤가 출구를 안 봄(장면 배치가 바뀜). 1절/`stations.yaml` 의 `exit_heading_deg` 확인 |
| 초기 위치가 다름 | 2절 출력 값으로: `ros2 launch smart_farm_navigation nav2.launch.py initial_x:=… initial_y:=… initial_yaw_deg:=…` |

## 6. 파일 지도

| 파일 | 실행 위치 | 역할 |
|---|---|---|
| `scripts/launch_scene.py` | 고피 | v011 열기, `/clock` 그래프 런타임 추가, Play |
| `scripts/make_map_from_usd.py` | 내피(usd-core 파이썬) | USD 에서 지도 생성. `maps/Collected_smartfarm_v011.{png,yaml}` 을 만들었음 |
| `launch/nav2.launch.py` | 내피 | Nav2 bringup + RViz2 + /scan 생성 |
| `config/nav2_params.yaml` | 내피 | 이번 수정 항목은 0절 표 참고 |
| `config/stations.yaml` | 내피 | 초기 위치, FEEDER_DOCK 등 작업점, 후진 탈출 구역 2개 |
| `smart_farm_navigation/go_to_station.py` | 내피 | 후진 탈출 + 경유점 + NavigateToPose |
| `smart_farm_navigation/nav2_link_check.py` | 내피 | Isaac → 내피 토픽 도달 점검 |
| `config/destinations_nav2.yaml`, `launch/navigation_node.launch.py` | 내피 | 방법 B |

## 7. 내피 모의 시험 결과와 한계

- 12차 실측의 "느린 회전" 을 모의 로봇에서 odom 각속도를 0 으로 고정해 재현한 뒤, 수정 후 FEEDER_DOCK 왕복 2회 모두 `SUCCEEDED`, recovery 0, 편도 8~12 s(실시간 배율 1 기준)였음. 방법 B 도 `SUCCEEDED`.
- 모의 도착 위치는 목표에서 0.2 m, 6° 안쪽이었음. 팔 Place 정밀도가 부족하면 도착 허용 오차를 다시 줄이거나, Nav2 도착 후 짧은 직진 보정 단계를 추가할 수 있음.
- ADR_navigation2 1.2 의 지도 경로는 v005 로 되어 있으나 이번 실행은 v011 지도를 씀. ADR 갱신 여부는 사용자 결정 사항임.
- 장면 디렉터리의 에셋 정리는 이쪽 작업에 영향 없음. 지도 생성과 좌표 확인은 `.usd` 본체와 `SubUSDs/` 만 있으면 됨 (v011 은 둘 다 있음).
