# guidance2_12차 — Collected_smartfarm_v008.usd 에서 Nav2 + RViz2 연동 (다중 PC)

- 작성일: 2026-09-21, 브랜치: `feature/navigation2`
- 목적: 고피는 Isaac Sim 만 실행하고, 내피에서 Nav2·RViz2 를 실행하여 카터를 `RACK_DOCK`(시작 위치) → `INSPECTION_DOCK`(컨베이어 앞) 으로 자율주행시킴.
- 기준 장면: `/home/rokey/ROKEY_P3_A1/cobot3_ws/isaacpjt/smart_farm/scenes/Collected_smartfarm_v008/Collected_smartfarm_v008.usd`
- 기준 지도: `/home/rokey/ROKEY_P3_A1/cobot3_ws/isaacpjt/smart_farm/maps/Collected_smartfarm_v005.yaml` (v008 의 랙·컨베이어·TurnTable 위치와 일치함을 확인함)
- 카터는 1대이며 USD 의 ROS namespace 는 비어 있음. 토픽은 `/cmd_vel`, `/chassis/odom`, `/tf`, `/front_2d_lidar/scan`, `/front_3d_lidar/lidar_points` 그대로임.

## 0. 동작 원리 (한 번만 읽으면 됨)

```
[고피] Isaac Sim (launch_scene.py)                     [내피] Nav2 (nav2.launch.py)
  /clock  /tf(odom->base_link)  /chassis/odom   ───►   AMCL · planner · controller · RViz2
  /front_2d_lidar/scan (없으면 3D 점군)          ───►   scan_sanitizer ─► /scan
  /cmd_vel                                      ◄───   collision_monitor
```

1. **/clock**: v008 장면에는 `ROS_Clock` 그래프가 없음. Isaac 이 보내는 `/tf`·`/chassis/odom` 은 시뮬레이션 시간으로 찍히므로, Nav2 도 `use_sim_time` 으로 같은 시계를 써야 함. `launch_scene.py` 가 장면을 연 뒤 `/World/ROS_Clock_runtime` 그래프를 **실행 중에만** 추가함 (USD 파일은 바뀌지 않음).
2. **/scan**: 리프트 기둥 4개가 라이다 바로 옆(0.2~0.55 m)에 있어 그대로 쓰면 Nav2 가 "발밑에 장애물" 로 보고 멈춤. `scan_sanitizer` 가 뒤쪽 반원의 0.6 m 이내 반사만 지우고 `/scan` 으로 다시 내보냄. 2D 스캔이 안 오면 자동으로 3D 점군 → `pointcloud_to_laserscan` 경로를 씀(`range_min 0.4`).
3. **통로 탈출**: 시작 위치는 Rack_1 옆 통로 안임. 여기서 제자리 회전하면 리그 뒤쪽(회전 반경 0.66 m)이 랙·팔레트에 닿음. 그래서 `go_to_station` 은 카터가 통로 안에 있으면 먼저 Nav2 `BackUp` 으로 **base_link −x(카터의 보이는 정면) 방향으로 2.2 m 직진**해 통로를 빠져나온 뒤 목표를 보냄. 1차 시연의 "전진 탈출" 과 같은 움직임임.
4. **도착 자세**: `INSPECTION_DOCK` 은 world (3.328, −5.251), yaw +90°. base_link +x 가 북쪽이므로 카터의 보이는 정면(−x)이 컨베이어를 수직으로 바라봄. 1차 시연과 같은 자리(컨베이어 x 2.33~4.33 구간 가운데)이며, v008 에서 이 구간의 prim 이름은 `Seg_4` 임.

## 1. 고피 — 터미널 1 (Isaac Sim 만)

터미널 환경은 튜터 지시대로 준비함(`ros_set`, `isaac_ros` 사용 여부 포함). 그 다음 아래 한 줄을 실행함.

```bash
isaac_python /home/rokey/ROKEY_P3_A1/cobot3_ws/src/smart_farm_navigation/scripts/launch_scene.py
```

기대 출력 (순서대로):

```
[launch_scene] opening /home/rokey/.../Collected_smartfarm_v008.usd
[launch_scene] robot prim /World/SmartFarm/Placed/LiftRig/Asset/nova_carter_ROS: x=-0.42 y=1.01 ... yaw=90.0deg
[launch_scene] /clock: added runtime graph /World/ROS_Clock_runtime (not saved to the USD)
[launch_scene] PLAY
```

- `x`, `y`, `yaw` 가 위 값과 0.05 m / 2° 이상 다르면 장면이 바뀐 것임. 4절의 "초기 위치가 다를 때" 를 따름.
- `/clock graph creation FAILED` 가 나오면 그 줄을 그대로 `errored/` 에 저장함.
- 고피에서 할 일은 여기까지임. 빌드도 필요 없음.

## 2. 내피 — 터미널 2 (빌드, 연결 점검)

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

기대 출력: `/clock`, `/tf odom->base_link`, `/chassis/odom` 이 각각 수십 Hz, 라이다 두 줄 중 하나 이상이 3 Hz 이상, 마지막 줄 `RESULT OK - nav2.launch.py scan_mode:=scan2d` (또는 `cloud`).

- `real-time factor` 가 1 보다 작아도 됨. Nav2 는 시뮬레이션 시간으로 움직임.
- `RESULT FAIL` 이면 Nav2 를 띄우지 말고 4절을 봄.
- 2D 스캔(수십 kB/s)이 오면 Wi-Fi 로도 충분함. 3D 점군만 오면 수 MB/s 이므로 유선(10.10.0.x)이어야 함.

## 3. 내피 — 터미널 3 (Nav2 + RViz2)

터미널 2 와 같은 5줄(`export` 3줄, `source` 2줄)을 먼저 실행한 뒤:

```bash
ros2 launch smart_farm_navigation nav2.launch.py 2>&1 | tee /home/rokey/ROKEY_P3_A1/cobot3_ws/src/smart_farm_navigation/results/nav2_$(date +%Y%m%d_%H%M).txt
```

기대 결과:

- 첫 줄 근처에 `[nav2.launch] scan_mode auto -> scan2d; AMCL initial pose (-0.421, 1.006, 90.0deg) ...`
- `scan_sanitizer: 10.0 Hz, NN self-returns removed per scan` 이 5초마다 나옴.
- `lifecycle_manager_navigation: Managed nodes are active` 가 나오면 준비 완료.
- RViz2 에서 지도 위에 카터가 Rack_1·Rack_4 사이 통로에 있고, 빨간 스캔 점이 랙·컨베이어 검은 영역의 가장자리와 겹쳐야 함. 어긋나 있으면 RViz2 의 `2D Pose Estimate` 로 실제 위치를 찍어 줌.

## 4. 내피 — 터미널 4 (주행 지시)

터미널 2 와 같은 5줄을 먼저 실행한 뒤, 둘 중 하나를 고름.

**방법 A. 직접 지시 (먼저 이것으로 확인)**

```bash
ros2 run smart_farm_navigation go_to_station --ros-args -p station:=INSPECTION_DOCK 2>&1 | tee /home/rokey/ROKEY_P3_A1/cobot3_ws/src/smart_farm_navigation/results/goto_$(date +%Y%m%d_%H%M).txt
```

기대 출력:

```
in zone RACK1_CORRIDOR at (-0.42, 1.01, 90deg): BackUp 2.21 m at 0.25 m/s before navigating
BackUp done
goToPose INSPECTION_DOCK: (3.33, -5.25, 90.0deg)
  remaining ... m, elapsed ...s, recoveries 0
RESULT SUCCEEDED for INSPECTION_DOCK after ..s
```

돌아오기: `station:=RACK_DOCK`. 통로 앞 `CORRIDOR_EXIT` 에서 방향을 맞춘 뒤 직진으로 들어감.

**방법 B. 1차 시연과 같은 지시 토픽 (`/navigation/command`)**

```bash
ros2 launch smart_farm_navigation navigation_node.launch.py
```

다른 터미널(같은 5줄 후)에서:

```bash
ros2 topic pub --once /navigation/command std_msgs/msg/String '{data: "{\"command_id\": \"TASK-20260921-001-CMD-001\", \"task_id\": \"TASK-20260921-001\", \"operation\": \"NAVIGATION\", \"destination\": \"INSPECTION_DOCK\"}"}'
ros2 topic echo /navigation/result --once --full-length
```

결과 JSON 의 `status` 가 `SUCCEEDED`, `reached_station` 이 `INSPECTION_DOCK` 이면 됨. `mode:=cmd_vel` 을 붙이면 1차 시연 방식(경로 주행)으로 되돌아감.

## 5. 문제가 생겼을 때

| 증상 | 원인과 조치 |
|---|---|
| link_check 에서 `/clock 0.0 Hz` | Isaac 을 `launch_scene.py` 가 아닌 다른 방법으로 띄움. 1절대로 다시 실행 |
| link_check 에서 전부 0 Hz | 도메인·화이트리스트 문제. 내피에서 `echo $ROS_DOMAIN_ID` 가 101 인지, `hostname -I` 의 IP(현재 10.10.0.3)가 양쪽 `~/.ros/fastdds_whitelist.xml` 에 있는지 확인 |
| 라이다 두 줄 모두 0 Hz | Isaac 화면 렌더링이 멈춰 있으면(최소화 등) RTX 라이다가 안 나옴. 창을 띄워 둠 |
| Nav2 로그에 `extrapolation into the future/past` 가 계속 나옴 | 두 PC 시계 문제가 아니라 `/clock` 유실임. 유선 연결 권장. 일시적이면 무시 |
| `RegulatedPurePursuitController detected collision ahead!` 반복 | 지도보다 실제 물체가 가깝게 보임. RViz2 의 local costmap 을 캡처해 `errored/` 에 저장 |
| `BackUp FAILED` | 카터 뒤(−x 방향)에 장애물이 보였거나 위치추정이 틀어짐. RViz2 에서 `2D Pose Estimate` 후 재시도 |
| 초기 위치가 다를 때 | 1절 출력의 x, y, yaw 를 그대로 넣어 실행: `ros2 launch smart_farm_navigation nav2.launch.py initial_x:=-0.42 initial_y:=1.01 initial_yaw_deg:=90.0` |
| 2D 스캔이 이상할 때 | `ros2 launch smart_farm_navigation nav2.launch.py scan_mode:=cloud` 로 3D 점군 경로 강제 |

실패 로그는 `/home/rokey/ROKEY_P3_A1/cobot3_ws/src/smart_farm_navigation/errored/` 에, 성공 로그(results/*.txt)는 그대로 커밋·푸시함.

## 6. 이번에 만든 것 (파일 지도)

| 파일 | 실행 위치 | 역할 |
|---|---|---|
| `scripts/launch_scene.py` | 고피 | v008 열기, `/clock` 그래프 런타임 추가, Play |
| `launch/nav2.launch.py` | 내피 | Nav2 bringup + RViz2 + /scan 생성(자동 선택) |
| `config/nav2_params.yaml` | 내피 | AMCL·costmap·RPP 컨트롤러 파라미터 |
| `config/stations.yaml` | 내피 | 초기 위치, 작업점, 통로 후진 탈출 구역 |
| `smart_farm_navigation/scan_sanitizer.py` | 내피 | 리프트 자기 반사 제거 |
| `smart_farm_navigation/go_to_station.py` | 내피 | 통로 탈출 + NavigateToPose |
| `smart_farm_navigation/nav2_link_check.py` | 내피 | Isaac → 내피 토픽 도달 점검 |
| `config/destinations_nav2.yaml`, `launch/navigation_node.launch.py` | 내피 | `/navigation/command` 를 Nav2 주행으로 연결 |

## 7. 내피 사전 시험 결과와 알려진 한계

- 내피에서 지도 기반 가상 로봇(/clock·/tf·odom·2D 스캔 + 리프트 반사 모사)으로 `RACK_DOCK → INSPECTION_DOCK → RACK_DOCK` 왕복을 3회 반복해 모두 `SUCCEEDED`, recovery 0회, 편도 18~21초였음. `/navigation/command` 경로도 같은 조건에서 확인함. Isaac Sim 실물 시험은 아직 안 함.
- 가상 시험에서 도착 시 위치추정 오차가 0.2~0.35 m 있었음. 가상 센서의 한계일 가능성이 크나, 실물에서도 비슷하면 M0609 Place 정밀도에 부족함. 4절 실행 후 Isaac 화면의 실제 정지 위치(1절 방식으로 prim 좌표 확인)와 (3.328, −5.251) 의 차이를 알려주면 AMCL 또는 최종 접근 단계를 조정함.
- 지도의 랙 가장자리는 x −1.10 인데 USD 의 랙·팔레트 외곽은 x −0.86 임(지도 생성 높이에 팔레트가 안 잡힘). Nav2 는 통로를 실제보다 0.24 m 넓게 봄. 통로 안 회전을 막은 이유이며, 장기적으로는 지도를 팔레트 높이까지 포함해 다시 만들어야 함.
- `launch_scene.py` 는 리프트·M0609 를 제어하지 않음. 팀 통합 standalone 과 동시에 실행할 수 없음(한 시뮬레이션에 SimulationApp 하나). 통합 시에는 팀 스크립트에 `/clock` 그래프 추가 부분만 옮기면 됨.
