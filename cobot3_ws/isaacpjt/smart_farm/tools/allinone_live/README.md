# allinone_live — 올인원 한 줄 실행 (Isaac Sim 5.1 + WSL2 Nav2)

수확(PICK_HARVEST) → Nav2 주행(FEEDER_DOCK) → 벨트에 내려놓기(PLACE_INSPECT) → 컨베이어 → 비전룸 YOLO 검사
→ 노랑·갈색 솎아내기(SortBin_1/2) → 재검사 → 배출까지 명령 하나로 돌린다.

```powershell
# 기본 (팀 기본 동작 그대로)
powershell -NoProfile -ExecutionPolicy Bypass -File cobot3_ws\isaacpjt\smart_farm\tools\allinone_live\run_allinone.ps1

# 사람 돌발상황 + 바닥 노란 차선 주행까지
powershell -NoProfile -ExecutionPolicy Bypass -File cobot3_ws\isaacpjt\smart_farm\tools\allinone_live\run_allinone.ps1 -Human -Lane
```

| 옵션 | 기본 | 내용 |
|---|---|---|
| `-Human` | 꺼짐 | 작업자가 카터 경로에 들어와 4초 섰다가 비킨다. 카터는 트레이 끝 0.8 m 앞에서 멈췄다가 1~2초 안에 다시 간다. 사람 에셋을 NVIDIA 서버에서 받으므로 **인터넷 필요** |
| `-Lane` | 꺼짐 | 랙 통로 노란 차선 중앙선(x -0.42)을 따라 후진 → 모서리 곡선 45° 까지 → Nav2 기본 경로로 FEEDER |
| `-OutDir` | `~\smartfarm_runs\allinone_run` | 로그·기록 폴더 |
| `-Scene` | `DEFAULT` | 씬 USD. DEFAULT = `scenes/Collected_smartfarm_v014/Collected_smartfarm_v014_room_core_cabbage.usd` (공유 zip) |
| `-NoFlow` | | Isaac·Nav2 만 띄우고 명령은 안 보냄 |
| `-NoRviz` | | RViz2 안 띄움 |
| `-IsaacDir` / `-Distro` | `D:\isaacsim` / `Ubuntu-24.04` | 설치 위치가 다르면 지정 |
| `-PyLib` | `~\smartfarm_runs\pylib` | Windows 쪽 중계기용 라이브러리 폴더. Isaac 내장 rclpy 가 `yaml`·`numpy` 를 쓰는데 Isaac 파이썬에 없어서, 비어 있으면 처음 한 번 pyyaml·numpy 를 자동 설치(인터넷 필요) |

**팀 파일은 바꾸지 않는다.** 옵션을 켜면 WSL 에서 팀 `nav2_params.yaml` 을 읽어 복사본 `OUT/nav2_params_test.yaml` 을 만들어 쓴다.
Isaac 쪽은 `runtime/standalone_app.py` 의 `--human-crossing` 옵션(표시 `[올인원 2026-09-25]`)과 `scripts/human_crossing.py` 가 맡는다.
씬(에셋) 파일도 그대로다: 사람은 실행할 때 원래 씬을 감싼 임시 장면(`%TEMP%\smartfarm_human\*_human.usda`)에 더해진다.

## 전제
- Windows: Isaac Sim 5.1 (`D:\isaacsim`), 공유 zip 을 `smart_farm/scenes/Collected_smartfarm_v014/` 에 푼 씬, `best.pt`
- WSL2 Ubuntu 24.04 (NAT 네트워크): 한 번만 `wsl -d Ubuntu-24.04 -- bash <이 폴더의 WSL 경로>/wsl/setup_wsl_nav2.sh`
  → ROS 2 Jazzy + Nav2 설치, 이 브랜치를 `~/rokey/ROKEY_P3_A1` 에 받아 Nav2 패키지 빌드, `~/nav2_env.sh` 설치
- Windows ↔ WSL 은 DDS 가 안 넘어가서 `wsl/ros_tcp_relay.py` 가 localhost TCP 로 토픽을 잇는다 (Windows 도메인 101, WSL 102)
- 자세한 설치 순서: `docs/beginner_manual_allinone_2026-09-25.md` 0~1절

## 파일
| 파일 | 어디서 | 하는 일 |
|---|---|---|
| `run_allinone.ps1` | Windows | 전체 실행 (WSL Nav2 → Isaac → 중계기 → 흐름 → 완료 대기) |
| `run_with_monitor.py` | Isaac 파이썬 | standalone_app 을 그대로 실행 + 기록(`monitor.json`: 카터 궤적 등) + 카메라 캡처(`CABBAGE_CAPTURE_CAMS`) |
| `floor_lines.py` | Isaac 파이썬 | 씬 바닥 선 좌표 추출·그림 (차선 좌표의 출처) |
| `wsl/setup_wsl_nav2.sh`, `wsl/nav2_env.sh` | WSL | 한 번 설치, 실행 환경(ROS_DOMAIN_ID 102) |
| `wsl/start_nav2_stack.sh` | WSL | 중계기 → (시험용 설정 복사본) → 팀 Nav2 → feeder_dock → navigation_node |
| `wsl/ros_tcp_relay.py` | 양쪽 | Windows ↔ WSL 토픽 중계 (`wsl` 서버 / `win` 클라이언트) |
| `wsl/run_flow.py` | WSL | 팀 절차대로 PICK_HARVEST → NAVIGATION → PLACE_INSPECT 명령 |
| `wsl/make_nav2_test_params.py` | WSL | 팀 nav2_params.yaml → 복사본 (`--human`: 트레이 윤곽·HumanStop/HumanSlow 등, `--lane`: 차선 BT) |
| `wsl/lane_planner.py` | WSL | 차선 경로 액션 서버 `lane_planner` + 도킹 구역에서 사람 정지 영역 끄기(안전 영역 전환) |
| `wsl/lane_route_bt.xml` | WSL | 차선 경로 → Nav2 기본 경로(하위 트리 DefaultNav) BT |
| `wsl/record_drive.sh`, `comm_panes.sh`, `snap_loop.sh` | WSL | RViz + ROS2 통신 화면 녹화 |

## 확인
- `OUT/flow.log`: `navigation result SUCCEEDED ... reached=FEEDER_DOCK`, `PLACE_INSPECT ... SUCCEEDED`
- `OUT/isaac.log`: `[솎아내기] Pallet_01 완료 — 제거 [...]`, (-Human) `[사람] WALK_IN / STAND_IN_PATH / WALK_OUT / CLEAR / ROBOT_RESUMED`
- (-Lane) `OUT/lane_planner.log`: `LaneRoute: 47 poses ...`, `safety field DOCK ...`
- (-Human) `OUT/collision_state.log`: `polygon_name: HumanStop`, `action_type: 1`

## 녹화
```powershell
$env:CABBAGE_CAPTURE_CAMS = "human=/World/Characters/HumanViewCam"; $env:CABBAGE_CAPTURE_EVERY = "0.2"   # 실행 전에. -> OUT\captures\*.jpg
```
```bash
HUMAN=1 bash wsl/record_drive.sh <OUT 의 WSL 경로>/linux 1500     # Nav2 준비 뒤, 흐름 시작 전에
```
Windows 창 녹화(gdigrab)로는 Isaac 3D 뷰포트가 갱신되지 않는다. Isaac 화면은 위 카메라 캡처로 만든다.

## 알려진 한계
- 카터+리프트+팔+트레이 리그가 제자리에서 돌지 못하고(원인 미확인) 주행 회전 반경이 약 1.2 m 라서, 차선 모서리 두 개(사이 1.77 m)를
  모두 차선대로 돌 수 없다. `-Lane` 은 첫 모서리 45° 까지만 차선을 따른다.
- FEEDER 도킹(feeder_dock) 중에는 사람 정지가 걸리지 않는다.
- `/navigation/result` 에 "사람 대기" 사유는 없다(팀 인터페이스). `/collision_monitor_state` 로 본다.
- 자세한 설명·트러블슈팅: `docs/beginner_manual_allinone_2026-09-25.md` 9절
