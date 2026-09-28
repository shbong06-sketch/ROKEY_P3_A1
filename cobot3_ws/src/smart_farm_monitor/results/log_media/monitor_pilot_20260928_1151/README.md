# 파일럿 실측 — 관제 웹·DB 첫 실데이터 (2026-09-28)

- 기기 **고피3** (`gc-isaacsim-lwh`, NVIDIA L4) · 시작 `2026-09-28T11:51:59Z` · 종료 12:10 (KST 기준 폴더명 시각)
- 실행자: 에이전트 대행 (ADR_basic §5-8). 사용자가 승인한 범위 = `guidance3_27차` 4장 전 구간 1사이클
- `task_id` **`TASK-20260928-115806`**

## 1. 무엇이 실측이고 무엇이 모의인가

| 구성 | 상태 |
|---|---|
| Isaac Sim (sim_task executor) | **실측.** v014 cabbage 씬, `--no-vision-station` 없이 기동 |
| Nav2 + `feeder_dock` (navigation executor) | **실측.** RViz2 포함, bag 기록 |
| Task Manager | **실측** |
| 관제 recorder + web | **실측** |
| **검사 (inspection executor)** | **모의.** 이 기기에 `docker` 가 설치되어 있지 않아 팀 `mock_executor` 로 대체함 |

## 2. 결과 — 8/12 단계 성공, CULL 관문에서 정지

| # | executor | operation | 결과 | 벽시계 | **sim** |
|---|---|---|---|---|---|
| 1 | sim_task | TRANSFER | SUCCEEDED | 265.6 s | 94.3 s |
| 2 | sim_task | PICK_HARVEST | SUCCEEDED | 173.5 s | 60.3 s |
| 3 | navigation | NAVIGATION (도킹 포함) | SUCCEEDED | 91.3 s | 31.2 s |
| 4 | sim_task | PLACE_INSPECT | SUCCEEDED | 66.1 s | 16.0 s |
| 5 | sim_task | CONVEY_TO_INSPECT | SUCCEEDED | 44.3 s | 15.8 s |
| 6 | sim_task | PREPARE_INSPECT | SUCCEEDED | 17.1 s | 6.0 s |
| 7 | sim_task | MOVE_TO_INSPECT | SUCCEEDED | 15.7 s | 5.0 s |
| 8 | inspection | INSPECT (모의) | SUCCEEDED | 0.2 s | 0.1 s |
| — | — | CULL 진입 | **ERROR** | — | — |

최종 `final_state = ERROR`, `terminal_status = FAILED`, `failure_reason = INSPECTION_DATA_TIMEOUT`.

**원인은 설계대로임.** `CULL` 은 `/sim_task/inspection_data_status` 가 `STORED` 여야 진행되고, 그 값은 `/inspection/detections_2d` 가 와야 생김. 모의 검사 executor 는 그 토픽을 발행하지 않음. 즉 **Docker 비전 노드 없이는 12단계를 끝까지 갈 수 없음**(코드 구조상 우회 불가).

## 3. 도킹 품질 (TurnTable 라이다 검출 면 기준 상대좌표계)

| 항목 | 값 | 기준 |
|---|---|---|
| `face_dist_m` | **0.938** | 목표 `standoff_m` 0.92 (+18 mm) |
| `yaw_err_deg` | **0.78** | 허용 3° |
| `lat_m` | **0.017** | 허용 0.06 (고피3 모의 최대 0.057 보다 좋음) |
| `retry` | **0** | BACKOFF 없이 1회 성공 |
| map 좌표 | x −2.235, y −2.680, yaw 88.42° | `stations.yaml` FEEDER_DOCK (−2.19, −2.73, 90°) |

`run_id` 가 Task Manager 명령 ID(`…-CMD-003`)와 같아 `dock_attempt` 행이 해당 `step` 에 정확히 붙음.

## 4. 팔레트(트레이) 추적 — 6회 전이 기록됨

`PALLET_001`: RACK_L1 → CARRY → INSPECT_STATION → INSPECT_STOP → INSPECT_WORK_POS → INSPECT_CAMERA_POSE → INSPECT

`pallet_locations` 발행 없이 명령의 `destination`·`reached_station` 대체 기록만으로 동작함.

## 5. 환경 지표

| 항목 | 값 |
|---|---|
| `/clock` | 4.8 Hz · **실시간 배율 0.37** |
| 라이다 | 1.1 Hz(벽시계) · **41,265 점/스캔** (fullScan 정상) · 시뮬 환산 2.97 Hz (기준 2.5 이상) |
| 자기 반사 | 0 / 41,265 점 (경량 카터. 26차 기록과 동일) |
| `nav2_link_check` 마지막 줄 | `RESULT FAIL` — 벽시계 기준 판정이라 이 기기에서는 그대로 믿지 않음(26차 규칙) |

## 6. 파일

| 파일 | 내용 |
|---|---|
| `state_trace.txt` | 공정 단계·map 좌표 전이 14줄 |
| `isaac_<단계>.png` (10장) | Isaac 창 1440×900 을 담은 1920×1080 화면. `ready`/`TRANSFER`/`PICK_HARVEST`/`NAVIGATION`/`PLACE_INSPECT`/`CONVEY_TO_INSPECT`/`PREPARE_INSPECT`/`MOVE_TO_INSPECT`/`CULL`/`ERROR` |
| `rviz_<단계>.png` (10장) | RViz2 창 1680×893 을 담은 1920×1080 화면. 같은 단계들 |
| `xvfb_99.log`, `xvfb_98.log` | 가상 디스플레이 로그 (xkb 경고는 무해) |
| `start_time.txt` | 시작 UTC 시각 |

창별 독립 가상 디스플레이로 캡처했음(`:99` Isaac, `:98` RViz2). 모든 스냅샷의 고유색이 17,000 이상이라 빈 화면이 아님을 확인했음.

**영상 녹화는 하지 않았음.** 이번은 절차를 처음 통과시키는 판이라 최소 스냅샷만 남겼음(ADR_basic §5-8). 발표 소재용 녹화는 12단계가 통과한 뒤에 함.

## 7. 로그 (다른 폴더)

| 경로 | 내용 |
|---|---|
| `smart_farm_navigation/results/log/isaac_20260928_1152.txt` | Isaac 전체 로그 |
| `smart_farm_navigation/results/log/nav2_20260928_1152.txt` | Nav2 · `feeder_dock` 로그 |
| `smart_farm_navigation/results/log/navnode_20260928_1152.txt` | 주행 노드 |
| `smart_farm_navigation/results/log/link_20260928_1152.txt` | 예비 점검 |
| `smart_farm_monitor/results/log/taskmgr_20260928_1152.txt` | Task Manager |
| `smart_farm_monitor/results/log/monitor_20260928_1152.txt` | 관제 recorder + web |
| `smart_farm_monitor/results/log/mockinspect_20260928_1152.txt` | 모의 검사 executor |
| `smart_farm_monitor/results/log/apistate_20260928_1152.json` | 종료 시점 `/api/state` 전체 |
| `~/.ros/smart_farm_navigation/bags/nav2_20260928_1154/` | bag 110 MB, 26 토픽 |
| `smart_farm_monitor/data/farm.db` | 관제 DB 원본 |
