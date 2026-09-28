# 실측 1차 · 시도 2 — **DEMO_HARVEST_01 12단계 완주** (2026-09-28)

- 기기 **고피3** · `task_id` **`TASK-20260928-130920`** · 시작 `2026-09-28T12:58:44Z`
- 실행자: 에이전트 대행 (ADR_basic §5-8)
- 결과: **`final_state = COMPLETE`, `terminal_status = SUCCEEDED`**
- **모의 없음.** 검사까지 전부 실제 노드임

## 1. 구성

| 구성 | 상태 |
|---|---|
| Isaac Sim (`:99`) | v014 cabbage, `fullScan=True` |
| Nav2 + RViz2 (`:98`) | `Managed nodes are active`, bag 기록 |
| 주행 노드 + `feeder_dock` | READY |
| 검사 노드 (Docker, GPU) | `torch 2.4.1+cu124`, `NVIDIA L4`, `/models/best.pt`, `cuda:0` |
| 관제 (recorder + web) | 기동 |
| Task Manager | READY |
| 시작 | **웹 대시보드 시작 버튼 경로** → ACCEPTED |

시작 전에 `ros2 node list` 로 **중복 노드 0** 을 확인했음(시도 1 의 실패 원인을 막는 관문).

## 2. 결과 — 12/12 단계 SUCCEEDED

사이클 전체 **sim 370.0 s / 벽시계 1139.3 s** (실시간 배율 약 0.32).

| # | executor | operation | 벽시계 | **sim** | 비중(sim) |
|---|---|---|---|---|---|
| 1 | sim_task | TRANSFER | 276.6 | **95.0** | 26 % |
| 2 | sim_task | PICK_HARVEST | 177.8 | 59.5 | 16 % |
| 3 | navigation | NAVIGATION (도킹) | 93.3 | 31.1 | 8 % |
| 4 | sim_task | PLACE_INSPECT | 69.2 | 16.0 | 4 % |
| 5 | sim_task | CONVEY_TO_INSPECT | 46.3 | 15.8 | 4 % |
| 6 | sim_task | PREPARE_INSPECT | 17.8 | 6.0 | 2 % |
| 7 | sim_task | MOVE_TO_INSPECT | 16.4 | 5.0 | 1 % |
| 8 | inspection | INSPECT | 1.5 | 0.5 | 0 % |
| **9** | sim_task | **CULL** | 318.5 | **98.4** | **27 %** |
| 10 | inspection | RECHECK | 0.2 | 0.1 | 0 % |
| 11 | sim_task | RELEASE_INSPECT | 18.0 | 6.2 | 2 % |
| 12 | sim_task | CONVEYOR_OUT | 102.9 | 36.3 | 10 % |

**병목은 CULL(27 %)과 TRANSFER(26 %)** 로 둘이 전체의 절반을 넘음. 사이클 시간 최적화의 1순위 근거임.

## 3. 검사 — 1차 검출부터 RECHECK 까지

1차(`pass_no=1`) 6건, 좌표는 원본 RGB 640×640 pixel(u=가로, v=세로).

| 슬롯 | 클래스 | conf | 중심 (u, v) | 판정 |
|---|---|---|---|---|
| SLOT_01 | `lettuce_dark_green` | 0.792 | (193.3, 282.2) | 정상 |
| SLOT_02 | `lettuce_dark_green` | 0.788 | (320.8, 286.7) | 정상 |
| SLOT_03 | `lettuce_yellow` | 0.873 | (453.9, 286.8) | **불량** |
| SLOT_04 | `lettuce_yellow` | 0.836 | (166.9, 375.5) | **불량** |
| SLOT_05 | `lettuce_brown` | 0.866 | (318.5, 383.1) | **불량** |
| SLOT_06 | `lettuce_dark_green` | 0.773 | (471.9, 388.9) | 정상 |

- `INSPECT` 판정: `defect_slots = ["SLOT_03","SLOT_04","SLOT_05"]`, `unknown_slots = []`
- `CULL` 로 3개 배출 (sim 98.4 s)
- 2차(`pass_no=2`) 검출 3건 → **`RECHECK` 판정 `defect_slots = []`** — 불량이 남지 않았음이 확인됨

## 4. 도킹

| 항목 | 값 |
|---|---|
| `face_dist_m` | 0.927 (목표 `standoff_m` 0.92) |
| `yaw_err_deg` | 0.36 |
| `lat_m` | 0.022 (허용 0.06) |
| `retry` | 0 |

누적 3회(파일럿 1 + 시도 1 + 시도 2) **전부 성공, 평균 재시도 0.0, 최대 횡 오차 0.022 m.**

## 5. 관제 계층

`step` 12행, `detection` 9행, `inspection_verdict` 2행, `pallet_move` **10행**, `dock_attempt` 1행, `live` 갱신. KPI: `cycle_total 3 / succeeded 1`, `cycle_avg_sim 369.95 s`, `dock 3/3`, `defect_detections 3`.

## 6. 미디어와 한계 — **Isaac 영상은 영상 소재로 쓸 수 없음**

| 파일 | 내용 |
|---|---|
| `isaac.mp4` | `:99` Isaac 창. 1920×1080, 10 fps, **2491.3 s**, 262 MB |
| `rviz.mp4` | `:98` RViz2 창. 1920×1080, 10 fps, **2491.3 s**, 27 MB |
| `isaac_<단계>.png`, `rviz_<단계>.png` | 단계별 창별 스냅샷 |
| `state_trace.txt` | 공정 단계·map 좌표 전이 |

**문제(2026-09-28 사용자 지적)**: Isaac 뷰포트가 씬 기본 `/OmniverseKit_Persp` 그대로라 **검사실만 찍혔음.** 랙·주행·턴테이블·컨베이어·퇴출구가 화면에 없음.

원인은 `standalone_app.py:1453` 의 `select_view_camera()`(2026-09-26 에 내가 넣은 기능)를 쓰지 않은 것임. 그 함수의 주석에 증상까지 적혀 있었는데 `SMARTFARM_VIEW_CAMERA` 를 설정하지 않고 띄웠음. guidance2_26차 에도 같은 경고가 있었음.

**판정 데이터(12단계 완주, 소요시간, 검출)는 영향 없음.** 영상만 소재로 못 씀. 공정 구역별 풀샷 녹화는 카메라 전환을 붙인 뒤 별도 회차로 진행함.

## 7. 관련 로그

`smart_farm_navigation/results/log/*_20260928_1258.txt` (Isaac·Nav2·주행), `smart_farm_monitor/results/log/*_20260928_1258.*` (Task Manager·관제·비전·`/api/state`), bag `~/.ros/smart_farm_navigation/bags/nav2_20260928_*`.
