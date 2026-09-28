# 실측 1차 · 녹화판 — 12단계 완주 + 공정별 카메라 전환 (2026-09-28)

- 기기 **고피3** · `task_id` **`TASK-20260928-142331`** · 녹화 시작 `2026-09-28T14:20:42Z`
- 실행자: 에이전트 대행 (ADR_basic §5-8)
- 결과: **`COMPLETE / SUCCEEDED`, 12/12 단계**. 조건은 실측 1차와 같고 **목적이 영상 소재**라 회차를 올리지 않고 `녹화` 를 붙임(ADR_web-monitor §3.5)

## 1. 이번 판에서 고친 것

| 항목 | 직전 판 | 이번 판 |
|---|---|---|
| Isaac 카메라 | 씬 기본 `/OmniverseKit_Persp` → **검사실만 찍힘** | **공정별 자동 전환**(`SMARTFARM_VIEW_FOLLOW=1`) |
| 컨베이어 구간 | `Cam2_Nav2Place` (컨베이어가 화면 귀퉁이) | **`Cam5_Pusher`** (컨베이어 베드 정면) |
| 마우스 포인터 | 화면 한가운데 찍힘 | `-draw_mouse 0` + `xdotool` 로 구석 이동 |
| 녹화 범위 | 기동~정리 전부 (2491 s, 절반이 정지 화면) | **사이클 구간만 (1166 s)** |
| RViz2 | 전 구간 | **`NAVIGATION` 단계만 (98.5 s)** |
| 터미널 창 | 없음 | **`:97` 에 `xterm`+`tmux` 4분할, 전 구간 녹화** |
| 클립 | 통짜 1개 | **공정 단계별 무손실 분할 24개** |

카메라 매핑은 이름으로 고르지 않고 `tools/preview_cameras.py` 로 **카메라 9개를 한 장씩 실제 렌더해서** 정했음(`results/log_media/camera_preview_20260928_1406/`). 그 과정에서 **`Cam0_Perspective` 는 벽과 천장만 보인다**는 것도 확인했음.

## 2. 결과 — 12/12 SUCCEEDED

사이클 **sim 370.5 s / 벽시계 1158.2 s**. 직전 판(sim 370.0 s)과 **0.1 % 차이**로 재현됨.

| # | executor | operation | 벽시계 | sim |
|---|---|---|---|---|
| 1 | sim_task | TRANSFER | 281.9 | 95.0 |
| 2 | sim_task | PICK_HARVEST | 182.2 | 60.4 |
| 3 | navigation | NAVIGATION | 96.2 | 31.1 |
| 4 | sim_task | PLACE_INSPECT | 71.5 | 16.0 |
| 5 | sim_task | CONVEY_TO_INSPECT | 47.4 | 15.7 |
| 6 | sim_task | PREPARE_INSPECT | 18.1 | 5.9 |
| 7 | sim_task | MOVE_TO_INSPECT | 16.7 | 5.0 |
| 8 | inspection | INSPECT | 1.4 | 0.5 |
| 9 | sim_task | CULL | 318.8 | 98.4 |
| 10 | inspection | RECHECK | 0.4 | 0.1 |
| 11 | sim_task | RELEASE_INSPECT | 18.7 | 6.1 |
| 12 | sim_task | CONVEYOR_OUT | 104.5 | 36.3 |

검사: 1차 검출 6건 → `defect_slots ["SLOT_03","SLOT_04","SLOT_05"]` → CULL 배출 → 2차 검출 3건 → **`RECHECK` `defect_slots []`**. 팔레트 논리 위치 이동 10회.

도킹(4번째 표본): `face_dist_m` 0.947 / `yaw_err_deg` 1.01 / `lat_m` **−0.013** / `retry` 0. 누적 **4/4 성공, 재시도 0**.

## 3. 미디어

| 파일 | 해상도 | 길이 | 크기 | 내용 |
|---|---|---|---|---|
| `isaac.mp4` | 1920×1080 | 1182 s | 121 MB | Isaac 창 전체(GUI 포함), 공정별 카메라 전환 |
| **`isaac_viewport.mp4`** | **936×518** | 1182 s | — | **3D 뷰포트만 크롭한 사본. 영상 소재용** |
| `terminal.mp4` | 1920×1080 | 1170 s | 127 MB | tmux 4분할 (Task Manager / Isaac / Nav2 / 비전) |
| `rviz_navigation.mp4` | 1920×1080 | **98.5 s** | 2.9 MB | `NAVIGATION` 단계만 |
| `clips/NN_<단계>_{isaac,terminal}.mp4` | 원본과 동일 | 각 단계 길이 | — | **무손실 분할**(`-c copy`), 24개 |
| `isaac_<단계>.png`, `term_<단계>.png` | 1920×1080 | — | — | 단계마다 1장씩 |
| `rviz_NAVIGATION_{start,end}.png` | 1920×1080 | — | — | 주행 구간 시작·끝 |
| `stage_offsets.txt` | — | — | — | 녹화 기준 단계 경계 초 (분할·편집용) |

### 단계별 클립 길이

TRANSFER 261 s · PICK_HARVEST 177 s · NAVIGATION 98 s · PLACE_INSPECT 74 s · CONVEY_TO_INSPECT 44 s · PREPARE_INSPECT 19 s · MOVE_TO_INSPECT 20 s · **CULL 315 s** · RECHECK 7 s · RELEASE_INSPECT 14 s · CONVEYOR_OUT 103 s · COMPLETE 7 s

## 4. 남은 한계 (다음 판에서 고칠 것)

1. **`NAVIGATION`·`INSPECT`·`RECHECK` 구간은 Isaac 카메라가 안 바뀜.** 그 세 단계는 주행 노드·검사 노드가 처리해 Isaac 의 `take_command()` 를 거치지 않기 때문임. 직전 단계 카메라가 유지되며, `NAVIGATION` 은 `Cam1_Harvest`(랙 쪽)에 머물러 **카터가 통로를 빠져나가면 화면 밖으로 사라짐**. 해결하려면 관제가 `/cycle/status` 로 본 단계를 카메라 지정 토픽으로 발행하고 `sim_task_node` 가 받아야 하는데, 팀 파일 수정이라 승인이 필요함.
2. **Isaac 창이 1440×900 이라 1920×1080 화면에 검은 여백이 생김.** 기동 인자 `--/app/window/width=1920 --/app/window/height=1080` 로 키우면 뷰포트가 커지고 여백이 사라짐.
3. **`ffmpeg` PID 를 잘못 저장해 `isaac.mp4` 마감이 한 번 실패했음.** 저장한 것이 ffmpeg 이 아니라 그것을 띄운 bash 래퍼였음. 실제 PID 를 찾아 `SIGINT` 로 마감해 복구했으나, 다음부터는 실행 직후 `pgrep -f` 로 실제 PID 를 잡음.

## 5. 관련 로그

`smart_farm_navigation/results/log/*_20260928_1420.txt`(Isaac·Nav2·주행), `smart_farm_monitor/results/log/*_20260928_1420.*`(Task Manager·관제·비전), bag `~/.ros/smart_farm_navigation/bags/nav2_20260928_*`.
