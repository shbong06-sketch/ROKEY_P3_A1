# 실측 2차 · 녹화판 — 12단계 완주 + OmniGraph 탭 고정 (2026-09-28)

- 기기 **고피3** · `task_id` **`TASK-20260928-225734`** · 녹화 시작 `2026-09-28T22:57:34Z`
- 실행자: 에이전트 대행 (ADR_basic §5-8)
- 결과: **`COMPLETE / SUCCEEDED`, 12/12 단계** · 사이클 **sim 370.0 s** / 녹화 **1218.1 s**
- **버린 판 2개**: 옛 `2차_녹화_20260928_1705`, `3차_녹화_20260928_1745`. 두 판 모두 **OmniGraph 창이 떠 있는 팝업(floating) 상태로 뷰포트를 가린 채** 녹화되어 영상 소재로 쓸 수 없었음. 사용자 지시로 폴더째 삭제하고 이 판을 **2차로 다시 돌렸음**

## 1. 버린 판의 원인과 조치

| | 옛 2·3차 | 이번 2차 |
|---|---|---|
| OmniGraph 창 | `dock_in()` 호출 후 **성공 여부를 확인하지 않고** 성공으로 보고 → 실제로는 뷰포트 위에 떠 있는 팝업 | `window.docked` 를 **직접 읽어 확인**, 200회까지 재시도, 끝까지 도킹 안 되면 **창을 닫음** |
| 녹화 전 점검 | 없음 | 녹화 시작 전 `setup_check.png` 를 남기고 **사람이 눈으로 확인한 뒤** 진행 |
| POV 카메라 | 프림 기준 오프셋 미검증 | 후보 8장 렌더 비교(`camera_try_pov_20260928_2252`) 후 Fork B · Wrist A · Pallet A 채택 |

`setup_check.png` 에서 확인한 것: `OmniGraph Toolkit` 이 화면 **하단의 `Content`/`Console` 옆 탭**으로 붙어 있고 3D 뷰포트가 가려지지 않음. 녹화 중간 프레임(+800 s CULL `Cam4_CullPickPlace`, +1150 s CONVEYOR_OUT `Cam6_Outfeed`)도 뽑아 가림 없음을 재확인했음.

## 2. 결과 — 12/12 SUCCEEDED

| # | cycle_state | operation | sim (s) |
|---|---|---|---|
| 1 | TRANSFER | TRANSFER | 95.10 |
| 2 | PICK_HARVEST | PICK_HARVEST | 59.50 |
| 3 | NAVIGATION | NAVIGATION | 30.95 |
| 4 | PLACE_INSPECT | PLACE_INSPECT | 16.05 |
| 5 | CONVEY_TO_INSPECT | CONVEY_TO_INSPECT | 15.70 |
| 6 | PREPARE_INSPECT | PREPARE_INSPECT | 5.95 |
| 7 | MOVE_TO_INSPECT | MOVE_TO_INSPECT | 5.00 |
| 8 | INSPECT | INSPECT | 0.50 |
| 9 | CULL | CULL | 98.55 |
| 10 | RECHECK | RECHECK | 0.05 |
| 11 | RELEASE_INSPECT | RELEASE_INSPECT | 6.05 |
| 12 | CONVEYOR_OUT | CONVEYOR_OUT | 36.35 |

- 사이클 sim **370.0 s** — 실측 1차 370.5 s, 1차 재시도 369.7 s 와 **0.2 % 이내** 재현
- 병목: **CULL 98.55 s (26.6 %)**, **TRANSFER 95.10 s (25.7 %)** — 둘이 절반 이상
- 검사: 1차 `defect_slots ["SLOT_03","SLOT_04","SLOT_05"]` → CULL 배출 → `RECHECK` `defect_slots []`. 검출 레코드 9건, 팔레트 논리 위치 이동 10회
- 도킹: `face_dist_m` 0.946 / `yaw_err_deg` 1.22 / `lat_m` **−0.049** / `retry` **0**. 최종 map 좌표 x **−2.233** y **−2.754** (월드좌표계). 누적 **6/6 성공, 평균 재시도 0**
- `late_result` 0 — 늦게 도착한 결과 없음

## 3. 미디어 (모두 **10배속본**. 1배속 원본은 ADR 규칙대로 삭제했음)

| 파일 | 해상도 | 길이(10배속) | 원본 길이 | 크기 | 내용 |
|---|---|---|---|---|---|
| `isaac.mp4` | 1920×1080 | 121.9 s | 1218.1 s | 6.5 MB | Isaac 창 전체. 하단에 **OmniGraph Toolkit 탭** 노출(툴 사용 인증용) |
| `isaac_viewport.mp4` | 1278×670 | 121.9 s | 1218.1 s | 5.8 MB | 3D 뷰포트만 크롭. **영상 소재용** |
| `terminal.mp4` | 1920×1080 | 121.9 s | 1218.1 s | 22 MB | tmux 4분할 (Task Manager / Isaac / Nav2 / 비전) |
| `rviz_navigation.mp4` | 1920×1080 | 9.6 s | 95.2 s | 556 KB | **`NAVIGATION` 단계만** |
| `clips/NN_<단계>_{isaac,viewport,terminal}.mp4` | 원본과 동일 | 각 단계 | — | — | 단계별 분할 **36개**(무손실 분할 후 10배속) |
| `isaac_<단계>.png` | 1920×1080 | — | — | — | **공정 단계마다 1장** (12장 + `isaac_final.png`) |
| `setup_check.png` | 1920×1080 | — | — | — | 녹화 직전 화면 점검용 |
| `stage_offsets.txt` | — | — | — | — | 녹화 기준 단계 경계 초(편집용, 1배속 기준) |

### 카메라 전환 (7회, 전부 의도대로 동작)

`Cam1_Harvest`(TRANSFER·PICK_HARVEST) → `Cam_Carter_Rear`/`Cam2_Nav2Place`(NAVIGATION·PLACE_INSPECT) → `Cam3_FeederEntry`(CONVEY_TO_INSPECT) → `Cam7_FeederClose`(메인라인 합류 후 0.5 s) → `Cam4_CullPickPlace`(INSPECT·CULL) → `Cam6_Outfeed`(CONVEYOR_OUT)

### 단계별 클립 길이 (10배속 기준)

TRANSFER 29 s · PICK_HARVEST 19 s · NAVIGATION 9 s · PLACE_INSPECT 7 s · CONVEY_TO_INSPECT 5 s · PREPARE_INSPECT 2 s · MOVE_TO_INSPECT 1 s · INSPECT <1 s · **CULL 32 s** · RELEASE_INSPECT 2 s · CONVEYOR_OUT 11 s · COMPLETE 1 s

## 4. 남은 한계

1. **`INSPECT`·`RECHECK` 클립이 1초 미만** — 검사 자체가 sim 0.5 s / 0.05 s 라 10배속에서 사실상 한 프레임임. 영상에서는 앞 단계(`MOVE_TO_INSPECT`)와 뒤(`CULL`) 클립에 얹어 쓰는 편이 나음.
2. **`NAVIGATION` 카메라 전환은 여전히 관제 쪽에서 못 지정함** — 주행·검사 단계는 Isaac `take_command()` 를 거치지 않아 `view_director` 가 직전 카메라를 유지함. 이번 판은 POV(`Cam_Carter_Rear`)가 그 구간을 커버했음. 근본 해결은 팀 파일(`sim_task_node`) 수정이 필요해 승인 대기.
3. `lat_m` −0.049 는 허용 0.06 에 여유 0.011 m. 6표본 중 최대치임.

## 5. 관련 로그

`smart_farm_navigation/results/log/*_20260928_2255.txt`(Isaac·Nav2·주행), `smart_farm_monitor/results/log/*_20260928_2255.*`(Task Manager·관제·비전), DB `smart_farm_monitor/data/farm.db` 의 `task_id='TASK-20260928-225734'`.
