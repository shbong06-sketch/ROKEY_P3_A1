# 3차 녹화 — 시점 연출 완성본 (2026-09-28)

- 기기 **고피3** · `task_id` **`TASK-20260928-174801`** · 녹화 시작 `2026-09-28T17:48:01Z`
- 결과: **`COMPLETE / SUCCEEDED`, 12/12 단계** · sim **369.7 s** / 벽시계 1181.7 s
- 실행자: 에이전트 대행 (ADR_basic §5-8)

## 1. 시점 전환 — 설계대로 한 번씩만

| 순서 | 카메라 | 전환 계기 |
|---|---|---|
| 1 | `Cam1_Harvest` | 시작. 랙 4단 + 카터 + 포크팔 |
| 2 | `Cam2_Nav2Place` | **카터 y ≤ −1.20** (통로 남단 통과) |
| 3 | `Cam3_FeederEntry` | `CONVEY_TO_INSPECT` 진입. 턴테이블 벨트 ~ 피더 ~ 비전룸 입구 |
| 4 | **`Cam7_FeederClose`** | **팔레트 y ≤ −6.20 (본선 합류) + 0.5 초** |
| 5 | `Cam4_CullPickPlace` | `PREPARE_INSPECT`. `INSPECT`·`CULL`·`RECHECK` 동안 **유지** |
| 6 | `Cam5_Pusher` | `RELEASE_INSPECT` |
| 7 | **`Cam6_Outfeed`** | **팔레트 x > 0.60** (비전룸 이탈). 맵 밖까지 추적 |

2차에서 반복되던 `(명령없음) → Cam2_Nav2Place` 되돌아감이 **사라졌음**.

## 2. 2차에서 고친 결함 2건

**① 팔레트 위치를 껍데기 Xform 에서 읽고 있었음**

`/World/SmartFarm/Placed/Pallet_01` 은 껍데기이고 실제로 움직이는 것은 그 아래 강체 `Cube_011_001` 임. 껍데기는 팔레트가 벨트로 가도 랙 좌표에 머물러 **`y ≤ −6.20` 이 영원히 거짓**이었고, 그래서 `Cam7`·`Cam6`·`PalletPOV` 가 모두 동작하지 않았음.

→ 경로를 `.../Pallet_01/Cube_011_001/Cube_011_001` 로 고쳤음. 덧붙여 **prim 을 못 찾으면 로그로 한 번 알리도록** 했음(조건이 영원히 거짓이 되는 실패를 빨리 알아채기 위함).

**② 명령 공백 구간에 시점이 튐**

Isaac 에 활성 명령이 없는 순간마다 위치 규칙이 발동해 비전룸 화면이 주행 롱샷으로 되돌아갔음.

→ **명령 공백 구간에는 현재 화면을 유지**하고, 직전 공정이 `PICK_HARVEST` 일 때(주행 단계)만 위치 규칙을 쓰도록 고쳤음.

## 3. 공정 결과

| # | executor | operation | 벽시계 | sim |
|---|---|---|---|---|
| 1 | sim_task | TRANSFER | 287.2 | 95.0 |
| 2 | sim_task | PICK_HARVEST | 186.2 | 60.4 |
| 3 | navigation | NAVIGATION | 97.2 | 30.3 |
| 4 | sim_task | PLACE_INSPECT | 71.8 | 16.1 |
| 5 | sim_task | CONVEY_TO_INSPECT | 48.5 | 15.8 |
| 6 | sim_task | PREPARE_INSPECT | 18.6 | 5.9 |
| 7 | sim_task | MOVE_TO_INSPECT | 16.9 | 5.0 |
| 8 | inspection | INSPECT | 1.3 | 0.4 |
| 9 | sim_task | **CULL** | 326.3 | **98.3** |
| 10 | inspection | RECHECK | 0.3 | 0.1 |
| 11 | sim_task | RELEASE_INSPECT | 19.2 | 6.0 |
| 12 | sim_task | CONVEYOR_OUT | 107.4 | 36.3 |

도킹: `face_dist_m` 0.947 / `yaw_err_deg` 0.53 / `lat_m` −0.035 / `retry` 0.
검사: 1차 검출 6건 → 불량 `SLOT_03`·`SLOT_04`·`SLOT_05` → CULL 배출 → 2차 검출 3건, **RECHECK 불량 0**.

## 4. 미디어 (전부 10배속, 원본 1배속은 삭제)

| 파일 | 해상도 | 길이 | 크기 | 내용 |
|---|---|---|---|---|
| `isaac.mp4` | 1920×1080 | 119.0 s | 4.4 MB | Isaac 창 전체. **아래 칸에 `OmniGraph Toolkit`** — Isaac 작업 인증용 |
| **`isaac_viewport.mp4`** | **1278×670** | 119.0 s | 3.9 MB | **3D 뷰포트만.** 좌표는 Kit 이 보고한 값(x 48, y 26)으로 잘랐음 |
| `terminal.mp4` | 1920×1080 | 120.1 s | 22.3 MB | tmux 4분할 (Task Manager / Isaac / Nav2 / 비전) |
| `rviz_navigation.mp4` | 1920×1080 | **9.7 s** | 0.6 MB | `NAVIGATION` 단계만 |
| `clips/` **33개** | 원본과 동일 | 단계별 | — | `NN_<단계>_{isaac,viewport,terminal}.mp4` |
| `isaac_<단계>.png`, `term_<단계>.png` | 1920×1080 | — | — | 단계마다 1장 |
| `stage_offsets.txt` | — | — | — | 녹화 기준 단계 경계 초 (편집용) |

화면 구성: Isaac 창 **1900×1022**(검은 여백 제거), 뷰포트 **1279×671**, 마우스 포인터 없음(`-draw_mouse 0` + 구석 이동).

## 5. 남은 것

- `ForkPOV`·`PalletPOV` 카메라는 **만들어 두었으나 이번 판에서 녹화하지 않았음.** 시점뷰 녹화는 별도 회차로 진행함
- 비전룸 그리퍼 시점(`RSD455` 컬러/깊이)은 씬에 이미 있음
