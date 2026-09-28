# 양배추 올인원 후속 시험·수정 정리 (2026-09-28)

브랜치 `feature/cabbage-place-fix` · Isaac Sim 5.1 · Windows 11 + RTX 3060 (+ WSL2 Ubuntu 24.04 / ROS 2 Jazzy)
앞선 결과: `docs/cabbage_grasp_speed_experiments_2026-09-27.md`. 모든 수치는 시뮬레이션 측정이며, 조건당 1~2회라 경향과 실패 모드를 보는 용도다.

## 1. 한눈에

| 주제 | 결론 |
|---|---|
| 그리퍼 기본값 | **16 N·m** (0.3·0.5·0.7 kg x 포기 마찰 0.8 / 실물 쪽 0.4 전 조건 6/6). 12 N·m 는 마찰 0.4 에서 0.5·0.7 kg 4/6 |
| 재검사 후 1회 재시도 | 동작 확인. 단 원인이 같은 실패(힘 부족)는 재시도도 실패 — 8 N·m·0.5 kg 재시도 0/5, 시간 231 → 396 s |
| 비전 치우침 원인 | 박스 중심을 되돌리는 높이 가정 0.090 m(포기 윗부분) ≠ 포기 중심 0.080 m. 고친 뒤 y −4.2 → −0.3 mm |
| 비전 전용 모드 | 포기마다 다시 찍기 + 문턱 3 mm: 뒤쪽 포기 집기 오차 5.8 → 3.5 mm. 전체 흐름(비전 전용 + `-Fast`) 판정 6/6·선별 3/3 |
| 실물 밀도 포기(0.14 kg, 같은 크기) | 6/6 |
| 무거운 포기(1.5 kg) 내려놓기 | 팀 `robot_motion` 구간 최소 시간 0.5 → 1.0 s (실행 중 값만, `CABBAGE_MIN_MOVE_S`): 포기 밀림 15.2 → 1.9 mm, 내려놓기 +9.5 s |
| 1.0 m/s 반복(10회) | 실패 3회(포기 낙하 2회), 성공 7회도 21~92 s. **0.6 m/s + 도킹 조정(`-Fast`) 유지** |
| DES | 추천 운반 설정이면 로봇 1대 25.6 → 31.9 트레이/h(+25 %), 스테이션 1곳은 불량률 10 % 에서 로봇 3대에 가동률 92 % → 로봇 3대부터 스테이션 2곳 검토 |
| 웹 DB | PostgreSQL 16 에 실제 적재 성공 (표 9개·뷰·JSON 조회) |
| 전체 흐름에서만 손가락이 덜 닫히는 원인 | 집는 위치는 같음(포기–TCP 35 mm) → 위치 원인 배제. 16 N·m 에서는 증상 없음. 근본 원인(포기 자세 추정)은 미확인 |
| 카메라 검은 화면 | render product 재생성 시도 → 해결 안 됨(되돌림). 재촬영·무효 회차 재실행으로 대응 중 |
| 0.6 m/s 에서 트레이가 포크 위로 미끄러진 1회 | 이후 재현 없음(0.6 m/s 10회 성공). 0.25 s 트레이 추적 기록을 추가해 다음에 잡을 수 있게 함 |

## 2. 팀원 올인원 최종본(`feature/task-managed-integration`) 검사·수정·통합

- **검사:** 구조(Task Manager 가 단계별 명령)는 좋고 순수 파이썬 시험은 통과. 위험: 비전 1장 판정(흰/검은 프레임 1장이면 사이클 ERROR), 그리퍼 8 N·m 고정, CULL 180 s 고정(6포기면 약 184 s), 재검사 불량 시 재시도 없음, `SortBin` 이름 고정, Nav2 준비 전 READY.
- **수정(패치, 팀원 브랜치에는 커밋하지 않음):** 비전 3장 다수결 + 이상 프레임 거르기, 그리퍼 16 N·m(환경변수), CULL 한도 = max(180, 30 + 45 x 대상 수), `SortBox` 대체, Task Manager CULL 1회 재시도, 자동 모드 포기 놓침 계속, `--no-vision-station` 10 s 자동 배출, Navigation READY 를 액션 서버 준비 뒤로. 시험 작업관리자 37·비전 36·Isaac 28 통과.
- **통합본:** 팀원 최종본 + 제 브랜치 + 수정. 복사로 들어온 파일은 원래 조상 버전으로 3방향 병합. `standalone_app.py --station-mode {managed, auto}` 로 두 흐름을 모두 살림(기본 managed, 올인원은 auto).
- **시뮬레이션 검증 (이 PC: Windows Isaac + WSL ROS, 중계기에 `/rgb`·검사 토픽 추가, 비전 CPU):**
  - 명령 모드 전체 사이클 **COMPLETE** — TRANSFER → PICK_HARVEST → NAVIGATION → PLACE_INSPECT → CONVEY_TO_INSPECT → PREPARE_INSPECT → MOVE_TO_INSPECT → INSPECT(불량 3칸 정답) → CULL(3/3, 색상별 통) → RECHECK → RELEASE_INSPECT → CONVEYOR_OUT. 시뮬레이션 394 s.
  - 자동 모드 전체 흐름(`-Fast`, 16 N·m): 판정 6/6, 선별 3/3, 재검사·배출 정상.
  - 통합본 스테이션 단독: 자동 모드 0.5 kg 6/6(색상별 통), 비전 전용 판정 6/6·선별 4/4.
- 결과물 위치: `D:\smartfarm-sim\out\patches_2026-09-28\` (패치 2개 + 설명서 `README_수정설명.md`).

## 3. 새 옵션 요약

| 옵션 | 기본 | 설명 |
|---|---|---|
| `SMARTFARM_GRIP_FORCE` | 16 | RG2 finger_joint maxForce (N·m) |
| `SMARTFARM_CULL_RETRY` | 1 | 재검사에서 불량이 남으면 다시 집는 횟수 |
| `SMARTFARM_AIM_ABOVE_TRAY` | 0.080 | 비전 박스 중심을 되돌리는 높이 (m, 트레이 원점 기준) |
| `SMARTFARM_VISION_REFRESH` | 1 | 비전 전용 모드: 두 번째 포기부터 집기 전에 다시 찍기 |
| `SMARTFARM_VISION_DEADBAND` | 0.003 | 비전 전용 모드 보정 문턱 (m) |
| `run_allinone.ps1 -Fast` | – | 0.6 m/s + feeder_dock 후진 0.15 · 미세 접근 0.08 m/s · 대기 1 s |
| `CABBAGE_MIN_MOVE_S` | – | 팀 robot_motion 구간 최소 보간 시간 (실행 중 값만 바꿈) |
| `run_conditions.ps1` | – | CSV 조건표로 올인원 무인 실행 |
| `tools/experiments/` | – | 스테이션·속도 실행기, 분석, DES, 웹DB, 그림 |

## 4. 하지 못한 것

- 실물 양배추 사진 검증 (사진 없음)
- 카메라 검은 화면의 근본 원인 (PC 재부팅 후 재확인 권고)
- 영상 파일을 외부 저장소(버킷)에 올려 웹 DB 에 주소 넣기 (대상 저장소 미정, 노션에는 첨부)
- 보고서 페이지 팀 공유 (공유 설정은 사용자가 직접)
- 팀 기본값(Nav2 속도·feeder_dock·robot_motion) 변경 — 팀 파일이라 옵션으로만 제공, 팀 합의 필요
