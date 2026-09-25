# CLAUDE.md — 이 저장소에서 일하는 에이전트의 상시 규칙

이 파일은 세션마다 자동으로 읽힘. **규칙의 원본은 ADR 이며 이 파일은 진입점임.**

---

## 0. 작업 시작 시 반드시 먼저 할 것

1. `docs/ADR/ADR_basic.md` 를 읽음 (역할·기기·쓰기 권한·답변 규칙).
2. Nav2·도킹·주행과 조금이라도 관련되면 `docs/ADR/ADR_navigation2.md` 를 이어서 읽음.
   - 브랜치 `feature/navigation2`, `feature/Inspection-Place-nav2` 에서 적용되며, 브랜치 이름과 무관하게 **Nav2 가 얽힌 모든 작업**에 적용함.
3. 충돌 시 **ADR_basic 이 ADR_navigation2 보다 우선**함.
4. **ADR 을 임의로 수정하지 않음.** 사용자가 직접 지시했거나, 수정 없이는 업무가 불가능함을 보고하고 승인받은 경우만 수정함.

> 2026-09-24 사용자 지적: ADR 항목을 어겼을 뿐 아니라 **ADR 파일을 아예 읽지 않는 상태**가 되었음.
> 구체적 위반: Nav2 감속 중 `feeder_dock` 시작(2.2 위반), `--once` 사용(2.3 위반), 모의 결과를 실측처럼 보고(2.7 위반).
> 가이던스 문서보다 ADR 이 상위임.

### ADR 이 최신이 아닐 때

ADR 은 작업보다 뒤처짐. ADR 과 **최근 대화에서 사용자와 합의한 값**이 충돌하면 **최근 합의가 이김**. 단, 그때는 낡은 ADR 항목을 답변의 "미해결" 항에 열거해 **갱신을 사용자에게 제안**해야 함. 직접 고치지 않음.

현재 확인된 낡은 ADR 항목 (2026-09-25 기준):

**반영 완료** — 사용자 승인(2026-09-25)으로 `ADR_navigation2.md` 에 적용함.

| ADR 항목 | 옛 값 | 반영된 현행값 |
|---|---|---|
| nav2 2.2 | 도킹 목표 `standoff_m` 0.90 → (9/24) 0.85 | **0.90 으로 복귀** (실측 재환산, 아래 설명) |
| nav2 2.2 | TurnTable 앞면 y −3.60 | **USD 기준 −3.60 / 라이다 검출 면 −3.645 를 구분해 명시.** `standoff_m` 은 검출 면 기준임 |
| nav2 2.6 | `feeder_dock` standoff 0.90 → 0.85 | **0.90** |
| nav2 2.6 | `cloud_self_filter` 상자 x −0.85~0.6 | **−0.65~0.60** (`cloud_self_filter.py:30`) |
| nav2 2.6 | `standoff_m`/`box_x[0]` 짝 표기 | **(0.90) / (−0.65)** |

> **도킹 거리를 두 번 바꾼 이유 (되풀이 금지)**
> 팔 밑동 ~ place 대상 = **`standoff_m` + 0.06** 임. 근거는 실측 2건(`results/isaac_20260923_2032.txt:704` + `nav2_20260923_2033.txt:433`, `isaac_20260923_2125.txt:722` + `nav2_20260923_2126.txt:435`)에서 나온 "검출 면 ~ place 대상 0.26 m" 와 팔 밑동 위치(base_link 보다 0.20 m 뒤, `isaac_20260923_2032.txt:557`)임.
> 2026-09-23 에 쓰던 `0.11 + standoff_m` 은 면을 USD 값 y −3.60 으로 가정해 **0.05 m 틀렸음**.
> 또한 그때 기준으로 삼은 `BASE_TO_PALLET_X`(0.89~1.05)는 `robot_motion.check_base_pose` 가 **`start_pick` 에서만** 호출하므로(`robot_motion.py:883`) **feeder place 에는 적용되지 않음**(`start_place_at_pose`, `:981`). place 의 실제 관문은 역기구학과 관절 연속성(한계 20°)임.
> 실제로 성공이 확인된 자세는 랙 pick 의 **앞뒤 0.932 m** 하나뿐임(`:556`). 0.90 은 그것을 0.96 으로 재현하는 값이고, 그 자세를 그대로 쓰려면 0.87 임.

**미결 — 사용자 결정 대기 (ADR 을 고치지 않음)**

| ADR 항목 | ADR 표기 | 실제 |
|---|---|---|
| nav2 2.3 | `ros2 topic pub -t 3 -r 1` (bag 기록기가 같은 토픽을 구독하므로 `--once` 는 놓칠 수 있음) | guidance2_25차는 `--once --max-wait-time-secs 15` |
| nav2 2.6 | RPP **0.6 m/s** | `config/nav2_params.yaml:162` `desired_linear_vel: **0.3**` |
| basic §3-4 / nav2 2.4 | 팀 파일 중 손댄 곳은 `standalone_app.py` `open_scene()` fullScan 6줄**뿐** | `ceeafd5` 에서 main loop 의 `hold()` 예외 처리도 수정함 (`docs/standalone_app_changes.md`) |

---

## 1. 기기 구분 (고피 / 내피)

| 이름 | 호스트 | 역할 |
|---|---|---|
| 고피1 | `IsaacSim03` / 10.10.0.2 / DOMAIN 101 | Isaac Sim, 명령 발행, Task Manager |
| 고피2 | 10.10.0.1 / DOMAIN 102 | 위와 동일(번갈아 사용) |
| 내피 | `lwh19180` / 10.10.0.3 | Nav2 · RViz2 · `feeder_dock` · `navigation_node` · 회귀 시험. Isaac 실행 불가 |
| (추가) | `gc-isaacsim-lwh` (GCP VM) | 고피급. **홈이 `/home/rokey` 가 아니고 레포 경로가 `/home/rokey/ROKEY_p3_a1` (소문자 `p3_a1`)** |

- **경로 주의**: ADR·가이던스·기존 메모리는 모두 `/home/rokey/ROKEY_P3_A1` (대문자)로 적혀 있음. GCP 기기에서는 실제 경로가 소문자임. 절대경로를 옮겨 적기 전에 `hostname` 과 `pwd` 를 확인함. **가이던스에 적는 경로는 계속 대문자 ADR 경로를 씀** (고피1/고피2/내피가 그 경로임).
- `ROS_DOMAIN_ID` 를 고정값으로 박지 않음. 그때 쓰는 고피에 맞춰 내피 전 터미널과 비전 컨테이너에 같은 값을 적용함.
- 내피에서 `git pull` 을 에이전트가 실행하지 않음. push 거부 시 사용자에게 알리고 대기함.

---

## 2. 답변 규칙 (ADR_basic §5·§6)

- **한국어, 개조식 평서체(`~함`, `~임`, `~하였음`).** 과장·아부·감탄사 금지. 사실과 데이터만.
- 사용자는 프로그래밍 비전공자임. 원리와 동작 순서를 직관적으로 설명하되 ROS 2/Isaac Sim 의 함수명·인터페이스·기술 용어는 원문 그대로 표기함.
- **매 답변에 다음 5개를 포함함**: (1) 확인한 로그·파일 (2) 원인 (3) 조치와 커밋 해시 (4) 고피/내피에서 사용자가 할 일 (5) 미해결·가정.
- 좌표·수치는 **x/y/z 중 무엇인지, 월드좌표계인지 상대좌표계인지**를 항상 함께 적음.
- 질문에 답만 요구된 경우 코드·문서를 바꾸지 않음. "반영할 것", "수행할 것" 같은 지시가 있을 때만 작업함.
- 팀은 ROS 2 부트캠프 수강생임. `rclpy` 노드·토픽·파라미터·타이머·런치·평범한 파이썬 클래스 범위를 넘지 않음. 메타프로그래밍, 데코레이터(`@dataclass`/`@property` 제외), 불필요한 스레드/asyncio 금지.

---

## 3. 쓰기 권한 범위 (ADR_basic §3)

열람은 저장소 전체 가능. **생성·수정은 아래 경로 안에서만** 함.

1. `cobot3_ws/src/smart_farm_navigation/`
2. `cobot3_ws/isaacpjt/smart_farm/`
3. `docs/`

- 위 경로 안이라도 **팀원 소유 파일**(`isaacpjt/smart_farm/runtime/`, `scripts/robot_motion*.py`, `scripts/pallet_transfer.py`, `scripts/lift.py`, `smart_farm_navigation/navigation_node.py`, `src/smart_farm_interfaces/`)은 수정 전 보고하고, 수정하면 주석 `[navigation YYYY-MM-DD]` 를 남김.
- `docs/reference/` 는 해당 주제를 다루는 중이 아니면 열람을 자제함(토큰).
- 저장소 트리 재구성 금지. 미사용 파일은 각 모듈의 `past/` 로 이동함.
- 병합 충돌은 **pull 된 쪽(development)이 이김**. 병합 뒤 ADR_nav2 §2.4 점검표로 내 추가분이 사라졌는지 확인함.

---

## 4. 가이던스 문서 규칙 (ADR_basic §6.1, ADR_nav2 §2.5)

- 경로 `cobot3_ws/src/smart_farm_navigation/guidance/`, 파일명 `guidance2_<n>차.md` (답변마다 차수 +1, 지난 차수는 `guidance/past/` 로).
- **자립형**: 위에서 아래로 실행만 하면 되게 씀. 이전 차수를 열어볼 필요가 없어야 함.
- 몇 번째 터미널인지 명시. **터미널 블록마다 환경 5줄을 매번 반복**함. "위와 같은 5줄" 식 참조 금지.
- 절대경로만 씀. `PROJECT_ROOT` 같은 셸 변수 금지. 어느 cwd 에서도 동작해야 함.
- `git pull`·브랜치 전환을 가이던스에 넣지 않음.
- 선택 단계는 "선택"이라고 표시하고, 각 방법이 무엇을 위한 것인지 설명함.
- 마지막 챕터에 **파일명-역할-요약 표**를 넣음. 로그는 `tee` 로 `results/` 에 남김.
- 고피에서 할 일은 최소로. **GUI 값 읽기 요청·터미널 출력 수동 복사 요청 금지.** 내피에서 계산 가능한 것은 내피에서 함.

---

## 5. 절대 어기지 않을 것 (ADR_nav2 §2.7)

- **내피 모의 결과를 실측 결과로 보고하지 않음.** 실측 여부는 `results/`·`errored/`·bag 으로만 판단함. 모의는 반드시 "내피 모의" 로 표기함.
- **시간 판단은 `/clock`(`use_sim_time: true`) 기준으로만.** `time.time()`·`time.monotonic()` 으로 타임아웃·신선도를 판단하지 않음 (실시간 배율 0.3 → 벽시계는 3배 빨리 걸림).
- `feeder_dock` 은 Nav2 `/cmd_vel` 이 **2초 이상 조용할 때만** 시작함. `/cmd_vel` 발행자는 Nav2 와 `feeder_dock` 둘뿐이며 세 번째를 추가하지 않음.
- 카터 **앞 = `base_link` +x = 구동륜 쪽**, 뒤 = −x = 캐스터·리프트·M0609. 초기 yaw 90°. 이 배치는 팀 P&P 때문에 바꾸지 않음. 랙 통로 안에서 제자리 회전 금지, 후진으로 빠져나감.
- 장면·지도 버전은 ADR 또는 `scenes/` 최신 경로만 신뢰함. 메모리 속 v004/v008 수치는 과거 값임.
- `ros2 topic pub` 이 노드에 닿았다고 가정하지 않음. 노드 로그의 `command …` 줄로 확인함.
- "잘 작동했다" 는 보고와 bag 결과가 다를 수 있음. 판정은 `/feeder_dock/result` 와 최종 world 좌표로 함.
- 카터가 "가만히 있다" 는 보고가 정지를 뜻하지 않음. `feeder_dock` 의 `idle:` 줄과 `/cmd_vel` 값을 먼저 확인함.
- 물리 파라미터·환경변수가 불명확하면 **가정해서 진행하지 말고 먼저 질문**함.
- 수정의 여파(Nav2 연동, TF 트리, 센서 수신)를 미리 예측해 답변에 함께 적음. 하드코딩식 임시방편 금지.

---

## 6. 저장소 위생

- **`.claude/`, `.claude.json*`, 대화 기록(`*.jsonl`), 자격증명 파일을 저장소에 커밋하지 않음.** `.gitignore` 에 등록되어 있음.
  기기 간 대화 기록 이전은 git 이 아니라 `scp`/`gcloud compute scp` 로 `~/.claude/projects/<경로>/` 에 직접 복사함.
- `results/bags/`, `media_log/`, 대용량 USD 에셋은 git 제외.
- 커밋 접두어: `feat/fix/test/refactor/docs/ci/chore`. 브랜치는 `feature/*` → `development` → `main` (PR).
- 작업 후 현재 원격 브랜치로 커밋·푸시까지 하고, **무엇이 들어갔는지 답변이나 가이던스에 반드시 보고**함.

---

## 7. 현재 상태 (2026-09-25)

- 브랜치 `feature/Inspection-Place-nav2` (통합 브랜치. feature/navigation2 + 팀의 PLACE_INSPECT + smart_farm_vision).
- 최신 가이던스 `guidance/guidance2_25차.md` — 전 구간(주행→도킹→Place) 절차.
- 직전 작업(`ceeafd5`): 굼뜬 차체(각속도 1~3초 지연)를 전제로 도킹 조향 재설계. `DockLogic`(ROS 없는 상태기계) + `FeederDock`(ROS 배선) 분리, `SETTLE` 단계 추가, 면 법선 추종 후진, 횡 오차 0.06 m 초과 시 0.9 m 물러나 재시도.
- **검증은 내피 모의뿐** (오프라인 격자 225/225, ROS 회귀 10/10). **실측 미실시** — 25차 절차로 실측하는 것이 다음 할 일.
- 실측 전 `sim_test/dock_regression.py` 를 내피에서 먼저 돌림. 실측 중에는 절대 돌리지 않음(같은 도메인에 Nav2 두 벌).
- 받아야 할 실측 로그: 고피 Isaac 터미널의 `[도킹] 카터 본체 world (…)` 줄 (팔 자세 허용 범위 결정용).

---

## 8. 더 자세한 배경

에이전트 메모리(`~/.claude/projects/<레포경로>/memory/`)에 다음이 있음: `adr-first`, `nav2-track-v008`(Nav2 트랙 전체 이력), `gopi-naepi-split`, `guidance-doc-conventions`, `minimize-gopi-work`, `stepwise-observable-workflow`, `carter-visual-front`, `project-milestones`, `integration-design`, `reference-docs-layout`, `ros2-bootcamp-level`. 메모리는 기기별로 저장되므로 **저장소에서의 단일 출처는 ADR 과 이 파일임**.
