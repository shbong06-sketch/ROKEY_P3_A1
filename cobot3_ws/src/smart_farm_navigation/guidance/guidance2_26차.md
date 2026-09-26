# guidance2_26차 — 고피3 한 대에서 파지 → 도킹 → 턴테이블 Place (모듈 단위 통합 시험)

- 작성일: 2026-09-25, 브랜치 `feature/Inspection-Place-nav2`.
- **이번 판의 범위는 딱 세 단계다: 랙 팔레트 파지(PICK_HARVEST) → 주행·정밀 도킹(NAVIGATION) → 턴테이블 줄기 벨트에 내려놓기(PLACE_INSPECT).** 그 뒤의 비전 검사·솎아내기는 이번 범위가 아니므로 끄고 돌린다.
- **기기는 고피3(GCP VM) 한 대뿐이다.** Isaac 도 Nav2 도 RViz2 도 도킹도 전부 이 한 대에서 돈다. 내피와의 원격 통신은 3일 뒤 교육장 복귀부터이고, 그때의 절차는 부록 A 에 적었다.
- **이 문서가 옛 26·27차를 대체한다(2026-09-25 사용자 지시).** 그 둘은 실측 없이 번호만 올라간 판이라 저장소에서 지웠고, **실측에 근거한 내역은 아래 "24차 실측에서 막힌 곳" 과 "지금까지 고친 것" 에 옮겨 담았다.** 25차까지는 `guidance/past/` 에 그대로 있다.
- 경로는 심볼릭 링크로 맞춰 두었으므로 **다른 기기와 같은 대문자 경로 `/home/rokey/ROKEY_P3_A1` 을 그대로 쓴다.**
- **아직 실측하지 않았다. 검증은 고피3 모의뿐이다** (ADR 2.7: 모의 통과 ≠ 실측 통과).

## 0. 이번 판에서 알아 둘 것 (읽기만)

| 항목 | 내용 |
|---|---|
| 환경 줄 | 고피3 는 **4줄**이다. `FASTRTPS_DEFAULT_PROFILES_FILE` 을 넣지 않는다(화이트리스트가 교육장 랜선 IP 전용이라 VM 안에서는 통신이 전부 끊긴다). 도메인은 101 |
| 한 대에서 도는 것 | Isaac(터미널 2), Nav2·RViz2·`/scan` 생성·정밀 도킹·bag(터미널 3), 주행 노드(터미널 4), 결과 구독(터미널 5·6), 명령 발행(터미널 1) |
| 화면 | 이 VM 에는 화면(`DISPLAY`)이 없다. **그렇다고 `--headless` 로 띄우면 안 된다**(0-2절: ROS 토픽이 하나도 안 나온다). **가상 디스플레이 `xvfb-run` 으로 띄운다.** Nav2 는 `use_rviz:=false` 로 띄운다. 판정은 화면이 아니라 터미널 로그와 bag 으로 한다 |
| Isaac 은 이 VM 에서 돈다 (2026-09-26 확인) | `xvfb-run` 으로 띄우면 씬이 열리고 `/clock`·`/tf`·`/chassis/odom`·3D 라이다가 모두 발행된다(0-2절). **단 실시간 배율이 0.37 이라 교육장(1.12)보다 3배 느리게 진행된다.** 센서 주기는 시뮬 기준으로는 교육장과 거의 같다 |
| 장면 | `scenes/Collected_smartfarm_v014/Collected_smartfarm_v014_room_core_cabbage.usd`. 그 폴더가 있으면 팀 앱이 `--scene` 없이 자동으로 이 씬을 연다 |
| 지도 | `maps/Collected_smartfarm_v014.yaml` 을 `nav2.launch.py` 가 자동으로 쓴다 |
| 집는 팔레트 | **`Pallet_01`** 이다(랙 L1 선반, `standalone_app.py:293` 의 `HARVEST_TASK = Task(PALLET_1_PATH, None, pick_only=True)`). 게다가 `PICK_HARVEST` 는 명령 네 필드가 **`recipe_id: HARVEST_RACK_L1` / `pallet_id: PALLET_001` / `source: RACK_L1` / `destination: CARRY`** 와 정확히 일치하지 않으면 `INVALID_COMMAND` 로 거부한다(`standalone_app.py:892~903`). 그래서 7-1 의 명령이 유일한 유효 조합이다. 참고로 `Pallet_02`·`Pallet_03` 은 랙 안에서 선반을 옮기는 `TRANSFER` 연산이 쓰는 것이고 이번 범위가 아니다 |
| 도킹 거리 | 라이다가 검출하는 면(world y −3.645)에서 **0.92 m**. 팀이 올인원을 끝까지 돌려 확인한 값이며 **팀의 place 수정과 한 쌍**이다. 임의로 바꾸지 않는다 |
| Place | 팀 수정으로 네 가지가 고쳐졌다 — 놓는 방향이 도킹 방향과 90° 어긋나던 것, TurnTable 쿼터니언 비정규화(0.507), 높이 25.9 mm 부족, 포크판이 롤러에 걸리던 것 |
| 컨베이어는 켠다 | Place 중 줄기 벨트를 멈추는 인터록(`hold_stem`)이 **팀 place 수정의 일부**다. 컨베이어를 끄면 이번에 확인하려는 조건이 달라지므로 켜 둔다. 검사 스테이션만 끈다 |
| 검사·솎아내기는 끈다 | `--no-vision-station` 을 붙인다. 이번 범위가 아니고, YOLO 가 별도 파이썬(`ultralytics`)을 요구해 변수를 늘린다. 이때 컨베이어는 트레이를 10 초 뒤 스스로 배출한다 |
| 도킹 시작 | 자동 시작은 꺼져 있다(`dock_auto` 기본 false). 주행 노드가 명령 식별자를 실어 시작시키고, 같은 식별자로 돌아온 결과만 인정한다 |
| 도킹 순서 | `SETTLE` → `ALIGN_TO_GOAL` → `REVERSE` → `CHECK` → (`CREEP` → `DONE`) 또는 `BACKOFF` 후 재시도. Nav2 `/cmd_vel` 이 2 s 조용하고 차체 각속도가 0.03 rad/s 아래여야 정렬을 시작한다(ADR 2.2) |
| 도킹 조향 | 차체 각속도가 명령을 1~3 s 늦게 따라오는 것을 전제로 짰다. 후진 0.10 m/s, 조향 상한 0.12 rad/s 라 느리게 보인다. 정상이다 |
| 제한 시간 | 주행 노드 600 s, 도킹 240 s(노드 자체 120 s). 모두 `/clock` 기준이다 |
| 발행 명령 | 모두 `--once --max-wait-time-secs 15`. 15 초 안에 구독자를 못 찾으면 무한 대기 대신 오류로 끝난다 |
| 결과 구독 | 결과 토픽은 보관되지 않는다. **6절 구독 터미널을 명령보다 먼저 띄운다** |
| 회귀 시험 | 2절에서 먼저 돌린다. **본 시험 중에는 절대 돌리지 않는다**(같은 기기에 Nav2 가 두 벌 뜬다) |

### 24차 실측에서 막힌 곳과 그 근거 (읽기만 · 교육장 실측 기록)

지금 쓰는 도킹 코드가 왜 이렇게 생겼는지의 근거다. **모의가 아니라 2026-09-24 교육장 실측 bag 기록이다.**
Nav2 주행은 성공했고 23차의 멈춤 오판도 사라졌다. 그 뒤 두 가지가 겹쳤다.

| 시점(bag) | 기록 |
|---|---|
| 291.4 | `ALIGN_TO_GOAL` 시작, 방향 오차 30° |
| 295.1 | `REVERSE` 시작, 방향 오차 0.1° — **그러나 차체는 아직 0.24 rad/s 로 돌고 있었다** |
| 296~297 | 관성으로 15° 더 돌아감. 후진하며 도킹 선에서 옆으로 0.16 m 벗어남 |
| 304.5 | `CHECK` 도착 방향 **+21.2°** → `BACKOFF` |
| 310.7 | 물러남 완료, 다시 정렬 |
| 313.7 | Isaac 종료: `RuntimeError: HOLD: 운반 중 팔레트가 30.1 mm 미끄러졌습니다` (팀 감시 예외가 main loop 에서 안 잡힘) |

기록된 `/scan` 을 도킹 노드와 같은 검출기에 다시 넣어 보면 **센서는 멀쩡했고 제어가 잘못 판단했다**는 것이 바로 보인다.

| bag 시각 | 면까지 | 방향 오차 | 횡 | 목표 방위 | 실제 cmd_w | odom 각속도 |
|---|---|---|---|---|---|---|
| 295.1 `[REVERSE]` 선언 | 1.99 | +0.1° | −0.01 | +0.3° | +0.116 | **+0.227** ← 아직 돌고 있다 |
| 296.1 | 1.99 | −8.2° | +0.34 | −11.2° | −0.165 | +0.144 |
| 297~302 | 1.9→1.2 | −15°→+9° | +0.60→0.02 | **−17~−24° 5 초 지속** | **−0.200 고정** | −0.10~−0.14 |
| 303.5 | 1.01 | **+20.7°** | −0.28 | −5.9° | −0.036 | −0.045 |

| 원인 | 내용 | 조치 (현재 코드에 들어 있음) |
|---|---|---|
| 차체 각속도 응답이 느리다 | 0.35 rad/s 를 명령해도 3 s 걸려 0.26, 끊어도 초당 0.14 씩만 줄어든다(Isaac DifferentialController 가속 제한). 22차엔 제자리 회전이 명령의 12% 만 나왔다 | 정렬 종료에 "실제 각속도 < 0.03" 조건. 모든 조향은 관성으로 더 돌 각도와 측정 지연을 뺀 오차로 계산. 시작 전 `SETTLE` 단계 |
| 점을 겨누는 후진 조향 | 남은 거리가 짧을 때 횡 오차를 방향으로 갚다가 도착 방향이 틀어진다 | 면 법선(도킹 선)을 따라가되 횡 보정용 방향 이탈을 10° 로 제한, 마지막 0.4 m 는 직각만. 정렬은 도킹 지점이 아니라 더 먼 면 가운데를 겨눔. 횡 0.06 m 넘으면 0.9 m 물러나 재시도(최대 2회) |
| 팀 감시 예외로 앱 종료 | 운반 중 팔레트 30 mm 이동 → 예외 → Isaac 종료 | main loop 에서 잡아 한 번 기록하고 계속. PLACE 때 같은 검사가 `MOTION_FAILED` 로 보고 |

같은 것을 다시 보려면(고피3 에서 가능, Isaac 불필요. **선택**):

```bash
export ROS_DOMAIN_ID=77
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
source /opt/ros/jazzy/setup.bash
source /home/rokey/ROKEY_P3_A1/cobot3_ws/install/setup.bash
python3 /home/rokey/ROKEY_P3_A1/cobot3_ws/src/smart_farm_navigation/sim_test/replay_bag_dock.py ~/.ros/smart_farm_navigation/bags/nav2_20260923_2154 --from 295 --to 306
```

(그 bag 이 이 기기에 없으면 돌지 않는다. 교육장 내피에 있던 기록이다.)

### 고피3 Isaac 기동 파일럿 결과 (2026-09-26 · 이 기기에서 직접 확인함)

이 기기에서 Isaac 을 처음 띄워 본 기록이다. **팔레트를 집거나 주행시키지는 않았고 기동과 토픽 발행만 봤다.**

| 시도 | 방법 | 결과 |
|---|---|---|
| 1차 | `python.sh standalone_app.py --autoplay --headless --no-vision-station` | **실패.** 씬은 열리고 `[READY]`·`[대기]` 까지 갔으나 **ROS 토픽이 하나도 흐르지 않았다** |
| 2차 | 위 명령에서 `--headless` 를 빼고 **`xvfb-run -a -s "-screen 0 1920x1080x24"`** 로 감쌈 | **성공.** `/clock` 7.1 Hz, `/tf` 7.4 Hz, `/chassis/odom` 7.4 Hz, 3D 라이다 41,272 점/스캔 |

1차가 왜 실패했는지 (`results/isaacpilot_20260925_2356.txt`, `linkpilot_20260926_0002.txt`):

| 관찰 | 뜻 |
|---|---|
| `/clock` 토픽은 목록에 있고 `Publisher count 1` 인데 메시지는 0 건 | 발행 노드는 만들어졌지만 **tick 이 돌지 않았다** |
| `/front_3d_lidar/lidar_points` 는 토픽 자체가 없음 | RTX 라이다는 렌더 파이프에 붙어 있는데 **렌더가 돌지 않았다** |
| GPU 사용률 0 % (메모리만 4.2 GB) | 같은 뜻 |
| `[대기] 팔 베이스 정지를 기다리는 중` 이 117 초까지 증가 (코드 제한은 sim 15 초) | **시뮬 시계가 사실상 흐르지 않았다** |

팀이 Windows 에서 남긴 기록(*headless 올인원에서 odom/clock 없음 → GUI 모드로 해결*)이 리눅스 standalone 에서도 그대로 재현된 것이다. **가상 디스플레이를 주면 GUI 모드로 뜨면서 렌더·물리·ROS 그래프가 모두 돈다.**

2차에서 실제로 나온 값 (`results/isaacpilot2_xvfb_20260926_0004.txt`, `linkpilot2_0008.txt`):

| 항목 | 값 | 판단 |
|---|---|---|
| 실시간 배율 | **0.37** | 벽시계 1 초에 시뮬 0.37 초. 교육장 고피보다 느릴 수 있다 |
| `/clock` | 7.1 Hz | 정상 |
| `/tf` odom→base_link, `/chassis/odom` | 7.4 Hz | 정상 |
| 3D 라이다 | **1.0 Hz**(벽시계), 41,272 점/스캔, frame `front_3d_lidar` | 점 수 정상(fullScan). **시뮬 기준 약 2.7 Hz 로 교육장과 거의 같다**(아래) |
| 자기 반사 | 리그 상자 안 **0 / 41,272 점** | 과거 실측 기준은 35~66 점이었다. 팀이 카터 몸체 속 부품을 뺀 경량본을 쓰기 때문으로 보인다 |
| GPU | 28~46 %, 4.5 GB | 여유 있음 |
| `nav2_link_check` 판정 | `RESULT FAIL - no lidar topic arrives at >= 3 Hz` | **기준이 벽시계라서 생긴 오판이다**(아래) |

**`RESULT FAIL` 은 성능 문제이지 센서 설정 문제가 아니다.** 같은 통합 앱으로 교육장에서 잰 값과 나란히 놓으면 분명하다.

| | 교육장 고피 (2026-09-23, `link_20260923_2154.txt`) | 고피3 (2026-09-26) |
|---|---|---|
| 실시간 배율 | **1.12~1.15** | **0.37** |
| 라이다 (벽시계) | 3.8~3.9 Hz | 1.0 Hz |
| 라이다 (시뮬 환산) | **약 3.4 Hz** | **약 2.7 Hz** |
| 점 수 | 41,538~41,880 | 41,272 |
| 판정 | `RESULT OK` | `RESULT FAIL` |

즉 **시뮬 기준 스캔율은 3.4 → 2.7 Hz 로 큰 차이가 없고, 벽시계 주기가 낮은 것은 순전히 실시간 배율이 3배 느리기 때문이다.** `nav2_link_check` 의 `>= 3 Hz` 는 벽시계 기준이라 배율 0.37 에서는 통과할 수 없는 기준이다. Nav2·`feeder_dock` 의 시간 판정은 모두 `/clock` 기준(`use_sim_time: true`)이므로 **시뮬 기준이 같으면 동작 조건은 교육장과 같다.** 실제로 원래부터 10 Hz 가 아니라 3.4 Hz 였다(팀 앱의 `RENDER_EVERY = 3` 때문으로 보인다).

**대신 벽시계 시간이 3배 걸린다.** 팀 기준 전 구간 4분 19초(시뮬)는 이 기기에서 벽시계 약 12분이다. 씬 로딩만 3~5분 더 든다.

부수로 확인한 씬 문제 두 가지:

| 증상 | 원인 | 조치 |
|---|---|---|
| `Could not open asset .../SubUSDs/materials.usd` | 실제 파일명은 **`Materials.usd`(대문자 M)**. 리눅스는 대소문자를 가리므로 팀 Windows PC 에서는 안 나던 오류다 | 심볼릭 링크 `materials.usd -> Materials.usd` 를 만들어 해결했다(씬 폴더는 git 제외라 저장소 영향 없음) |
| `Could not load sublayer .../env_dressing.usd; skipping` | 그 파일이 **팀 공유 zip 에 아예 없다**(`env_dressing_tex/`, `env_dressing_navmap/` 만 있다) | 우리 v014 지도도 같은 USD 를 읽어 만들었으므로 지도와 시뮬은 서로 어긋나지 않는다(v011 지도와 36 픽셀만 달랐던 것과 일치). **팀에 확인이 필요하다** |

### 화면 녹화 가능 여부 (2026-09-26 검증 · 읽기만)

실측 영상을 남길 수 있는지 확인한 결과다. **둘 다 된다.**

| 검증 | 방법 | 결과 |
|---|---|---|
| Isaac 화면이 가상 디스플레이에 실제로 그려지는가 | `DISPLAY=:99` 에서 `xwininfo -root -tree`, `import -window root` | **그려진다.** 창 `Isaac Sim Python 5.1.0` 1440x900, 뷰포트·Stage·Content·Property 패널 전부 정상 |
| RViz2 를 같은 화면에 띄울 수 있는가 | 같은 `DISPLAY=:99` 로 `rviz2 -d rviz/nav2_smartfarm.rviz` | **뜬다. `OpenGl version: 4.5` — 소프트웨어 렌더가 아니라 GPU 가속이다.** Displays(Grid/LaserScan/Map/Stations/Global Planner/Controller)와 Nav2 패널 정상 |
| 영상으로 받아지는가 | `ffmpeg -f x11grab -framerate 10 -video_size 1920x1080 -i :99` 로 8 초 | **된다.** 1920x1080 / 10 fps / 8.0 s 정상 생성 |
| 자원 | Isaac + RViz2 동시 | GPU 49 %, 4.5 GB / 23 GB. 디스크 여유 190 GB |

**걸림돌 하나: 창 관리자가 없어 창이 겹치고 잘린다.** RViz2 창이 화면 좌표 +1091+222 에 1610x893 으로 떠서, 그 안의 3D 뷰(x=1455 부터 897 폭)가 화면 오른쪽 끝 1920 을 넘어가 **잘린다.** 그대로 녹화하면 RViz2 의 3D 화면이 안 나온다. 해법은 셋이다.

| 방법 | 내용 | 평가 |
|---|---|---|
| 디스플레이를 둘로 나눈다 | `Xvfb :99` 에 Isaac, `Xvfb :98` 에 RViz2 를 띄우고 **각각 따로 녹화** | **가장 단순하고 확실하다.** 편집에서 두 영상을 나란히 붙이면 된다 |
| 화면을 키운다 | `Xvfb :99 -screen 0 2560x1440x24` | 창은 여전히 겹칠 수 있지만 잘림은 줄어든다 |
| 창 관리자를 넣는다 | `matchbox-window-manager` 등을 같은 디스플레이에 띄워 배치 | 배치가 자유롭지만 설치·설정이 는다 |

녹화용 참고:

- 디스플레이 번호를 **고정**해서 띄우면 캡처가 쉽다: `Xvfb :99 -screen 0 1920x1080x24 &` → `export DISPLAY=:99` 후 3절 명령을 `xvfb-run` 없이 그대로 실행.
- `xvfb-run -a` 로 띄웠다면 번호는 `ps -ef | grep Xvfb` 로 찾고, `XAUTHORITY` 도 그 줄의 `-auth` 경로를 써야 한다.
- 렌더가 초당 7 회 수준이므로 캡처 `-framerate` 는 **10** 이면 충분하다. 그 이상은 용량만 는다.
- Isaac 뷰포트의 기본 카메라는 Perspective 라 비전룸을 보고 있다. 씬의 `/World/ProcessCameras` 에 `Cam0_Perspective`, `Cam1_Harvest`, `Cam2_Nav2Place`, `Cam4_CullPickPlace`, `Cam5_Pusher` 가 있으므로 **녹화 전에 보고 싶은 공정의 카메라로 바꾼다.**
- 이 검증에서 남긴 증거는 `results/media_log/` 에 있다(`pilot_isaac_only.png`, `pilot_isaac_rviz.png`, `pilot_capture_8s.mp4`). 이 경로는 git 에 올라가지 않는다.

### 지금까지 고친 것 (읽기만)

| 판 | 증상 | 조치 |
|---|---|---|
| 24차 실측 이후 | PLACE 에서 IK 110° 점프 / `DESCEND_5` 20.7°(한계 20°) | 팀 `feature/cabbage-place-fix` 의 `turntable_place_pose()` 수정 + 도킹 거리 0.92 를 함께 반입(`a95ba6c`) |
| 24차 | 정렬 직후에도 차체가 0.23 rad/s 로 돌아 후진 중 0.16 m 이탈, `CHECK` +21.2° | `SETTLE` 단계, 관성·측정 지연을 뺀 오차로 조향, 면 법선 추종, 횡 0.06 m 초과 시 0.9 m 물러나 재시도 |
| 24차 | 운반 중 팔레트 30 mm 미끄러짐 예외로 Isaac 종료 | main loop 에서 잡아 한 줄 남기고 계속 |
| 23차 | 후진 0.6 s 만에 `STALLED_REVERSE` | 멈춤 감지가 방향 변화도 이동으로 인정, 단계마다 기준점 재설정 |
| 22차 | 후진 1 m 중 방향 25° 틀어짐 | 도킹 지점 제자리 회전 금지, 물러나 재진입(최대 2회) |
| 21차 | Place 지시 후 결과가 안 보임 | 결과 구독 터미널을 명령보다 먼저 띄운다(6절) |
| 19차 | 오류가 로그에 안 남음 | `PYTHONUNBUFFERED=1`. `stdbuf` 는 별칭이라 쓸 수 없다 |
| 18차 | 명령에서 메시지 형식 오류 | 명령 보내는 터미널에도 워크스페이스 빌드·`source`(1절) |

### 검증 상태 (고피3 모의, 실측 아님)

| 시험 | 조건 | 결과 |
|---|---|---|
| ROS 회귀 10 시나리오 (standoff 0.92) | 현장과 같은 launch + 합성 로봇 | **10/10 통과.** 도착 면 거리 0.916~0.950 m, 최대 횡 오차 0.057 m (허용 0.06). `results/regression_20260925_standoff092.txt` |
| 오프라인 격자 (standoff 0.92) | 시작 자세 45종 × 차체 5종 = 225 | 방향·거리·충돌 225/225. **횡 오차는 224/225** — 1 케이스가 0.063 m (시작 world (−2.39, −1.85) yaw 70°, 제자리 회전이 명령의 12% 만 나오는 차체). `results/docksim_20260925_standoff092.txt:52` |
| v014 지도 교체 후 normal | 같은 launch, 지도만 v014 | 1/1 통과 (면 0.947 m, 횡 −0.006 m) |

**실측 전에는 도킹 이득을 고치지 않는다.** 고치면 위 10/10 의 근거가 무효가 된다. 실측에서 팔 쪽 실제 횡 허용치를 받으면 `lat_tol_m`·`max_retry` 로 줄인다.

---

## 1. 터미널 1 — 워크스페이스 빌드 (처음 한 번만)

```bash
export ROS_DOMAIN_ID=101
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
source /opt/ros/jazzy/setup.bash
cd /home/rokey/ROKEY_P3_A1/cobot3_ws
colcon build --symlink-install --packages-select smart_farm_interfaces smart_farm_navigation 2>&1 | tee /home/rokey/ROKEY_P3_A1/cobot3_ws/src/smart_farm_navigation/results/build_$(date +%Y%m%d_%H%M).txt
source /home/rokey/ROKEY_P3_A1/cobot3_ws/install/setup.bash
ros2 interface show smart_farm_interfaces/msg/TaskCommand
```

기대: `Summary: 2 packages finished` 뒤에 `string task_id` 로 시작하는 필드 목록이 나온다.

- **YAML·코드를 고쳤으면 반드시 이 빌드를 다시 한다.** launch 는 `install/` 복사본을 읽으므로 빌드하지 않으면 옛 값으로 돈다.
- 이 터미널은 7절 명령 발행에서 그대로 다시 쓴다. **빌드는 한 번이면 되지만 `source` 는 새 터미널마다 해야 한다.**

## 2. 터미널 1 — 도킹 회귀 시험 (본 시험 전 1회, 20~30 분)

Isaac 없이 돈다. 현장과 같은 `nav2.launch.py` 를 띄우고 합성 로봇으로 도킹 10 시나리오를 돌린다. 실패가 있으면 본 시험을 미룬다.

```bash
export ROS_DOMAIN_ID=77
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
source /opt/ros/jazzy/setup.bash
source /home/rokey/ROKEY_P3_A1/cobot3_ws/install/setup.bash
python3 /home/rokey/ROKEY_P3_A1/cobot3_ws/src/smart_farm_navigation/sim_test/dock_regression.py 2>&1 | tee /home/rokey/ROKEY_P3_A1/cobot3_ws/src/smart_farm_navigation/results/regression_$(date +%Y%m%d_%H%M).txt
```

기대: 마지막에 `10/10` 표가 나온다. 도메인 77 은 본 시험(101)과 섞이지 않게 일부러 다르게 둔 것이다.

빠르게 상태기계만 보려면(2 분, **선택**):

```bash
source /opt/ros/jazzy/setup.bash
python3 /home/rokey/ROKEY_P3_A1/cobot3_ws/src/smart_farm_navigation/sim_test/dock_sim.py --fail-only
```

**이 절을 마친 뒤에는 회귀 시험 프로세스가 모두 끝났는지 확인한다.** 본 시험 중에 남아 있으면 Nav2 가 두 벌이 되어 서로 `/cmd_vel` 을 쏜다.

## 3. 터미널 2 — Isaac 기동과 예비 점검 (본 시험 전에 반드시 한 번)

**이 터미널에서는 워크스페이스를 `source` 하지 않는다**(팀 앱이 Isaac 번들 ROS 라이브러리를 먼저 써야 한다).
**`--headless` 를 쓰지 않는다. 대신 가상 디스플레이로 감싼다**(0-2절에서 확인한 사실).

```bash
export ROS_DOMAIN_ID=101
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export PYTHONUNBUFFERED=1
xvfb-run -a -s "-screen 0 1920x1080x24" /home/nitrouriah92/isaacsim/python.sh /home/rokey/ROKEY_P3_A1/cobot3_ws/isaacpjt/smart_farm/runtime/standalone_app.py --autoplay --no-vision-station 2>&1 | tee /home/rokey/ROKEY_P3_A1/cobot3_ws/src/smart_farm_navigation/results/isaac_$(date +%Y%m%d_%H%M).txt
```

- `xvfb-run` 이 없으면 `sudo apt-get install -y xvfb` 로 넣는다(2026-09-26 에 이 기기에 설치해 두었다).
- Isaac 경로의 `$HOME` 만은 다른 기기와 다르다(`/home/nitrouriah92`). 위 명령의 `python.sh` 경로를 그대로 쓴다.
- 씬 로딩까지 **3~5 분** 걸린다. 기대 순서:

```
[시작] Scene을 불러옵니다: /home/rokey/ROKEY_P3_A1/.../Collected_smartfarm_v014_room_core_cabbage.usd
[라이다] 3D 라이다 fullScan=True (1개 helper)
[컨베이어] 통로 확인됨 — 벽 개구부 이상 없음
[READY] Collected_smartfarm_v014_room_core_cabbage scene ready; ...
[대기] /sim_task/command의 String/JSON 명령을 기다립니다
```

- 첫 줄 씬 이름에 `_cabbage` 가 없으면 v014 폴더를 못 찾은 것이다. `--scene /home/rokey/ROKEY_P3_A1/cobot3_ws/isaacpjt/smart_farm/scenes/Collected_smartfarm_v014/Collected_smartfarm_v014_room_core_cabbage.usd` 를 붙여 다시 띄운다.
- Isaac 의 Stop 버튼은 누르지 않는다(팀 앱이 종료 처리에서 죽는다). 끝낼 때는 Ctrl+C, 또는 다른 터미널에서 `pkill -f standalone_app.py`.
- **Isaac 을 다시 띄우면 4·5절도 다시 띄운다.** 시뮬 시각이 0 으로 돌아가 TF·센서 시각이 어긋난다.

`[대기]` 가 뜨면 **터미널 1** 에서 토픽이 실제로 흐르는지 본다.

```bash
export ROS_DOMAIN_ID=101
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
source /opt/ros/jazzy/setup.bash
source /home/rokey/ROKEY_P3_A1/cobot3_ws/install/setup.bash
ros2 run smart_farm_navigation nav2_link_check 2>&1 | tee /home/rokey/ROKEY_P3_A1/cobot3_ws/src/smart_farm_navigation/results/link_$(date +%Y%m%d_%H%M).txt
```

2026-09-26 에 이 기기에서 나온 값과 견주어 판단한다.

| 항목 | 2026-09-26 관측값 | 판단 |
|---|---|---|
| `/clock` | 7.1 Hz, 실시간 배율 **0.37** | 정상 |
| `/tf` odom→base_link, `/chassis/odom` | 7.4 Hz | 정상 |
| `/front_3d_lidar/lidar_points` | 1.0 Hz(벽시계), **41,272 점/스캔** | 정상 (아래 환산으로 판단한다) |
| 자기 반사 | 0 / 41,272 점 | 경량 카터라서 그렇다. `cloud_self_filter` 의 제거 상자가 할 일이 없을 뿐 오류가 아니다 |
| 마지막 줄 | `RESULT FAIL - no lidar topic arrives at >= 3 Hz` | **이 기기에서는 이 판정을 그대로 믿지 않는다**(아래) |

**판정은 이렇게 한다.** `nav2_link_check` 의 `>= 3 Hz` 는 벽시계 기준이라 실시간 배율 0.37 인 이 기기에서는 통과할 수 없다. 대신 **시뮬 기준으로 환산해서 본다.**

> 시뮬 기준 스캔율 = (라이다 Hz) ÷ (실시간 배율). 예: 1.0 ÷ 0.37 = **2.7 Hz**.
> 교육장 통합 앱 실측이 3.8 ÷ 1.12 = **3.4 Hz** 였으므로, **2.5 Hz 이상이면 진행한다.**

- 점 수가 **41,000 안팎**이면 fullScan 이 제대로 걸린 것이다. 6,900 점 근처면 fullScan 이 꺼진 것이므로 진행하지 않는다.
- 시뮬 환산이 2.5 Hz 미만이거나 점 수가 이상하면 `link_*.txt` 와 `isaac_*.txt` 를 보고한다.
- `/clock` 이 0 Hz 이면 `xvfb-run` 없이 띄웠거나 `--headless` 가 붙어 있는 것이다. 명령을 다시 확인한다.
- **벽시계로는 교육장의 3배가 걸린다.** 7-1~7-3 을 다 돌리는 데 벽시계 12분 안팎을 잡는다. 제한 시간은 모두 `/clock` 기준이라 타임아웃이 앞당겨지지는 않는다.
- GPU 를 함께 보려면(**선택**) 다른 터미널에서:

```bash
nvidia-smi --query-gpu=timestamp,utilization.gpu,memory.used,memory.total,temperature.gpu --format=csv -l 2 2>&1 | tee /home/rokey/ROKEY_P3_A1/cobot3_ws/src/smart_farm_navigation/results/gpu_$(date +%Y%m%d_%H%M).csv
```

## 4. 터미널 3 — Nav2 · `/scan` 생성 · 정밀 도킹 · bag 기록

3절 Isaac 이 `[대기]` 이고 `RESULT OK` 를 받은 뒤에 띄운다.

```bash
export ROS_DOMAIN_ID=101
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
source /opt/ros/jazzy/setup.bash
source /home/rokey/ROKEY_P3_A1/cobot3_ws/install/setup.bash
ros2 launch smart_farm_navigation nav2.launch.py record:=true use_rviz:=false 2>&1 | tee /home/rokey/ROKEY_P3_A1/cobot3_ws/src/smart_farm_navigation/results/nav2_$(date +%Y%m%d_%H%M).txt
```

기대: `rosbag -> ~/.ros/smart_farm_navigation/bags/nav2_…` → `scan_mode auto -> cloud` → `AMCL initial pose (-0.421, 1.006, 90.0deg)` → `Managed nodes are active` → `[feeder_dock]: [IDLE] waiting`.

- **`use_rviz:=false` 는 이 VM 에 화면이 없기 때문이다.** RViz2 없이도 절차는 전부 돈다. 초기 위치는 launch 가 AMCL 에 직접 넣으므로 `2D Pose Estimate` 클릭이 필요 없다.
- `record:=true` 는 판정 근거다. 화면이 없으니 이번 시험의 증거는 사실상 이 bag 과 로그뿐이다.

## 5. 터미널 4 — 주행 노드

```bash
export ROS_DOMAIN_ID=101
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
source /opt/ros/jazzy/setup.bash
source /home/rokey/ROKEY_P3_A1/cobot3_ws/install/setup.bash
ros2 launch smart_farm_navigation navigation_node.launch.py 2>&1 | tee /home/rokey/ROKEY_P3_A1/cobot3_ws/src/smart_farm_navigation/results/navnode_$(date +%Y%m%d_%H%M).txt
```

기대: `navigation_node ready; destinations: ['FEEDER_DOCK', 'RACK_DOCK', 'CORRIDOR_EXIT']`.

## 6. 터미널 5·6 — 결과 구독 (명령을 보내기 전에 먼저 띄운다)

결과 토픽은 보관되지 않으므로, 명령 뒤에 구독하면 지나간 결과를 못 본다.

터미널 5 (Isaac 작업 결과):

```bash
export ROS_DOMAIN_ID=101
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
source /opt/ros/jazzy/setup.bash
source /home/rokey/ROKEY_P3_A1/cobot3_ws/install/setup.bash
ros2 topic echo /sim_task/result std_msgs/msg/String 2>&1 | tee /home/rokey/ROKEY_P3_A1/cobot3_ws/src/smart_farm_navigation/results/simresult_$(date +%Y%m%d_%H%M).txt
```

터미널 6 (주행 결과):

```bash
export ROS_DOMAIN_ID=101
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
source /opt/ros/jazzy/setup.bash
source /home/rokey/ROKEY_P3_A1/cobot3_ws/install/setup.bash
ros2 topic echo /navigation/result smart_farm_interfaces/msg/TaskResult 2>&1 | tee /home/rokey/ROKEY_P3_A1/cobot3_ws/src/smart_farm_navigation/results/navresult_$(date +%Y%m%d_%H%M).txt
```

둘 다 아무것도 찍히지 않은 채 대기하는 것이 정상이다.

## 7. 터미널 1 — 명령 발행 (1절 터미널을 그대로 쓴다)

### 7-1. 팔레트 파지 (PICK_HARVEST)

```bash
export ROS_DOMAIN_ID=101
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
source /opt/ros/jazzy/setup.bash
source /home/rokey/ROKEY_P3_A1/cobot3_ws/install/setup.bash
ros2 topic pub --once --max-wait-time-secs 15 /sim_task/command std_msgs/msg/String '{data: "{\"task_id\": \"TASK-20260925-001\", \"command_id\": \"TASK-20260925-001-CMD-001\", \"operation\": \"PICK_HARVEST\", \"recipe_id\": \"HARVEST_RACK_L1\", \"pallet_id\": \"PALLET_001\", \"source\": \"RACK_L1\", \"destination\": \"CARRY\"}"}' 2>&1 | tee -a /home/rokey/ROKEY_P3_A1/cobot3_ws/src/smart_farm_navigation/results/cmd_$(date +%Y%m%d).txt
```

**터미널 5** 에 `"status": "SUCCEEDED"` 와 `"safe_to_navigate": true` 가 나오면 다음으로 간다. 팀 기준 소요 약 72 s(시뮬 시간)이며, 이 VM 은 실시간 배율이 낮을 수 있어 벽시계로는 더 걸린다.

### 7-2. 주행 + 정밀 도킹 (NAVIGATION)

```bash
export ROS_DOMAIN_ID=101
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
source /opt/ros/jazzy/setup.bash
source /home/rokey/ROKEY_P3_A1/cobot3_ws/install/setup.bash
ros2 topic pub --once --max-wait-time-secs 15 /navigation/command smart_farm_interfaces/msg/TaskCommand "{task_id: 'TASK-20260925-001', command_id: 'TASK-20260925-001-CMD-002', operation: 'NAVIGATION', destination: 'FEEDER_DOCK'}" 2>&1 | tee -a /home/rokey/ROKEY_P3_A1/cobot3_ws/src/smart_farm_navigation/results/cmd_$(date +%Y%m%d).txt
```

터미널 4 의 기대 출력:

```
goToPose FEEDER_APPROACH: -2.19, -1.55
정밀 도킹 시작 요청: run_id=TASK-20260925-001-CMD-002
도킹 결과 SUCCEEDED: face_dist=0.9x yaw_err=-0.x lat=0.0x
result SUCCEEDED/NONE for TASK-20260925-001-CMD-002 (phase ARRIVED)
```

`face_dist` 는 **0.92 근처**여야 한다(모의 범위 0.916~0.950). **터미널 6** 에 `status: SUCCEEDED`, `reason: NONE`, `reached_station: FEEDER_DOCK` 이 나와야 한다.

### 7-3. 턴테이블에 내려놓기 (PLACE_INSPECT) — 이번 범위의 마지막

```bash
export ROS_DOMAIN_ID=101
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
source /opt/ros/jazzy/setup.bash
source /home/rokey/ROKEY_P3_A1/cobot3_ws/install/setup.bash
ros2 topic pub --once --max-wait-time-secs 15 /sim_task/command std_msgs/msg/String '{data: "{\"task_id\": \"TASK-20260925-001\", \"command_id\": \"TASK-20260925-001-CMD-003\", \"operation\": \"PLACE_INSPECT\", \"recipe_id\": \"PLACE_AT_INSPECTION\", \"pallet_id\": \"PALLET_001\", \"source\": \"CARRY\", \"destination\": \"INSPECT_STATION\"}"}' 2>&1 | tee -a /home/rokey/ROKEY_P3_A1/cobot3_ws/src/smart_farm_navigation/results/cmd_$(date +%Y%m%d).txt
```

터미널 2(Isaac)의 기대 출력:

```
[Place] 차체 정지 확인 (0.5초 대기)
[도킹] 카터 본체 world (…), place 대상까지 x … y … 직선 … m
[컨베이어] 줄기 벨트 정지 (로봇 놓기 중)  →  재가동
[컨베이어] PALLET_001 벨트 감지 — 반송 시작
[컨베이어] PALLET_001 검사 신호 없이 … 배출      ← 검사 스테이션을 껐으므로 10 초 뒤 스스로 배출한다. 정상이다
```

터미널 5 에 `"status": "SUCCEEDED"` 가 나오면 이번 범위는 끝이다.

- **`[도킹] 카터 본체 world (…)` 줄을 그대로 남겨 주면** 팔 자세 허용 범위를 숫자로 정할 수 있다. 이번 시험에서 받아야 할 값이다.

## 8. 실패했을 때

| 증상 | 조치 |
|---|---|
| Isaac 이 안 뜨거나 `/clock` 0 Hz | 3절 표를 따른다. 본 시험으로 넘어가지 않는다 |
| `/clock` 0 Hz, 라이다 토픽 없음, GPU 0 % | `--headless` 로 띄웠다. 3절대로 `xvfb-run` 으로 다시 띄운다 |
| `nav2_link_check` 가 `RESULT FAIL - no lidar topic arrives at >= 3 Hz` | 이 기기에서는 정상일 수 있다. 3절의 시뮬 환산(라이다 Hz ÷ 실시간 배율 ≥ 2.5)과 점 수 41,000 안팎을 보고 판단한다 |
| `Could not open asset .../SubUSDs/materials.usd` | 대소문자 문제다. `ln -sfn Materials.usd /home/rokey/ROKEY_P3_A1/cobot3_ws/isaacpjt/smart_farm/scenes/Collected_smartfarm_v014/SubUSDs/materials.usd` (2026-09-26 에 이미 걸어 두었다) |
| `Could not load sublayer .../env_dressing.usd; skipping` | 그 파일이 팀 zip 에 없다. 지도와는 어긋나지 않으므로 진행해도 되나 팀에 확인한다 |
| 3절 첫 줄 씬 이름에 `_cabbage` 가 없음 | v014 폴더를 못 찾았다. `--scene /home/rokey/ROKEY_P3_A1/cobot3_ws/isaacpjt/smart_farm/scenes/Collected_smartfarm_v014/Collected_smartfarm_v014_room_core_cabbage.usd` 를 붙여 다시 띄운다 |
| `Unknown package 'smart_farm_interfaces'` / `The passed message type is invalid` | 그 터미널에서 `source .../install/setup.bash` 를 하지 않았다. 블록을 통째로 붙인다 |
| 발행 명령이 15 초 뒤 오류로 끝남 | 구독자를 못 찾았다. `/navigation/command` 면 5절 터미널이, `/sim_task/command` 면 3절 Isaac 이 `[대기]` 인지 본다 |
| 명령을 보냈는데 결과가 안 보임 | 6절 구독 터미널을 명령보다 **먼저** 띄웠는지 확인한다 |
| `navigate_to_pose 액션 서버가 없습니다` | 4절 Nav2 가 아직 안 떴다. `Managed nodes are active` 를 본 뒤 보낸다 |
| 도킹이 시작되지 않음 | 4절 터미널에 `FEEDER_APPROACH 부근에 서 있으나 자동 시작이 꺼져 있습니다` 경고가 뜬다. 7-2 명령으로 시작한다 |
| 도킹 중 `BACKOFF` 가 보임 | 정상이다. 도착 방향이 틀어져 물러나 다시 맞추는 중이다(최대 2회) |
| 도킹 결과 `STALLED_REVERSE` / `STALLED_CREEP` | 카터가 걸렸다. bag 이름과 4절 로그를 보고한다 |
| 도킹 결과 `YAW_OFF_..DEG` | 두 번 다시 맞췄는데도 방향이 3° 안에 안 들어왔다. `navresult_*` 와 bag 을 보고한다 |
| 도킹 결과 `FACE_NOT_FOUND` | 라이다가 TurnTable 앞면을 못 봤다. 4절의 `idle:` 줄과 `link_*.txt` 를 함께 보고한다 |
| Place 가 `BASE_NOT_SETTLED` | 차체가 아직 미세하게 움직인다. 2~3 초 뒤 7-3 을 다시 보낸다 |
| Place 가 `MOTION_FAILED` (관절 한계) | **도킹 거리를 임의로 바꾸지 않는다.** 0.92 는 팀 place 수정과 한 쌍이다. 실패 단계 이름(`DESCEND_n`)과 `[도킹] 카터 본체 world` 줄을 보고한다 |
| 주행이 시간 초과 | 실시간 배율이 낮은 것이다. `gpu_*.csv` 와 함께 보고한다. 이 VM 은 GPU 한 장으로 Isaac 과 Nav2 를 같이 돌린다 |
| 무언가가 `/cmd_vel` 을 계속 쏨 | 2절 회귀 시험이 아직 살아 있을 수 있다. 그 프로세스를 먼저 끝낸다 |

## 9. 결과 보고

- `results/` 의 `build_*`, `regression_*`, `isaac_*`, `link_*`, `nav2_*`, `navnode_*`, `cmd_*`, `simresult_*`, `navresult_*`, 있으면 `gpu_*.csv`
- bag 디렉터리 이름 (`~/.ros/smart_farm_navigation/bags/`)
- 7-3 의 `[도킹] 카터 본체 world (…)` 줄 **(필수)**
- 실패했으면 그 파일을 `errored/` 에 둔다

## 10. 파일과 역할

| 파일 | 역할 |
|---|---|
| `isaacpjt/smart_farm/runtime/standalone_app.py` | 장면 실행, 팔레트 파지·내려놓기, 라이다 fullScan, Place 전 차체 정지 확인과 도킹 위치 기록, 컨베이어·비전 스테이션 기동. 팀 파일 |
| `isaacpjt/smart_farm/scripts/conveyor.py`, `conveyor_rollers.py` | 트레이 반송, Place 중 줄기 벨트 인터록(`hold_stem`). 팀 파일 |
| `isaacpjt/smart_farm/scripts/inspection_cull_station.py`, `cull_motion.py`, `human_crossing.py` | 검사·솎아내기·돌발상황. **이번 범위에서는 쓰지 않는다**(`--no-vision-station`). 팀 파일 |
| `isaacpjt/smart_farm/scenes/Collected_smartfarm_v014/` | v014 양배추 씬·에셋. git 제외 |
| `isaacpjt/smart_farm/maps/Collected_smartfarm_v014.{png,yaml}` | Nav2 지도. `nav2.launch.py` 기본값 |
| `launch/nav2.launch.py` | Nav2 · `/scan` 생성 · 정밀 도킹 노드 · 작업점 마커 · bag 기록. `use_rviz:=false` 로 화면 없이 돈다 |
| `launch/navigation_node.launch.py` | `/navigation/command` 를 받아 NavigateToPose 와 정밀 도킹을 구동. 600 s / 도킹 240 s(`/clock` 기준) |
| `smart_farm_navigation/feeder_dock.py` | 라이다로 TurnTable 앞면을 보며 후진 도킹. `DockParams` 가 파라미터 단일 출처(`standoff_m` 0.92) |
| `smart_farm_navigation/cloud_self_filter.py` | 결합카터 자기 반사 제거와 0.25 s 점군 합치기 |
| `smart_farm_navigation/nav2_link_check.py` | 토픽 도달과 자기 반사 점검(3절 예비 점검에 씀) |
| `config/stations.yaml` | 작업점 좌표(FEEDER_APPROACH, FEEDER_DOCK)와 기준 장면 이름 |
| `config/nav2_params.yaml` | Nav2 설정. Smac Hybrid-A*, RPP `desired_linear_vel` 0.3 m/s |
| `sim_test/dock_regression.py` | 본 시험 전 회귀(2절). 합성 로봇 + 현장 launch 로 10 시나리오 |
| `sim_test/dock_sim.py` | 도킹 상태기계 오프라인 격자 모의(225 케이스) |
| `smart_farm_interfaces` | 명령·결과 메시지. 명령을 보내는 터미널에 빌드·`source` 되어 있어야 한다 |

---

## 부록 A. 3일 뒤 교육장(고피1 + 내피) 복귀 시 달라지는 점

지금 절차와 **명령·판정·수치는 모두 같다.** 달라지는 것은 아래 다섯 가지뿐이다.

| 항목 | 고피3 한 대 (지금) | 교육장 2대 (복귀 후) |
|---|---|---|
| 환경 줄 | **4줄** (도메인 101, `RMW_IMPLEMENTATION`, ROS, 워크스페이스) | **5줄** — 위 4줄에 `export FASTRTPS_DEFAULT_PROFILES_FILE=/home/rokey/.ros/fastdds_whitelist.xml` 를 더한다. 두 PC 모두 같은 도메인(고피1 이면 101, 고피2 면 102) |
| 어디서 무엇을 | 전부 이 한 대 | Isaac·명령 발행·결과 구독은 **고피**, Nav2·`/scan`·도킹·주행 노드는 **내피** |
| 빌드 | 이 한 대에서 1회 | **양쪽 모두** 빌드해야 한다. 고피에는 최소한 `smart_farm_interfaces` 가 있어야 명령을 보낼 수 있다 |
| 화면 | 없음 → Isaac `--headless`, Nav2 `use_rviz:=false` | 있음 → `--headless` 를 빼고, `use_rviz:=false` 도 뺀다(RViz2 로 스캔·경로를 본다) |
| 검사·솎아내기 | 범위 밖이라 `--no-vision-station` | 전 구간을 볼 때는 이 인자를 빼고, 고피에 `ultralytics` 가 있는지 먼저 확인한다(`python3 -c "import ultralytics"`). 가중치는 씬 폴더의 `best.pt` 를 자동으로 찾는다 |

복귀 후 전 구간(검사·솎아내기 포함)을 돌릴 때 추가로 필요한 것은 아래 셋뿐이다. 나머지는 1~9절 그대로다.

1. **고피에서 `ultralytics` 확인** — 검사는 Isaac 파이썬이 아니라 별도 `python3` 하위 프로세스로 돈다.

```bash
export ROS_DOMAIN_ID=101
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE=/home/rokey/.ros/fastdds_whitelist.xml
source /opt/ros/jazzy/setup.bash
source /home/rokey/ROKEY_P3_A1/cobot3_ws/install/setup.bash
python3 -c "import ultralytics, torch; print('ultralytics', ultralytics.__version__, '| torch', torch.__version__)"
ls -l /home/rokey/ROKEY_P3_A1/cobot3_ws/isaacpjt/smart_farm/scenes/Collected_smartfarm_v014/best.pt
```

없으면 `python3 -m pip install ultralytics`. 다른 파이썬에 있으면 Isaac 터미널에 `export SMARTFARM_YOLO_PYTHON=<그 파이썬 경로>` 를 더한다. 가중치는 씬 폴더의 `best.pt` 를 자동으로 찾는다.

2. **Isaac 을 검사 스테이션까지 켜서 띄운다** — `--no-vision-station` 과 `--headless` 를 빼고, 결과물이 모일 곳을 지정한다.

```bash
export ROS_DOMAIN_ID=101
export PYTHONUNBUFFERED=1
export SMARTFARM_STATION_OUT=/home/rokey/ROKEY_P3_A1/cobot3_ws/src/smart_farm_navigation/results/station_$(date +%Y%m%d_%H%M)
isaac_python /home/rokey/ROKEY_P3_A1/cobot3_ws/isaacpjt/smart_farm/runtime/standalone_app.py --autoplay 2>&1 | tee /home/rokey/ROKEY_P3_A1/cobot3_ws/src/smart_farm_navigation/results/isaac_$(date +%Y%m%d_%H%M).txt
```

`SMARTFARM_STATION_OUT` 폴더에 검사 사진, `station_results.json`, 실패 시 `yolo_worker.err` 가 쌓인다.

3. **7-3 이후로는 명령을 더 보내지 않는다** — 트레이가 벨트에 놓이면 컨베이어 반송 → 이송 프레임 밀어 넣기 → YOLO 검사 → 노랑·갈색 솎아내기(SortBin 1/2 번갈아) → 재검사 → 배출이 자동으로 이어진다. 팀 기준 전 구간 약 4분 19초(시뮬 시간)다. Isaac 터미널에 `[컨베이어]`·`[솎아내기]`·`[비전]` 줄이 순서대로 찍히는지만 본다.

돌발상황(카터가 통로를 나올 때 작업자가 앞을 막았다 비킴)을 넣으려면 2 의 명령에 `--human-crossing` 을 붙인다. 첫 전 구간 실측에서는 붙이지 않는다.
