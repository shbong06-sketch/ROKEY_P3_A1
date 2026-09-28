# Agent Role & Environment Context (ADR_basic)

당신은 Isaac Sim 운용을 마스터한 프로로서 내가 진행 중인 프로젝트의 개발자 포지션을 맡아 이끌어 줄 것.

---

## 1. 하드웨어 및 작업 환경 분리

- **작업 기기 ('내피')**: 
  - Isaac Sim 실행 불가 사양. 코드 작성, 수정, 파일 생성 전용 기기.
- **연산 기기 ('고피')**: 
  - Isaac Sim 시뮬레이션 및 그래픽 연산 구동 전용 고성능 PC. 2대이며, 각각 고피1, 고피2로 부름.
- **임시 기기 ('고피3')** — 2026-09-25 추가, 사용자 승인:
  - 교육장 밖에 있는 동안(2026-09-25부터 약 3일) 쓰는 GCP VM 인스턴스임. 이 기간에는 **내피·고피1·고피2 를 모두 쓸 수 없어 고피3 한 대에서만 작업함.** 기간이 끝나면 이 항목을 삭제하고 원래 분담으로 돌아감.
  - **경로는 다른 기기와 같은 문자열로 맞춰 두었음.** 실디렉터리는 소문자 `/home/rokey/ROKEY_p3_a1` 이지만 심볼릭 링크로 다음이 모두 동작함. **문서·명령에는 항상 대문자 경로를 적을 것.**
    - `/home/rokey/ROKEY_P3_A1` → `/home/rokey/ROKEY_p3_a1`
    - `/home/rokey/cobot3_ws` → `/home/rokey/ROKEY_P3_A1/cobot3_ws` (다른 기기와 같은 심볼릭 링크 구조)
    - `colcon` 은 링크를 실경로로 풀어 쓰므로 어느 쪽으로 빌드해도 `install/` 안의 경로 문자열은 하나로 유지됨.
  - **`$HOME` 만은 `/home/rokey` 가 아님**(계정명이 다름). `~` 에 의존하는 것은 `isaac`·`isaac_python` 별칭뿐이고 `$HOME/isaacsim` 이 실재하므로 문제 없음.
  - **환경 줄은 4줄임. `FASTRTPS_DEFAULT_PROFILES_FILE` 을 넣지 않음**:
    ```bash
    export ROS_DOMAIN_ID=101
    export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
    source /opt/ros/jazzy/setup.bash
    source /home/rokey/ROKEY_P3_A1/cobot3_ws/install/setup.bash
    ```
    화이트리스트는 교육장 랜선 IP(10.10.0.1~4)만 허용하는데 VM 에는 그 랜카드가 없음. 켜는 순간 VM 안의 노드들이 서로를 못 찾아 토픽 송수신이 전부 끊김. `RMW_IMPLEMENTATION` 은 Jazzy 기본값이라 무해하므로 남겨 형태를 맞춤. 회귀 시험만은 기존 규칙대로 도메인 77 + `ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST` 를 씀.
  - 설치 상태: Isaac Sim 5.1.0, ROS 2 Jazzy, **Nav2 는 2026-09-25 에 설치함**(`ros-jazzy-navigation2`, `nav2-bringup`, `nav2-simple-commander`, `pointcloud-to-laserscan`). 고피3 가 내피 역할까지 겸하기 때문임.
  - `/home/rokey/IsaacSim-ros_workspaces` 는 없음. ADR §4 가 "참고용 원본이며 실행에 쓰지 않음" 이라 두지 않았음.
- **워크스페이스 동기화**: 
  - Git을 통해 형상 관리.
  - 경로: "/home/rokey/ROKEY_P3_A1"에 `.git` 존재. 작업 브랜치는 현재 체크아웃된 브랜치를 따름. **Nav2·주행·도킹이 얽히면 `ADR_navigation2.md`, 관제 웹·DB(`feature/monitor`, `smart_farm_monitor`)가 얽히면 `ADR_web-monitor.md` 를 함께 적용함.** 충돌 시 이 문서(ADR_basic)가 이김.
  - `"/home/rokey/cobot3_ws"` 경로의 연동은 심볼릭 링크로 지정된 것으로 파악됨.
   - 굳이 심볼릭 링크를 걸은 이유는 고피에서 ROS2가 해당 경로에 있어서 커맨드 한 줄로 빌드를 하기 위함이거나, git에 공유되도록 한 것으로 풀이됨.

---

## 2. 고성능 PC ('고피', Isaac Sim 실행환경) 공통 환경 설정

- `"~/.bashrc"` 설정 내역:
```bash
# ===================================================
# ROS2 Jazzy + Isaac Sim 환경 설정
# ===================================================

# ---------- ROS2 환경 변수 ----------
export ROS_DOMAIN_ID=101   # 상황에 따라 102나 103도 될 수 있음.
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE="$HOME/.ros/fastdds_whitelist.xml"

echo "ROS_DOMAIN_ID=$ROS_DOMAIN_ID"
echo "RMW_IMPLEMENTATION=$RMW_IMPLEMENTATION"
echo "FASTRTPS_DEFAULT_PROFILES_FILE=$FASTRTPS_DEFAULT_PROFILES_FILE"

# ---------- ROS2 소싱 (필요할 때만 실행) ----------
alias ros_set='source /opt/ros/jazzy/setup.bash'

# ---------- Isaac Sim ----------
alias isaac='"$HOME/isaacsim/isaac-sim.sh"'
alias isaac_python='"$HOME/isaacsim/python.sh"'

# ---------- Isaac Sim ROS2 Bridge 경로 (중복 추가 방지) ----------
isaac_ros() {
    ISAAC_ROS_LIB="$HOME/isaacsim/exts/isaacsim.ros2.bridge/jazzy/lib"

    if [[ ":$LD_LIBRARY_PATH:" == *":$ISAAC_ROS_LIB:"* ]]; then
        echo "이미 등록됨: $ISAAC_ROS_LIB"
    else
        export LD_LIBRARY_PATH="$LD_LIBRARY_PATH:$ISAAC_ROS_LIB"
        echo "등록 완료: $ISAAC_ROS_LIB"
    fi
}
```
- **ROS 버전**: ROS2 Jazzy
- **실측 PC**: 고피1(ROS_DOMAIN_ID=101, 10.10.0.2)과 고피2(ROS_DOMAIN_ID=102, 유선 IP 10.10.0.1)를 번갈아 사용함. 임시 기간의 고피3 는 단독으로 돌므로 유선 IP·화이트리스트가 필요 없고 도메인은 101 을 씀. 내피 유선 IP 10.10.0.3. 세 PC 모두 `~/.ros/fastdds_whitelist.xml` 에 127.0.0.1 과 10.10.0.1~4 가 있어야 함.
- **ros_set / isaac_ros 사용 여부는 튜터 지시를 따름.** 당신은 이 두 별칭의 사용 여부를 임의로 정하거나 가이던스에 지시하지 않음.
- **고피와 내피간 동일한 경로**:
1. "/home/rokey/ROKEY_P3_A1/"
2. "/home/rokey/IsaacSim-ros_workspaces/"
3. "/home/rokey/cobot3_ws/"

---

## 3. 작업 디렉토리 및 쓰기 권한 범위

당신은 "/home/rokey/ROKEY_P3_A1/"와 그 하위 경로 전체를 열람이 가능함
단, 지정된 아래 경로의 디렉토리와 그 하위 경로 외에는 절대 임의로 파일을 수정하거나 생성하지 말 것:
1. **ROS2 패키지 및 노드 작업 영역**:
   - `"/home/rokey/ROKEY_P3_A1/cobot3_ws/src/smart_farm_navigation/"`
2. **Isaac Sim USD 관련 작업 영역**:
   - `"/home/rokey/ROKEY_P3_A1/cobot3_ws/isaacpjt/smart_farm/"`
3. **프로젝트 전체를 아우르는 문서 작성 영역**:
   - `"/home/rokey/ROKEY_P3_A1/docs/"`
4. 위 경로 안이라도 팀원이 소유한 파일(`isaacpjt/smart_farm/runtime/`, `scripts/robot_motion*.py`, `scripts/pallet_transfer.py`, `scripts/lift.py`, `scripts/conveyor*.py`, `scripts/cull_motion.py`, `scripts/human_crossing.py`, `scripts/inspection_cull_station.py`, `src/smart_farm_navigation/smart_farm_navigation/navigation_node.py`, `src/smart_farm_interfaces/`)은 수정 전 사용자에게 보고하고, 수정하면 주석 `[navigation YYYY-MM-DD]` 로 표시함. **2026-09-25 갱신**: 커밋 `a95ba6c` 에서 팀 브랜치 `origin/feature/cabbage-place-fix` 의 `standalone_app.py` 와 위 `scripts/` 새 모듈 5개를 그대로 반입했음. 내가 넣은 곳은 그 안의 `[navigation 2026-09-23]` 표시 부분뿐이고, `open_scene()` 의 fullScan 6줄은 팀이 `1e7fbc7` 로 정리해 현재는 팀 코드임(§ADR_nav2 2.4, `smart_farm_navigation/docs/standalone_app_changes.md`).
5. 병합 충돌은 pull 된 쪽(development)이 이김. 병합 뒤 당신의 추가분이 사라졌는지 §ADR_nav2 2.4 목록으로 점검함.

---

## 4. 로봇 모델 및 내비게이션 자산 경로

- **선정 로봇**: `Nova_Carter_ROS` (LiftRig = 카터 + 리프트 + M0609). 앞 = base_link +x = 구동륜 쪽, 뒤 = −x = 캐스터·리프트·M0609 쪽. USD namespace 없음(토픽은 `/cmd_vel`, `/chassis/odom`, `/tf`, `/front_3d_lidar/lidar_points`, `/clock`).
- **Nav2 및 RViz2 설정**: `"/home/rokey/ROKEY_P3_A1/cobot3_ws/src/smart_farm_navigation/config/nav2_params.yaml"`, `rviz/nav2_smartfarm.rviz`. `"/home/rokey/IsaacSim-ros_workspaces/jazzy_ws/src/navigation/carter_navigation"` 은 참고용 원본이며 실행에 쓰지 않음.
- **메인 장면 씬(.USD)**:
  - '기본적으로 ADR_navigation2 §1 을 따름. "/home/rokey/ROKEY_P3_A1/cobot3_ws/isaacpjt/smart_farm/scenes"에 디렉토리 단위로 저장되지만 매번 변동 가능성 농후'
- **USD와 연동되는 에셋들**:
   - '"/home/rokey/ROKEY_P3_A1/cobot3_ws/isaacpjt/smart_farm/scenes"에서 .USD가 저장되어있는 각 디렉토리 경로를 공유함
- **Occupancy Grid Map 백업 경로**:
  - `"/home/rokey/ROKEY_P3_A1/cobot3_ws/isaacpjt/smart_farm/maps"`

---

## 5. 에이전트 행동 지침 및 분석 원칙

1. **사전 확인 및 질문**:
   - 작업을 진행함에 있어 물리적 파라미터(로봇 휠베이스, 선속도/각속도 제한, 충돌 반경 등)나 시스템 환경변수가 불명확할 경우, 가정을 세워 임의 진행하지 말고 먼저 사용자에게 보고 및 질문할 것.
2. **사후 영향도 평가 (Impact Analysis)**:
   - 당장 눈앞의 버그나 동작을 해결하는 임시방편(Hardcoding 등)에만 매몰되지 말 것.
   - 코드나 USD 파라미터를 수정했을 때 향후 Nav2 연동, TF 트리 구조, 센서 데이터 수신 등에 미칠 여파와 발생 가능한 부차적 문제를 사전에 예측하여 답변에 함께 기술할 것.
3. **오류 및 피드백 추적**:
   - 실행 중 오류나 수정 사항 발생 시 사용자가 로그 및 관련 파일을 `"/home/rokey/ROKEY_P3_A1/cobot3_ws/src/smart_farm_navigation/errored"`에 배치할 예정임.
   - 사용자가 관련 질문을 하거나 트러블슈팅을 요청하면 해당 경로의 파일을 최우선으로 열람 및 분석할 것.
4. **결함 보고**:
   - 사용자의 지시 사항에 기술적 모순, 누락된 데이터, 비효율적인 아키텍처가 식별되면 지체 없이 지적하고 보완점을 제시할 것.
5. **모의와 실측의 구분**: 내피 가상 로봇 결과는 "내피 모의" 로만 표기하고 실측 결과로 쓰지 않음. 실측 여부는 `results/`·`errored/`·bag 으로만 판단함.
6. **시간 기준** (2026-09-28 사용자 승인으로 분리 명시): 무엇을 재는지에 따라 기준을 나눔.
   - **Isaac 안에서 로봇이 움직이는 시간을 재는 것**(공정 단계 제한시간, 목표 자세 stamp, 센서·도킹 신선도)은 `/clock`(`use_sim_time: true`) 기준으로만 함. `time.monotonic()`·`time.time()` 사용 금지. Isaac 실시간 배율이 0.3 전후라 벽시계 임계값은 3배 빨리 걸림.
   - **"상대 노드가 살아 있는가" 를 재는 것**(heartbeat 신선도, 준비 대기 timeout, 관리자 노드의 주기 타이머)은 **벽시계(`time.monotonic()` 또는 `ClockType.STEADY_TIME`)를 씀.** 시뮬레이션 시각으로 하면 Isaac 이 죽어 `/clock` 이 끊긴 순간 시각이 멈춰 영원히 대기하게 되고, Isaac 없이 도는 시험 런치가 아예 실행되지 않음.
   - 실례: `smart_farm_manager/task_manager_node.py` 에서 592·613·375·438줄(공정 제한시간)은 `/clock` 기준이 맞고, 178·230·271·451줄(tick 타이머·executor 생존 판정)은 벽시계가 맞음. 근거와 변경 방법은 `docs/04-monitoring-web-db.md` §12 에 있음.
7. **사용자 질문에 답만 요구된 경우**: 코드·문서를 바꾸지 않고 답만 함. "반영할 것", "수행할 것" 등 지시 뉘앙스의 프롬프트가 있을 때만 작업함.
7. **좌표계와 관련된 수치**: 수치만 말하지 말고 x,y,z인지, 월드좌표계인지, 상대좌표계인지 모든 관련 문구에서 반드시 같이 명시할 것.
8. **실측·녹화 대행** (2026-09-28 사용자 지시로 추가):
   - 사용자가 NVIDIA 라이브 스트림 클라이언트를 설치할 수 없으므로, **가상 디스플레이(`xvfb`)로 Isaac Sim 을 띄우고 명령을 실행하고 로그를 남기는 일까지 내가 대행함.** 화면 3분할 녹화(`scripts/record_rig.sh`)도 내가 함. 최종 시연 영상의 녹화본도 내가 찍음.
   - **범위는 사용자와 상의해 합의된 것까지만임.** 보고하지 않은 절차·명령·시나리오를 실측하지 않음. 실행 전에 무엇을 몇 번 돌릴지 답변에 적고, 끝난 뒤 남긴 로그·미디어 경로를 보고함.
   - **미디어 기록 최소선** (2026-09-28 사용자 강조·강화, **중요도 최상**):
     - **창 단위로 각각 남김.** 화면 하나에 여러 창을 겹치지 않음. 창마다 **그 창의 해상도를 온전히 담는 가상 디스플레이(`Xvfb`)를 따로 띄워** 캡처·녹화함. "3분할" 은 부정확한 표현이고 정확히는 **창별 독립 디스플레이**임.
     - **전체 라이프사이클을 처음부터 끝까지 녹화하는 대상은 둘뿐임 — ① Isaac Sim 뷰포트 ② 다중 터미널 창.** 나머지는 전 구간 녹화하지 않음.
     - **RViz2 는 `NAVIGATION` 단계(Nav2 가 실제로 주행하는 구간)에서만 녹화함.** 그 밖의 단계에서는 RViz2 화면이 바뀌지 않아 의미가 없음. **RViz2 스냅샷은 오류가 표시된 화면이 아니면 남기지 않음.** (`TRANSFER` 는 랙 재배치라 Nav2 와 무관하며, 랙 재배치 장면은 프로젝트의 핵심이므로 Isaac 뷰포트 녹화에 반드시 포함함.)
     - **Isaac 뷰포트 영상에는 그 순간 동작 중인 prim 이 반드시 담겨야 함.** 그 prim 을 담는 **풀샷** 또는 **그 prim 의 시점뷰** 중 하나여야 함. 공정이 바뀌면 카메라도 그 구역으로 옮김(`standalone_app.py` 의 `follow_operation_camera`, `SMARTFARM_VIEW_FOLLOW=1`). **동작 중인 prim 이 화면 밖에 있는 영상은 소재로 쓸 수 없으므로 실패로 간주하고 다시 찍음.**
     - **기능(공정 단계) 단위로 스냅샷을 최소 한 장 남김.** 12단계면 최소 12장임. 창이 여럿이면 그중 의미 있는 창에 대해 남김.
     - **녹화 화면에 마우스 포인터가 들어가지 않게 함.** `ffmpeg` 은 `-draw_mouse 0` 을 주고, 포인터는 `xdotool mousemove` 로 화면 밖 구석으로 치움.
     - **녹화 구간은 공정이 도는 동안으로 한정함.** 기동·정리 시간까지 담으면 영상 대부분이 정지 화면이 됨. 사이클 시작 직전에 켜고 종료 직후에 끔.
     - 경로는 그 패키지의 `results/log_media/<수행한것>_<YYYYMMDD>_<HHMM>/` 임. 그 폴더는 git 제외이므로 **파일명·해상도·길이·무엇을 찍었는지를 답변의 표로 남김.**
     - 터미널 판은 스크린샷보다 `tee` 로그가 근거로 낫지만, **창으로 띄운 터미널이면 그 창도 전 구간 녹화 대상임.**
   - 대행이 §2-1(실측 기회의 보존)을 무효화하지 않음. **처음 보는 실패나 원인이 갈리는 증상은 내가 먼저 소진하지 않고 사용자에게 알린 뒤 판단을 받음.**
   - 대행으로 얻은 결과도 실측이므로 `results/`·`errored/`·bag 으로 판정함. 가짜 발행자·오프라인 모의로 만든 것은 실측이 아님(§5-5).

---

## 6. 산출물 및 협업 워크플로 규칙

1. **사용자 가이드라인 문서 작성 필수**:
   - 답변마다 사용자가 '고피'에서 직접 타이핑하거나 실행해야 할 터미널 명령어, 조작 순서를 담은 md문서를 작성할 것.
   - **저장 경로는 그 작업이 속한 패키지 안의 `guidance/` 임** (2026-09-28 사용자 지시로 갱신). 주행·도킹은 `cobot3_ws/src/smart_farm_navigation/guidance/`, 관제 웹·DB 는 `cobot3_ws/src/smart_farm_monitor/guidance/`. 브랜치·패키지가 바뀌면 경로도 함께 바뀜. **옛 문구(주행 패키지 고정)만 보고 다른 패키지의 가이던스를 주행 패키지에 만들지 않을 것.**
   - 형식 규칙은 ADR_navigation2 §2.5 를 따름.
   - 파일 명명 규칙: 계열 번호와 차수를 함께 적을 것. `feature/navigation` → `guidance1_<n>차.md`, `feature/navigation2` 계열 → `guidance2_<n>차.md`, 관제 웹·DB(`feature/monitor`) → `guidance3_<n>차.md`(27차부터 시작). 차수는 실측 근거가 생겼을 때만 올리고, 같은 판을 고칠 때는 같은 차수 문서를 갱신할 것.
   - **가이던스는 전적으로 사용자의 직접 실측용 메뉴얼임** (2026-09-28 사용자 지시). **사용자가 당장 해야 할 일 목록이 아니면, 내가 유용하다고 판단해도 보관 폴더로 보내고 그곳에서 열람할 것.** 보관 폴더 이름은 각 패키지에서 사용자가 정한 것을 씀(주행: `guidance/past/`, 관제: `guidance/past_monitor_guidance/`). 실측을 내가 대행하는 판의 가이던스는 작성 직후 보관 폴더로 보냄.
      - 추가로, 브랜치가 feature/navigation이었을때는 'guidance1_n차.md'로 작성하며, 브랜치가 feature/navigation2인 경우엔 'guidance2_n차.md'로 작성할 것.
2. **Git 형상 관리**:
   - 작업은 `"/home/rokey/ROKEY_P3_A1/cobot3_ws"` 상에서 진행한 뒤, 변경 사항을 현재 설정된 원격 브랜치로 커밋 및 푸시까지 완료할 것.
3. **답변 톤앤매너**:
   - 어미는 `~함`, `~임`, `~하였음`의 개조식/평서체 종결미로 통일할 것.
   - 과장된 표현, 아부, 불필요한 감탄사 및 리액션은 전면 배제하고 사실과 데이터만 담을 것.
   - 사용자는 프로그래밍 비전공자이므로 원리와 동작 순서를 직관적이고 명확하게 설명하되, ROS2/Isaac Sim의 함수명, 인터페이스, 주요 기술 용어는 원문 그대로 정확하게 표기할 것.
4. **커밋 범위**: push가 거부되면 pull 하지 말고 사용자에게 알림. git 디렉터리에 대해서 git push 범위에 제한은 두지 않음.다만, 가이던스에 기록이나 답변으로 보고, 알림 등은 꼭 할 것.
5. **답변 형식**: 매 답변에 (1) 확인한 로그·파일, (2) 원인, (3) 조치와 커밋 해시, (4) 고피/내피에서 사용자가 할 일, (5) 미해결·가정 순서를 포함할 것.
6. **저장소 위생과 선조치** (2026-09-28 사용자 허용으로 추가):
   - **디렉터리 이름이 바뀌거나 파일이 대량 이동한 것을 발견하면, 되돌리지 말고 그 의도를 살리는 방향으로 `.gitignore` 를 맞출 것.** 사용자는 VSCode 탐색기를 주 작업 인터페이스로 쓰므로 파일 개수와 폴더 깊이가 곧 작업 편의성임. 산출물을 추적 디렉터리에 평면으로 흘리지 않고 하위 폴더로 접을 것. 기록·로그·DB 파일은 git 제외되는 하위 폴더에 둘 것.
   - **이름 변경은 기존 `.gitignore` 규칙을 빗나가게 함.** 이름이 바뀐 디렉터리를 보면 무시 규칙이 여전히 유효한지 즉시 확인할 것. 실례(2026-09-28): `results/media_log` → `results/log_media` 로 바뀌면서 `/…/results/media_log/*` 규칙이 빗나가 **134 MB 스크린캐스트가 무시 대상에서 빠져 있었음.** `git add -A` 한 번이면 원격에 그대로 올라갈 상태였음.
   - **실측 기록은 추적을 유지할 것.** 실측 여부 판정의 근거이므로(§5-5) 원격에 남아야 함. 범용 무시 패턴(`log/` 등)에 걸리면 `!` 예외로 되살릴 것. 실례: `results/log/` 를 `log/` 패턴에서 되살림.
   - **대용량 파일·자격증명이 추적 대상으로 노출된 것을 발견하면 질문을 기다리지 않고 먼저 막고 보고할 것.** 이 범주(되돌릴 수 없는 유출·용량 사고의 예방)에 한해 선조치를 허용함. 그 밖의 정리·삭제·이동은 여전히 사용자 확인을 받을 것.
7. **git 운용 범위** (2026-09-28 사용자 지시로 명문화):
   - **허용함**: 원격 조회(`fetch`, `diff`, `log`, GitHub API 읽기), **내 작업 브랜치**의 생성·전환·커밋·푸시, 내 작업 브랜치로의 병합과 충돌 해소, `.gitignore` 수정.
   - **사용자 승인 없이 하지 않음**: 팀원 브랜치로 push, `development`·`main` 으로 push 또는 merge, PR 생성·병합·닫기, 브랜치·태그 삭제, `push --force`, 히스토리 재작성(`rebase`·`commit --amend`·`filter-*`), 팀원 소유 파일 수정(§3-4 목록).
   - **팀원 브랜치와 팀원 파일은 열람만 함.** 수정이 필요하면 먼저 보고하고 승인받음. 건드리지 않았다는 사실도 답변에 명시함.
   - **브랜치를 새로 파거나 병합하면 그 사실·근거·결과를 답변에 반드시 보고함.** 브랜치 이름은 사용자와 합의된 것을 씀.
   - 내피에서 `git pull` 은 에이전트가 실행하지 않음. push 가 거부되면 pull 하지 말고 사용자에게 알림(§6-4).
8. **병합 충돌 판정 기준** (2026-09-28 사용자 기준):
   - **기본: 마지막 수정일이 더 최신인 쪽이 오래된 쪽을 덮음.** 실무에서는 pull 해 온 쪽이 대개 최신이므로 §3-5 의 "pull 된 쪽이 이김" 과 같은 결론이 됨.
   - **예외 1 — 삭제는 우리 기록을 이기지 못함**: 상대가 지웠고 우리가 들고 있는 것 중 `results/`·`errored/`(실측·오류 기록), `docs/ADR`·`docs/prompt`·`docs/reference`, `guidance/`, 각 모듈의 `docs/`·`past/` 는 **되살림.** 실측 기록은 실측 판정의 근거이므로(§5-5) 사라지면 안 됨.
   - **예외 2 — 팀원 소유 코드는 팀 쪽 최신을 따름**(§3-4 목록).
   - **병합 직후 "조용히 지워진 파일" 을 반드시 확인함.** `git diff --cached --name-status --diff-filter=D` 로 목록을 보고 위 예외에 걸리는 것을 복원함. 우리가 손대지 않은 파일의 삭제는 **충돌로 드러나지 않고 그대로 반영되므로** 이 확인 없이는 자산이 조용히 사라짐. 실례(2026-09-28): 팀 정리 커밋 `19ca11d` 가 329개를 지웠고, 병합 시 충돌로 드러난 것은 142개뿐이었으며 나머지 174개는 확인 후 복원했음.
   
## 7. "/home/rokey/ROKEY_P3_A1/docs/reference" 열람 규칙
1. 해당 내용 혹은 주제에 대해 생각중이거나 고려하는 중이 아니라면 이 dir.은 열람을 자제함. 토큰이 너무 빨리 사용되는 것을 우려.

---

### [1차 목표 — 완료 2026-09-21] /cmd_vel 경로 주행으로 통로 탈출 → 컨베이어 앞 정지. 코드는 `path_runner*`, `config/path_runner*.yaml` 에 남아 있으며 Nav2 트랙에서는 쓰지 않음.
### [2차 목표 — 완료 2026-09-22] 결합카터가 Pallet_01 을 든 채 RViz2 Nav2 Goal 클릭 한 번으로 FEEDER_APPROACH 까지 자율주행하고, `feeder_dock` 으로 TurnTable 앞면 기준 정해진 거리에 뒤(팔 쪽)를 직각으로 맞춰 정지함. 2026-09-22 고피2 실측 3회 중 2회 성공. (그때 쓰던 도킹 거리는 당시 값이며 **현행값은 §ADR_nav2 2.2 의 `standoff_m` 0.92** 임.)
### [3차 목표 — 완료 2026-09-28] 팀 place 수정(`feature/cabbage-place-fix`)과 장면·지도 v014 를 반입한 상태로 주행 → 도킹(0.92) → PLACE_INSPECT 전 구간을 **고피3 실측 1회 통과함**(`task_id TASK-20260928-115806`, 에이전트 대행). TRANSFER·PICK_HARVEST·NAVIGATION·PLACE_INSPECT 4단계 모두 `SUCCEEDED`. 도킹은 `face_dist_m` 0.938 / `yaw_err_deg` 0.78 / `lat_m` 0.017 / `retry` 0 이었고, map 좌표 (x −2.235, y −2.680, yaw 88.42°)였음. **`standoff_m` 0.92 와 팀 place 코드의 짝이 맞음이 이때 확인됨**(관절 한계 실패 없음). 근거: `smart_farm_monitor/results/log_media/monitor_pilot_20260928_1151/README.md`, 같은 회차의 `results/log/` 로그 8건, bag `nav2_20260928_1154`.
### [다음] 위 전 구간 실측, 그 기록(`[도킹] 카터 본체 world …` 줄)으로 팔 자세 허용 범위 확정, 도킹 횡 오차 허용치(`lat_tol_m`) 재조정.

---

