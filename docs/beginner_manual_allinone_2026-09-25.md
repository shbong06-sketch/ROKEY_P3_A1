# 양배추 스마트팜 올인원 — 초심자용 따라하기 매뉴얼 (2026-09-25)

> 이 문서는 **새 PC 에서 처음부터(배경 세팅부터)** 따라 해 보며 검증하기 위한 매뉴얼이다.
> 순서대로 하면 오늘(2026-09-25) 만든 것 전부 — 양배추 씬, 올인원 실행(수확 → Nav2 주행 → 컨베이어 → 비전 검사 → 솎아내기 → 배출),
> 공정 녹화, Nav2·ROS2 통신 녹화 — 를 다시 돌릴 수 있다.
>
> - 각 단계는 **목적 → 명령(복사해서 붙여넣기) → 성공하면 보이는 것 → 안 되면(트러블슈팅)** 순서로 적었다.
> - 경로는 작성 PC 기준(`D:\isaacsim`, `D:\smartfarm-sim`)이다. 다른 드라이브를 쓰면 **스크립트 안의 경로도 같이** 바꿔야 한다(스크립트에 경로가 박혀 있음).
> - "**확인 필요**" 라고 적은 곳은 작성 PC 에서 실제로 확인하지 못한 부분이다. 새 PC 테스트 때 결과를 여기에 채워 넣으면 된다.
> - 명령 표기: `PS>` = Windows PowerShell, `$` = WSL(Ubuntu) 터미널. 기호 자체는 입력하지 않는다.

---

## 목차

0. [준비물·사양·폴더 구조](#0-준비물사양폴더-구조)
1. [환경 설치](#1-환경-설치)
2. [양배추 씬 만들기 · 경량 카터](#2-양배추-씬-만들기--경량-카터)
3. [올인원 실행 (한 줄 명령)](#3-올인원-실행-한-줄-명령)
4. [비전 검사 · YOLO · 솎아내기 · 푸셔](#4-비전-검사--yolo--솎아내기--푸셔)
5. [녹화 (공정 카메라 · 검출 박스 · Nav2 RViz · ROS2 통신)](#5-녹화)
6. [깃 업로드 규칙](#6-깃-업로드-규칙)
7. [트러블슈팅 총정리](#7-트러블슈팅-총정리)
8. [용어 설명](#8-용어-설명)
9. [사람 돌발상황 + 바닥 노란 차선 주행](#9-사람-돌발상황--바닥-노란-차선-주행)
10. [진행 중 작업 메모](#10-진행-중-작업-메모)

---

## 0. 준비물·사양·폴더 구조

### 0-1. 사양 (작성 PC 에서 검증한 구성)

| 항목 | 작성 PC | 비고 |
|---|---|---|
| OS | Windows 11 Pro (10.0.26100) | |
| GPU | NVIDIA GeForce RTX 3060 **12 GB**, 드라이버 581.57 | RTX 필수(Isaac Sim 렌더링). VRAM 12 GB 에서 녹화까지 동작 |
| Isaac Sim | **5.1.0** standalone (Windows zip), 설치 위치 `D:\isaacsim` (약 15 GB) | 버전 문자열 `5.1.0-rc.19+release.26219` |
| WSL2 | Ubuntu **24.04.5 LTS** | 배포판 이름 `Ubuntu-24.04` (스크립트에 이 이름이 박혀 있음) |
| ROS 2 | **Jazzy** (WSL 안) + Nav2 (`ros-jazzy-navigation2`) | Isaac 안의 ROS 브리지도 Jazzy |
| WSL 메모리 | `.wslconfig` 에 memory=12GB, processors=8 | 아래 1-3 |
| 디스크 | Isaac zip 7.9 GB + 설치 15 GB + 씬 0.5 GB + 공유 zip 73 MB + 녹화 여유 | **여유 40 GB 이상 권장** |
| 인터넷 | 설치 때 필요 (apt, pip, GitHub) | |

### 0-2. 받아야 하는 파일

| 파일 | 어디서 | 작성 PC 위치 |
|---|---|---|
| `isaac-sim-standalone-5.1.0-windows-x86_64.zip` | NVIDIA Isaac Sim 다운로드 페이지 | `D:\isaac-dl\` |
| 팀 v013 씬 (`Collected_smartfarm_v013_room_core.usd` + `SubUSDs/` 등이 든 폴더) | 팀 공유 (작성 PC 에는 `Downloads\Collected_smartfarm_v013_room_draft.zip` 이 있음 — 이것이 원본인지 **확인 필요**) | `D:\v013_extract\Collected_smartfarm_v013\` |
| 공유 zip `cabbage_smartfarm_v014.zip` (양배추 씬 v014 · 에셋 · best.pt · 경량 카터 · 영상) | 작성자 공유 (Slack 등) | `D:\smartfarm-sim\share\` |
| 코드 | GitHub `shbong06-sketch/ROKEY_P3_A1`, 브랜치 **`feature/cabbage-place-fix`** | `D:\smartfarm-sim\ROKEY_P3_A1\` |
| Nav2 쪽 팀 코드 | 같은 저장소, 브랜치 `feature/Inspection-Place-nav2` (WSL 에 따로 clone) | WSL `~/rokey/ROKEY_P3_A1` |

### 0-3. 폴더 구조 (작성 PC 기준, 이대로 만들면 스크립트 수정 없이 돈다)

```
D:\isaacsim\                                  Isaac Sim 5.1 (python.bat, isaac-sim.bat, kit\python\python.exe)
D:\v013_extract\Collected_smartfarm_v013\     팀 v013 씬 폴더 (+ 공유 zip 을 여기에 풀어 v014 양배추 씬 추가)
D:\smartfarm-sim\
 ├─ ROKEY_P3_A1\                              GitHub 저장소 (Windows 쪽, 브랜치 feature/cabbage-place-fix)
 │   └─ cobot3_ws\isaacpjt\smart_farm\
 │       ├─ runtime\standalone_app.py         팀 올인원 (오늘 추가분은 "[올인원 2026-09-25]" 표시)
 │       ├─ scripts\conveyor.py, conveyor_rollers.py, cull_motion.py   (팀 코드, 수정 안 함)
 │       ├─ scripts\inspection_cull_station.py 비전 검사·솎아내기·푸셔·YOLO 워커 (오늘 신규)
 │       ├─ tools\cabbage\make_cabbage_scene.py 양배추 씬 생성
 │       ├─ tools\allinone_live\            ← 올인원 한 줄 실행 도구 (run_allinone.ps1, wsl\*.sh·*.py) — 저장소에 있음
 │       └─ scenes\Collected_smartfarm_v014\  ← 씬 폴더 (작성 PC 는 D:\v013_extract\... 로 가는 junction)
 ├─ scripts\cabbage\, scripts\wsl\             작성 PC 의 개발용 도구 (저장소 밖. 실행에는 필요 없음)
 ├─ team_ref\pylib\                           Isaac 파이썬용 추가 라이브러리 (pyyaml, numpy, scipy, pillow, trimesh, usd-core)
 ├─ team_ref\pylib_yolo\                      YOLO 용 (ultralytics, opencv-python-headless 등)
 ├─ team_ref\pylib_ffmpeg\                    imageio-ffmpeg (Windows 용 ffmpeg.exe)
 ├─ assets\cabbage_pallet_6\                  양배추 트레이 에셋 원본
 ├─ share\                                    공유 zip, 공유 폴더
 └─ out\                                      실행 결과·로그·녹화가 쌓이는 곳 (실행마다 하위 폴더)
```

> **중요:** 실행 도구는 저장소 `smart_farm\tools\allinone_live\` 에 있고 경로는 스크립트 위치 기준이다.
> Isaac 설치 위치·WSL 배포판 이름만 옵션으로 준다: `run_allinone.ps1 -IsaacDir D:\isaacsim -Distro Ubuntu-24.04` (둘 다 기본값).

---

## 1. 환경 설치

### 1-1. Isaac Sim 5.1 설치 (Windows)

**목적:** 시뮬레이터 설치.

```powershell
PS> Expand-Archive -Path D:\isaac-dl\isaac-sim-standalone-5.1.0-windows-x86_64.zip -DestinationPath D:\isaacsim
PS> D:\isaacsim\post_install.bat
PS> D:\isaacsim\isaac-sim.bat
```

- zip 을 풀었을 때 `D:\isaacsim\isaac-sim.bat`, `D:\isaacsim\python.bat` 이 바로 보여야 한다(한 단계 더 들어간 폴더가 생기면 한 단계 올려서 옮긴다).
- `post_install.bat` 을 작성 PC 에서 실행했는지는 **확인 필요** (파일은 존재). 실행해도 해는 없다.

**성공:** Isaac Sim 창이 뜬다(첫 실행은 셰이더 캐시 때문에 5~15분 걸릴 수 있음).

**안 되면:**
- 창이 안 뜨고 꺼짐 → GPU 드라이버 최신화, `D:\isaacsim\kit\logs\` 의 최신 로그 확인.
- 경로에 한글·공백이 있으면 문제가 생길 수 있으니 `D:\isaacsim` 처럼 짧은 영문 경로를 쓴다.

### 1-2. Isaac 파이썬용 추가 라이브러리 (Windows)

**목적:** 도구 스크립트·ROS 중계기·YOLO·영상 제작에 필요한 파이썬 패키지를 Isaac 파이썬과 **섞이지 않게** 별도 폴더에 설치한다.

```powershell
PS> D:\isaacsim\kit\python\python.exe -m pip install --target D:\smartfarm-sim\team_ref\pylib pyyaml numpy scipy pillow trimesh usd-core
PS> D:\isaacsim\kit\python\python.exe -m pip install --target D:\smartfarm-sim\team_ref\pylib_yolo ultralytics opencv-python-headless --no-deps
PS> D:\isaacsim\kit\python\python.exe -m pip install --target D:\smartfarm-sim\team_ref\pylib_yolo typing_extensions sympy networkx jinja2 filelock fsspec psutil tqdm requests matplotlib ultralytics-thop polars pandas
PS> D:\isaacsim\kit\python\python.exe -m pip install --target D:\smartfarm-sim\team_ref\pylib_ffmpeg imageio-ffmpeg
```

- 작성 PC 에 실제로 설치된 버전: ultralytics 8.4.158, opencv-python-headless 5.0.0.93, numpy 2.4.6, scipy 1.17.1, pillow 12.3.0, trimesh 5.1.0, usd-core 26.8.
- `pylib` 줄의 패키지 목록은 작성 PC 폴더 내용에서 거꾸로 적은 것이라 **정확한 설치 명령은 확인 필요** (작성 PC 에서 확인된 명령은 `pyyaml`, `imageio-ffmpeg`, `opencv-python-headless --no-deps`, 세 번째 줄 목록).
- **torch 는 설치하지 않는다.** Isaac 에 들어 있는 torch(`D:\isaacsim\exts\omni.isaac.ml_archive\pip_prebundle`)를 같이 쓴다. 그래서 ultralytics 를 `--no-deps` 로 설치한다(의존성 설치 시 torch 를 새로 받아 충돌).

**성공:** 아래가 버전을 출력하면 OK.

```powershell
PS> $env:PYTHONPATH="D:\smartfarm-sim\team_ref\pylib_yolo;D:\isaacsim\exts\omni.isaac.ml_archive\pip_prebundle;D:\smartfarm-sim\team_ref\pylib"; D:\isaacsim\kit\python\python.exe -c "import ultralytics, torch, cv2; print(ultralytics.__version__, torch.__version__, cv2.__version__)"
```

**안 되면:**
- `ModuleNotFoundError: X` → 해당 패키지를 `pylib_yolo` 에 `--target` 으로 추가 설치.
- numpy 버전 충돌 경고 → `pylib` 와 `pylib_yolo` 순서를 위 PYTHONPATH 순서 그대로 둔다.

### 1-3. WSL2 + Ubuntu 24.04 (Windows)

**목적:** Nav2 를 돌릴 리눅스 환경.

```powershell
PS> wsl --install -d Ubuntu-24.04
PS> wsl -l -v
```

- 처음 실행 시 Ubuntu 사용자 이름/비밀번호를 만든다(재부팅이 필요할 수 있음).
- `wsl -l -v` 에 `Ubuntu-24.04   ...   2` (VERSION 2) 가 보이면 OK.

**`.wslconfig` 작성** — 파일 `%USERPROFILE%\.wslconfig` (예: `C:\Users\<이름>\.wslconfig`) 에 아래 내용을 그대로 넣고 저장:

```ini
[wsl2]
networkingMode=nat
localhostForwarding=true
memory=12GB
processors=8
```

```powershell
PS> wsl --shutdown
```

- **mirrored 모드를 쓰지 말 것.** mirrored 에서는 WSL 안에서도 늦게 뜬 ROS 노드끼리 데이터가 안 와서 Nav2 가 멈췄다(트러블슈팅 N2).

### 1-4. ROS 2 Jazzy + Nav2 + 팀 Nav2 패키지 (WSL)

**목적:** Nav2 와 팀 `smart_farm_navigation` 패키지 설치·빌드.

Windows 쪽 저장소(1-6)를 먼저 받은 뒤, 그 안의 설치 스크립트를 WSL 에서 실행한다:

```powershell
PS> wsl -d Ubuntu-24.04 -- bash /mnt/d/smartfarm-sim/ROKEY_P3_A1/cobot3_ws/isaacpjt/smart_farm/tools/allinone_live/wsl/setup_wsl_nav2.sh
```

이 스크립트가 하는 일(파일 `tools/allinone_live/wsl/setup_wsl_nav2.sh`):
1. ROS 2 apt 저장소 등록, `ros-jazzy-ros-base ros-jazzy-navigation2 ros-jazzy-nav2-bringup ros-jazzy-pointcloud-to-laserscan ros-jazzy-rmw-fastrtps-cpp ros-jazzy-rviz2 python3-colcon-common-extensions python3-rosdep python3-yaml` 설치
2. 녹화 도구 `ffmpeg xdotool tmux xterm` 설치
3. `~/rokey/ROKEY_P3_A1` 에 저장소를 **`feature/cabbage-place-fix`** 브랜치로 clone (다른 브랜치는 `BRANCH=... bash setup_wsl_nav2.sh`)
4. `rosdep install` 후 `colcon build --symlink-install --packages-up-to smart_farm_navigation smart_farm_manager`
5. 실행용 환경 파일 `nav2_env.sh` 를 `~/nav2_env.sh` 로 복사 (모든 WSL 스크립트가 이것을 읽는다: Jazzy + 팀 워크스페이스, `ROS_DOMAIN_ID=102`, `RMW_IMPLEMENTATION=rmw_fastrtps_cpp`)

> 수동으로 `ros2 topic list` 등을 칠 때도 먼저 `source ~/nav2_env.sh` 를 해야 같은 도메인(102)을 본다.

**성공:**
```bash
$ source ~/nav2_env.sh
$ ros2 pkg list | grep smart_farm
```
→ `smart_farm_interfaces`, `smart_farm_manager`, `smart_farm_navigation` 이 보이면 OK.

**안 되면:**
- `colcon build` 실패 → `rosdep install --from-paths src --ignore-src -y -r` 를 다시 돌리고 빌드 재시도.
- `sudo` 비밀번호 요구 → 스크립트는 중간에 비밀번호를 물어본다. 터미널에서 직접 `wsl -d Ubuntu-24.04` 로 들어가 `bash /mnt/d/smartfarm-sim/ROKEY_P3_A1/cobot3_ws/isaacpjt/smart_farm/tools/allinone_live/wsl/setup_wsl_nav2.sh` 를 실행하면 입력할 수 있다.

### 1-5. WSL 녹화·화면 도구 (WSL)

**목적:** RViz·통신 화면 녹화(5장)에 필요한 도구.

```powershell
PS> wsl -d Ubuntu-24.04 -u root -- bash -c "apt-get update && apt-get install -y ffmpeg x11-utils xdotool wmctrl tmux xterm ros-jazzy-rqt-graph ros-jazzy-rqt-topic"
```

작성 PC 에 설치 확인됨: ffmpeg, xdotool, tmux, xterm, wmctrl, xwininfo(x11-utils), ros-jazzy-rqt-graph, ros-jazzy-rqt-topic, ros-jazzy-rviz2.
(xdotool·wmctrl·tmux·xterm 을 설치한 정확한 명령은 기록에 없어 위 명령은 **확인 필요** — 패키지 이름은 표준 Ubuntu 이름)

### 1-6. 코드 받기 (Windows 쪽 저장소)

**목적:** 올인원 코드(오늘 브랜치).

```powershell
PS> wsl -d Ubuntu-24.04 -- bash -c "mkdir -p /mnt/d/smartfarm-sim && cd /mnt/d/smartfarm-sim && git clone -b feature/cabbage-place-fix https://github.com/shbong06-sketch/ROKEY_P3_A1.git"
```

- 작성 PC 는 Windows PowerShell 에 `git` 이 PATH 에 없어서 **WSL 의 git** 으로 `/mnt/d/...` 저장소를 다뤘다. Git for Windows 가 있으면 PowerShell 에서 `git clone` 해도 된다.

**성공:** `D:\smartfarm-sim\ROKEY_P3_A1\cobot3_ws\isaacpjt\smart_farm\scripts\inspection_cull_station.py` 가 있다.

### 1-7. 실행 도구 (저장소 `tools\allinone_live\`)

**목적:** 올인원 실행·Nav2 기동·중계기·녹화 도구는 저장소에 들어 있다. 따로 복사할 것은 없다.

| 파일 | 하는 일 |
|---|---|
| `run_allinone.ps1` | 전체 한 줄 실행 (Windows) |
| `run_with_monitor.py` | standalone_app 실행 + 기록(monitor.json) + 카메라 캡처 |
| `floor_lines.py` | 씬 바닥 선 좌표 추출 |
| `wsl\setup_wsl_nav2.sh`, `nav2_env.sh` | WSL 한 번 설치 (1-4), 실행 환경 |
| `wsl\start_nav2_stack.sh`, `ros_tcp_relay.py`, `run_flow.py` | Nav2 기동, Windows↔WSL 중계, 명령 흐름 |
| `wsl\make_nav2_test_params.py`, `lane_planner.py`, `lane_route_bt.xml` | `-Human` / `-Lane` 옵션용 (팀 Nav2 설정 복사본, 차선 경로) |
| `wsl\record_drive.sh`, `comm_panes.sh`, `snap_loop.sh` | RViz + ROS2 통신 화면 녹화 |

- `.sh`·WSL 파이썬은 `.gitattributes` 로 LF 줄끝으로 받는다. 그래도 `$'\r': command not found` 가 나면 `sed -i 's/\r$//' 파일.sh` (트러블슈팅 W6).
- 작성 PC 의 `D:\smartfarm-sim\scripts\` 는 개발 중 쓰던 도구다(저장소 밖). 이 매뉴얼의 다른 절에 나오는 `scripts\cabbage\...` 녹화·영상 도구(10·13번 등)는 작성 PC 에만 있다.

### 1-8. 씬 폴더 준비 (v013 원본 + 공유 zip v014)

**목적:** 올인원이 `--scene` 없이 바로 양배추 씬을 열게 한다.

기본 씬 경로(`standalone_app.py` 의 `CABBAGE_SCENE_PATH`):
`cobot3_ws\isaacpjt\smart_farm\scenes\Collected_smartfarm_v014\Collected_smartfarm_v014_room_core_cabbage.usd`

방법 A — 작성 PC 방식(원본 폴더를 따로 두고 junction 으로 연결):

```powershell
PS> Expand-Archive -Path <팀 v013 zip> -DestinationPath D:\v013_extract
PS> Expand-Archive -Path D:\smartfarm-sim\share\cabbage_smartfarm_v014.zip -DestinationPath D:\v013_extract\Collected_smartfarm_v013 -Force
PS> New-Item -ItemType Directory -Force D:\smartfarm-sim\ROKEY_P3_A1\cobot3_ws\isaacpjt\smart_farm\scenes
PS> cmd /c mklink /J D:\smartfarm-sim\ROKEY_P3_A1\cobot3_ws\isaacpjt\smart_farm\scenes\Collected_smartfarm_v014 D:\v013_extract\Collected_smartfarm_v013
```

방법 B — 씬 폴더에 직접: `scenes\Collected_smartfarm_v014\` 폴더를 만들고 거기에 **v013 폴더 내용 전체** 를 넣은 뒤, 같은 폴더에 공유 zip 을 푼다.

- 공유 zip 안: `Collected_smartfarm_v014_room_core_cabbage.usd`, `assets\cabbage_pallet_6\`, `best.pt`, `SubUSDs\nova_carter_sim_optimized.usd`(경량 카터, 기존 파일 **덮어씀**), `videos\`, 문서 2개.
- 양배추 씬은 같은 폴더의 `Collected_smartfarm_v013_room_core.usd` 와 `SubUSDs\` 를 참조한다. **v013 폴더 내용이 같은 폴더에 있어야 한다.**
- `scenes/` 폴더는 저장소에 올리지 않는다(작성 PC 는 `.git/info/exclude` 에 `cobot3_ws/isaacpjt/smart_farm/scenes/` 추가).
- v013 zip 에 폴더가 한 겹 더 있으면 최종적으로 `...\Collected_smartfarm_v013\Collected_smartfarm_v013_room_core.usd` 가 되도록 맞춘다.

**성공:** `Test-Path` 두 개가 모두 True.
```powershell
PS> Test-Path D:\smartfarm-sim\ROKEY_P3_A1\cobot3_ws\isaacpjt\smart_farm\scenes\Collected_smartfarm_v014\Collected_smartfarm_v014_room_core_cabbage.usd
PS> Test-Path D:\smartfarm-sim\ROKEY_P3_A1\cobot3_ws\isaacpjt\smart_farm\scenes\Collected_smartfarm_v014\best.pt
```

### 1-9. best.pt (YOLO 가중치)

찾는 순서(`inspection_cull_station._yolo_setup`):
1. 환경변수 `SMARTFARM_YOLO_WEIGHTS`
2. **씬 폴더의 `*best*.pt`** ← 공유 zip 을 풀면 여기 들어감 (권장)
3. `smart_farm\models\*.pt`
4. 작성 PC 경로 `C:\Users\kangm\Downloads\best.pt`

YOLO 를 돌리는 파이썬: `SMARTFARM_YOLO_PYTHON` → (`D:\isaacsim\kit\python\python.exe` 가 있으면) Isaac 번들 파이썬 → `python3`.
라이브러리 경로: `SMARTFARM_YOLO_PYTHONPATH` → (Isaac 파이썬이 있으면) `pylib_yolo;ml_archive\pip_prebundle;pylib` (1-2 의 폴더 위치 그대로여야 함).

- best.pt = `romaine3_v011` 가중치, 클래스 dark_green / yellow / brown. 오늘 검사 사진 90장에서 v012 와 90/90 같은 판정.

---

## 2. 양배추 씬 만들기 · 경량 카터

> 공유 zip 에 **완성된 v014 씬이 들어 있으므로 보통은 이 장을 건너뛴다.** 원본 씬이 바뀌었거나 다시 만들어야 할 때만.

### 2-1. 양배추 씬 생성 (`make_cabbage_scene.py`)

**목적:** 원본 v013 씬(로메인)을 건드리지 않고, 옆에 양배추 씬 v014 복사본을 만든다.

```powershell
PS> D:\isaacsim\python.bat D:\smartfarm-sim\ROKEY_P3_A1\cobot3_ws\isaacpjt\smart_farm\tools\cabbage\make_cabbage_scene.py D:\v013_extract\Collected_smartfarm_v013\Collected_smartfarm_v013_room_core.usd D:\smartfarm-sim\assets\cabbage_pallet_6
```

- 인자: `씬USD 에셋폴더 [출력이름]`. 에셋 폴더는 공유 zip 의 `assets\cabbage_pallet_6` 을 써도 된다.
- **Isaac 이 켜져 있으면 끄고 실행한다**(씬 파일이 잠겨 저장 실패, 트러블슈팅 S7).
- 스크립트가 Isaac 을 headless 로 띄워서 저장한다(씬 파일은 Isaac 5.1 의 USD 버전으로 써야 해서 일반 파이썬 불가).
- 선택 환경변수: `SAVED_VIEW_CAMERAS=D:\smartfarm-sim\out\saved_view_cameras.json` (GUI 에서 저장한 시점을 카메라로 굽기, 5-1), `SKIP_MAST=1`(리프트 텔레스코픽 생략), `SKIP_SCENEFIX=1`(벽·통·교차점 수정 생략).

하는 일: 로메인 트레이 → 양배추(랙 트레이는 검사 패턴), 비전 M0609 받침대 +0.146 m, SortBox → 윗면 열린 SortBin_1/2(바닥 5 cm), 벽 통로 윗부분 평판, 컨베이어 교차점 A49 충돌 끔, 카터 리프트 텔레스코픽, 공정 카메라 `/World/ProcessCameras/Cam0~Cam5`, 시험 트레이·Lettuce_1~3 삭제, 카터 속 부품 끔.

**성공:** 씬 폴더에 `Collected_smartfarm_v014_room_core_cabbage.usd` 가 새로 생긴다. 로그 끝에 오류가 없다.

**안 되면:**
- 아무 출력 없이 끝나고 파일이 없음 → `make_cabbage_scene.py.error.txt` (스크립트 옆)를 연다. Kit 종료 때 오류 출력이 사라지는 문제가 있어 파일로 남긴다(트러블슈팅 S8).
- 저장 실패/권한 오류 → Isaac GUI 종료 후 재시도.

### 2-2. 경량 카터 (선택)

**목적:** 카터 파일 193 MB → 57 MB (보이지 않는 몸체 속 부품 제거). 공유 zip 에 이미 경량본이 들어 있다.

```powershell
PS> D:\isaacsim\python.bat D:\smartfarm-sim\scripts\cabbage\run_in_isaac.py D:\smartfarm-sim\scripts\cabbage\12_slim_carter_layer.py <원본>\SubUSDs\nova_carter_sim_optimized.usd <출력>\nova_carter_sim_slim.usd
```

- 작성 PC 는 원본을 `SubUSDs\nova_carter_sim_optimized.usd.orig_backup` 으로 백업하고 경량본으로 교체했다.
- 경량본으로 올인원 전체 실행 성공 확인함(겉모습·동작 같음).

---

## 3. 올인원 실행 (한 줄 명령)

### 3-1. 전체 그림

```
tools\allinone_live\run_allinone.ps1 (Windows, 한 번에 실행)
 ├─ [1/5] 이전 Isaac·중계기 종료, wsl --terminate Ubuntu-24.04
 ├─ [2/5] WSL: start_nav2_stack.sh
 │     ├─ ros_tcp_relay.py wsl  (TCP 서버 127.0.0.1:47100) — Isaac /clock 이 들어올 때까지 대기
 │     ├─ ros2 launch smart_farm_navigation nav2.launch.py use_rviz:=true   (Nav2 + RViz2)
 │     ├─ feeder_dock 재기동 (standoff_m=0.92)
 │     └─ navigation_node.launch.py
 ├─ [3/5] Windows
 │     ├─ D:\isaacsim\python.bat run_with_monitor.py runtime\standalone_app.py --autoplay [--scene ...] [--human-crossing]   (Isaac GUI)
 │     │     ROS_DOMAIN_ID=101, SMARTFARM_ROS_REEXEC=1, PATH 에 Isaac 내장 Jazzy DLL
 │     └─ [READY] 뜨면 ros_tcp_relay.py win (Isaac 내장 rclpy 로 WSL 서버에 접속)
 ├─ [4/5] "navigation_node ready" 까지 대기 (최대 10분)
 └─ [5/5] WSL: run_flow.py → PICK_HARVEST → NAVIGATION FEEDER_DOCK → PLACE_INSPECT
        이후 컨베이어 → 비전 검사 → 솎아내기 → 재검사 → 배출은 Isaac 안에서 자동, "Pallet_01 완료" 까지 대기 (최대 30분)
```

도메인: Windows(Isaac) **101**, WSL(Nav2) **102**. 두 쪽 DDS 가 서로 반쯤 발견하는 문제를 피하려고 일부러 나눴고, 토픽은 `ros_tcp_relay.py` 가 옮긴다.
중계 토픽: Isaac→Nav2 `/clock /tf /tf_static /chassis/odom /chassis/imu /front_3d_lidar/lidar_points /front_2d_lidar/scan /sim_task/status /sim_task/result`, Nav2→Isaac `/cmd_vel /sim_task/command`.

### 3-2. 실행

```powershell
PS> powershell -NoProfile -ExecutionPolicy Bypass -File D:\smartfarm-sim\ROKEY_P3_A1\cobot3_ws\isaacpjt\smart_farm\tools\allinone_live\run_allinone.ps1 -OutDir D:\smartfarm-sim\out\test01
```

옵션:

| 옵션 | 뜻 | 기본값 |
|---|---|---|
| `-OutDir` | 로그·결과 폴더 | `~\smartfarm_runs\allinone_run` |
| `-Scene` | 씬 USD 경로. `DEFAULT` 면 standalone_app 기본(v014, `scenes\Collected_smartfarm_v014\...cabbage.usd`) | `DEFAULT` |
| `-Human` | 사람 돌발상황 (9절) | 끔 |
| `-Lane` | 바닥 노란 차선 주행 (9절) | 끔 |
| `-Standoff` | feeder_dock 도킹 거리(m) | `0.92` |
| `-NoFlow` | Nav2 준비까지만 하고 명령 흐름(run_flow)은 안 보냄 (녹화 준비용) | 끔 |
| `-NoRviz` | RViz2 안 띄움 | 끔 |
| `-IsaacDir` / `-Distro` | Isaac 설치 폴더 / WSL 배포판 이름 | `D:\isaacsim` / `Ubuntu-24.04` |
| `-PyLib` | Windows 쪽 중계기가 `yaml` 등을 못 찾을 때(`relay_win.err`) 추가 라이브러리 폴더 (예: `D:\smartfarm-sim\team_ref\pylib`) | 없음 |

- 이 스크립트 출력을 `| Out-Null`, `| Select-Object` 로 넘기지 말 것. Isaac·중계기 자식 프로세스가 출력 핸들을 잡고 있어 명령이 끝나지 않는다.

**화면에 나오는 순서 (성공):**
```
[1/5] 이전 실행 정리 (Isaac·중계기·WSL ROS)
[2/5] WSL: 중계기 + Nav2(+RViz2) + navigation_node  (Human=False Lane=False)
[3/5] Windows: Isaac Sim GUI + standalone_app (+기록) + 중계기
Isaac READY (NNN s)
[4/5] Nav2 + navigation_node 대기
Nav2 ready (NN s)
[5/5] 흐름: PICK_HARVEST -> NAVIGATION FEEDER_DOCK -> PLACE_INSPECT
[flow] sim command PICK_HARVEST ...
[flow] sim result SUCCEEDED ...
[flow] navigation command FEEDER_DOCK ...
[flow] navigation result SUCCEEDED reason=NONE phase=ARRIVED reached=FEEDER_DOCK
[flow] sim command PLACE_INSPECT ...
[flow] sim result SUCCEEDED phase=RESULT reason=NONE
컨베이어 + 비전 검사·선별 (Pallet_01 완료까지 최대 30분)
  [컨베이어] ... / [비전] ... / [솎아내기] ...
```
Isaac 창에서는: 포크 로봇이 랙에서 트레이를 들고 → 컨베이어 앞으로 주행·도킹 → 벨트에 내려놓음 → 벨트가 비전룸으로 → 주황 판(PlateN)이 로봇 쪽으로 밀기 → 팔이 검사 자세 → 노랑·갈색 포기를 SortBin_1/2 에 번갈아 버림 → 재검사 → 빨강 판(PlateS)이 벨트로 되밀기 → 배출.

소요(시뮬레이션 시간, 녹화 켠 경우 벽시계는 2~3배): PICK 72 s, 주행 37 s, 놓기 9 s, 컨베이어 23 s, 밀기 6 s, 검사 5 s, 솎아내기 3포기 92 s, 재검사 7 s, 되밀기 6 s → **약 4분 19초**.

### 3-3. 로그 보는 법 (`-OutDir` 안)

| 파일 | 내용 | 무엇을 찾나 |
|---|---|---|
| `isaac.log` / `isaac.log.err` | Isaac·standalone_app 출력 | `[READY]`, `[컨베이어]`, `[비전]`, `[솎아내기]`, `Pallet_01 완료`, Traceback |
| `stack.txt` / `stack.err` | WSL 스택 기동 출력 | `nav2 active after`, `navigation_node ready` |
| `nav2.log` | Nav2 launch | `Managed nodes are active`, `Aborting bringup`(실패) |
| `navnode.log` | navigation_node | `goToPose FEEDER_APPROACH`, `result SUCCEEDED` |
| `feeder_dock.log` | 도킹 | `[DONE] {... "status": "SUCCEEDED", "face_dist_m": ...}` |
| `relay_win.log` / `relay_wsl.log` | 중계기 | `msgs /clock=..., /cmd_vel=...` 숫자가 늘어나면 정상 |
| `flow.log` / `flow.json` | 명령 흐름 | `[flow] ... SUCCEEDED` |
| `monitor.json` | 포기·트레이 감시, 카터 궤적 `chassis_track` (run_with_monitor.py) | 포기별 최대 이탈 mm·기울기 |
| `station\station_results.json`, `station_events.json`, `*_yolo.jpg` | 비전 스테이션 결과 | 칸별 판정, 이벤트 시각 |

빠른 확인 명령:
```powershell
PS> Select-String -Path D:\smartfarm-sim\out\test01\isaac.log -Encoding UTF8 -Pattern "\[(컨베이어|비전|솎아내기)\]|완료|Traceback" | % Line
PS> Get-Content D:\smartfarm-sim\out\test01\flow.log -Tail 5
```

### 3-4. 수동 명령 모드 (-NoFlow)

Nav2 가 준비된 상태에서 흐름을 직접 보낼 때(녹화 준비 후 시작 등):

```powershell
PS> powershell -NoProfile -ExecutionPolicy Bypass -File D:\smartfarm-sim\ROKEY_P3_A1\cobot3_ws\isaacpjt\smart_farm\tools\allinone_live\run_allinone.ps1 -OutDir D:\smartfarm-sim\out\test02 -NoFlow
PS> $env:WSL_UTF8="1"; wsl -d Ubuntu-24.04 -- bash -c "source ~/nav2_env.sh; python3 /mnt/d/smartfarm-sim/ROKEY_P3_A1/cobot3_ws/isaacpjt/smart_farm/tools/allinone_live/wsl/run_flow.py /mnt/d/smartfarm-sim/out/test02/flow.json 2>&1 | tee /mnt/d/smartfarm-sim/out/test02/flow.log | grep '\[flow\]'"
```

### 3-5. Isaac 단독 실행 (Nav2 없이, 팀 방식)

```powershell
PS> D:\isaacsim\python.bat D:\smartfarm-sim\ROKEY_P3_A1\cobot3_ws\isaacpjt\smart_farm\runtime\standalone_app.py --autoplay
```
- 옵션: `--scene <usd>`, `--headless`, `--demo`(검증용 TRANSFER 자동), `--no-conveyor`, `--no-vision-station`(컨베이어가 10초 뒤 스스로 배출).
- Windows 에서 ROS 를 쓰려면 `run_allinone.ps1` 처럼 `SMARTFARM_ROS_REEXEC=1`, `RMW_IMPLEMENTATION=rmw_fastrtps_cpp`, `ROS_DISTRO=jazzy`, PATH 에 `D:\isaacsim\exts\isaacsim.ros2.bridge\jazzy\lib` 를 넣어야 한다(팀 코드는 리눅스 기준).
- **headless 에서는 ROS OmniGraph 가 돌지 않아 /clock·odom·lidar 가 안 나온다 → Nav2 연동 시 GUI 필수.**

### 3-6. 끝내기

- Isaac 창을 닫는다.
- WSL 쪽 정리: `PS> wsl --terminate Ubuntu-24.04` (다음 실행 때 스크립트가 자동으로도 한다).
- 남은 중계기: `PS> Get-CimInstance Win32_Process | ? { $_.CommandLine -like '*ros_tcp_relay*' -or $_.CommandLine -like '*run_with_monitor.py*' } | % { Stop-Process -Id $_.ProcessId -Force }`

---

## 4. 비전 검사 · YOLO · 솎아내기 · 푸셔

모두 `scripts\inspection_cull_station.py` 하나(`VisionCullStation`) 가 처리한다. 올인원에서는 자동이므로 따로 실행할 필요는 없고, **단독 시험**만 여기서 한다.

### 4-1. 상태 순서

```
IDLE → PUSH_IN(이송 프레임이 트레이를 로봇 쪽으로) → MOVE_INSPECT → CAPTURE(손목 카메라 3프레임 + YOLO)
     → CULL ⇄ HOME (포기마다: 집기 → VIA 중간점 → SortBin 에 놓기 → 검사 자세 복귀)
     → RECHECK_MOVE → RECHECK(빈 칸 확인) → PUSH_OUT(벨트로 되밀기) → conveyor.inspection_done() → IDLE
```

| 항목 | 값 |
|---|---|
| 컨베이어 정지 위치 | vision_x = -0.69 (트레이 중심이 비전 M0609 base 와 같은 x) |
| 이송 | 벨트 줄 y -6.73 → 로봇 앞 y -7.00 (TRAY_REACH 0.48 m), 속도 0.08 m/s |
| 판정 | green = 정상, **yellow · brown = 제거** (팀 문서의 "yellow 보류" 규칙은 무시하기로 결정) |
| 버리는 곳 | SortBin_1 / SortBin_2 번갈아, 같은 통은 긴 변을 따라 자리 바꿈, 통 윗면 10 cm 위에서 놓기 |
| 그리퍼 힘 | 8 N·m (팀 기본 1e4 는 관통) |
| 팔 실행기 | 팀 `cull_motion.CullMotion` (파일 수정 없음) + 계획 덮어쓰기 |
| YOLO | 같은 파일을 `--yolo-worker` 로 하위 프로세스 실행 (Isaac 파이썬 라이브러리 충돌 방지) |

### 4-2. 단독 시험 (포크 로봇·Nav2 없이)

```powershell
PS> D:\isaacsim\python.bat D:\smartfarm-sim\scripts\cabbage\09_inspection_cull_station_test.py --scene D:\smartfarm-sim\ROKEY_P3_A1\cobot3_ws\isaacpjt\smart_farm\scenes\Collected_smartfarm_v014\Collected_smartfarm_v014_room_core_cabbage.usd --out D:\smartfarm-sim\out\station_test01 --pattern G,Y,B,G,Y,B --gui
```

- `--pattern` 칸 01~06 색(G/Y/B). 기본 `G,Y,B,G,Y,G`.
- `--start=-2.186,-4.2` 트레이 시작 위치. **음수 값은 반드시 `=` 로 붙여 쓴다**(PowerShell 이 `-2.186` 을 옵션으로 오해, 트러블슈팅 W2).
- `--snap /World/ProcessCameras/Cam5_Pusher` 1초마다 사진, `--contacts` 트레이 접촉 보고(걸림 진단).
- 09 는 `D:\smartfarm-sim\scripts\cabbage\` 에서 실행하면 `D:\smartfarm-sim\ROKEY_P3_A1\cobot3_ws\isaacpjt\smart_farm\scripts` 의 `conveyor`, `inspection_cull_station` 을 불러온다(최종 코드와 호환, YOLO 워커는 inspection_cull_station 안에 있음).

**성공:** `--out` 폴더의 `result.json`, `station_results.json`(칸별 판정), `*_yolo.jpg`. 작성 PC 결과: 불량 4칸 4/4, 3칸 3/3 제거, 판정 6/6 일치.

### 4-3. 이송 프레임(푸셔) 모양

실행 시 코드가 `/World/VisionTransfer` 에 만든다(씬 파일에는 없음).
PlateN(주황, 로봇 쪽으로 밀기) · PlateS(빨강, 벨트로 되밀기) · ArmW/E · Rod0/1 · Carriage · Column0/1.
롤러·레일·벽을 뚫지 않도록: 판 바닥 0.775 > 롤러 윗면 0.769, 레일(y -7.19 / -6.31) 안쪽, 기둥은 컨베이어 밖(y -6.05).
검사 중에만 남쪽 옆가이드 충돌을 끈다(트레이가 가이드 자리를 지나감).

---

## 5. 녹화

### 5-1. 공정별 씬 카메라 녹화 → 영상

**목적:** 수확 / Nav2+놓기 / 검사 화면 / 픽앤플레이스 / 푸셔를 씬 안 카메라로 찍기.

1) 캡처를 켜고 올인원 실행(0.2초마다 한 장):

```powershell
PS> $env:CABBAGE_SNAP_FROM=""; $env:CABBAGE_CAPTURE_EVERY="0.2"; $env:CABBAGE_CAPTURE_CAMS="cam1_harvest=/World/ProcessCameras/Cam1_Harvest;cam2_nav2place=/World/ProcessCameras/Cam2_Nav2Place;cam3_inspection=/World/SmartFarm/Placed/M0609/Asset/onrobot_rg2ft/angle_bracket/realsense_d455/RSD455/Camera_OmniVision_OV9782_Color;cam4_cullpickplace=/World/ProcessCameras/Cam4_CullPickPlace;cam5_pusher=/World/ProcessCameras/Cam5_Pusher"; powershell -NoProfile -ExecutionPolicy Bypass -File D:\smartfarm-sim\ROKEY_P3_A1\cobot3_ws\isaacpjt\smart_farm\tools\allinone_live\run_allinone.ps1 -OutDir D:\smartfarm-sim\out\rec01
```

→ `D:\smartfarm-sim\out\rec01\captures\cap_<카메라>_<시각>.jpg`

2) 공정별 영상 자르기:

```powershell
PS> $env:PYTHONPATH="D:\smartfarm-sim\team_ref\pylib_yolo;D:\smartfarm-sim\team_ref\pylib"; D:\isaacsim\kit\python\python.exe D:\smartfarm-sim\scripts\cabbage\10_make_process_videos.py D:\smartfarm-sim\out\rec01 10
```
→ `rec01\videos\00_all_processes.mp4 ~ 05_transfer_frame_pusher.mp4`

3) 검출 박스 영상(rqt_image_view 모양):

```powershell
PS> $env:PYTHONPATH="D:\smartfarm-sim\team_ref\pylib_yolo;D:\isaacsim\exts\omni.isaac.ml_archive\pip_prebundle;D:\smartfarm-sim\team_ref\pylib"; D:\isaacsim\kit\python\python.exe D:\smartfarm-sim\scripts\cabbage\13_make_detection_video.py D:\smartfarm-sim\out\rec01 D:\smartfarm-sim\ROKEY_P3_A1\cobot3_ws\isaacpjt\smart_farm\scenes\Collected_smartfarm_v014\best.pt
```
→ `rec01\videos\06_vision_detection_boxes.mp4`
(가중치 인자를 안 주면 작성 PC 경로 `Downloads\romaine3_v012_640sq_yolo11n_best.pt` 를 찾으니 **새 PC 에서는 반드시 준다**. 클래스 이름이 `lettuce_*` 가 아닌 가중치는 색 표시가 기본색이 될 수 있음 — **확인 필요**)

- 브라우저에서 재생되지 않는 mp4(mp4v 코덱) → H.264 로 다시 인코딩:
  `PS> & D:\smartfarm-sim\team_ref\pylib_ffmpeg\imageio_ffmpeg\binaries\ffmpeg-win-x86_64-v7.1.exe -y -i 입력.mp4 -c:v libx264 -pix_fmt yuv420p 출력.mp4`

**GUI 시점을 카메라로 저장**(실행 중): 뷰포트를 원하는 시점으로 맞춘 뒤
```powershell
PS> powershell -File D:\smartfarm-sim\scripts\cabbage\save_view_camera.ps1 -Name Cam0_MyView
```
→ `isaac.log` 에 `[MONITOR] viewport view saved`, `D:\smartfarm-sim\out\saved_view_cameras.json` 에 누적 → 2-1 에서 `SAVED_VIEW_CAMERAS` 로 씬에 굽는다.

### 5-2. Nav2 RViz + ROS2 통신 화면 녹화 (주행 중)

**원리:** WSLg 창(RViz·xterm)은 Windows 화면 캡처(gdigrab)로는 검게 나오거나 안 잡힌다 → **WSL 안에서 `ffmpeg -f x11grab`** 으로 창을 직접 녹화한다.

1) Nav2 까지만 띄우기:
```powershell
PS> powershell -NoProfile -ExecutionPolicy Bypass -File D:\smartfarm-sim\ROKEY_P3_A1\cobot3_ws\isaacpjt\smart_farm\tools\allinone_live\run_allinone.ps1 -OutDir D:\smartfarm-sim\out\drivecomm01 -NoFlow
```

2) 통신 화면(tmux 6칸 xterm "ROS2 comm") 띄우고 RViz 크기 맞춘 뒤 녹화 시작(1500초):
```powershell
PS> wsl -d Ubuntu-24.04 -e bash /mnt/d/smartfarm-sim/ROKEY_P3_A1/cobot3_ws/isaacpjt/smart_farm/tools/allinone_live/wsl/record_drive.sh /mnt/d/smartfarm-sim/out/drivecomm01/linux 1500
```
6칸 내용: `/sim_task/command`, `/sim_task/result`, `/navigation/command`(TaskCommand), `/navigation/result`(TaskResult), `/cmd_vel`(linear.x), `/chassis/odom`(위치).
출력: `linux\nav2_rviz.mp4`, `linux\ros2_comm.mp4`, `linux\comm_frames\*.png`(1초 1장 예비).

3) 흐름 시작: 3-4 의 run_flow 명령(OutDir 만 drivecomm01 로).

4) 끝나면 녹화 중지:
```powershell
PS> wsl -d Ubuntu-24.04 -e bash -c "pkill -INT -f 'ffmpeg -y'; pkill -f snap_loop"
```

5) 주행 구간만 자르기·나란히 붙이기(예: 녹화 시작 후 150초부터 115초):
```powershell
PS> $ff="D:\smartfarm-sim\team_ref\pylib_ffmpeg\imageio_ffmpeg\binaries\ffmpeg-win-x86_64-v7.1.exe"; $d="D:\smartfarm-sim\out\drivecomm01\linux"
PS> & $ff -y -ss 150 -t 115 -i "$d\nav2_rviz.mp4" -ss 150 -t 115 -i "$d\ros2_comm.mp4" -filter_complex "[0:v]scale=-2:810[a];[1:v]scale=-2:810[b];[a][b]hstack,scale=trunc(iw/2)*2:trunc(ih/2)*2" -c:v libx264 -preset veryfast -crf 23 -pix_fmt yuv420p "$d\drive_rviz_plus_comm.mp4"
```
- 주행 시작 시각 찾기: `navnode.log` 의 `goToPose` 줄 시간(유닉스 초) − `ros2_comm.mp4` 생성 시각.

작성 PC 결과: `D:\smartfarm-sim\out\allinone_drivecomm\linux\drive_rviz_plus_comm.mp4` (1분 55초).

**다른 창 녹화 도구** (참고):
- `ros_viewers.sh` / `ros_viewers_fix.sh`: rqt_graph, rqt_topic, sim_task xterm 을 열고 창을 화면 안에 맞춤.
- `ros_record.sh OUT 초`: RViz, rqt_graph, rqt_topic, sim_task 창 녹화 (`SKIP_RVIZ=1` 로 RViz 제외).
- `snap_loop.sh OUT 이름 창제목정규식 초`: 연속 녹화가 멈추는 창(rqt_graph 등)을 1초 1장으로.
- `rec_status.sh OUT`: 어떤 녹화가 돌고 있는지.

---

## 6. 깃 업로드 규칙

| 규칙 | 내용 |
|---|---|
| 브랜치 | **`feature/cabbage-place-fix` 하나만.** 새 브랜치를 여러 개 만들지 말고 이것만 계속 갱신 |
| 작성자 | `git config user.name "kangmulj-max"` (작성 PC 저장소 설정: name kangmulj-max, email kangmulj@gmail.com) |
| 팀원 파일 | **절대 수정·삭제 금지.** 예외: `runtime/standalone_app.py` 는 `[올인원 2026-09-25]` 표시 추가분 + PLACE 수정만 (합의됨). `conveyor.py`, `conveyor_rollers.py`, `cull_motion.py` 는 원본 그대로 |
| main | 손대지 않음, 강제 푸시(force push) 금지 |
| 올리는 것 | 코드만. 에셋·씬·yaml 중 안 쓰는 것·시험 도구는 올리지 않음(에셋은 공유 zip). `scenes/` 는 `.git/info/exclude` |
| 인증 | WSL 에서 `gh auth login` 후 `gh auth setup-git` (작성 PC 는 WSL gh 로 푸시) |

작성 PC 의 푸시 스크립트(`scripts/wsl/push_branch.sh`)는 현재 브랜치가 `feature/cabbage-place-fix` 이고 커밋 안 된 변경이 없을 때만 그 브랜치만 푸시한다:

```powershell
PS> wsl -d Ubuntu-24.04 -- bash /mnt/d/smartfarm-sim/scripts/wsl/push_branch.sh
```

커밋 전 확인(팀원 파일이 안 섞였는지):
```bash
$ cd /mnt/d/smartfarm-sim/ROKEY_P3_A1
$ git status --short
$ git diff --stat origin/feature/cabbage-place-fix
```

현재 브랜치 최근 커밋: `255860e` (v014 기본 경로) ← `8cdc819` (영상) ← `683b779` (YOLO 워커 합침) ← `5368fa5` (최종 코드만 남김) ← `ee1043d` (도구 전부 있던 마지막 커밋).

---

## 7. 트러블슈팅 총정리

### 7-1. 설치·실행 환경 (Windows / PowerShell / WSL)

| # | 증상 | 원인 | 해결 |
|---|---|---|---|
| W1 | `git` 을 찾을 수 없음 (PowerShell) | Git for Windows 없음/PATH 없음 | WSL 에서 `/mnt/d/...` 저장소로 git 사용, 또는 Git for Windows 설치 |
| W2 | 인자 `-2.0` 이 옵션으로 해석됨 | PowerShell 이 `-` 로 시작하는 값을 파라미터로 봄 | `--start=-2.186,-4.2` 처럼 `=` 로 붙이기 |
| W3 | 인자 안의 `\|` 가 파이프로 동작 | cmd/PowerShell 해석 | 인자를 작은따옴표로 감싸거나 `;` 구분자 사용 |
| W4 | `-File` 로 배열 인자가 안 넘어감 | `powershell -File` 은 배열 파싱 안 함 | `"a=b;c=d"` 처럼 문자열 하나로 넘기고 스크립트에서 `Split(";")` |
| W5 | 한글 파일명 README 가 비거나 깨짐 | `Set-Content` 기본 인코딩이 ANSI | `-Encoding utf8` 명시, 가능하면 편집기로 저장 |
| W6 | WSL 에서 `$'\r': command not found` | Windows 줄끝(CRLF) | `sed -i 's/\r$//' 파일.sh` |
| W7 | 실행 정책 오류 (`.ps1` 실행 불가) | ExecutionPolicy | `powershell -NoProfile -ExecutionPolicy Bypass -File ...` |
| W8 | WSL 출력 한글/경로 깨짐 | wsl 출력이 UTF-16 | `$env:WSL_UTF8="1"` |
| W9 | `Remove-Item` 이 변수 경로로 막힘 | 권한 확인 정책 | `-LiteralPath "실제경로"` 로 직접 지정 |

### 7-2. Isaac / USD

| # | 증상 | 원인 | 해결 |
|---|---|---|---|
| A1 | 만든 .usd 를 Isaac 이 못 읽음 | Isaac 5.1 USD(24.05) ≠ pip usd-core(26.x) 이진 포맷 | 씬·에셋 저장은 Isaac 파이썬으로(`python.bat`, `run_in_isaac.py`, make_cabbage_scene 은 스스로 Isaac 을 띄움) |
| S1 | 양배추로 바꿨는데 로메인과 겹쳐 폭발 | 서브레이어 편집은 루트 레이어보다 약함 | 루트 레이어 복사본에서 참조 교체 (make_cabbage_scene 방식) |
| S7 | 씬 재생성 실패(파일 잠김) | Isaac GUI 가 씬을 열고 있음 | Isaac 종료 후 실행 |
| S8 | 씬 스크립트가 오류 없이 끝나는데 결과 없음 | 삭제 API 이름 오류 + Kit 종료 때 stderr 사라짐 | `<스크립트>.error.txt` 확인, prim 삭제는 `del parent.nameChildren[name]` |
| S3 | 씬 열면 옛 트레이가 몇 초 보였다 사라짐 | 벨트 위 시험 트레이·Lettuce | v014 에서 삭제됨 (옛 씬이면 make_cabbage_scene 재실행) |
| N1 | headless 에서 /clock·odom 없음 | headless 에서 ROS OmniGraph 미동작 | GUI 로 실행 |
| I1 | 첫 실행이 매우 느림 | 셰이더 캐시 생성 | 기다린다(두 번째부터 빨라짐) |

### 7-3. Nav2 · 통신

| # | 증상 | 원인 | 해결 |
|---|---|---|---|
| N2 | Windows↔WSL 토픽이 안 옴 | DDS 가 WSL 경계를 못 넘음(mirrored: 늦게 뜬 노드 데이터 유실, NAT: 방화벽·멀티캐스트) | `.wslconfig` NAT + `ros_tcp_relay.py`, 도메인 101/102 분리 |
| N3 | Nav2 가 use_sim_time 에서 멈춤 | 씬에 /clock 그래프 없음(v011) | v013/v014 사용 |
| N4 | `Aborting bringup` | 이전 WSL 프로세스 잔존 | `wsl --terminate Ubuntu-24.04` 후 재실행 (스크립트가 자동) |
| N5 | `Nav2 stack did not come up` (10분) | Isaac 이 READY 전에 멈춤, 중계기 미연결 | `isaac.log` 에 `[READY]` 있는지, `relay_wsl.log` 에 `/clock=` 숫자 있는지 확인 |
| N6 | `ros2 topic list` 에 아무것도 없음 | 도메인 다름 | `source ~/nav2_env.sh` (도메인 102) 후 확인 |
| P3 | PLACE 중 DESCEND_5 관절 한계 실패 | 도킹 거리 0.85 m 에서 팔이 너무 뻗음 | feeder_dock `standoff_m` 0.92 (`-Standoff 0.92` 기본) |

### 7-4. 컨베이어 · 비전 · 솎아내기

| # | 증상 | 원인 | 해결 |
|---|---|---|---|
| C2 | 트레이가 교차점 출구(x -1.424)에서 멈춤 `CONVEYOR_FAILED` | 교차점 프레임 부품 `SM_ConveyorBelt_A49_01` 충돌면이 롤러보다 2 mm 높음 (접촉 보고로 찾음, 시험 14회) | 그 부품 충돌 끔 (v014 에 반영) |
| V2 | 비전 팔이 트레이에 닿지 않음 | 줄(y -6.73)이 base 에서 0.89 m | 받침대 +0.146 m + 이송 프레임이 y -7.00 으로 밀기 |
| V3 | 옮긴 뒤 포기가 칸에서 빠짐 | kinematic 순간이동, 남쪽 가이드와 겹침 | 이송 프레임이 물리로 밀기 + 검사 중 남쪽 가이드 충돌 끔 |
| V4 | 푸셔가 롤러·레일을 뚫음 / 한쪽만 보임 | 롤러 아래에서 올라오는 판 | 사각 이송 프레임(PlateN/PlateS 둘 다 보임), 롤러 위·레일 안쪽에서만 움직임 |
| V5 | 옮기다 포기를 떨어뜨림 | LIFT→PLACE 직선이 base 위로 지나 팔이 꺾임 | VIA 중간점 + 포기마다 HOME 복귀 |
| V6 | 같은 통 두 번째 포기가 굴러 나감 | 같은 자리에 20 cm 위에서 떨어뜨림 | DROP_SPREAD 자리 바꿈, 10 cm 위에서 놓기 |
| V7 | 먼 줄 포기 놓침 | 비전 좌표 오차 2~10 mm | 집는 좌표 = 트레이 자세 + 칸 배치 |
| S4/S5 | 포기가 SortBox 뚜껑 위에 얹힘 / 통 바닥 뚫림 | 속이 찬 큐브 / 바닥 2 cm | 윗면 열린 SortBin, 바닥 5 cm |
| A5 | 그리퍼가 포기를 관통 | 팀 그리퍼 힘 1e4 N·m | 8 N·m + 포기 contactOffset 4 mm |
| Y1 | YOLO 결과가 안 나옴 / 워커 오류 | ultralytics 경로·가중치 없음 | 1-2 설치 확인, 씬 폴더에 best.pt, 필요시 `SMARTFARM_YOLO_WEIGHTS`·`SMARTFARM_YOLO_PYTHON`·`SMARTFARM_YOLO_PYTHONPATH` 지정 |

### 7-5. 씬 모양 · 로봇

| # | 증상 | 원인 | 해결 |
|---|---|---|---|
| S6 | 비전룸 벽 통로가 "뻐큐" 모양 | 통로 윗부분 메시에 가운데 홈 | 평판으로 교체 (v014) |
| L1 | 팔이 카터 리프트 기둥을 뚫고 지나감 | 기둥 꼭대기 1.48 m > 팔 베이스 | 텔레스코픽 기둥(바깥 0.78 m + 안쪽 0.63 m), 로봇 위치 그대로 |

### 7-6. 녹화

| # | 증상 | 원인 | 해결 |
|---|---|---|---|
| R1 | 캡처가 초당 1장뿐 | 캡처 코드가 15스텝 감시 조건 뒤 | run_with_monitor.py (캡처가 조건 앞) 사용 |
| R2 | WSLg 창(RViz 등)이 Windows 녹화에서 검게 나옴/안 잡힘 | gdigrab 이 WSLg 창을 못 봄, 최소화 시 멈춤 | WSL 안에서 `ffmpeg -f x11grab -window_id` |
| R3 | x11grab 이 오류로 멈춤 | 창이 1920x1080 X 화면 밖으로 삐져나감 | `xdotool windowsize/windowmove` 로 화면 안에 맞추기 (map/activate 쓰면 창이 사라짐) |
| R4 | rqt 가 `-qwindowgeometry` 에서 오류 | rqt 가 이 옵션 미지원 | 띄운 뒤 xdotool 로 크기 조정 (`ros_viewers_fix.sh`) |
| R5 | rqt_graph 연속 녹화가 멈춤 | 정적 창 갱신 없음 | `snap_loop.sh` 1초 1장 |
| R6 | 녹화 중 강제 종료하면 mp4 가 안 열림 | mp4 끝 정보 미기록 | `-movflags +frag_keyframe+empty_moov -g 12` (스크립트에 적용됨), 종료는 `pkill -INT` |
| R7 | 전체 최소화(Win+D 등) 후 WSLg 창이 사라짐 | WSLg 창도 같이 최소화 | 최소화하지 말고 Isaac 창만 옮기기 |
| R8 | 브라우저에서 mp4 재생 안 됨 | mp4v 코덱 | H.264 재인코딩 (5-1) |
| R9 | `/cmd_vel` 칸이 대부분 0.0 | 확인 필요 — Nav2 속도 체인(`cmd_vel_smoothed` → collision_monitor → `/cmd_vel`) 중 화면에 찍히는 값 | 필요하면 `/cmd_vel_smoothed` 칸 추가 |

---

## 8. 용어 설명

| 용어 | 뜻 |
|---|---|
| **Isaac Sim** | NVIDIA 로봇 시뮬레이터. 물리(PhysX)와 렌더링(RTX)으로 로봇·센서를 가상으로 돌린다 |
| **USD** | 3D 장면 파일 형식(.usd/.usda). 씬·에셋 모두 USD |
| **prim** | USD 장면 안의 물체 하나(경로 예: `/World/ProcessCameras/Cam1_Harvest`) |
| **레이어 / 서브레이어 / 루트 레이어** | USD 파일을 겹쳐 쌓는 구조. 위(루트)에 쓴 값이 아래(서브레이어)보다 우선 |
| **variant** | 한 prim 의 여러 버전 중 고르는 스위치 (포기의 `condition` = green/yellow/brown) |
| **rigid body / kinematic** | 물리로 움직이는 물체 / 코드가 위치를 정해 주고 다른 물체를 밀 수 있는 물체 (이송 프레임) |
| **collider(충돌체)** | 물리 충돌에 쓰는 모양. 보이는 모양과 다를 수 있음 |
| **articulation** | 관절로 연결된 로봇 몸체 (M0609 팔, 카터) |
| **headless** | 화면 없이 실행. 이 프로젝트에서는 ROS 연동이 안 됨 |
| **ROS 2 / Jazzy** | 로봇 소프트웨어 통신 프레임워크 / 그 배포판 이름 |
| **노드** | ROS 프로그램 하나 (navigation_node, feeder_dock 등) |
| **토픽** | 노드끼리 메시지를 주고받는 이름 붙은 통로 (`/cmd_vel`, `/sim_task/command`) |
| **DDS / ROS_DOMAIN_ID** | ROS 2 의 통신 방식 / 같은 번호끼리만 통신하는 채널 번호 (여기선 Windows 101, WSL 102) |
| **중계기(ros_tcp_relay)** | 도메인·OS 경계를 넘어 토픽을 TCP 로 옮기는 오늘 만든 프로그램 |
| **Nav2** | ROS 2 자율주행 스택 (지도·위치추정·경로계획·주행) |
| **costmap** | 장애물 비용 지도. 로봇 주변(local)과 전체(global) 두 개 |
| **AMCL** | 라이다와 지도로 로봇 위치를 추정하는 노드 |
| **RViz2** | ROS 데이터를 보여 주는 화면(지도·로봇·경로) |
| **feeder_dock** | 컨베이어 앞 정밀 도킹 노드 (라이다로 면을 찾아 붙음), `standoff_m` = 면까지 거리 |
| **use_sim_time / /clock** | Nav2 가 시뮬레이션 시간을 쓰게 하는 설정 / 그 시간 토픽 |
| **WSL2 / WSLg** | Windows 안의 리눅스 / 그 리눅스 GUI 창 표시 기능 |
| **YOLO / best.pt** | 물체 검출 딥러닝 모델 / 학습된 가중치 파일 |
| **SortBin** | 솎아낸 포기를 버리는 윗면 열린 통 (1, 2) |
| **이송 프레임(푸셔)** | 트레이를 벨트 줄 ↔ 로봇 앞으로 미는 장치 (PlateN 주황, PlateS 빨강) |
| **PICK_HARVEST / NAVIGATION / PLACE_INSPECT** | 랙에서 수확 / 컨베이어 앞으로 주행 / 벨트에 내려놓기 명령 |
| **x11grab / gdigrab** | ffmpeg 의 리눅스 창 녹화 / Windows 창 녹화 방식 |

---

## 9. 사람 돌발상황 + 바닥 노란 차선 주행

### 9-1. 무엇을 하나
- **-Lane (바닥 차선 주행, A안)**: 카터가 RACK_DOCK 에서 **노란 차선의 흰 점선 중앙선(x -0.42)을 따라 남쪽으로 후진**하고,
  모서리에서 곡선(반경 0.85 m)을 45° 까지 돈 뒤 **Nav2 기본 경로 계획(Hybrid-A*)이 이어받아** FEEDER_APPROACH → feeder_dock 도킹.
- **-Human (사람 돌발상황)**: 작업자(안전모·조끼)가 차선 동쪽 가장자리(0.25, -1.45)에 서 있다가 카터가 출발하면 차선 중앙으로 들어와
  **4초 서 있다가** (1.3, -1.2) 로 비킨다. 카터는 **트레이 끝에서 약 0.5~0.8 m 앞에서 멈추고**(로봇 중심~사람 1.7~1.9 m),
  사람이 영역을 벗어나면 **1~2초 안에 다시 출발**한다.
- 두 옵션은 따로도, 같이도 쓸 수 있다. 최종 검증(lane_run8)은 둘 다 켜고 수확 → 차선 주행(사람 정지·재출발) → 도킹 → 내려놓기 → 검사 → 선별 3/3 → 배출 성공.

| 쪽 | 파일 | 하는 일 |
|---|---|---|
| Isaac | `smart_farm/scripts/human_crossing.py` (새 파일) | 사람을 넣은 감싼 장면(.usda) 생성, 걷기 제어, 녹화 카메라 `/World/Characters/HumanViewCam`, 기록 `station/human_events.json` |
| Isaac | `runtime/standalone_app.py` (`[올인원 2026-09-25]` 표시 줄만) | `--human-crossing` 옵션, 장면 열기 전 사람 준비, 매 스텝 `human.update` |
| Nav2 | `tools\allinone_live\wsl\make_nav2_test_params.py` (새 파일) | 팀 `nav2_params.yaml` 을 **읽기만** 하고 복사본 `OUT/nav2_params_test.yaml` 을 만든다 (아래 표) |
| Nav2 | `tools\allinone_live\wsl\lane_planner.py` (새 파일) | 차선 경로를 주는 ComputePathToPose 액션 서버 `lane_planner` + 도킹 구역 안전 영역 전환 |
| Nav2 | `tools\allinone_live\wsl\lane_route_bt.xml` (새 파일) | 차선 경로 → Nav2 기본 경로(하위 트리 DefaultNav) 순서의 BT. 조건이 안 맞으면 처음부터 기본 경로 |
| 도구 | `tools\allinone_live\floor_lines.py` (새 파일) | 씬 바닥 선 메쉬 좌표 추출 + 지도 겹침 그림 (`out\floor_lines\floor_lines_zoom.png`) |
| 실행 | `tools\allinone_live\run_allinone.ps1 -Human -Lane`, `wsl\start_nav2_stack.sh`(`HUMAN=1`, `LANE=1`) | 위를 켬 + `collision_state.log`, `lane_planner.log` 기록 |

Nav2 복사본에 더하는 것 (팀 파일은 그대로):

| 옵션 | 항목 | 값 | 이유 |
|---|---|---|---|
| -Human | 로봇 윤곽(footprint) 뒤끝 | -0.607 → **-1.10 m** | 주행 중 트레이가 차체 뒤 0.8 m(끝 약 1.1 m), 높이 1.02 m 에 있음 |
| -Human | HumanStop (정지) | 후진 -0.62 ~ **-1.90 m**, 전진 0.15 ~ 1.20 m, 폭 ±0.38 m, 속도 0 근처는 반경 1.25 m 팔각형 | 트레이 끝에서 0.8 m 앞에서 정지. 폭 0.38 은 랙 통로 팔레트(중심에서 0.44 m) 안쪽 |
| -Human | HumanSlow (50 % 감속) | 후진 -0.62 ~ -2.60 m, 전진 0.15 ~ 1.90 m | 멈추기 전 속도를 줄임 |
| -Human | 장애물 지도 `inf_is_valid` | true | 사람 지나간 자국(유령 장애물) 지우기 |
| -Human | RPP `use_collision_detection` | false | 멈춤은 HumanStop 이 맡음. 켜 두면 실패→복구로 10초 늦게 출발 |
| -Human | `movement_time_allowance` | 30 s | 사람 대기 중 목표 실패 방지 |
| -Lane | bt_navigator `default_nav_to_pose_bt_xml` | `lane_route_bt.xml` | 차선 경로 BT |

도킹 구역(x < -1.5, y < -1.7)에서는 컨베이어가 정지 영역에 들어오므로 `lane_planner.py` 가 HumanStop/HumanSlow 를 끄고 나오면 켠다
(실제 AGV 의 안전 스캐너 영역 전환과 같은 방식).

### 9-2. 실행 (한 줄)
```powershell
$env:CABBAGE_CAPTURE_CAMS = "human=/World/Characters/HumanViewCam"; $env:CABBAGE_CAPTURE_EVERY = "0.2"   # 사람 카메라 영상이 필요할 때만
powershell -NoProfile -ExecutionPolicy Bypass -File D:\smartfarm-sim\ROKEY_P3_A1\cobot3_ws\isaacpjt\smart_farm\tools\allinone_live\run_allinone.ps1 -OutDir D:\smartfarm-sim\out\lane_run -Human -Lane
```
- 처음 한 번은 사람 에셋을 NVIDIA 서버(`omniverse-content-production.s3-us-west-2.amazonaws.com/.../Isaac/5.1/Isaac/People/Characters`)에서 받으므로 **인터넷 필요**.
- GUI 뷰포트가 자동으로 `HumanViewCam` 으로 바뀌고, 카터 재출발 40초 뒤 원래 카메라로 돌아간다.
- 통신 화면 녹화(-NoFlow 로 띄운 뒤): `wsl -d Ubuntu-24.04 -- env HUMAN=1 bash /mnt/d/smartfarm-sim/ROKEY_P3_A1/cobot3_ws/isaacpjt/smart_farm/tools/allinone_live/wsl/record_drive.sh <OUT의 linux 경로> 1500`
  (odom 칸 대신 `/collision_monitor_state`: `action_type: 1` = 정지, `2` = 감속, `0` = 해제).
- **주의**: `run_allinone.ps1` 출력을 `| Out-Null`, `| Select-Object` 로 넘기지 말 것. Isaac·중계기 자식 프로세스가 출력 핸들을 잡고 있어 명령이 끝나지 않는다.

### 9-3. 성공 확인
- `lane_planner.log`: `LaneRoute: 47 poses from (-0.42, 1.01) to (-0.67, -1.29)`, `safety field DOCK (HumanStop/HumanSlow off)`
- `isaac.log`: `[사람] WALK_IN` → `STAND_IN_PATH ... robot_gap=1.7~1.9` → `WALK_OUT ... robot_speed=0.0` → `CLEAR` / `ROBOT_RESUMED ... robot_speed=0.3`
- `flow.log`: `navigation result SUCCEEDED ... reached=FEEDER_DOCK`
- `monitor.json` 의 `chassis_track`: 남쪽 구간 x 가 -0.38 ~ -0.43 (차선 중앙 -0.42)

### 9-4. 트러블슈팅 (실제로 겪은 것)
| 증상 | 원인 | 해결 |
|---|---|---|
| 캐릭터가 걷지 않음, `getCharacter ... does not apply AnimationGraphAPI`, `attribute animationGraph not found` | 장면을 연 **뒤에** 사람을 넣으면 Fabric 에 애니메이션 속성이 안 올라감. 확장도 비동기로 늦게 켜짐 | 확장을 먼저 켜고 `app.update()` 여러 번 → 사람을 넣은 감싼 장면을 **먼저 만들고** 연다 (`prepare_scene`) |
| `Type mismatch for xformOp:rotateXYZ: expected GfVec3d` | 사람 에셋이 double 회전값을 가짐 | 기존 속성 타입대로 값 설정 |
| 사람이 섰는데 카터가 옆으로 지나감 | 기본 경로는 남서 대각선. 예측 지점이 경로에서 0.8 m 벗어남 | 실제 경로 위 한 점(`CROSS_POINT`)에 세움 |
| 걷기가 목표 0.25 m 앞에서 멈춤 | 애니메이션 도착 판정 | 목표를 걷는 방향으로 0.25 m 더 잡음 (-0.67 → 실제 -0.43) |
| 멈춘 뒤 못 감, `detected collision ahead!` 반복, Recoveries 6 | 사람 자국이 장애물 지도에 남음 | `inf_is_valid: true` |
| 정지 중 기어감, 정지/해제 반복 | 속도 0 근처에서 정지 영역이 차체 크기로 바뀜 | 속도 0 근처 영역을 반경 1.25 m 팔각형으로 |
| 사람이 비켜도 10초 늦게 출발 | RPP 충돌 실패 → BT 복구(spin → wait) | RPP `use_collision_detection: false` |
| **트레이가 사람에 닿음** | 정지 판정이 차체 끝(0.6 m) 기준, 트레이는 1.1 m 까지 나옴 | 윤곽 -1.10, HumanStop -1.90 |
| 모서리 제자리 회전(Spin)이 30초 막힘 | 회전 판정 영역(앞뒤 긴 상자)이 돌면서 옆 1 m 사람에 걸림 | 회전 영역 = 반경 1.25 m 팔각형, 사람 비키는 곳을 1.75 m 밖으로 |
| **제자리 회전이 원호가 되어 0.6~0.9 m 밀려남** | 카터+리프트+팔+트레이 리그가 제자리에서 못 돎 (명령 직진 속도 0 인데 0.7 m 떨어진 점을 축으로 끌림). 원인 미확인 | 제자리 회전을 쓰지 않음 |
| 반경 0.5 / 0.85 m 곡선에서 바깥으로 밀려 앞뒤로 떨며 갇힘 | 실제 회전 반경 약 1.2 m. 두 모서리 사이 1.77 m 라 90° 두 번 불가 | **A안**: 첫 모서리 45° 까지만 차선, 나머지는 Nav2 기본 경로 |
| 도착 판정기 추가 뒤 `FollowPath called with goal_checker name  ... which does not exist` 로 전부 실패 | 판정기가 둘 이상이면 이름 없는 FollowPath 가 거부됨 | 판정기는 팀 것 하나만 |
| 차선 끝에서 컨베이어를 사람으로 보고 정지 | 정지 영역이 컨베이어 면까지 닿음 | 도킹 구역 안전 영역 전환 (`lane_planner.py`) |
| Windows 창 녹화(gdigrab)의 Isaac 화면이 멈춰 있음 | 창 캡처로는 3D 뷰포트가 갱신되지 않음 | Isaac 내부 카메라 캡처(`CABBAGE_CAPTURE_CAMS`) |

### 9-5. 한계 (실물 적용 전 보완)
- 차선을 정확히 따르는 것은 **남쪽 직선 구간과 첫 모서리 앞부분**까지다. 모서리 45° 이후는 Nav2 가 서쪽 차선 남쪽 가장자리(y ≈ -1.95)를 따라가고, FEEDER 앞에서 한 번 앞뒤로 방향을 고친다.
  차선을 끝까지 따르려면 리그가 제자리 회전할 수 있게 물리를 고치거나(B안, 팀 에셋 변경) 차선 모서리를 회전 반경에 맞게 바꿔야 한다(C안, 씬·지도 변경).
- FEEDER 앞 마지막 도킹(`feeder_dock`)은 `/cmd_vel` 을 직접 내고 도킹 구역에서는 정지 영역도 꺼지므로 **도킹 중에는 사람 정지가 걸리지 않는다**.
- `/navigation/result` 에 "사람 대기" reason 은 없다(팀 인터페이스 변경 필요). `/collision_monitor_state` 로 확인한다.
- 차선·사람 좌표는 v014 씬과 v011 지도 기준 고정값이다. 씬이 바뀌면 `15_floor_lines.py` 로 다시 뽑고 `lane_planner.py`, `human_crossing.py` 상수를 고친다.

## 10. 진행 중 작업 메모

- 바닥 차선 끝까지 따라가기(B안/C안)는 미착수.

---

### 부록: 새 PC 점검표 (배경 세팅 테스트용)

| 순서 | 확인 | 결과(채우기) |
|---|---|---|
| 1 | `D:\isaacsim\isaac-sim.bat` 실행 → 창 뜸 | |
| 2 | 1-2 의 ultralytics/torch/cv2 버전 출력 | |
| 3 | `wsl -l -v` 에 Ubuntu-24.04 VERSION 2 | |
| 4 | `ros2 pkg list \| grep smart_farm` 3개 | |
| 5 | 저장소 `smart_farm\tools\allinone_live\run_allinone.ps1`, `wsl\start_nav2_stack.sh` 존재 | |
| 6 | 씬 폴더 `Test-Path` 두 개 True (1-8) | |
| 7 | 3-2 실행 → `[flow] ... PLACE_INSPECT ... SUCCEEDED` | |
| 8 | `isaac.log` 에 `Pallet_01 완료` | |
| 9 | 5-1 공정 영상 7개 생성 | |
| 10 | 5-2 `drive_rviz_plus_comm.mp4` 생성 | |
