# [guidance1 11차] smartfarm_v1.usd 곡선 주행(path_runner_smooth) + ros_set 단독 운용 실측

feature/navigation(/cmd_vel 모션 제어) 트랙의 가이드임. Nav2 트랙은 feature/navigation2의 guidance2_12차부터 이어짐.

## 이번 변경 사항
- **전진 방향 부호**: 11차 관찰(팀원 모두 "후진으로 보임")에 따라 path_runner·path_runner_smooth의 `drive_direction_sign`을 `1.0`으로 바꿨음. 이제 Nav2와 같은 규칙(base_link +x = 전방)임. 장면에서 carter의 초기 yaw를 180도 돌려, base_link +x가 통로 탈출 방향(+y)을 보게 해 둘 것. 경유지는 시작 기준 좌표계라 값은 그대로임.
- **곡선 주행 노드 path_runner_smooth 신설**: 통로는 직선으로 나가고, 탈출점부터 도착 축 위의 접근점(도착점 0.8 m 앞)까지 하나의 곡선(3차 베지어, pure pursuit)으로 이동한 뒤, 도착 축을 따라 직진 접근하여 정지함. 내피 시험 결과 도착 오차 5 cm, 방향 오차 1.7도.
- **launch_scene.py**: `ISAAC_EXPERIENCE=full`을 앞에 붙이면 GUI(`isaac`)와 같은 확장 세트로 뜸(아래 3절).

## 고정 경로
- 워크스페이스 `/home/rokey/ROKEY_P3_A1/cobot3_ws`
- 장면 `/home/rokey/ROKEY_P3_A1/cobot3_ws/isaacpjt/smart_farm/scenes/smartfarm_v1/smartfarm_v1.usd`
- 설정 `/home/rokey/ROKEY_P3_A1/cobot3_ws/src/smart_farm_navigation/config/path_runner_smooth.yaml`
- 결과 `/home/rokey/ROKEY_P3_A1/cobot3_ws/src/smart_farm_navigation/results`

---

## 1. 곡선 주행 실행 (터미널 3개)

### [터미널 1] Isaac Sim
```bash
ros_set
bash /home/rokey/ROKEY_P3_A1/cobot3_ws/src/smart_farm_navigation/scripts/env_check.sh
isaac_python /home/rokey/ROKEY_P3_A1/cobot3_ws/src/smart_farm_navigation/scripts/launch_scene.py /home/rokey/ROKEY_P3_A1/cobot3_ws/isaacpjt/smart_farm/scenes/smartfarm_v1/smartfarm_v1.usd
```
(2절의 실측 B에서는 `ros_set` 대신 `isaac_ros`를 씀.)

### [터미널 2] 빌드·점검
```bash
ros_set
cd /home/rokey/ROKEY_P3_A1/cobot3_ws
rm -rf build/smart_farm_navigation install/smart_farm_navigation
colcon build --packages-select smart_farm_navigation
source install/setup.bash
ros2 run smart_farm_navigation scene_check 2>&1 | tee -a /home/rokey/ROKEY_P3_A1/cobot3_ws/src/smart_farm_navigation/results/check_$(date +%Y%m%d_%H%M).txt
```

### [터미널 3] 곡선 주행
```bash
ros_set
source /home/rokey/ROKEY_P3_A1/cobot3_ws/install/setup.bash
ros2 launch smart_farm_navigation path_smooth.launch.py auto_start:=true 2>&1 | tee -a /home/rokey/ROKEY_P3_A1/cobot3_ws/src/smart_farm_navigation/results/smooth_$(date +%Y%m%d_%H%M).txt
```

---
이미 옮겨놓은 컨베이어 큐브와 다시 반대방향으로 진행함. 그래서 또 후진으로만 주행했음.
곡선 주행 자체는 매끄럽게 잘됨
---

- 기대 로그: `path: N points, curve from (2.30,0.00) to approach (4.60,2.44), final (5.40,2.44)` → `Phase TRACK` → `approach point reached` → `Phase APPROACH` → `final point reached ... cross-track ±0.05` → `Path COMPLETE ... heading error ±2deg`.
- 뷰포트 기대: 통로를 직진으로 나온 뒤 멈춤 없이 왼쪽으로 휘어 컨베이어 앞 축에 올라탄 다음 짧게 직진해 정지. 정면이 앞서 가야 함(뒤로 가면 초기 yaw 180도 회전이 안 된 것).
- 직선·제자리회전 방식이 필요하면 `path.launch.py`(path_runner)를 그대로 쓰면 됨.

---

## 2. ros_set 단독 운용 검증 실측

목적: 모든 터미널에서 `ros_set`만 써도 Isaac Sim bridge와 외부 노드가 같은 ROS 2(Jazzy) 라이브러리로 동작함을 사실로 확인함. A와 B를 각각 한 번씩 수행하고 결과를 이 파일 하단에 붙임.

### 실측 A — 모든 터미널 `ros_set`만 (권장안)
1. 터미널 1~3을 1절 그대로 수행함(터미널 1에 `ros_set`만).
2. Isaac Sim이 PLAY 상태일 때 터미널 2에서 아래를 실행하고 출력 전체를 기록함.
```bash
ls /home/rokey/isaacsim/exts/isaacsim.ros2.bridge/
PID=$(pgrep -f "launch_scene.py" | head -1); echo "PID=$PID"
grep -oE "/[^ ]*/lib(rcl|rmw|rosidl|fastrtps|fastcdr|rclpy)[^ ]*\.so[^ ]*" /proc/$PID/maps | sed 's#/lib[^/]*\.so.*##' | sort | uniq -c
ros2 topic list | wc -l
```
- 판정: `uniq -c` 결과의 디렉터리가 `/opt/ros/jazzy/lib`만이면 Isaac Sim이 시스템 Jazzy를 쓰는 것임. `isaacsim.ros2.bridge/humble` 이 함께 나오면 두 배포판이 섞인 것임.
- scene_check RESULT: OK, 곡선 주행 COMPLETE 이면 A 통과.

---
rokey@IsaacSim03:~/ROKEY_P3_A1/cobot3_ws$ ls /home/rokey/isaacsim/exts/isaacsim.ros2.bridge/
PID=$(pgrep -f "launch_scene.py" | head -1); echo "PID=$PID"
grep -oE "/[^ ]*/lib(rcl|rmw|rosidl|fastrtps|fastcdr|rclpy)[^ ]*\.so[^ ]*" /proc/$PID/maps | sed 's#/lib[^/]*\.so.*##' | sort | uniq -c
ros2 topic list | wc -l
bin     humble    ogn
config  include   PACKAGE-LICENSES
data    isaacsim
docs    jazzy
PID=46269
16
rokey@IsaacSim03:~/ROKEY_P3_A1/cobot3_ws$ 

---

### 실측 B — 수업 방식 (터미널 1 `isaac_ros`만, 터미널 2·3 `ros_set`)
1. 터미널 1을 닫고 새로 열어 `ros_set` 없이 아래만 실행함.
```bash
isaac_ros
bash /home/rokey/ROKEY_P3_A1/cobot3_ws/src/smart_farm_navigation/scripts/env_check.sh
isaac_python /home/rokey/ROKEY_P3_A1/cobot3_ws/src/smart_farm_navigation/scripts/launch_scene.py /home/rokey/ROKEY_P3_A1/cobot3_ws/isaacpjt/smart_farm/scenes/smartfarm_v1/smartfarm_v1.usd
```

---
rokey@IsaacSim03:~/ROKEY_P3_A1/cobot3_ws$ ls /home/rokey/isaacsim/exts/isaacsim.ros2.bridge/
PID=$(pgrep -f "launch_scene.py" | head -1); echo "PID=$PID"
grep -oE "/[^ ]*/lib(rcl|rmw|rosidl|fastrtps|fastcdr|rclpy)[^ ]*\.so[^ ]*" /proc/$PID/maps | sed 's#/lib[^/]*\.so.*##' | sort | uniq -c
ros2 topic list | wc -l
bin     humble    ogn
config  include   PACKAGE-LICENSES
data    isaacsim
docs    jazzy
PID=46979
16
rokey@IsaacSim03:~/ROKEY_P3_A1/cobot3_ws$ 
---

2. 실측 A의 2번 명령을 그대로 실행하여 기록함. 터미널 3의 곡선 주행도 한 번 더 실행함.

---
위에서 말한 주행 결과와 일치함
---

### 판정 규칙
| 결과 | 의미 | 결정 |
| --- | --- | --- |
| A: `/opt/ros/jazzy/lib`만, OK, COMPLETE | 시스템 Jazzy 단독으로 정상 | 모든 터미널 `ros_set`만 사용 |
| B: humble 디렉터리가 보이는데도 동작 | 내장 humble bridge + 외부 Jazzy 노드가 통신은 됨 | 동작은 하나 배포판 혼용이므로 A를 표준으로 |
| A에서 topic이 안 보임 | 시스템 Jazzy 라이브러리로 bridge가 못 뜸 | 터미널 1 Isaac Sim 콘솔의 `ros2 bridge` 관련 줄을 기록해 올릴 것 |

---

## 3. standalone(isaac_python)과 GUI(isaac)의 확장 차이 확인
1절과 동일하되 터미널 1을 아래로 바꾸면 GUI와 같은 확장 세트로 뜸.
```bash
ros_set
ISAAC_EXPERIENCE=full isaac_python /home/rokey/ROKEY_P3_A1/cobot3_ws/src/smart_farm_navigation/scripts/launch_scene.py /home/rokey/ROKEY_P3_A1/cobot3_ws/isaacpjt/smart_farm/scenes/smartfarm_v1/smartfarm_v1.usd
```
- 확인: Window > Examples, Tools > Robotics(Occupancy Map) 메뉴가 보이는지. `ls /home/rokey/isaacsim/apps/`로 사용 가능한 .kit 목록도 기록함.
- 로딩 시간과 GPU 사용은 GUI와 같아짐. ROS 통신·주행에는 차이가 없어야 함.

---
==============NVSMI LOG==============

Timestamp                                              : Sun Sep 20 14:23:04 2026
Driver Version                                         : 580.173.02
CUDA Version                                           : 13.0

Attached GPUs                                          : 1
GPU 00000000:01:00.0
    Product Name                                       : NVIDIA GeForce RTX 5080 Laptop GPU
    Product Brand                                      : GeForce
    Product Architecture                               : Blackwell
    Display Mode                                       : Requested functionality has been deprecated
    Display Attached                                   : Yes
    Display Active                                     : Enabled
    Persistence Mode                                   : Disabled
    Addressing Mode                                    : HMM
    MIG Mode
        Current                                        : N/A
        Pending                                        : N/A
    Accounting Mode                                    : Disabled
    Accounting Mode Buffer Size                        : 4000
    Driver Model
        Current                                        : N/A
        Pending                                        : N/A
    Serial Number                                      : 0
    GPU UUID                                           : GPU-49db0f63-042c-20f9-5de2-99d6a91a9d34
    GPU PDI                                            : 0x8ae21ce04abcb64b
    Minor Number                                       : 0
    VBIOS Version                                      : 98.03.5C.00.73
    MultiGPU Board                                     : No
    Board ID                                           : 0x100
    Board Part Number                                  : N/A
    GPU Part Number                                    : 2C19-725-A1
    FRU Part Number                                    : N/A
    Platform Info
        Chassis Serial Number                          : 
        Slot Number                                    : 0
        Tray Index                                     : 0
        Host ID                                        : 1
        Peer Type                                      : Direct Connected
        Module Id                                      : 1
        GPU Fabric GUID                                : 0x0000000000000000
    Inforom Version
        Image Version                                  : G005.0000.98.01
        OEM Object                                     : 2.1
        ECC Object                                     : N/A
        Power Management Object                        : N/A
    Inforom BBX Object Flush
        Latest Timestamp                               : N/A
        Latest Duration                                : N/A
    GPU Operation Mode
        Current                                        : N/A
        Pending                                        : N/A
    GPU C2C Mode                                       : Disabled
    GPU Virtualization Mode
        Virtualization Mode                            : None
        Host VGPU Mode                                 : N/A
        vGPU Heterogeneous Mode                        : N/A
    GPU Recovery Action                                : None
    GSP Firmware Version                               : 580.173.02
    IBMNPU
        Relaxed Ordering Mode                          : N/A
    PCI
        Bus                                            : 0x01
        Device                                         : 0x00
        Domain                                         : 0x0000
        Base Classcode                                 : 0x3
        Sub Classcode                                  : 0x0
        Device Id                                      : 0x2C1910DE
        Bus Id                                         : 00000000:01:00.0
        Sub System Id                                  : 0x14741462
        GPU Link Info
            PCIe Generation
                Max                                    : 5
                Current                                : 1
                Device Current                         : 1
                Device Max                             : 5
                Host Max                               : 5
            Link Width
                Max                                    : 16x
                Current                                : 8x
        Bridge Chip
            Type                                       : N/A
            Firmware                                   : N/A
        Replays Since Reset                            : 0
        Replay Number Rollovers                        : 0
        Tx Throughput                                  : 13903 KB/s
        Rx Throughput                                  : 1077 KB/s
        Atomic Caps Outbound                           : N/A
        Atomic Caps Inbound                            : FETCHADD_32 FETCHADD_64 SWAP_32 SWAP_64 CAS_32 CAS_64 
    Fan Speed                                          : N/A
    Performance State                                  : P8
    Clocks Event Reasons
        Idle                                           : Not Active
        Applications Clocks Setting                    : Not Active
        SW Power Cap                                   : Not Active
        HW Slowdown                                    : Not Active
            HW Thermal Slowdown                        : Not Active
            HW Power Brake Slowdown                    : Not Active
        Sync Boost                                     : Not Active
        SW Thermal Slowdown                            : Not Active
        Display Clock Setting                          : Not Active
    Clocks Event Reasons Counters
        SW Power Capping                               : 12234991553 us
        Sync Boost                                     : 0 us
        SW Thermal Slowdown                            : 11591399010 us
        HW Thermal Slowdown                            : 0 us
        HW Power Braking                               : 0 us
    Sparse Operation Mode                              : N/A
    FB Memory Usage
        Total                                          : 16303 MiB
        Reserved                                       : 464 MiB
        Used                                           : 570 MiB
        Free                                           : 15271 MiB
    BAR1 Memory Usage
        Total                                          : 16384 MiB
        Used                                           : 30 MiB
        Free                                           : 16354 MiB
    Conf Compute Protected Memory Usage
        Total                                          : 0 MiB
        Used                                           : 0 MiB
        Free                                           : 0 MiB
    Compute Mode                                       : Default
    Utilization
        GPU                                            : 0 %
        Memory                                         : 0 %
        Encoder                                        : 0 %
        Decoder                                        : 0 %
        JPEG                                           : 0 %
        OFA                                            : 0 %
    Encoder Stats
        Active Sessions                                : 0
        Average FPS                                    : 0
        Average Latency                                : 0
    FBC Stats
        Active Sessions                                : 0
        Average FPS                                    : 0
        Average Latency                                : 0
    DRAM Encryption Mode
        Current                                        : Disabled
        Pending                                        : Disabled
    ECC Mode
        Current                                        : N/A
        Pending                                        : N/A
    ECC Errors
        Volatile
            SRAM Correctable                           : N/A
            SRAM Uncorrectable Parity                  : N/A
            SRAM Uncorrectable SEC-DED                 : N/A
            DRAM Correctable                           : N/A
            DRAM Uncorrectable                         : N/A
        Aggregate
            SRAM Correctable                           : N/A
            SRAM Uncorrectable Parity                  : N/A
            SRAM Uncorrectable SEC-DED                 : N/A
            DRAM Correctable                           : N/A
            DRAM Uncorrectable                         : N/A
            SRAM Threshold Exceeded                    : N/A
        Aggregate Uncorrectable SRAM Sources
            SRAM L2                                    : N/A
            SRAM SM                                    : N/A
            SRAM Microcontroller                       : N/A
            SRAM PCIE                                  : N/A
            SRAM Other                                 : N/A
        Channel Repair Pending                         : No
        TPC Repair Pending                             : No
        Unrepairable Memory                            : N/A
    Retired Pages
        Single Bit ECC                                 : N/A
        Double Bit ECC                                 : N/A
        Pending Page Blacklist                         : N/A
    Remapped Rows                                      : N/A
    Temperature
        GPU Current Temp                               : 49 C
        GPU T.Limit Temp                               : 37 C
        GPU Shutdown T.Limit Temp                      : -5 C
        GPU Slowdown T.Limit Temp                      : -2 C
        GPU Max Operating T.Limit Temp                 : 0 C
        GPU Target Temperature                         : 87 C
        Memory Current Temp                            : N/A
        Memory Max Operating T.Limit Temp              : N/A
    GPU Power Readings
        Average Power Draw                             : 12.72 W
        Instantaneous Power Draw                       : 14.47 W
        Current Power Limit                            : 80.00 W
        Requested Power Limit                          : 80.00 W
        Default Power Limit                            : 80.00 W
        Min Power Limit                                : 5.00 W
        Max Power Limit                                : 175.00 W
    GPU Memory Power Readings 
        Average Power Draw                             : N/A
        Instantaneous Power Draw                       : N/A
    Module Power Readings
        Average Power Draw                             : N/A
        Instantaneous Power Draw                       : N/A
        Current Power Limit                            : N/A
        Requested Power Limit                          : N/A
        Default Power Limit                            : N/A
        Min Power Limit                                : N/A
        Max Power Limit                                : N/A
    Power Smoothing                                    : N/A
    Workload Power Profiles
        Requested Profiles                             : N/A
        Enforced Profiles                              : N/A
    Clocks
        Graphics                                       : 427 MHz
        SM                                             : 427 MHz
        Memory                                         : 405 MHz
        Video                                          : 772 MHz
    Applications Clocks
        Graphics                                       : N/A
        Memory                                         : N/A
    Default Applications Clocks
        Graphics                                       : N/A
        Memory                                         : N/A
    Deferred Clocks
        Memory                                         : N/A
    Max Clocks
        Graphics                                       : 3090 MHz
        SM                                             : 3090 MHz
        Memory                                         : 14001 MHz
        Video                                          : 3090 MHz
    Max Customer Boost Clocks
        Graphics                                       : N/A
    Clock Policy
        Auto Boost                                     : N/A
        Auto Boost Default                             : N/A
    Fabric
        State                                          : N/A
        Status                                         : N/A
        CliqueId                                       : N/A
        ClusterUUID                                    : N/A
        Health
            Summary                                    : N/A
            Bandwidth                                  : N/A
            Route Recovery in progress                 : N/A
            Route Unhealthy                            : N/A
            Access Timeout Recovery                    : N/A
            Incorrect Configuration                    : N/A
            Partition Assigned                         : N/A
    Processes
        GPU instance ID                   : N/A
        Compute instance ID               : N/A
        Process ID                        : 2573
            Type                          : G
            Name                          : ............./Xorg
            Used GPU Memory               : 276 MiB
        GPU instance ID                   : N/A
        Compute instance ID               : N/A
        Process ID                        : 2829
            Type                          : G
            Name                          : ......../gnome-shell
            Used GPU Memory               : 49 MiB
        GPU instance ID                   : N/A
        Compute instance ID               : N/A
        Process ID                        : 3346
            Type                          : G
            Name                          : ............/xdg-desktop-portal-gnome
            Used GPU Memory               : 4 MiB
        GPU instance ID                   : N/A
        Compute instance ID               : N/A
        Process ID                        : 4217
            Type                          : G
            Name                          : ......../nautilus
            Used GPU Memory               : 25 MiB
        GPU instance ID                   : N/A
        Compute instance ID               : N/A
        Process ID                        : 6013
            Type                          : G
            Name                          : ................../chrome
            Used GPU Memory               : 52 MiB
        GPU instance ID                   : N/A
        Compute instance ID               : N/A
        Process ID                        : 12286
            Type                          : G
            Name                          : .............../code
            Used GPU Memory               : 65 MiB
        GPU instance ID                   : N/A
        Compute instance ID               : N/A
        Process ID                        : 44735
            Type                          : G
            Name                          : ......../gnome-text-editor
            Used GPU Memory               : 20 MiB
    Capabilities
        EGM                                            : disabled

'
2026-09-20T05:23:16Z [93ms] [Warning] [carb.crashreporter-breakpad.plugin] [crash]  osDistro = 'ubuntu'
2026-09-20T05:23:16Z [94ms] [Warning] [carb.crashreporter-breakpad.plugin] [crash]  osName = '24.04.5 LTS (Noble Numbat)'
2026-09-20T05:23:16Z [95ms] [Warning] [carb.crashreporter-breakpad.plugin] [crash]  osVersion = '24.04.5'
2026-09-20T05:23:16Z [95ms] [Warning] [carb.crashreporter-breakpad.plugin] [crash]  portableMode = '1'
2026-09-20T05:23:16Z [96ms] [Warning] [carb.crashreporter-breakpad.plugin] [crash]  primaryDisplayRes = '1920x1200x32bit@60Hz'
2026-09-20T05:23:16Z [97ms] [Warning] [carb.crashreporter-breakpad.plugin] [crash]  runEnvironment = 'Individual'
2026-09-20T05:23:17Z [98ms] [Warning] [carb.crashreporter-breakpad.plugin] [crash]  runningInContainer = '0'
2026-09-20T05:23:17Z [98ms] [Warning] [carb.crashreporter-breakpad.plugin] [crash]  shaderdb_debugSymbols = '0'
2026-09-20T05:23:17Z [99ms] [Warning] [carb.crashreporter-breakpad.plugin] [crash]  shaderdb_dumpIncludeOverrides = '0'
2026-09-20T05:23:17Z [100ms] [Warning] [carb.crashreporter-breakpad.plugin] [crash]  shaderdb_dumpIntermediates = '0'
2026-09-20T05:23:17Z [100ms] [Warning] [carb.crashreporter-breakpad.plugin] [crash]  shaderdb_obfuscateCode = '1'
2026-09-20T05:23:17Z [101ms] [Warning] [carb.crashreporter-breakpad.plugin] [crash]  shaderdb_optimizationLevel = '1'
2026-09-20T05:23:17Z [102ms] [Warning] [carb.crashreporter-breakpad.plugin] [crash]  systemInfo = '
|---------------------------------------------------------------------------------------------|
| Driver Version: 580.173.02    | Graphics API: Vulkan
|=============================================================================================|
| GPU | Name                             | Active | LDA | GPU Memory | Vendor-ID | LUID       |
|     |                                  |        |     |            | Device-ID | UUID       |
|     |                                  |        |     |            | Bus-ID    |            |
|---------------------------------------------------------------------------------------------|
| 0   | NVIDIA GeForce RTX 5080 Laptop.. | Yes: 0 |     | 16303   MB | 10de      | 0          |
|     |                                  |        |     |            | 2c19      | 49db0f63.. |
|     |                                  |        |     |            | 1         |            |
|---------------------------------------------------------------------------------------------|
| 1   | Intel(R) Graphics (ARL)          |        |     | 47779   MB | 8086      | 0          |
|     |                                  |        |     |            | 7d67      | 8680677d.. |
|     |                                  |        |     |            | 0         |            |
|=============================================================================================|
| OS: 24.04.5 LTS (Noble Numbat) ubuntu, Version: 24.04.5, Kernel: 6.14.0-27-generic
| XServer Vendor: The X.Org Foundation, XServer Version: 12101011 (1.21.1.11)
| Processor: Intel(R) Core(TM) Ultra 9 275HX
| Cores: 24 | Logical Cores: 24
|---------------------------------------------------------------------------------------------|
| Total Memory (MB): 63706 | Free Memory: 56265
| Total Page/Swap (MB): 8191 | Free Page/Swap: 8191
|---------------------------------------------------------------------------------------------|
'
2026-09-20T05:23:17Z [102ms] [Warning] [carb.crashreporter-breakpad.plugin] [crash]  telemetrySessionId = '10104021940587809249'
2026-09-20T05:23:17Z [103ms] [Warning] [carb.crashreporter-breakpad.plugin] [crash]  terminatedByAbort = '0'
2026-09-20T05:23:17Z [104ms] [Warning] [carb.crashreporter-breakpad.plugin] [crash]  totalRamBareMetalMB = '63706'
2026-09-20T05:23:17Z [104ms] [Warning] [carb.crashreporter-breakpad.plugin] [crash]  totalRamLimitedMB = '63706'
2026-09-20T05:23:17Z [105ms] [Warning] [carb.crashreporter-breakpad.plugin] [crash]  totalSwapBareMetalMB = '8191'
2026-09-20T05:23:17Z [106ms] [Warning] [carb.crashreporter-breakpad.plugin] [crash]  totalSwapLimitedMB = '8191'
2026-09-20T05:23:17Z [106ms] [Warning] [carb.crashreporter-breakpad.plugin] [crash]  userId = 'default'
2026-09-20T05:23:17Z [107ms] [Warning] [carb.crashreporter-breakpad.plugin] [crash]  workingDirectory = '/home/<username>/ROKEY_P3_A1'
2026-09-20T05:23:17Z [108ms] [Fatal] [carb.crashreporter-breakpad.plugin] [crash] Thread 47745 backtrace follows:
2026-09-20T05:23:17Z [207ms] [Fatal] [carb.crashreporter-breakpad.plugin] 000: libc.so.6!__sigaction+0x50 (libc_sigaction.c:?)
2026-09-20T05:23:17Z [214ms] [Fatal] [carb.crashreporter-breakpad.plugin] 001: libomni.graph.core.plugin.so!std::vector<unsigned long, std::allocator<unsigned long> >::_M_default_append(unsigned long)+0x1038 (??:?)
2026-09-20T05:23:17Z [217ms] [Fatal] [carb.crashreporter-breakpad.plugin] 002: libomni.graph.image.core.plugin.so!void std::vector<unsigned long, std::allocator<unsigned long> >::_M_realloc_insert<unsigned long>(__gnu_cxx::__normal_iterator<unsigned long*, std::vector<unsigned long, std::allocator<unsigned long> > >, unsigned long&&)+0x2cf4 (??:?)
2026-09-20T05:23:17Z [220ms] [Fatal] [carb.crashreporter-breakpad.plugin] 003: libomni.graph.image.core.plugin.so!std::pair<std::__detail::_Node_iterator<unsigned long, true, false>, bool> std::_Hashtable<unsigned long, unsigned long, std::allocator<unsigned long>, std::__detail::_Identity, std::equal_to<unsigned long>, std::hash<unsigned long>, std::__detail::_Mod_range_hashing, std::__detail::_Default_ranged_hash, std::__detail::_Prime_rehash_policy, std::__detail::_Hashtable_traits<false, true, true> >::_M_emplace<unsigned long const&>(std::integral_constant<bool, true>, unsigned long const&)+0x6230 (??:?)
2026-09-20T05:23:17Z [224ms] [Fatal] [carb.crashreporter-breakpad.plugin] 004: libomni.kit.exec.core.plugin.so!std::_Sp_counted_base<(__gnu_cxx::_Lock_policy)2>::_M_release()+0x80b4 (??:?)
2026-09-20T05:23:17Z [228ms] [Fatal] [carb.crashreporter-breakpad.plugin] 005: libomni.kit.exec.core.plugin.so!std::_Sp_counted_base<(__gnu_cxx::_Lock_policy)2>::_M_release()+0x82d1 (??:?)
2026-09-20T05:23:17Z [232ms] [Fatal] [carb.crashreporter-breakpad.plugin] 006: libtbb.so!tbb::empty_task::~empty_task()+0x47b5 (??:?)
2026-09-20T05:23:17Z [236ms] [Fatal] [carb.crashreporter-breakpad.plugin] 007: libtbb.so!tbb::empty_task::~empty_task()+0x4afc (??:?)
2026-09-20T05:23:17Z [239ms] [Fatal] [carb.crashreporter-breakpad.plugin] 008: libomni.kit.exec.core.plugin.so!std::_Sp_counted_base<(__gnu_cxx::_Lock_policy)2>::_M_release()+0x7744 (??:?)
2026-09-20T05:23:17Z [241ms] [Fatal] [carb.crashreporter-breakpad.plugin] 009: libomni.kit.exec.core.plugin.so!+0xbd75
2026-09-20T05:23:17Z [243ms] [Fatal] [carb.crashreporter-breakpad.plugin] 010: libomni.kit.exec.core.plugin.so!+0xd261
2026-09-20T05:23:17Z [245ms] [Fatal] [carb.crashreporter-breakpad.plugin] 011: libomni.kit.exec.core.plugin.so!+0xeebf
2026-09-20T05:23:17Z [249ms] [Fatal] [carb.crashreporter-breakpad.plugin] 012: libomni.usd.so!std::_Function_handler<std::unique_ptr<std::__future_base::_Result_base, std::__future_base::_Result_base::_Deleter> (), std::__future_base::_Task_setter<std::unique_ptr<std::__future_base::_Result<void>, std::__future_base::_Result_base::_Deleter>, std::thread::_Invoker<std::tuple<omni::usd::UsdContext::Impl::spawnLoaderThread<omni::usd::UsdContext::Impl::reopenUsd(bool, bool, omni::fabric::SampleIndex)::{lambda()#1}>(omni::usd::UsdContext::Impl::reopenUsd(bool, bool, omni::fabric::SampleIndex)::{lambda()#1}&&, bool)::{lambda()#1}> >, void> >::_M_invoke(std::_Any_data const&)+0x34e8 (??:?)
2026-09-20T05:23:17Z [252ms] [Fatal] [carb.crashreporter-breakpad.plugin] 013: libomni.usd.so!std::_Function_handler<std::unique_ptr<std::__future_base::_Result_base, std::__future_base::_Result_base::_Deleter> (), std::__future_base::_Task_setter<std::unique_ptr<std::__future_base::_Result<void>, std::__future_base::_Result_base::_Deleter>, std::thread::_Invoker<std::tuple<omni::usd::UsdContext::Impl::spawnLoaderThread<omni::usd::UsdContext::Impl::reopenUsd(bool, bool, omni::fabric::SampleIndex)::{lambda()#1}>(omni::usd::UsdContext::Impl::reopenUsd(bool, bool, omni::fabric::SampleIndex)::{lambda()#1}&&, bool)::{lambda()#1}> >, void> >::_M_invoke(std::_Any_data const&)+0x4080 (??:?)
2026-09-20T05:23:17Z [256ms] [Fatal] [carb.crashreporter-breakpad.plugin] 014: libomni.usd.so!std::__detail::_Compiler<std::__cxx11::regex_traits<char> >::_M_assertion()+0x1260 (??:?)
2026-09-20T05:23:17Z [261ms] [Fatal] [carb.crashreporter-breakpad.plugin] 015: libcarb.events.plugin.so!std::_Sp_counted_base<(__gnu_cxx::_Lock_policy)2>::_M_release()+0x6f6f (??:?)
2026-09-20T05:23:17Z [265ms] [Fatal] [carb.crashreporter-breakpad.plugin] 016: libcarb.events.plugin.so!std::_Sp_counted_base<(__gnu_cxx::_Lock_policy)2>::_M_release()+0x7504 (??:?)
2026-09-20T05:23:17Z [268ms] [Fatal] [carb.crashreporter-breakpad.plugin] 017: libomni.timeline.plugin.so!std::_Sp_counted_base<(__gnu_cxx::_Lock_policy)2>::_M_release()+0x2c69 (??:?)
2026-09-20T05:23:17Z [271ms] [Fatal] [carb.crashreporter-breakpad.plugin] 018: libomni.timeline.plugin.so!std::_Sp_counted_base<(__gnu_cxx::_Lock_policy)2>::_M_release()+0x41a9 (??:?)
2026-09-20T05:23:17Z [275ms] [Fatal] [carb.crashreporter-breakpad.plugin] 019: libcarb.eventdispatcher.plugin.so!carbOnPluginRegisterEx+0x3441 (??:?)
2026-09-20T05:23:17Z [279ms] [Fatal] [carb.crashreporter-breakpad.plugin] 020: libomni.kit.loop-isaac.plugin.so!void std::__detail::__to_chars_10_impl<unsigned int>(char*, unsigned int, unsigned int)+0x1a48d (??:?)
2026-09-20T05:23:17Z [281ms] [Fatal] [carb.crashreporter-breakpad.plugin] 021: libomni.kit.app.plugin.so!+0x235e1
2026-09-20T05:23:17Z [285ms] [Fatal] [carb.crashreporter-breakpad.plugin] 022: libomni.kit.app.plugin.so!carbOnPluginPreStartup+0x1fa0 (??:?)
2026-09-20T05:23:17Z [290ms] [Fatal] [carb.crashreporter-breakpad.plugin] 023: _app.cpython-311-x86_64-linux-gnu.so!std::__detail::_Map_base<std::__cxx11::basic_string<char, std::char_traits<char>, std::allocator<char> >, std::pair<std::__cxx11::basic_string<char, std::char_traits<char>, std::allocator<char> > const, void*>, std::allocator<std::pair<std::__cxx11::basic_string<char, std::char_traits<char>, std::allocator<char> > const, void*> >, std::__detail::_Select1st, std::equal_to<std::__cxx11::basic_string<char, std::char_traits<char>, std::allocator<char> > >, std::hash<std::__cxx11::basic_string<char, std::char_traits<char>, std::allocator<char> > >, std::__detail::_Mod_range_hashing, std::__detail::_Default_ranged_hash, std::__detail::_Prime_rehash_policy, std::__detail::_Hashtable_traits<true, false, true>, true>::operator[](std::__cxx11::basic_string<char, std::char_traits<char>, std::allocator<char> >&&)+0xcfd7 (??:?)
2026-09-20T05:23:17Z [294ms] [Fatal] [carb.crashreporter-breakpad.plugin] 024: _app.cpython-311-x86_64-linux-gnu.so!std::__detail::_Map_base<std::__cxx11::basic_string<char, std::char_traits<char>, std::allocator<char> >, std::pair<std::__cxx11::basic_string<char, std::char_traits<char>, std::allocator<char> > const, void*>, std::allocator<std::pair<std::__cxx11::basic_string<char, std::char_traits<char>, std::allocator<char> > const, void*> >, std::__detail::_Select1st, std::equal_to<std::__cxx11::basic_string<char, std::char_traits<char>, std::allocator<char> > >, std::hash<std::__cxx11::basic_string<char, std::char_traits<char>, std::allocator<char> > >, std::__detail::_Mod_range_hashing, std::__detail::_Default_ranged_hash, std::__detail::_Prime_rehash_policy, std::__detail::_Hashtable_traits<true, false, true>, true>::operator[](std::__cxx11::basic_string<char, std::char_traits<char>, std::allocator<char> >&&)+0x88ae (??:?)
2026-09-20T05:23:17Z [338ms] [Fatal] [carb.crashreporter-breakpad.plugin] 025: libpython3.11.so.1.0!cfunction_call+0x37 (/builds/omniverse/externals/python-build/_build/Python-3.11.13/Objects/methodobject.c:542)
2026-09-20T05:23:17Z [361ms] [Fatal] [carb.crashreporter-breakpad.plugin] 026: libpython3.11.so.1.0!_PyObject_MakeTpCall+0x95 (/builds/omniverse/externals/python-build/_build/Python-3.11.13/./Include/internal/pycore_ceval.h:123)
2026-09-20T05:23:17Z [426ms] [Fatal] [carb.crashreporter-breakpad.plugin] 027: libpython3.11.so.1.0!_PyEval_EvalFrameDefault+0x8b8c (/builds/omniverse/externals/python-build/_build/Python-3.11.13/Python/ceval.c:7285)
2026-09-20T05:23:17Z [491ms] [Fatal] [carb.crashreporter-breakpad.plugin] 028: libpython3.11.so.1.0!_PyEval_Vector+0xba (/builds/omniverse/externals/python-build/_build/Python-3.11.13/./Include/internal/pycore_ceval.h:73)
2026-09-20T05:23:17Z [515ms] [Fatal] [carb.crashreporter-breakpad.plugin] 029: libpython3.11.so.1.0!_PyObject_FastCallDictTstate+0xb9 (/builds/omniverse/externals/python-build/_build/Python-3.11.13/Objects/call.c:154)
2026-09-20T05:23:17Z [538ms] [Fatal] [carb.crashreporter-breakpad.plugin] 030: libpython3.11.so.1.0!_PyObject_Call_Prepend+0xe4 (/builds/omniverse/externals/python-build/_build/Python-3.11.13/Objects/call.c:489)
2026-09-20T05:23:17Z [587ms] [Fatal] [carb.crashreporter-breakpad.plugin] 031: libpython3.11.so.1.0!slot_tp_init+0xcc (/builds/omniverse/externals/python-build/_build/Python-3.11.13/./Include/object.h:537)
2026-09-20T05:23:17Z [637ms] [Fatal] [carb.crashreporter-breakpad.plugin] 032: libpython3.11.so.1.0!type_call+0x91 (/builds/omniverse/externals/python-build/_build/Python-3.11.13/Objects/typeobject.c:1104)
2026-09-20T05:23:17Z [660ms] [Fatal] [carb.crashreporter-breakpad.plugin] 033: libpython3.11.so.1.0!_PyObject_Call+0x58 (/builds/omniverse/externals/python-build/_build/Python-3.11.13/./Include/internal/pycore_ceval.h:123)
2026-09-20T05:23:17Z [724ms] [Fatal] [carb.crashreporter-breakpad.plugin] 034: libpython3.11.so.1.0!_PyEval_EvalFrameDefault+0x5d8d (/builds/omniverse/externals/python-build/_build/Python-3.11.13/Python/ceval.c:7349)
2026-09-20T05:23:17Z [788ms] [Fatal] [carb.crashreporter-breakpad.plugin] 035: libpython3.11.so.1.0!PyEval_EvalCode+0x4ba (/builds/omniverse/externals/python-build/_build/Python-3.11.13/./Include/internal/pycore_ceval.h:73)
2026-09-20T05:23:17Z [868ms] [Fatal] [carb.crashreporter-breakpad.plugin] 036: libpython3.11.so.1.0!run_mod+0xa5 (/builds/omniverse/externals/python-build/_build/Python-3.11.13/Python/pythonrun.c:1742)
2026-09-20T05:23:17Z [944ms] [Fatal] [carb.crashreporter-breakpad.plugin] 037: libpython3.11.so.1.0!_PyRun_SimpleFileObject+0x130 (/builds/omniverse/externals/python-build/_build/Python-3.11.13/Python/pythonrun.c:1662)
2026-09-20T05:23:17Z [1,023ms] [Fatal] [carb.crashreporter-breakpad.plugin] 038: libpython3.11.so.1.0!_PyRun_AnyFileObject+0x3c (/builds/omniverse/externals/python-build/_build/Python-3.11.13/Python/pythonrun.c:79)
2026-09-20T05:23:18Z [1,106ms] [Fatal] [carb.crashreporter-breakpad.plugin] 039: libpython3.11.so.1.0!Py_RunMain+0x793 (/builds/omniverse/externals/python-build/_build/Python-3.11.13/Modules/main.c:361)
2026-09-20T05:23:18Z [1,192ms] [Fatal] [carb.crashreporter-breakpad.plugin] 040: libpython3.11.so.1.0!Py_BytesMain+0x43 (/builds/omniverse/externals/python-build/_build/Python-3.11.13/Modules/main.c:739)
2026-09-20T05:23:18Z [1,227ms] [Fatal] [carb.crashreporter-breakpad.plugin] 041: libc.so.6!__libc_init_first+0x8a (./csu/../sysdeps/x86/libc-start.c:74)
2026-09-20T05:23:18Z [1,260ms] [Fatal] [carb.crashreporter-breakpad.plugin] 042: libc.so.6!__libc_start_main+0x8b (./csu/../csu/libc-start.c:128)
2026-09-20T05:23:18Z [1,264ms] [Fatal] [carb.crashreporter-breakpad.plugin] 043: python3!_start+0x29 (??:?)
/home/rokey/isaacsim/python.sh: line 73: 47745 Segmentation fault      (core dumped) $python_exe "${filtered_args[@]}" $args
There was an error running python

---
에러가 난것으로 보임. 일단은 중요도는 낮아졌으니 나중에 확인하겠음.
---

## 기록
- results/의 check_, smooth_ 로그와 2·3절 출력을 커밋·푸시함. 최신 smartfarm_v1.usd(회전한 carter, 옮긴 컨베이어)도 커밋함.

---
용량이 너무 커서 assets/은 옮기지 않음.
무엇보다 이제는 진짜 메인장면 usd로 옮기므로 가이던스 13차까지 진행하고 최종 피드백 넣겠음
---
