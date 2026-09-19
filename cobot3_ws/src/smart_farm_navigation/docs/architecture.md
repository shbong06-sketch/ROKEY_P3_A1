# smart_farm_navigation 노드 구성도 및 흐름도

기준: feature/navigation, 2026-09-19. 대상 장면 smart_farm_nav2_01.usd, ROS 2 Jazzy, ROS_DOMAIN_ID 101.
GitHub 또는 VS Code(Markdown Preview Mermaid Support)에서 그림으로 렌더링된다.

---

## 1. 배치도: 내피 · GitHub · 고피

```mermaid
flowchart LR
    subgraph NAEPI["내피 (코드 작성, ROS 2 Jazzy 있음, Isaac Sim 없음)"]
        DEV["코드·가이드 작성<br>내피 통합 시험<br>(fake odom 노드)"]
    end
    subgraph GH["GitHub  shbong06-sketch/ROKEY_P3_A1<br>branch: feature/navigation"]
        REPO[("저장소")]
    end
    subgraph GOPI["고피 (Isaac Sim 5.1 + ROS 2 Jazzy)"]
        T1["터미널 1<br>Isaac Sim + 장면 Play"]
        T2["터미널 2<br>colcon build · scene_check"]
        T3["터미널 3<br>ros2 launch escape.launch.py"]
        RES["results/*.txt"]
    end
    DEV -->|"commit · push"| REPO
    REPO -->|"git pull (사용자)"| T2
    T1 <-->|"DDS (domain 101)"| T2
    T1 <-->|"DDS (domain 101)"| T3
    T2 --> RES
    T3 --> RES
    RES -->|"commit · push (사용자)"| REPO
```

---

## 2. 노드·토픽 그래프 (rqt_graph 형식)

타원 = 노드, 사각형 = 토픽. 화살표 방향이 발행(→토픽) / 구독(토픽→)이다.
Isaac Sim 안의 노드 이름은 Action Graph 경로에서 자동 생성된 것이다.

```mermaid
flowchart LR
    subgraph ISAAC["Isaac Sim  smart_farm_nav2_01.usd  (ROS2 Bridge)"]
        SUB(["_World_nova_carter_ROS_differential_drive_ros2_subscribe_twist"])
        DC["DifferentialController<br>wheelRadius 0.14 · wheelDistance 0.413<br>max 1.0 m/s · 1.2 rad/s"]
        ODOMN(["ros2_publish_odometry<br>(IsaacComputeOdometry)"])
        TFN(["ros2_publish_transform_tree ×3"])
        LID(["ros2 RTX lidar / camera / imu helpers"])
    end

    CMD["/cmd_vel<br>geometry_msgs/Twist"]
    ODOM["/chassis/odom<br>nav_msgs/Odometry<br>odom → base_link, 60 Hz"]
    TF["/tf<br>tf2_msgs/TFMessage"]
    PC["/front_3d_lidar/lidar_points<br>sensor_msgs/PointCloud2"]
    IMG["/front_stereo_camera/*/image_raw<br>/…/imu"]

    ESC(["/escape_controller<br>(escape_controller.py)"])
    CHK(["/scene_check<br>(scene_check.py)"])
    CLI(["ros2 topic / rviz2<br>(관측용)"])

    ESC -->|"20 Hz"| CMD
    CMD --> SUB --> DC
    ODOMN --> ODOM
    TFN --> TF
    LID --> PC
    LID --> IMG
    ODOM --> ESC
    ODOM --> CHK
    TF --> CHK
    PC --> CLI
    ODOM --> CLI

    classDef topic fill:#fff7d6,stroke:#b8860b,color:#000;
    class CMD,ODOM,TF,PC,IMG topic;
```

현재 장면에서 발행되지 않는 것: `/clock`, `/tf_static`, `/front_2d_lidar/scan`. 에셋의 `/ros_clock` 그래프가 defaultPrim 밖에 있어 참조에 포함되지 않기 때문이며, Nav2 단계에서 `/World/ros_clock` 참조 추가로 해결한다.

### 2.1 TF 트리 (현재)

```mermaid
flowchart TB
    odom --> base_link
    base_link --> nova_carter
    nova_carter --> wheel_left
    nova_carter --> wheel_right
    nova_carter --> nova_carter_caster_frame_base
    base_link --> front_3d_lidar
    base_link --> front_2d_lidar
    base_link --> back_2d_lidar
    base_link --> cams["front/left/right/rear stereo · fisheye 카메라 프레임"]
    base_link --> chassis_imu
    map -.->|"Nav2 단계에서 AMCL 이 발행 (미구현)"| odom
```

---

## 3. escape_controller 상태 머신

```mermaid
stateDiagram-v2
    [*] --> WAITING : 노드 시작, zero Twist 발행
    WAITING --> WAITING : auto_start=false 또는 파라미터 미설정 또는 odom 없음
    WAITING --> ARC_REVERSE : auto_start=true 이고 신선한 odom 수신, 시작 자세·중앙선 기록
    ARC_REVERSE --> ARC_STOP : 누적 yaw 변화가 turn_angle_rad 도달
    ARC_REVERSE --> ABORTED : 경로가 reverse_max_distance_m 초과, 또는 stall_timeout_s 동안 yaw 정체, 또는 단계 제한시간 초과
    ARC_STOP --> FORWARD : settle_duration_s 경과
    FORWARD --> COMPLETE : 중앙선 방향 진행거리가 forward_distance_m 도달
    FORWARD --> ABORTED : stall_timeout_s 동안 거리 정체, 또는 단계 제한시간 초과
    ARC_REVERSE --> ABORTED : odom_timeout_s 동안 odom 없음
    FORWARD --> ABORTED : odom_timeout_s 동안 odom 없음
    COMPLETE --> [*] : zero Twist 를 stop_hold_duration_s 유지 후 종료 코드 0
    ABORTED --> [*] : zero Twist 를 stop_hold_duration_s 유지 후 종료 코드 2

    note right of ARC_REVERSE
        linear.x = -drive_direction_sign × reverse_speed_mps
        angular.z = sign(turn_angle_rad) × turn_speed_radps
        회전반경 = reverse_speed / turn_speed = 0.4 m
    end note
    note right of FORWARD
        linear.x = drive_direction_sign × forward_speed_mps
        angular.z = _follow_line():
          횡오차 e → 접근각 atan(cross_track_gain × e)
          → 진행방향 오차 × heading_hold_gain (상한 heading_hold_max_radps)
    end note
```

### 3.1 매 tick(0.05 s) 판정 순서

```mermaid
flowchart TD
    A["타이머 tick"] --> B{"COMPLETE 또는 ABORTED ?"}
    B -->|"예"| B1["zero Twist 발행<br>stop_hold 경과 시 finished=true"] --> Z["끝"]
    B -->|"아니오"| C{"auto_start ?"}
    C -->|"false"| C1["zero Twist 발행"] --> Z
    C -->|"true"| D{"파라미터 유효 ?"}
    D -->|"아니오"| D1["zero Twist 발행<br>오류 로그 1회"] --> Z
    D -->|"예"| E{"odom 신선 ?"}
    E -->|"아니오, WAITING"| Z
    E -->|"아니오, 주행 중"| E1["_abort(odom timeout)"] --> Z
    E -->|"예"| F{"현재 Phase"}
    F -->|"WAITING"| F0["시작 자세·중앙선 기록<br>_enter(ARC_REVERSE)"] --> Z
    F -->|"ARC_REVERSE"| F1["후진+회전 발행<br>각도 도달 → ARC_STOP<br>거리 초과·정체·시간 초과 → ABORTED"] --> Z
    F -->|"ARC_STOP"| F2["정지 발행<br>settle 경과 → FORWARD"] --> Z
    F -->|"FORWARD"| F3["직진+중앙선 조향 발행<br>거리 도달 → COMPLETE<br>정체·시간 초과 → ABORTED"] --> Z
```

### 3.2 기하 (odom 좌표계, 시작 자세 = 원점, yaw 0)

```mermaid
flowchart LR
    subgraph CORR["통로 (안쪽 폭 4.0 m, y −7.5 … +7.5)"]
        direction TB
        W1["/World/Cube  x = −2.5"]
        S["시작 (0, 0)<br>보이는 정면 = −x"]
        W2["/World/Cube_01  x = +2.5"]
    end
    S -->|"① 원호 후진 (후단이 우측으로)<br>반경 0.4 m, yaw +90°"| P1["(+0.4, +0.4)"]
    P1 -->|"② 정지 0.5 s"| P1
    P1 -->|"③ 중앙선 x=0 으로 복귀 후<br>−y 방향 9.0 m 직진"| P2["(0, −9.0)"]
```

---

## 4. 실행 절차 시퀀스 (터미널 3개)

```mermaid
sequenceDiagram
    autonumber
    participant U as 사용자
    participant T1 as 터미널 1
    participant SIM as Isaac Sim
    participant T2 as 터미널 2
    participant T3 as 터미널 3
    participant ESC as escape_controller

    U->>T1: ros_set · isaac_ros · env_check.sh
    T1-->>U: 판정 OK / WARN (CONFLICT 면 중단)
    U->>T1: isaac_python launch_scene.py
    T1->>SIM: SimulationApp → bridge 확장 → open_stage → Play
    SIM-->>T1: "[launch_scene] PLAY"
    Note over SIM: /chassis/odom, /tf, 센서 발행 시작<br>/cmd_vel 구독 대기

    U->>T2: ros_set · isaac_ros · env_check.sh
    U->>T2: colcon build → source → ros2 pkg executables
    U->>T2: ros2 run smart_farm_navigation scene_check
    T2->>SIM: /cmd_vel 구독자 수 조회, odom · tf 구독
    SIM-->>T2: odom 60 Hz, tf
    T2-->>U: [1]~[4] 점검 로그, RESULT: OK

    U->>T3: ros_set · isaac_ros · env_check.sh · source
    U->>T3: ros2 launch escape.launch.py auto_start:=true
    T3->>ESC: 노드 시작 (YAML + auto_start 인자)
    loop 20 Hz
        SIM-->>ESC: /chassis/odom
        ESC->>SIM: /cmd_vel (Twist)
    end
    ESC-->>T3: Phase ARC_REVERSE → ARC_STOP → FORWARD → COMPLETE
    ESC-->>T3: 종료 코드 0 (중단 시 2)
    U->>T3: Ctrl+C (launch 종료)
    U->>T1: Ctrl+C (Isaac Sim 종료, 재실행 시 초기 자세 복원)
    U->>U: results/*.txt 커밋 · 푸시
```

---

## 5. 향후 Nav2 단계에서의 위치 (참고, 미구현)

escape_controller 자리에 Nav2 스택이 들어가고, 상위에 task_manager · navigation_node 가 붙는다. Isaac Sim 쪽 토픽은 그대로 재사용한다.

```mermaid
flowchart LR
    TM(["task_manager"]) -->|"/navigation/command"| NAV(["navigation_node<br>BasicNavigator"])
    NAV -->|"NavigateToPose action"| N2["Nav2<br>map_server · amcl · planner · controller"]
    N2 -->|"/cmd_vel"| SIM["Isaac Sim<br>nova_carter_ROS"]
    SIM -->|"/chassis/odom · /tf · LiDAR"| N2
    SIM -.->|"/clock (추가 필요)"| N2
    N2 -->|"map → odom TF (AMCL)"| N2
    NAV -->|"/navigation/result"| TM
    RV(["rviz2"]) -.->|"Nav2 Goal · costmap 확인"| N2
```

선결 과제: `/clock` 발행 추가, `use_sim_time` 통일, 지도 YAML의 image 경로 수정, 로봇의 보이는 정면(chassis −x)과 base_link +x 불일치 해소.
