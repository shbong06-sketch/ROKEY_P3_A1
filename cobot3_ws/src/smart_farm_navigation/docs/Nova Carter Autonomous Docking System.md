# **Nova Carter Autonomous Docking System.md**

*User prompt: https://github.com/NVIDIA-ISAAC-ROS/nova\_carter/blob/main/nova\_carter\_docking/scripts/fp\_dock\_pose\_publisher.py \-- nova carter 3D Array(아마도 3D point cloud..? 잘 모르겠습니다.) 기반으로 현재 도킹 위치를 발행해주는 코드인 듯 합니다. https://github.com/open-navigation/opennav\_docking-- 도킹 프레임워크 https://github.com/Fonyuy45/nova\_carter\_mpc/blob/main/README.md \-- 센서 추적 기반 도킹 위치 정렬 알고리즘? 인 것 같습니다. \--- 각각의 github url이 어떤 코드에 대한 저장소인지 정확한 설명과 활용 방법을 나한테 알려줄 것.*

Response: 각 링크에 대한 정확한 코드/저장소의 정체와 실제 활용 방법입니다.

### **1\. nova\_carter/.../fp\_dock\_pose\_publisher.py**

> * **URL:** \[https://github.com/NVIDIA-ISAAC-ROS/nova\_carter/blob/main/nova\_carter\_docking/scripts/fp\_dock\_pose\_publisher.py\](https://github.com/NVIDIA-ISAAC-ROS/nova\_carter/blob/main/nova\_carter\_docking/scripts/fp\_dock\_pose\_publisher.py)

#### **1\) 정확한 설명**

> * **이름의 'fp' 의미:** 3D Point Cloud(점군)가 아니라 **FoundationPose**\[\*NVIDIA에서 개발한 객체 6D(위치 및 회전) 자세 추정 및 추적 딥러닝 파이프라인\]를 의미합니다.  
> * **코드의 역할:** NVIDIA의 AMR(자율이동로봇) 참조 플랫폼인 **Nova Carter** 환경에서, AI 기반 객체 포즈 추정 모델인 FoundationPose가 도킹 스테이션을 인식하여 출력한 vision\_msgs/msg/Detection3DArray 메시지를 구독합니다. 이 배열 데이터에서 스테이션의 3차원 위치 및 방향을 추출한 뒤, 내비게이션 및 도킹 시스템이 소비하기 쉬운 **geometry\_msgs/msg/PoseStamped 형태의 외부 감지 포즈(External Detection Pose)** 로 변환하여 퍼블리시하는 어댑터(노드)입니다.

#### **2\) 활용 방법**

> * **Isaac Sim 가상 환경 및 실물 로봇 연동:** 도킹 스테이션에 별도의 물리 마커(예: AprilTag)를 부착하지 않고, 카메라 영상 기반의 딥러닝(FoundationPose)을 통해 도킹 스테이션 자체의 CAD 형상을 인식하여 정밀 접근하고자 할 때 사용합니다.  
> * **OpenNav Docking과의 연계:** 후술할 opennav\_docking의 감지 토픽(detected\_dock\_pose)으로 본 노드의 출력 토픽을 매핑하여, 로봇이 도킹 직전 도킹 스테이션의 정밀한 상대 위치를 실시간 보정(Refinement)받도록 구성합니다.

### **2\. open-navigation/opennav\_docking**

> * **URL:** \[https://github.com/open-navigation/opennav\_docking\](https://github.com/open-navigation/opennav\_docking)

#### **1\) 정확한 설명**

> * **정체:** ROS 2 표준 내비게이션 스택인 Nav2(Navigation2) 생태계에서 제공하는 **공식 오픈소스 자율 도킹 프레임워크(Docking Task Server)** 입니다 (Open Navigation 제작, NVIDIA 후원).  
> * **주요 아키텍처 및 원리:**  
  1. **DockingServer:** 도킹/언도킹 액션 요청(DockRobot.action)을 관리하는 메인 액션 서버.  
  2. **Navigator:** 도킹을 시작하기 전, 안전하게 센서 인식이 가능한 대기 위치인 **스테이징 포즈(Staging Pose)**\[\*도킹 절차를 시작하기 위해 로봇이 진입하는 기준 대기 위치. 센서가 도킹 스테이션을 정면으로 포착할 수 있는 지점\]로 로봇을 일반 Nav2 내비게이션을 통해 이동시킵니다.  
  3. **DockDatabase:** 작업 환경에 설치된 여러 도킹 스테이션의 위치 및 타입을 YAML/파라미터 기반으로 등록·관리합니다.  
  4. **Controller:** 나선형(Spiral-based) 또는 부드러운 호를 그리는 **Graceful Controller**를 사용하여 정밀 비전-제어 루프(Vision-control loop)로 도크에 진입합니다.  
  5. **ChargingDock 플러그인:** 도크 종류에 따라 마커(AprilTag), 카메라, 라이다 등으로 정밀 포즈를 보정(getRefinedPose)하고 충전 접촉 여부(isDocked, isCharging)를 판단할 수 있게 인터페이스를 추상화한 플러그인 구조입니다.

#### **2\) 활용 방법**

> * **BT(Behavior Tree) 통합:** 로봇의 전체 미션 완료 후 또는 배터리 부족 시점(BatteryLow)에 Nav2 Behavior Tree 내부에서 DockRobot 액션을 호출하여 자동 복귀 및 충전 시퀀스를 자동화합니다.  
> * **플러그인 커스텀:** AprilTag 인식기나 앞선 FoundationPose 기반 퍼블리셔(fp\_dock\_pose\_publisher.py)의 토픽을 구독하도록 파라미터를 설정하여, 환경에 맞는 고정밀 도킹 파이프라인을 구축할 수 있습니다.

### **3\. Fonyuy45/nova\_carter\_mpc**

> * **URL:** \[https://github.com/Fonyuy45/nova\_carter\_mpc\](https://github.com/Fonyuy45/nova\_carter\_mpc)

#### **1\) 정확한 설명**

> * **정체:** Nova Carter 로봇의 하드웨어 스펙을 기반으로 한 **MATLAB 환경의 차동 구동(Differential Drive) 비선형 모델 예측 제어기(NMPC)\[\*로봇의 동역학 모델과 제약 조건을 기반으로 미래 궤적을 실시간 최적화하여 제어 입력을 산출하는 비선형 모델 예측 제어\] 및 확장 칼만 필터(EKF)\[\*센서 잡음이 존재하는 비선형 시스템에서 로봇의 위치 및 속도 상태를 추정하는 필터 알고리즘\] 센서 퓨전 시뮬레이션 프로젝트**입니다.  
> * **핵심 내용:**  
  * 도킹 특화 전용 알고리즘이라기보다는, 휠 엔코더, IMU, 초음파/거리 센서 등에서 들어오는 잡음 섞인 데이터를 **EKF**로 융합하여 로봇의 실제 상태(X, Y, Yaw, 선속도, 각속도)를 추정합니다.  
  * 추정된 상태를 기반으로 비선형 제약 조건(모터 속도 한계, 장애물 회피, 경로 오차 최소화)을 만족하며 목표 경로를 추종하는 **NMPC** 제어 알고리즘을 MATLAB/Simulink 코드로 구현해 둔 개인/연구용 저장소입니다.

#### **2\) 활용 방법**

> * **알고리즘 검증 및 파라미터 튜닝:** ROS 2 환경에 제어기를 배포하기 전, 가혹한 센서 노이즈 환경에서 로봇의 경로 추종 성능 및 제약 조건 만족 여부를 MATLAB 수치 해석 환경에서 사전 시뮬레이션하고 검증할 때 참고할 수 있습니다.  
> * **고성능 경로 제어기 설계:** 고정밀 도킹의 마지막 접근 구간이나 좁은 공간에서의 이동 제어를 위해, 표준 비례-적분(PI) 제어나 Graceful Controller 대신 제약 조건 최적화가 가능한 비선형 제어기(MPC)를 직접 구현하고자 할 때 모델링 수식 및 필터 설계 레퍼런스로 활용합니다.

### **3개 프로젝트의 전체적인 파이프라인 구성**

이 세 컴포넌트를 실제 자율 도킹 시스템으로 엮는 구조는 다음과 같이 정리할 수 있습니다.

\[카메라 입력\]&nbsp;  
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;↓  
\[FoundationPose 모델\]&nbsp;  
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;↓ (Detection3DArray)  
\[fp\_dock\_pose\_publisher.py\] ──→ (정밀 도킹 상대 위치: PoseStamped)  
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;↓  
\[opennav\_docking (전체 도킹 상태 머신)\] ──→ \[경로 생성 및 오차 계산\]  
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;↓  
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;\[nova\_carter\_mpc 스타일의 NMPC / 제어 루프\] ──→ \[바퀴 구동 속도(cmd\_vel)\]

> * open-navigation/opennav\_docking이 상위에서 전체 도킹 프로세스(내비게이션 이동 → 비전 정밀 정렬 → 접촉 감지 → 충전 시작)를 총괄 관리하는 **뼈대 프레임워크**입니다.  
> * fp\_dock\_pose\_publisher.py는 그 프레임워크에 들어가는 **센서 정밀 인식단(FoundationPose 인식 결과를 ROS 2 포즈로 변환)** 역할을 수행합니다.  
> * nova\_carter\_mpc는 도킹이나 일반 주행 시 센서 노이즈를 걸러내고(EKF) 목표 자세로 정밀하게 수렴시키는 **하위 수학적 궤적 추종 제어기(MPC)** 연구용 레퍼런스로 기능합니다.

*참고: 위 설명 중 본문에서 처음 등장한 생소한 용어(FoundationPose, 스테이징 포즈, NMPC, EKF)는 주석으로 설명을 추가하였습니다.*

---

*User prompt: 아이작 심을 통해 주행로봇 카터를 Nav2 연결 및 RViz2 상에서 시각화와 커맨딩을 하고 있음. 다음과 같은 코드가 관여하고 있을때, 해당 컴포넌트들을 어떻게 적용해야할지 나한테 알려줄 것.*

Response: 현재 시스템(Isaac Sim 시뮬레이션, Nav2 스택, M0609 매니퓰레이터 시퀀서)에 앞서 언급된 **opennav\_docking**, **fp\_dock\_pose\_publisher.py (FoundationPose 어댑터)**, **nova\_carter\_mpc** 컴포넌트를 결합하고 적용하는 아키텍처와 구체적인 적용 절차입니다.

### **1\. 현행 시스템의 문제점 및 컴포넌트 도입 배경**

제공해주신 guidance2\_15차.md와 integration\_v1.py를 보면 다음과 같은 한계가 명시되어 있습니다.

> 1. **정밀 도킹 자세 오차:** RPP(Regulated Pure Pursuit)\[\*사전 정의된 경로 상의 전방 주시 거리를 추종하며 곡률과 장애물에 따라 속도를 감속하는 제어기\] 컨트롤러는 후진 시 제자리 회전을 지원하지 못해 도킹 지점에서 **방향 오차가 7\~33° 가량 잔여**합니다.  
> 2. **매니퓰레이터 작업의 엄격한 도킹 허용선:** robot\_motion.py의 check\_base\_pose는 팔레트 기준 오차(앞뒤 0.89\~1.05m, 좌우 \-0.21\~0.34m, 각도 ±25°)를 엄격히 제한하며, integration\_v1.py의 도착 허용 오차는 0.15m로 규정되어 있어 일반 Nav2 경로 추종만으로는 IK(역기구학) 실패 또는 팔레트 충돌 위험이 상존합니다.  
> 3. **자체 반사 및 위치 추정 드리프트:** 팔 처짐이나 팔레트 파지로 인한 센서 반사 노이즈가 AMCL 위치 추정에 간섭을 줍니다.

### **2\. 컴포넌트별 배치 및 전체 파이프라인**

각 컴포넌트의 역할과 연결 구조는 다음과 같습니다.

\[ Isaac Sim (Collected\_smartfarm) \]  
&nbsp;&nbsp;│  
&nbsp;&nbsp;├─ 3D Lidar / Camera (Isaac ROS Bridge)  
&nbsp;&nbsp;│    │  
&nbsp;&nbsp;│    ├─ (RGB-D / PointCloud) ──→ \[ FoundationPose \]  
&nbsp;&nbsp;│    │                                │ (vision\_msgs/Detection3DArray)  
&nbsp;&nbsp;│    │                                ↓  
&nbsp;&nbsp;│    │                     \[ fp\_dock\_pose\_publisher.py \]  
&nbsp;&nbsp;│    │                                │ (geometry\_msgs/PoseStamped: /detected\_dock\_pose)  
&nbsp;&nbsp;│    │                                ↓  
&nbsp;&nbsp;│    └─ \[ Nav2 Global Navigation \] ──→ \[ opennav\_docking \]  
&nbsp;&nbsp;│          (대기 지점 Staging Pose까지 이동)   │ (Vision-control Loop 실행)  
&nbsp;&nbsp;│                                             │  
&nbsp;&nbsp;│                                             ├─ \[ Graceful Controller / nova\_carter\_mpc \]  
&nbsp;&nbsp;│                                             │    └─ cmd\_vel 산출 (정밀 제어)  
&nbsp;&nbsp;│                                             ↓  
&nbsp;&nbsp;└─ \[ differential\_drive /cmd\_vel \] ←──────────┘  
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;│  
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;▼  
&nbsp;\[ 정밀 도킹 완료 (isDocked & settled) \] ──→ \[ integration\_v1.py (Place 시퀀스 트리거) \]

### **3\. 단계별 상세 적용 가이드**

#### **1단계: 도킹 스테이션 포즈 발행 (fp\_dock\_pose\_publisher.py 적용)**

> * **목적:** 피더(Feeder) 또는 랙(Rack) 도킹 스테이션의 정밀 6D 포즈를 ROS 2 토픽으로 변환.  
> * **적용 절차:**  
  1. Isaac Sim의 카터 전방/후방 카메라 센서에서 나오는 RGB-D 영상을 ROS 2 Bridge로 스트리밍합니다.  
  2. FoundationPose 추론 노드가 도킹 스테이션의 CAD 메시를 기반으로 Detection3DArray 토픽을 발행하도록 설정합니다.  
  3. fp\_dock\_pose\_publisher.py를 실행하여 이를 단일 geometry\_msgs/msg/PoseStamped 형태의 /detected\_dock\_pose 토픽으로 변환합니다. *(만약 FoundationPose 대신 AprilTag나 라이다 ICP를 사용하는 경우에도 동일하게 PoseStamped 형태로 /detected\_dock\_pose에 발행하면 하위 파이프라인이 호환됩니다.)*

#### **2단계: Nav2 도킹 프레임워크 연동 (opennav\_docking 적용)**

> * **목적:** FEEDER\_APPROACH에서 FEEDER\_DOCK으로 진입하는 수동 클릭/스크립트 방식을 표준 도킹 상태머신으로 대체.  
> * **설정 방법 (opennav\_docking 파라미터 구성):**  
  * dock\_database 또는 파라미터 파일에 피더/랙 도킹 지점을 등록합니다.  
  * 기존 guidance2\_15차.md의 FEEDER\_APPROACH(-2.19, \-1.55)를 **Staging Pose**로, FEEDER\_DOCK(-2.19, \-2.60)을 **Dock Pose**로 매핑합니다.  
  * SimpleChargingDock 플러그인을 활성화하고 외부 포즈 구독 설정을 켭니다.  
    docking\_server:  
    &nbsp;&nbsp;ros\_\_parameters:  
    &nbsp;&nbsp;&nbsp;&nbsp;controller\_frequency: 50.0  
    &nbsp;&nbsp;&nbsp;&nbsp;base\_frame: "base\_link"  
    &nbsp;&nbsp;&nbsp;&nbsp;fixed\_frame: "odom"  
    &nbsp;&nbsp;&nbsp;&nbsp;dock\_backwards: true            \# 후진 진입 시 true  
    &nbsp;&nbsp;&nbsp;&nbsp;dock\_plugins: \["feeder\_dock"\]  
    &nbsp;&nbsp;&nbsp;&nbsp;feeder\_dock:  
    &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;plugin: "opennav\_docking::SimpleChargingDock"  
    &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;use\_external\_detection\_pose: true  
    &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;\# 1단계에서 발행한 토픽 이름 매핑  
    &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;external\_detection\_pose\_topic: "detected\_dock\_pose"  
    &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;\# dock 기준 staging pose 오프셋 (FEEDER\_APPROACH 와의 거리: 약 1.05m 전방)  
    &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;staging\_x\_offset: \-1.05  
    &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;staging\_yaw\_offset: 0.0  
    &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;docking\_threshold: 0.03       \# 3cm 이내 정밀 진입 허용  
    &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;use\_stall\_detection: false    \# Isaac Sim에서는 위치/감지 기반 완료 판정

#### **3단계: 정밀 도킹 궤적 제어기 튜닝 (nova\_carter\_mpc 알고리즘 적용)**

> * **목적:** RPP의 후진 회전 불가 문제를 해결하고, 차동 구동 기구학 제약 조건 하에서 목표 자세(X, Y, Yaw)를 오차 없이 수렴.  
> * **적용 절차:**  
  1. opennav\_docking 내장 Graceful Controller의 파라미터를 튜닝하거나, nova\_carter\_mpc의 제어 모델(AMR 가감속 한계: 0.8 m/s², 차폭: 카터 베이스 규격)을 기반으로 한 MPC 컨트롤러 노드를 구성합니다.  
  2. 도킹 접근 구간에서는 글로벌 맵 기반 AMCL 대신 **/chassis/odom (스무스 오도메트리) 프레임**을 기준으로 오차를 최소화하도록 제어 입력을 생성하여 cmd\_vel로 전달합니다.

#### **4단계: integration\_v1.py 및 전체 시퀀스 연결**

기존 integration\_v1.py는 WAIT\_NAVIGATION 단계에서 단순히 위치 변화와 바퀴 속도(commanded\_wheel\_speed)만을 모니터링하여 도착 여부를 판단했습니다. 이를 **도킹 액션 클라이언트(DockRobot Action Client)** 구조로 개선합니다.

> * **동작 흐름 수정:**  
  1. **Pick 단계:** 기존과 동일하게 M0609가 팔레트를 인양하고 운반 자세(CARRY\_ROTATE)로 전환합니다.  
  2. **Navigation & Docking 요청:**  
     * integration\_v1.py가 직접 좌표 오차를 체크하는 대신, opennav\_docking 액션 서버로 DockRobot.action (dock\_id: feeder\_dock)을 전송합니다.  
     * Nav2가 카터를 FEEDER\_APPROACH까지 이동시킨 뒤, 자동으로 비전 제어 루프로 전환되어 fp\_dock\_pose\_publisher가 주는 상대 좌표를 보며 후진 도킹합니다.  
  3. **Place 단계 전환:**  
     * 액션 결과가 SUCCEEDED로 반환되고 BaseWatcher().settled가 확인되면 브레이크(rig\_switch.work())를 걸고 M0609의 build\_place\_sequence()를 즉시 실행합니다.

이와 같이 구성하면 수동 RViz2 더블 클릭에 의존하던 후진 정렬 오차와 팔레트 충돌 문제를 센서 피드백 기반 폐루프(Closed-loop) 제어로 완전히 해소할 수 있습니다.

*참고: 본문에서 처음 등장한 생소한 제어 알고리즘인 RPP(Regulated Pure Pursuit)는 주석으로 설명을 추가하였습니다.*

---

*User prompt: 엔비디아 아이작심 extention 에셋 중 Nova\_Carter\_ROS가 2D 라이다도 토픽 발행을 켤 수 있는지 여부를 알려줄 것.*

Response: **결론부터 말씀드리면, 토픽 발행을 켤 수 있습니다.**

Isaac Sim의 Nova\_Carter\_ROS 에셋 및 내장 라이다 퍼블리셔 파이프라인에서 2D 라이다(sensor\_msgs/msg/LaserScan) 토픽을 활성화하는 방법과 원리는 다음과 같습니다.

### **1\. 활성화 가능 원리 (ROS2RtxLidarHelper)**

Isaac Sim 내 Nova Carter의 라이다 파이프라인은 OmniGraph 노드인 ROS2RtxLidarHelper를 통해 ROS 2 토픽을 내보냅니다. 이 헬퍼 노드의 inputs:type 속성은 3D 점군인 point\_cloud뿐만 아니라 2D 평면 스캔인 laser\_scan을 기본 지원합니다.

> * **3D PointCloud2 모드:** inputs:type \= "point\_cloud"  
> * **2D LaserScan 모드:** inputs:type \= "laser\_scan"

### **2\. 설정 및 활성화 방법 (2가지)**

#### **방법 1\. 파이썬 런처 스크립트에서 동적 전환 (권장)**

USD 원본 파일을 수정하지 않고 launch\_scene.py 등 실행 스크립트 단계에서 라이다 헬퍼 노드의 속성을 2D 스캔 형태로 변경하여 켤 수 있습니다.

import omni.usd

stage \= omni.usd.get\_context().get\_stage()

\# Nova Carter 내부의 ROS2RtxLidarHelper 노드 탐색 및 2D 라이다 설정  
for prim in stage.Traverse():  
&nbsp;&nbsp;&nbsp;&nbsp;if prim.GetTypeName() \== "OmniGraphNode" and str(  
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;prim.GetAttribute("node:type").Get() or ""  
&nbsp;&nbsp;&nbsp;&nbsp;).endswith("ROS2RtxLidarHelper"):  
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;\# 1\) 출력 타입을 2D LaserScan으로 변경  
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;prim.GetAttribute("inputs:type").Set("laser\_scan")

&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;\# 2\) 토픽 이름 설정 (필요 시 지정, 기본값은 보통 /scan)  
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;if prim.GetAttribute("inputs:topicName"):  
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;prim.GetAttribute("inputs:topicName").Set("/scan")

&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;\# 3\) 전체 스캔 주기 발행 활성화 (10 Hz 동기화)  
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;if prim.GetAttribute("inputs:fullScan"):  
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;prim.GetAttribute("inputs:fullScan").Set(True)

print("\[Lidar Setup\] Nova Carter 라이다를 2D LaserScan(/scan) 모드로 전환했습니다.")

#### **방법 2\. Isaac Sim GUI (Action Graph)에서 직접 수정**

> 1. Isaac Sim 상단 메뉴에서 **Window \> Visual Scripting \> Action Graph**를 엽니다.  
> 2. Stage 패널에서 Nova\_Carter\_ROS 하위에 위치한 라이다 OmniGraph(예: ros2\_rtx\_lidar 등)를 선택합니다.  
> 3. 그래프 내부의 **ROS2 RTX Lidar Helper** 노드를 클릭합니다.  
> 4. 우측 Property 패널의 **Inputs** 항목에서:  
   * **type**: 드롭다운에서 laser\_scan을 선택합니다.  
   * **topicName**: 발행을 원하는 토픽명(예: scan 또는 chassis/scan)으로 설정합니다.

### **3\. 적용 시 주의사항**

> * **마운트 프레임(frameId):** inputs:type을 laser\_scan으로 변경할 경우 헤더에 찍히는 frame\_id가 실제 라이다 센서 축(예: front\_3d\_lidar 또는 base\_link 기준)과 맞는지 확인해야 Nav2 Costmap 및 AMCL의 TF 변환 오류가 발생하지 않습니다.  
> * **센서 각도(FOV/Pitch):** Nova Carter에 장착된 센서가 전방을 향해 아래로 기울어져(Pitch tilt) 있다면 2D 투영 평면이 지면을 장애물로 인식할 수 있습니다. 수평 단면 2D 스캔이 정상 작동하는지 RViz2의 /scan 토픽 시각화로 점검하시기 바랍니다.

---

