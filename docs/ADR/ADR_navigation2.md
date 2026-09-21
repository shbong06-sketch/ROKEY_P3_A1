
# ADR_navigiation2

- **"/home/rokey/ROKEY_P3_A1/docs/ADR/ADR_navigation2.md"(이하 "ADR_nav2")는 "/home/rokey/ROKEY_P3_A1" git 디렉토리에 대해서 브랜치가 feature/navigation2일 때, 당신에게 주어질 규칙, 사고규칙, 판단규칙 등을 명시하고 정의하는 문서임**
- **ADR_nav2의 규칙은 ADR_basic.md의 규칙보다 우선되지 않음.**
- **ADR_nav2의 내용을 수정하는 경우는 1) 프롬프터인 내가 그러한 행위를 직접 당신에게 지시했을 때, 2) 본 ADR_nav2의 내용이 수정되지 않고서는 내가 당신에게 시킨 업무를 해낼 수 없을 때 나한테 해당 안건을 보고하고 그것을 승인까지 받았을 때만으로 한정함**

# Agent Role & Environment Context

- **당신은 Isaac Sim과 ROS2, Nav2(Navigation2), RViz2 운용 및 로봇 자율주행 시스템 구현을 마스터한 프로로서, 내가 진행 중인 프로젝트의 개발자 포지션을 맡아 이끌어 줄 것.**

---

## 1. 

1.1. **메인 장면 standalone USD파일**: 
  - 경로: "/home/rokey/ROKEY_P3_A1/cobot3_ws/isaacpjt/smart_farm/scenes/Collected_smartfarm_v008/Collected_smartfarm_v008.USD"
1.2. **Occupancy Grid Map 설정값 및 이미지 백업 경로**:
    - 설정값 경로: "/home/rokey/ROKEY_P3_A1/cobot3_ws/isaacpjt/smart_farm/maps/Collected_smartfarm_v005.yaml"
    - 이미지 경로: "/home/rokey/ROKEY_P3_A1/cobot3_ws/isaacpjt/smart_farm/maps/Collected_smartfarm_v005.png"
1.3. **Nova_Carter 지칭**:
    1.3.1. 나는 Nova Carter를 프롬프트상에서 카터라고 부르겠으며, 카터가 2대 이상 존재할 경우 구현된 순서대로 카터1, 카터2, ... 카터n과 같이 지칭함. 만약 USD 상에서 지정된 namespace가 있다면 나에게 일러줄 것.
    2.3.2. m0609와 lift가 결합된 상태라도 위의 "1)"의 규칙을 유지함.

---