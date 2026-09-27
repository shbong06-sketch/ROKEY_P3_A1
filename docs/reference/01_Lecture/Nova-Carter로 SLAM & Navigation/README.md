# Nova-Carter로 SLAM & Navigation (수업 자료 정리)

- 출처: https://sonmiran9.oopy.io/3c1450ef-7c59-8055-9f65-cc04e8572496 (Isaac Sim_협동3_9기 > 24.04+5.1.0)
- 정리일: 2026-09-19
- 정리 기준: 각 페이지를 폴더 하나로 두고, 제목 구조·명령·코드·설정값은 원문 그대로, 설명문은 요약하여 md로 작성함. 이미지는 `<페이지>_<번호>_<내용>.png/gif`로 같은 폴더에 저장함. 동영상(mp4)은 링크만 남김.
- 수업 환경은 ROS_DOMAIN_ID=50, 예제 워크스페이스 `~/IsaacSim-ros_workspaces/jazzy_ws` 기준임. 우리 프로젝트는 ROS_DOMAIN_ID=101이며 워크스페이스는 `/home/rokey/ROKEY_P3_A1/cobot3_ws`임.

## 상위 페이지의 작업 환경

```shell
#아이작심용
mkdir -p ~/cobot3_ws/isaacpjt/nova_carter

#ROS용
mkdir -p ~/cobot3_ws/src/nova_carter
```

## 페이지 목록

아이작심 예제 환경에서 동작 확인
1. [SLAM 개요](SLAM%20개요/SLAM%20개요.md)
2. [Nova Carter 로드 및 Map 제작](Nova%20Carter%20로드%20및%20Map%20제작/Nova%20Carter%20로드%20및%20Map%20제작.md)
3. [Nav2개요](Nav2개요/Nav2개요.md)
4. [Nav2 자율주행 + Rviz2 연동](Nav2%20자율주행%20+%20Rviz2%20연동/Nav2%20자율주행%20+%20Rviz2%20연동.md)
5. [Python Simple Commander API — nav_to_pose](Python%20Simple%20Commander%20API%20—%20nav_to_pose/Python%20Simple%20Commander%20API%20—%20nav_to_pose.md)
6. [Python Simple Commander — nav_through_pose](Python%20Simple%20Commander%20—nav_through_pose/Python%20Simple%20Commander%20—nav_through_pose.md)
7. [경비 로봇 Nova-Carter](경비%20로봇%20Nova-Carter/경비%20로봇%20Nova-Carter.md) (이미지 1장뿐인 과제 페이지)

나의 워크스페이스로 가져와서 동작 개발
8. [나만의 USD 환경 구성 + navigation 마이그레이션](나만의%20USD%20환경%20구성%20+%20navigation%20마이그레이션/나만의%20USD%20환경%20구성%20+%20navigation%20마이그레이션.md)
9. [Nova Carter USD Standalone 로드](Nova%20Carter%20USD%20Standalone%20로드/Nova%20Carter%20USD%20Standalone%20로드.md)
10. [Multi-Robot — Nova Carter2 추가](Multi-Robot%20—%20Nova%20Carter2%20추가/Multi-Robot%20—%20Nova%20Carter2%20추가.md)
11. [Multi-Robot — Nav2 Launch + Rviz2](Multi-Robot%20—%20Nav2%20Launch%20+%20Rviz2/Multi-Robot%20—%20Nav2%20Launch%20+%20Rviz2.md) (launch.py, params_1/2.yaml 원본 파일 포함)
12. [Nova Carter가 보행자를 LiDAR로 감지](Nova%20Carter가%20보행자를%20LiDAR로%20감지/Nova%20Carter가%20보행자를%20LiDAR로%20감지.md)

## 우리 프로젝트에 바로 쓰이는 항목

- Nav2 단계: 2(지도 제작 절차와 YAML 형식), 4(launch·Rviz2 확인 순서), 11(params 값의 출처. 저장소 params/smart_farm/123.py와 동일 계열).
- 자동 실행: 9(Standalone 로더 구조가 smart_farm_navigation/scripts/launch_scene.py와 같음).
- 주의: 이 수업의 /clock은 예제 씬의 ROS_Clock 그래프에서 나옴. smart_farm_nav2_01.usd에는 그 그래프가 없어 /clock이 없음.
