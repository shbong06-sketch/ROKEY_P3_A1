"""[navigation 2026-09-28] 씬의 카메라들이 실제로 무엇을 비추는지 한 장씩 렌더해 본다.

공정별 녹화 카메라 매핑을 정하기 전에 화각을 눈으로 확인하기 위한 도구다.
Isaac 을 한 번만 띄우고 카메라를 차례로 바꿔 가며 뷰포트를 캡처한다.

사용:
  export DISPLAY=:99            # 가상 디스플레이가 떠 있어야 한다
  /home/nitrouriah92/isaacsim/python.sh preview_cameras.py --out <폴더>
"""

import argparse
import os

from isaacsim import SimulationApp

DEFAULT_SCENE = (
    "/home/rokey/ROKEY_P3_A1/cobot3_ws/isaacpjt/smart_farm/scenes/"
    "Collected_smartfarm_v014/Collected_smartfarm_v014_room_core_cabbage.usd"
)

# 공정별 후보 카메라. 이름만 보고 고르지 않고 실제 화각을 확인한다.
CANDIDATES = [
    # 공정 구역 고정 카메라
    "/World/ProcessCameras/Cam1_Harvest",
    "/World/ProcessCameras/Cam2_Nav2Place",
    "/World/ProcessCameras/Cam4_CullPickPlace",
    "/World/ProcessCameras/Cam5_Pusher",
    # Cam0_Perspective 는 2026-09-28 확인 결과 벽과 천장만 보여 후보에서 뺐다.
    # 비전룸 고정 카메라
    "/World/VisionRoom/Cameras/Inspect_Cam",
    "/World/VisionRoom/Cameras/M0609_Cam",
    "/World/VisionRoom/Cameras/M0609_Front",
    "/visionroom",
    # 카터에 달린 카메라 (움직이는 prim 의 시점뷰)
    "/World/SmartFarm/Placed/LiftRig/Asset/nova_carter_ROS/chassis_link/sensors/front_hawk/left/camera_left",
    "/World/SmartFarm/Placed/LiftRig/Asset/nova_carter_ROS/chassis_link/sensors/back_hawk/left/camera_left",
    "/World/SmartFarm/Placed/LiftRig/Asset/nova_carter_ROS/chassis_link/sensors/left_hawk/left/camera_left",
    "/World/SmartFarm/Placed/LiftRig/Asset/nova_carter_ROS/chassis_link/sensors/right_hawk/left/camera_left",
    "/World/SmartFarm/Placed/LiftRig/Asset/nova_carter_ROS/chassis_link/sensors/front_owl/camera",
    "/World/SmartFarm/Placed/LiftRig/Asset/nova_carter_ROS/chassis_link/sensors/back_owl/camera",
    # 비전룸 매니퓰레이터 엔드이펙터의 RealSense D455 (컬러/깊이)
    "/World/SmartFarm/Placed/M0609/Asset/onrobot_rg2ft/angle_bracket/realsense_d455/RSD455/Camera_OmniVision_OV9782_Color",
    "/World/SmartFarm/Placed/M0609/Asset/onrobot_rg2ft/angle_bracket/realsense_d455/RSD455/Camera_Pseudo_Depth",
]


def main() -> None:
    """카메라마다 뷰포트를 바꿔 한 장씩 저장한다."""

    parser = argparse.ArgumentParser()
    parser.add_argument("--out", required=True, help="PNG 를 저장할 폴더")
    parser.add_argument("--scene", default=DEFAULT_SCENE)
    parser.add_argument("--settle", type=int, default=60, help="카메라마다 돌릴 프레임 수")
    args = parser.parse_args()

    os.makedirs(args.out, exist_ok=True)

    app = SimulationApp({"headless": False, "width": 1920, "height": 1080})

    import omni.usd  # noqa: E402
    from omni.kit.viewport.utility import capture_viewport_to_file, get_active_viewport  # noqa: E402

    omni.usd.get_context().open_stage(args.scene)
    for _ in range(120):
        app.update()

    viewport = get_active_viewport()
    if viewport is None:
        print("### 뷰포트가 없습니다", flush=True)
        app.close()
        return

    for camera in CANDIDATES:
        name = camera.strip("/").replace("/", "_")
        path = os.path.join(args.out, f"cam_{name}.png")
        viewport.camera_path = camera
        for _ in range(args.settle):
            app.update()
        capture_viewport_to_file(viewport, path)
        for _ in range(20):
            app.update()
        print(f"### SAVED {camera} -> {path}", flush=True)

    app.close()


if __name__ == "__main__":
    main()
