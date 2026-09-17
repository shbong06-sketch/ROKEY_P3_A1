# ==============================================================================
# 🤖 MiR100 ROS 2 Bridge Action Graph 자동 생성 스크립트 (Isaac Sim용)
# ==============================================================================
# [사용 방법]
# 1. Isaac Sim에서 260916_AMR_test.usd 스테이지를 열고 mir100.usd를 불러옵니다.
# 2. Isaac Sim 상단 메뉴: Window -> Script Editor 를 엽니다.
# 3. 이 파일의 내용 전체를 복사하여 Script Editor에 붙여넣고 [Run] 버튼을 누릅니다.
# 4. /World/ActionGraph 가 자동 생성되며, Play(▶)를 누르면 ROS2 토픽이 발행됩니다.
# 5. 스테이지를 Ctrl + S 로 저장해 두면 다음부터 자동으로 유지됩니다.
# ==============================================================================

import omni.graph.core as og
import omni.usd
from pxr import Gf, UsdGeom

stage = omni.usd.get_context().get_stage()

# 1. 로봇 Prim 경로 탐색 (/World/mir100 확인)
robot_prim_path = "/World/mir100"
found_prim = stage.GetPrimAtPath(robot_prim_path)
if not found_prim or not found_prim.IsValid():
    for prim in stage.Traverse():
        if prim.GetName() in ["mir100", "MiR100"]:
            robot_prim_path = prim.GetPath().pathString
            break

print(f"[ActionGraph Setup] 타겟 로봇 Prim 경로: {robot_prim_path}")

base_footprint_path = f"{robot_prim_path}/base_footprint"
graph_path = "/World/ActionGraph"

# 2. OmniGraph 생성 및 노드 연결
keys = og.Controller.Keys
try:
    og.Controller.edit(
        {"graph_path": graph_path, "evaluator_name": "execution"},
        {
            keys.CREATE_NODES: [
                ("on_tick", "omni.graph.action.OnPlaybackTick"),
                ("ros2_context", "omni.isaac.ros2_bridge.ROS2Context"),
                ("publish_clock", "omni.isaac.ros2_bridge.ROS2PublishClock"),
                ("subscribe_twist", "omni.isaac.ros2_bridge.ROS2SubscribeTwist"),
                ("diff_controller", "omni.isaac.wheeled_robots.DifferentialController"),
                ("articulation_controller", "omni.isaac.core_nodes.IsaacArticulationController"),
                ("publish_odom", "omni.isaac.ros2_bridge.ROS2PublishOdometry"),
                ("publish_tf", "omni.isaac.ros2_bridge.ROS2PublishTransformTree"),
            ],
            keys.SET_VALUES: [
                # ROS 2 기본 설정
                ("ros2_context.inputs:domain_id", 103),  # 사용자 설정 ROS_DOMAIN_ID
                ("subscribe_twist.inputs:topicName", "/cmd_vel"),
                ("publish_clock.inputs:topicName", "/clock"),
                # MiR100 차동 구동 파라미터 (휠 간격 0.4452m, 휠 반지름 0.0625m)
                ("diff_controller.inputs:wheelDistance", 0.4452),
                ("diff_controller.inputs:wheelRadius", 0.0625),
                ("articulation_controller.inputs:targetPrim", robot_prim_path),
                # 오도메트리 & TF 설정
                ("publish_odom.inputs:topicName", "/odom"),
                ("publish_odom.inputs:chassisFrameId", "base_footprint"),
                ("publish_odom.inputs:odomFrameId", "odom"),
            ],
            keys.CONNECT: [
                # Tick 실행 흐름 연결
                ("on_tick.outputs:tick", "publish_clock.inputs:execIn"),
                ("on_tick.outputs:tick", "subscribe_twist.inputs:execIn"),
                ("on_tick.outputs:tick", "publish_odom.inputs:execIn"),
                ("on_tick.outputs:tick", "publish_tf.inputs:execIn"),
                ("on_tick.outputs:tick", "articulation_controller.inputs:execIn"),
                # Twist 속도 명령 -> Differential Controller -> Articulation Controller
                ("subscribe_twist.outputs:linearVelocity", "diff_controller.inputs:linearVelocity"),
                ("subscribe_twist.outputs:angularVelocity", "diff_controller.inputs:angularVelocity"),
                ("diff_controller.outputs:velocityCommand", "articulation_controller.inputs:velocityCommand"),
                # ROS2 Context 연결
                ("ros2_context.outputs:context", "publish_clock.inputs:context"),
                ("ros2_context.outputs:context", "subscribe_twist.inputs:context"),
                ("ros2_context.outputs:context", "publish_odom.inputs:context"),
                ("ros2_context.outputs:context", "publish_tf.inputs:context"),
            ],
        },
    )
    print("\n✅ [성공] /World/ActionGraph 노드 생성 및 연결이 완료되었습니다!")
    print("👉 Isaac Sim 좌측 상단의 Play(▶) 버튼을 누르면 ROS2 토픽이 정상 발행됩니다.\n")
except Exception as e:
    print(f"\n❌ [오류] Action Graph 생성 중 예외 발생: {e}\n")
