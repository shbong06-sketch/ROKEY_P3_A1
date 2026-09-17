# ==============================================================================
# 🎮 Isaac Sim Stage Loader & Action Graph Auto-Setup Async Runner
# ==============================================================================
# Isaac Sim 시작 시 --exec 로 자동 실행되어:
# 1. 지정된 USD 스테이지를 비동기로 엽니다.
# 2. ROS 2 Bridge 확장을 활성화하고 /World/ActionGraph가 없으면 자동 생성합니다.
# 3. 타임라인 Play를 호출하여 시뮬레이션을 즉시 가동합니다.
# ==============================================================================

import argparse
import asyncio
import os
import sys
import traceback

import carb
import omni.client
import omni.graph.core as og
import omni.kit.app
import omni.kit.async_engine
import omni.timeline
import omni.usd


def parse_arguments():
    parser = argparse.ArgumentParser()
    parser.add_argument("--path", type=str, required=True, help="Full path to the USD stage")
    parser.add_argument("--start-on-play", action="store_true", help="Auto start simulation playback")
    args, _ = parser.parse_known_args()
    return args


def setup_action_graph_if_needed(stage):
    """스테이지 내 /World/ActionGraph 유무를 확인하고 없으면 자동 생성"""
    graph_path = "/World/ActionGraph"
    existing_graph = stage.GetPrimAtPath(graph_path)
    if existing_graph and existing_graph.IsValid():
        carb.log_info(f"[ActionGraph] Already exists at {graph_path}")
        return

    # 타겟 로봇 Prim 탐색
    robot_prim_path = "/World/mir100"
    for prim in stage.Traverse():
        if prim.GetName() in ["mir100", "MiR100"]:
            robot_prim_path = prim.GetPath().pathString
            break

    carb.log_info(f"[ActionGraph] Creating ROS 2 Bridge for robot: {robot_prim_path}")

    # 확장 기능 활성화 시도
    try:
        from omni.isaac.core.utils.extensions import enable_extension
        for ext in ["isaacsim.ros2.bridge", "omni.isaac.ros2_bridge"]:
            try:
                enable_extension(ext)
            except Exception:
                pass
    except Exception as e:
        carb.log_warn(f"[ActionGraph] Extension enable warning: {e}")

    # 노드 타입 식별
    def resolve_type(candidates):
        for c in candidates:
            try:
                if og.Controller.node_type(c) is not None:
                    return c
            except Exception:
                pass
        return candidates[0]

    node_context = resolve_type(["isaacsim.ros2.bridge.ROS2Context", "omni.isaac.ros2_bridge.ROS2Context"])
    node_clock = resolve_type(["isaacsim.ros2.bridge.ROS2PublishClock", "omni.isaac.ros2_bridge.ROS2PublishClock"])
    node_twist = resolve_type(["isaacsim.ros2.bridge.ROS2SubscribeTwist", "omni.isaac.ros2_bridge.ROS2SubscribeTwist"])
    node_odom = resolve_type(["isaacsim.ros2.bridge.ROS2PublishOdometry", "omni.isaac.ros2_bridge.ROS2PublishOdometry"])
    node_tf = resolve_type(["isaacsim.ros2.bridge.ROS2PublishTransformTree", "omni.isaac.ros2_bridge.ROS2PublishTransformTree"])
    node_diff = resolve_type(["omni.isaac.wheeled_robots.DifferentialController", "isaacsim.robot.wheeled_robots.DifferentialController"])
    node_artic = resolve_type(["omni.isaac.core_nodes.IsaacArticulationController", "isaacsim.core.nodes.IsaacArticulationController"])

    keys = og.Controller.Keys
    try:
        og.Controller.edit(
            {"graph_path": graph_path, "evaluator_name": "execution"},
            {
                keys.CREATE_NODES: [
                    ("on_tick", "omni.graph.action.OnPlaybackTick"),
                    ("ros2_context", node_context),
                    ("publish_clock", node_clock),
                    ("subscribe_twist", node_twist),
                    ("diff_controller", node_diff),
                    ("articulation_controller", node_artic),
                    ("publish_odom", node_odom),
                    ("publish_tf", node_tf),
                ],
                keys.SET_VALUES: [
                    ("ros2_context.inputs:domain_id", 103),
                    ("subscribe_twist.inputs:topicName", "/cmd_vel"),
                    ("publish_clock.inputs:topicName", "/clock"),
                    ("diff_controller.inputs:wheelDistance", 0.4452),
                    ("diff_controller.inputs:wheelRadius", 0.0625),
                    ("articulation_controller.inputs:targetPrim", robot_prim_path),
                    ("publish_odom.inputs:topicName", "/odom"),
                    ("publish_odom.inputs:chassisFrameId", "base_footprint"),
                    ("publish_odom.inputs:odomFrameId", "odom"),
                ],
                keys.CONNECT: [
                    ("on_tick.outputs:tick", "publish_clock.inputs:execIn"),
                    ("on_tick.outputs:tick", "subscribe_twist.inputs:execIn"),
                    ("on_tick.outputs:tick", "publish_odom.inputs:execIn"),
                    ("on_tick.outputs:tick", "publish_tf.inputs:execIn"),
                    ("on_tick.outputs:tick", "articulation_controller.inputs:execIn"),
                    ("subscribe_twist.outputs:linearVelocity", "diff_controller.inputs:linearVelocity"),
                    ("subscribe_twist.outputs:angularVelocity", "diff_controller.inputs:angularVelocity"),
                    ("diff_controller.outputs:velocityCommand", "articulation_controller.inputs:velocityCommand"),
                    ("ros2_context.outputs:context", "publish_clock.inputs:context"),
                    ("ros2_context.outputs:context", "subscribe_twist.inputs:context"),
                    ("ros2_context.outputs:context", "publish_odom.inputs:context"),
                    ("ros2_context.outputs:context", "publish_tf.inputs:context"),
                ],
            },
        )
        carb.log_info("✅ [ActionGraph] Successfully created and connected /World/ActionGraph!")
    except Exception as exc:
        carb.log_error(f"❌ [ActionGraph] Failed to create ActionGraph: {exc}")


async def open_stage_and_start(path: str, start_on_play: bool):
    carb.log_info(f"⏳ [IsaacSim Loader] Opening stage: {path}")

    # 스테이지 비동기 로드
    success, error = await omni.usd.get_context().open_stage_async(path)
    if not success:
        carb.log_error(f"❌ [IsaacSim Loader] Failed to open stage {path}: {error}")
        return

    carb.log_info(f"✅ [IsaacSim Loader] Stage opened successfully: {path}")

    # 1프레임 렌더링 대기
    await omni.kit.app.get_app().next_update_async()

    # Action Graph 자동 확인 및 주입
    stage = omni.usd.get_context().get_stage()
    setup_action_graph_if_needed(stage)

    # 타임라인 Play 시작
    if start_on_play:
        await omni.kit.app.get_app().next_update_async()
        await omni.kit.app.get_app().next_update_async()
        timeline = omni.timeline.get_timeline_interface()
        timeline.play()
        carb.log_info("▶️ [IsaacSim Loader] Timeline PLAY started! ROS 2 topics are now publishing.")


def main():
    args = parse_arguments()
    if not args.path:
        carb.log_error("No stage path provided.")
        return

    omni.kit.async_engine.run_coroutine(
        open_stage_and_start(args.path, args.start_on_play)
    )


if __name__ == "__main__":
    main()
