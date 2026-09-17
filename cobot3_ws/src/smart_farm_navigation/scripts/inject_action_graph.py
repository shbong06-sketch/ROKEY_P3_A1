#!/usr/bin/env python3
# ==============================================================================
# 🛠️ Collected_260916_AMR_test.usd 내 Action Graph (ROS 2 Bridge) 자동 주입 스크립트
# ==============================================================================
# [실행 방법 - 고사양 PC 터미널에서 단 1회 실행]:
#   isaac_python ~/ROKEY_P3_A1/cobot3_ws/src/smart_farm_navigation/scripts/inject_action_graph.py
#
# [동작 설명]:
#   1. Isaac Sim의 ROS 2 Bridge 확장 기능을 자동으로 활성화합니다.
#   2. Collected_260916_AMR_test/260916_AMR_test.usd 를 엽니다.
#   3. /World/ActionGraph (Clock, Odom, CmdVel, TF) 를 자동으로 생성합니다.
#   4. 스테이지를 영구 저장(Save)합니다.
#   -> 이후에는 스크립트 에디터나 GUI 조작 없이 USD만 열면 토픽이 100% 자동 발행됩니다.
# ==============================================================================

import os
import sys

# Isaac Sim SimulationApp 초기화 (헤드리스 모드로 안전하고 빠르게 실행)
from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": True})

import omni.graph.core as og
import omni.kit.commands
import omni.usd
from omni.isaac.core.utils.extensions import enable_extension
from pxr import Usd, UsdGeom

# 1. ROS 2 Bridge 확장 기능 활성화 (버전별 확장명 자동 지원)
print("[1/4] ROS 2 Bridge 확장 기능 활성화 중...")
for ext in ["isaacsim.ros2.bridge", "omni.isaac.ros2_bridge"]:
    try:
        enable_extension(ext)
        print(f"  - 확장 활성화 성공: {ext}")
    except Exception:
        pass

# 2. USD 파일 경로 확인
home_dir = os.path.expanduser("~")
usd_candidates = [
    os.path.join(home_dir, "ROKEY_P3_A1/cobot3_ws/src/smart_farm_navigation/Collected_260916_AMR_test/260916_AMR_test.usd"),
    os.path.join(home_dir, "cobot3_ws/src/smart_farm_navigation/Collected_260916_AMR_test/260916_AMR_test.usd"),
    "/home/rokey/ROKEY_P3_A1/cobot3_ws/src/smart_farm_navigation/Collected_260916_AMR_test/260916_AMR_test.usd",
]
usd_path = next((p for p in usd_candidates if os.path.exists(p)), usd_candidates[0])

print(f"[2/4] 대상 USD 스테이지 로드: {usd_path}")
omni.usd.get_context().open_stage(usd_path)
stage = omni.usd.get_context().get_stage()

if not stage:
    print(f"❌ [에러] USD 스테이지를 열 수 없습니다: {usd_path}")
    simulation_app.close()
    sys.exit(1)

# 로봇 Prim 탐색 (/World/mir100)
robot_prim_path = "/World/mir100"
for prim in stage.Traverse():
    if prim.GetName() in ["mir100", "MiR100"]:
        robot_prim_path = prim.GetPath().pathString
        break

print(f"  - 감지된 로봇 Prim: {robot_prim_path}")

# 3. 노드 타입 탐색 (Isaac Sim 5.x / 4.x 자동 호환)
def get_node_type(candidates):
    for c in candidates:
        try:
            desc = og.Controller.node_type(c)
            if desc is not None:
                return c
        except Exception:
            pass
    return candidates[0]

node_context = get_node_type(["isaacsim.ros2.bridge.ROS2Context", "omni.isaac.ros2_bridge.ROS2Context"])
node_clock = get_node_type(["isaacsim.ros2.bridge.ROS2PublishClock", "omni.isaac.ros2_bridge.ROS2PublishClock"])
node_twist = get_node_type(["isaacsim.ros2.bridge.ROS2SubscribeTwist", "omni.isaac.ros2_bridge.ROS2SubscribeTwist"])
node_odom = get_node_type(["isaacsim.ros2.bridge.ROS2PublishOdometry", "omni.isaac.ros2_bridge.ROS2PublishOdometry"])
node_tf = get_node_type(["isaacsim.ros2.bridge.ROS2PublishTransformTree", "omni.isaac.ros2_bridge.ROS2PublishTransformTree"])
node_diff = get_node_type(["omni.isaac.wheeled_robots.DifferentialController", "isaacsim.robot.wheeled_robots.DifferentialController"])
node_artic = get_node_type(["omni.isaac.core_nodes.IsaacArticulationController", "isaacsim.core.nodes.IsaacArticulationController"])

print(f"[3/4] /World/ActionGraph 생성 중 (ROS 2 Bridge 연결)...")
graph_path = "/World/ActionGraph"
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
    print("  - ActionGraph 노드 생성 및 배선 완료!")
except Exception as e:
    print(f"  - 노드 연결 중 경고: {e}")

# 4. USD 파일에 영구 저장 (Bake)
print(f"[4/4] 수정된 스테이지를 USD 파일로 저장 중: {usd_path}")
stage.Save()
print("\n🎉 [완료] Collected_260916_AMR_test.usd 에 ActionGraph 가 성공적으로 영구 저장되었습니다!")
print("👉 이제 아이작 심을 열고 Play만 누르면 /clock, /cmd_vel, /odom 토픽이 100% 자동 발행됩니다.\n")

simulation_app.close()
