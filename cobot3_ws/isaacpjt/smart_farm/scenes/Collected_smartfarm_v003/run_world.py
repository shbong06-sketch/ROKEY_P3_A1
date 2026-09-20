"""
스마트팜 월드 한 번에 실행 (Isaac Sim 5.1)

    ~/isaacsim/python.sh run_world.py

하는 일
  1. ROS 2 브리지 켜기 (터미널에 환경변수가 없으면 넣고 스스로 다시 실행)
  2. 이 폴더의 World0_carter_tuned.usd 열기  (USD 파일은 저장하지 않습니다)
  3. 바닥 확인: 지면(충돌 평면)이 USD 에 들어 있으면 그대로 사용. 없는 옛 월드면 실행 중에만 보정
       큐브 바닥 충돌 그대로면 Play 10초에 리그가 55 cm 떠돌고 넘어지기도 합니다. 끄려면 --no-ground-fix
  4. Play 하면 리그의 주행/작업 자동 전환 (rig_mode.py)
       /cmd_vel 이 오면 바퀴 브레이크 해제, 1초 동안 없으면 브레이크 -> 팔·리프트 작업 중 카터가 밀리지 않음
       주의: 주행을 멈출 때는 속도 0 을 한 번 보내야 합니다 (발행만 끊으면 마지막 속도로 계속 굴러갑니다)

옵션
  --world 파일이름     다른 월드 USD (이 폴더 기준)
  --no-ground-fix      바닥 보정 끄기
  --no-rig-mode        주행/작업 자동 전환 끄기
  --autoplay           열자마자 Play
  --headless --test-seconds N   화면 없이 N초 돌리고 리그가 얼마나 움직였는지 출력 (점검용)
"""
import argparse
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))

parser = argparse.ArgumentParser()
parser.add_argument("--world", default="World0_carter_tuned.usd")
parser.add_argument("--no-ground-fix", action="store_true")
parser.add_argument("--no-rig-mode", action="store_true")
parser.add_argument("--autoplay", action="store_true")
parser.add_argument("--headless", action="store_true")
parser.add_argument("--test-seconds", type=float, default=0.0)
args, kit_args = parser.parse_known_args()

# ROS 2 브리지는 Isaac Sim 내장 jazzy 라이브러리를 LD_LIBRARY_PATH 에서 찾습니다. 프로세스 시작 때만 읽히므로
# 빠져 있으면 넣고 자기 자신을 한 번 다시 실행합니다. (ROS 를 source 한 터미널이면 그대로 둡니다)
isaac_root = os.environ.get("ISAAC_PATH") or os.path.abspath(
    os.path.join(os.path.dirname(os.path.abspath(sys.executable)), "..", "..", ".."))
ros_lib = os.path.join(isaac_root, "exts", "isaacsim.ros2.bridge", "jazzy", "lib")
if (os.path.isdir(ros_lib) and ros_lib not in os.environ.get("LD_LIBRARY_PATH", "")
        and not os.environ.get("ROS_DISTRO") and os.environ.get("SMARTFARM_ROS_REEXEC") != "1"):
    os.environ["LD_LIBRARY_PATH"] = os.environ.get("LD_LIBRARY_PATH", "") + ":" + ros_lib
    os.environ.setdefault("RMW_IMPLEMENTATION", "rmw_fastrtps_cpp")
    os.environ["SMARTFARM_ROS_REEXEC"] = "1"
    print(f"[ROS2] LD_LIBRARY_PATH 에 {ros_lib} 를 추가하고 다시 실행합니다.", flush=True)
    os.execv(sys.executable, [sys.executable] + sys.argv)

from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": args.headless, "extra_args": kit_args})

import omni.timeline
import omni.usd
from pxr import Gf, UsdGeom, UsdPhysics
from isaacsim.core.utils.extensions import enable_extension

sys.path.insert(0, HERE)
from rig_mode import RigModeWatcher

FLOOR_PATH = "/World/SmartFarm/Room/Floor"
GROUND_PATH = "/World/SmartFarm/Room/GroundCollider"

# 1. ROS 2 브리지 (월드를 열기 전에 켜야 카터의 ROS 그래프가 정상 로드됩니다)
enable_extension("isaacsim.ros2.bridge")
simulation_app.update()

# 2. 월드 열기
world_path = os.path.join(HERE, args.world)
if not os.path.isfile(world_path):
    raise FileNotFoundError(world_path)
omni.usd.get_context().open_stage(world_path)
while omni.usd.get_context().get_stage_loading_status()[2] > 0:
    simulation_app.update()
for _ in range(15):
    simulation_app.update()
stage = omni.usd.get_context().get_stage()
stage.SetEditTarget(stage.GetSessionLayer())          # 아래 변경은 전부 실행 중에만. Ctrl+S 해도 USD 에 들어가지 않습니다
print(f"[월드] {world_path}")

# 3. 바닥 보정
if not args.no_ground_fix:
    floor = stage.GetPrimAtPath(FLOOR_PATH)
    if stage.GetPrimAtPath(GROUND_PATH):
        print("[바닥] 지면(충돌 평면)이 이미 USD 에 들어 있습니다 -> 보정 불필요")
    elif floor and floor.HasAPI(UsdPhysics.CollisionAPI):
        UsdPhysics.CollisionAPI(floor).GetCollisionEnabledAttr().Set(False)
        ground = UsdGeom.Plane.Define(stage, GROUND_PATH)          # 무한 충돌 평면, z=0 (큐브 바닥 윗면과 같은 높이)
        ground.GetAxisAttr().Set("Z")
        ground.GetPurposeAttr().Set(UsdGeom.Tokens.guide)          # 화면에는 안 보임
        UsdPhysics.CollisionAPI.Apply(ground.GetPrim())
        print("[바닥] 큐브 바닥 충돌 끔 + 지면(충돌 평면) 추가 (실행 중에만)")
    else:
        print(f"[바닥] {FLOOR_PATH} 충돌체를 찾지 못해 보정을 건너뜁니다")

# 4. 리그 찾기 + 주행/작업 자동 전환
rig_root = next((str(p.GetPath()) for p in stage.Traverse() if p.GetName() == "nova_carter_ROS"), None)
watcher = None
if rig_root and not args.no_rig_mode:
    watcher = RigModeWatcher(rig_root)
    print(f"[리그] {rig_root}\n[리그] Play 하면 /cmd_vel 이 없을 때 바퀴 브레이크가 자동으로 걸립니다.")
elif not rig_root:
    print("[리그] nova_carter_ROS 를 찾지 못했습니다 -> 자동 전환 없이 진행")

timeline = omni.timeline.get_timeline_interface()
if args.autoplay or args.test_seconds > 0:
    timeline.play()


def chassis_xy():
    matrix = omni.usd.get_world_transform_matrix(stage.GetPrimAtPath(f"{rig_root}/chassis_link"))
    t = matrix.ExtractTranslation()
    return Gf.Vec2d(t[0], t[1]), matrix


if args.test_seconds > 0 and rig_root:
    for _ in range(5):
        simulation_app.update()
    start, _ = chassis_xy()
    while timeline.get_current_time() < args.test_seconds and simulation_app.is_running():
        simulation_app.update()
        if watcher:
            watcher.update()
    end, matrix = chassis_xy()
    up_z = matrix.ExtractRotationMatrix()[2][2]          # 1 이면 똑바로 서 있음
    print(f"[점검] {args.test_seconds:.0f}초 동안 카터 이동 {1000 * (end - start).GetLength():.1f} mm, "
          f"기울기 cos={up_z:.4f}, 모드={watcher.mode if watcher else '-'}", flush=True)
else:
    print("준비 완료. Play 를 누르세요." if not args.autoplay else "실행 중.", flush=True)
    while simulation_app.is_running():
        simulation_app.update()
        if watcher:
            watcher.update()

simulation_app.close()
