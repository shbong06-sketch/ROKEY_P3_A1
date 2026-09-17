"""Standalone parameter runner.

  ~/isaacsim/python.sh control/run_control.py --robot mobile --v 0.3 --drive-time 3 --arm 90,20,70,0,90,0 --gripper close
  ~/isaacsim/python.sh control/run_control.py --robot fixed  --arm 45,30,60,0,60,0 --gripper close --headless

Sequence: settle 1 s -> base (v, w for drive-time) -> stop -> arm to --arm -> gripper -> hold -> report.
"""
import argparse, math, os, re, sys

# allow negative values such as "--arm -60,20,90,0,70,45" or "--gripper -10"
_VALUE_OPTS = {"--v", "--w", "--drive-time", "--turn-deg", "--distance", "--arm", "--arm-speed", "--gripper", "--hold"}
_argv = []
_it = iter(range(len(sys.argv)))
_i = 1
while _i < len(sys.argv):
    tok = sys.argv[_i]
    if tok in _VALUE_OPTS and _i + 1 < len(sys.argv) and re.fullmatch(r"-[0-9.][0-9.,\-]*", sys.argv[_i + 1]):
        _argv.append(f"{tok}={sys.argv[_i + 1]}"); _i += 2
    else:
        _argv.append(tok); _i += 1
sys.argv = sys.argv[:1] + _argv

ap = argparse.ArgumentParser()
ap.add_argument("--robot", choices=["mobile", "fixed"], default="mobile")
ap.add_argument("--usd", default=None, help="stage to open (default: the demo scene of --robot)")
ap.add_argument("--root", default=None, help="robot prim path (default: /World/mobile_manipulator or /World/m0617_fixed_base)")
ap.add_argument("--v", type=float, default=0.0, help="base forward speed [m/s]")
ap.add_argument("--w", type=float, default=0.0, help="base yaw rate [rad/s], + = left")
ap.add_argument("--drive-time", type=float, default=0.0, help="seconds to apply v/w (open loop)")
ap.add_argument("--turn-deg", type=float, default=None, help="closed-loop turn [deg], + = left (runs before --distance)")
ap.add_argument("--distance", type=float, default=None, help="closed-loop straight drive [m]")
ap.add_argument("--arm", default=None, help="joint_1..joint_6 in degrees, comma separated")
ap.add_argument("--arm-speed", type=float, default=45.0, help="arm ramp speed [deg/s] (capped by M0617 limits)")
ap.add_argument("--gripper", default=None, help="open | close | angle in degrees (-30..30)")
ap.add_argument("--hold", type=float, default=1.0, help="seconds to keep simulating at the end")
ap.add_argument("--headless", action="store_true")
ap.add_argument("--keep-open", action="store_true", help="GUI: keep the window open after the sequence")
args = ap.parse_args()

from isaacsim import SimulationApp
app = SimulationApp({"headless": args.headless})

import numpy as np
import omni.usd
from pxr import Usd, UsdGeom, UsdPhysics
from isaacsim.core.api import World
from isaacsim.core.utils.stage import open_stage

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from mm_control import MobileManipulatorControl

pkg = os.path.dirname(HERE)
usd = args.usd or os.path.join(pkg, "scenes", "mobile_manipulator_demo.usda" if args.robot == "mobile" else "m0617_fixed_base_demo.usda")
root = args.root or ("/World/mobile_manipulator" if args.robot == "mobile" else "/World/m0617_fixed_base")
open_stage(usd)
for _ in range(20):
    app.update()
stage = omni.usd.get_context().get_stage()
hz = 60
for p in stage.Traverse():
    if p.IsA(UsdPhysics.Scene):
        hz = p.GetAttribute("physxScene:timeStepsPerSecond").Get() or 60
        break
dt = 1.0 / hz
world = World(stage_units_in_meters=1.0, physics_dt=dt, rendering_dt=1.0 / 60.0)
world.reset()
ctl = MobileManipulatorControl(root, has_base=(args.robot == "mobile"), arm_speed_deg=args.arm_speed)
ctl.initialize()
render = not args.headless


def run(sec, cond=None):
    for _ in range(int(sec * hz)):
        ctl.step(dt)
        world.step(render=render)
        if cond and cond():
            break


def pose_str():
    x, y, yaw = ctl.base_pose()
    return f"x={x:.3f} y={y:.3f} yaw={yaw:.1f}deg"


print(f"[run_control] stage={usd} root={root} physics={hz}Hz")
run(1.0)
report = {}
if args.drive_time > 0:
    x0, y0, yaw0 = ctl.base_pose()
    ctl.set_base_velocity(args.v, args.w)
    run(args.drive_time)
    ctl.stop_base()
    run(0.5)
    x1, y1, yaw1 = ctl.base_pose()
    dyaw = (yaw1 - yaw0 + 180) % 360 - 180
    report["base"] = f"commanded v={args.v} w={args.w} for {args.drive_time}s | moved {math.hypot(x1-x0, y1-y0):.3f}m (ideal straight {abs(args.v)*args.drive_time:.3f}) yaw {dyaw:.1f}deg (ideal {math.degrees(args.w*args.drive_time):.1f})"
if args.turn_deg is not None:
    x0, y0, yaw0 = ctl.base_pose()
    ctl.turn_by_deg(args.turn_deg)
    run(60.0, cond=ctl.base_done)
    run(0.5)
    x1, y1, yaw1 = ctl.base_pose()
    report["turn_closed_loop"] = f"target={args.turn_deg}deg measured={(yaw1 - yaw0 + 180) % 360 - 180:.1f}deg (center shifted {math.hypot(x1-x0, y1-y0):.3f}m while turning)"
if args.distance is not None:
    x0, y0, yaw0 = ctl.base_pose()
    ctl.drive_distance(args.distance)
    run(60.0, cond=ctl.base_done)
    run(0.5)
    x1, y1, yaw1 = ctl.base_pose()
    h = math.radians(yaw0)
    along = (x1 - x0) * math.cos(h) + (y1 - y0) * math.sin(h)
    report["distance_closed_loop"] = f"target={args.distance}m measured={along:+.3f}m heading drift={(yaw1 - yaw0 + 180) % 360 - 180:.1f}deg"
if args.arm:
    q = [float(s) for s in args.arm.split(",")]
    ctl.set_arm_joints_deg(q)
    run(60.0, cond=ctl.arm_done)
    run(0.5)
    got = ctl.arm_joints_deg()
    report["arm"] = f"target={q} reached={np.round(got, 2).tolist()} max_err={np.abs(got - np.clip(q, ctl._arm_lo, ctl._arm_hi)).max():.2f}deg"
if args.gripper:
    if args.gripper == "open":
        ctl.open_gripper(); tgt = 30.0
    elif args.gripper == "close":
        ctl.close_gripper(); tgt = -30.0
    else:
        tgt = float(args.gripper); ctl.set_gripper_deg(tgt)
    run(2.0)
    report["gripper"] = f"target={tgt}deg reached={ctl.gripper_deg():.2f}deg"
run(args.hold)
print("[run_control] REPORT")
print(f"  final pose: {pose_str()}")
for k, v in report.items():
    print(f"  {k}: {v}")
print(f"  warnings: {ctl.warnings}")
sys.stdout.flush()
if args.keep_open and not args.headless:
    while app.is_running():
        ctl.step(dt)
        world.step(render=True)
# app.close() can hang on some headless setups; exit directly after flushing
os._exit(0)
