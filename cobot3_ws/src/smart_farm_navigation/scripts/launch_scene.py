"""Isaac Sim standalone launcher for the Nav2 test (default scene: Collected_smartfarm_v014 cabbage).

Opens the scene, enables the ROS 2 bridge, adds a /clock publisher graph when the
scene has none (Nav2 runs with use_sim_time, so /clock is mandatory), presses Play,
and keeps the simulation running until SIGTERM/SIGINT.  Run with Isaac Sim's python:

    ~/isaacsim/python.sh launch_scene.py [scene.usd] [--pose carry | --arm-joints 270,0,0,0,0,0] [--lift 0.35]

Also (all at runtime only, the USD on disk is never modified):
  * the 3D lidar helper is switched to fullScan so /front_3d_lidar/lidar_points comes at the
    sensor scan rate (10 Hz) instead of once per rendered frame (20-60 Hz);
  * --pose <name> / --arm-joints / --lift move the M0609 joints and the lift to a preset from
    config/arm_poses.yaml after Play (guidance2 14차 test 2: carry pose without a pallet).
"""

import argparse
import datetime
import faulthandler
import os
import signal
import sys

# Every print goes to the terminal AND to results/launch_scene_<time>.log; a native crash (segfault inside
# PhysX/Kit) leaves a Python stack dump in the same file via faulthandler.
_LOG_DIR = "/home/rokey/ROKEY_P3_A1/cobot3_ws/src/smart_farm_navigation/results"
os.makedirs(_LOG_DIR, exist_ok=True)
_LOG_PATH = os.path.join(_LOG_DIR, f"launch_scene_{datetime.datetime.now():%Y%m%d_%H%M}.log")
_log_file = open(_LOG_PATH, "a", buffering=1)
faulthandler.enable(file=_log_file, all_threads=True)


class _Tee:
    def __init__(self, *streams): self.streams = streams
    def fileno(self): return self.streams[0].fileno()          # SimulationApp calls faulthandler.enable() on sys.stderr
    def isatty(self): return self.streams[0].isatty()
    @property
    def encoding(self): return getattr(self.streams[0], "encoding", "utf-8")
    def write(self, data):
        for st in self.streams:
            try: st.write(data); st.flush()
            except Exception: pass
    def flush(self):
        for st in self.streams:
            try: st.flush()
            except Exception: pass


sys.stdout = _Tee(sys.__stdout__, _log_file)
sys.stderr = _Tee(sys.__stderr__, _log_file)
print(f"[launch_scene] log -> {_LOG_PATH}  argv={sys.argv[1:]}", flush=True)

# 폴더 이름과 파일 이름이 다르다(ADR_nav2 1.1.2). 폴더에 딸려 온 README_사용법.md 가 여는 파일을 밝힌다.
DEFAULT_SCENE = ("/home/rokey/ROKEY_P3_A1/cobot3_ws/isaacpjt/smart_farm/scenes/"
                 "Collected_smartfarm_v014/Collected_smartfarm_v014_room_core_cabbage.usd")
_ap = argparse.ArgumentParser()
_ap.add_argument("scene", nargs="?", default=DEFAULT_SCENE)
_ap.add_argument("--pose", default="", help="preset name in config/arm_poses.yaml (e.g. carry)")
_ap.add_argument("--arm-joints", default="", help="6 joint angles in degrees, comma separated (joint_1..joint_6)")
_ap.add_argument("--lift", type=float, default=None, help="lift_prismatic_joint position in metres (0..0.61)")
_ap.add_argument("--cloud-full-scan", default="true", help="true: publish the 3D cloud once per full scan (10 Hz)")
args, _unknown = _ap.parse_known_args()
scene = args.scene
ARM_POSES = "/home/rokey/ROKEY_P3_A1/cobot3_ws/src/smart_farm_navigation/config/arm_poses.yaml"
RIG_PATH = "/World/SmartFarm/Placed/LiftRig/Asset/nova_carter_ROS"
ARM_PATH = RIG_PATH + "/m0609_with_fork"
LIFT_JOINT = RIG_PATH + "/lift_v3_physics/lift_prismatic_joint"

from isaacsim import SimulationApp  # noqa: E402  (must precede other omni imports)

import os as _os
_exp = _os.environ.get("ISAAC_EXPERIENCE", "")        # e.g. ISAAC_EXPERIENCE=full -> same extension set as the `isaac` GUI
_kw = {}
if _exp:
    _path = _exp if _exp.endswith(".kit") else _os.path.expanduser(f"~/isaacsim/apps/isaacsim.exp.{_exp}.kit")
    _kw["experience"] = _path
    print(f"[launch_scene] experience: {_path}", flush=True)
app = SimulationApp({"headless": False}, **_kw)

running = True


def _request_stop(_sig, _frame):
    global running
    running = False


try:
    import omni.timeline
    from isaacsim.core.utils.extensions import enable_extension
    from isaacsim.core.utils.stage import is_stage_loading, open_stage

    enable_extension("isaacsim.ros2.bridge")
    app.update()

    print(f"[launch_scene] opening {scene}", flush=True)
    if not open_stage(scene):
        print("[launch_scene] open_stage failed", flush=True)
        raise SystemExit(2)
    while is_stage_loading():
        app.update()
    for _ in range(60):  # let OmniGraph / ROS2 contexts initialise
        app.update()

    # Record the robot's world pose (= AMCL initial pose in the map frame) for nav2.launch.py.
    try:
        import math as _m
        import omni.usd as _ou
        from pxr import UsdGeom as _UG, Usd as _U
        stage = _ou.get_context().get_stage()
        xc = _UG.XformCache()
        found = []
        for prim in stage.Traverse():
            path = str(prim.GetPath())
            if prim.GetName().lower() not in ("nova_carter_ros", "nova_carter", "carter", "carter1", "carter2"):
                continue
            m = xc.GetLocalToWorldTransform(prim)
            t = m.ExtractTranslation(); q = m.ExtractRotationQuat()
            w = q.GetReal(); qx, qy, qz = q.GetImaginary()
            yaw = _m.degrees(_m.atan2(2 * (w * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz)))
            found.append((path, float(t[0]), float(t[1]), float(t[2]), yaw))
        for path, x, y, z, yaw in found:
            print(f"[launch_scene] robot prim {path}: x={x:.3f} y={y:.3f} z={z:.3f} yaw={yaw:.1f}deg", flush=True)
        if found:
            import datetime as _dt
            path, x, y, z, yaw = found[0]
            out = "/home/rokey/ROKEY_P3_A1/cobot3_ws/src/smart_farm_navigation/results/robot_spawn.yaml"
            with open(out, "w") as f:
                f.write(f"# written by launch_scene.py {_dt.datetime.now():%Y-%m-%d %H:%M:%S}\n"
                        f"scene: {scene}\nprim: {path}\nx: {x:.4f}\ny: {y:.4f}\nyaw_deg: {yaw:.2f}\n")
                f.write("robots:\n")
                for i, (p2, x2, y2, z2, yaw2) in enumerate(found, 1):
                    f.write(f"  - name: carter{i}\n    prim: {p2}\n    x: {x2:.4f}\n    y: {y2:.4f}\n    yaw_deg: {yaw2:.2f}\n")
            print(f"[launch_scene] spawn pose(s) written to {out} ({len(found)} robot prim(s))", flush=True)
        else:
            print("[launch_scene] WARNING: no nova_carter prim found; check config/stations.yaml initial_pose by hand", flush=True)
    except Exception as exc:  # noqa: BLE001
        print(f"[launch_scene] spawn pose record failed: {exc}", flush=True)

    # /clock: Nav2 (use_sim_time) needs it. Collected_smartfarm_v002+ scenes ship without a ROS_Clock graph.
    try:
        import omni.graph.core as og
        import omni.usd as _ou2
        _stage = _ou2.get_context().get_stage()
        has_clock = any(
            p.GetTypeName() == "OmniGraphNode"
            and str(p.GetAttribute("node:type").Get() or "").endswith("ROS2PublishClock")
            for p in _stage.Traverse()
        )
        if has_clock:
            print("[launch_scene] /clock: scene already has a ROS2PublishClock node", flush=True)
        else:
            og.Controller.edit(
                {"graph_path": "/World/ROS_Clock_runtime", "evaluator_name": "execution"},
                {
                    og.Controller.Keys.CREATE_NODES: [
                        ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                        ("ReadSimTime", "isaacsim.core.nodes.IsaacReadSimulationTime"),
                        ("Context", "isaacsim.ros2.bridge.ROS2Context"),
                        ("PublishClock", "isaacsim.ros2.bridge.ROS2PublishClock"),
                    ],
                    og.Controller.Keys.CONNECT: [
                        ("OnPlaybackTick.outputs:tick", "PublishClock.inputs:execIn"),
                        ("ReadSimTime.outputs:simulationTime", "PublishClock.inputs:timeStamp"),
                        ("Context.outputs:context", "PublishClock.inputs:context"),
                    ],
                    og.Controller.Keys.SET_VALUES: [
                        ("ReadSimTime.inputs:resetOnStop", False),
                        ("PublishClock.inputs:topicName", "clock"),
                    ],
                },
            )
            print("[launch_scene] /clock: added runtime graph /World/ROS_Clock_runtime (not saved to the USD)", flush=True)
    except Exception as exc:  # noqa: BLE001
        print(f"[launch_scene] /clock graph creation FAILED: {exc}", flush=True)

    # Hold the M0609 joints where the scene saved them (same drive values as the team's robot_motion.setup_arm_drives).
    # Without this the arm sags under gravity within a minute, its links leave the self-filter box and the local
    # costmap gets lethal cells on the rig itself (2026-09-22 test 1: 114 -> 313 self points/scan, "collision ahead").
    try:
        import omni.usd as _ou5
        from pxr import UsdPhysics as _UP5
        _st5 = _ou5.get_context().get_stage()
        n_drv = 0
        for i in range(1, 7):
            prim = _st5.GetPrimAtPath(f"{ARM_PATH}/joints/joint_{i}")
            if not prim:
                continue
            drv = _UP5.DriveAPI.Get(prim, "angular")
            drv.GetStiffnessAttr().Set(1e8); drv.GetDampingAttr().Set(1e4); drv.GetMaxForceAttr().Set(1e8); n_drv += 1
        print(f"[launch_scene] M0609 joint drives held ({n_drv} joints, stiffness 1e8)", flush=True)
    except Exception as exc:  # noqa: BLE001
        print(f"[launch_scene] arm drive hold FAILED: {exc}", flush=True)

    # 3D lidar: one message per full scan (sensor scanRateBaseHz = 10) instead of one per rendered frame.
    try:
        import omni.usd as _ou3
        from pxr import Usd as _U3
        _st = _ou3.get_context().get_stage()
        n_set = 0
        for prim in _st.Traverse():
            if prim.GetTypeName() == "OmniGraphNode" and str(prim.GetAttribute("node:type").Get() or "").endswith("ROS2RtxLidarHelper"):
                if str(prim.GetAttribute("inputs:type").Get() or "") == "point_cloud":
                    prim.GetAttribute("inputs:fullScan").Set(args.cloud_full_scan.lower() == "true")
                    n_set += 1
        print(f"[launch_scene] 3D lidar fullScan={args.cloud_full_scan.lower() == 'true'} on {n_set} helper node(s) -> about 10 Hz", flush=True)
    except Exception as exc:  # noqa: BLE001
        print(f"[launch_scene] lidar fullScan setting FAILED: {exc}", flush=True)

    signal.signal(signal.SIGTERM, _request_stop)
    signal.signal(signal.SIGINT, _request_stop)

    omni.timeline.get_timeline_interface().play()
    print("[launch_scene] PLAY", flush=True)
    for _ in range(30):
        app.update()

    # Optional arm / lift pose (guidance2 14차 test 2): joint drive targets (USD DriveAPI, degrees / metres).
    arm_deg, lift_m = None, args.lift
    if args.pose:
        import yaml as _yaml
        preset = (_yaml.safe_load(open(ARM_POSES)) or {}).get("poses", {}).get(args.pose)
        if preset is None:
            print(f"[launch_scene] pose '{args.pose}' not in {ARM_POSES}", flush=True)
        else:
            arm_deg = [float(v) for v in preset.get("arm_joints_deg", [])] or None
            lift_m = preset.get("lift_m", lift_m) if lift_m is None else lift_m
    if args.arm_joints:
        arm_deg = [float(v) for v in args.arm_joints.split(",")]
    if arm_deg is not None or lift_m is not None:
        try:
            import omni.usd as _ou4
            from pxr import UsdPhysics as _UP4
            _st4 = _ou4.get_context().get_stage()
            drives = []          # (DriveAPI, start, goal, label)
            if arm_deg is not None:
                for i, d in enumerate(arm_deg, 1):
                    drv = _UP4.DriveAPI.Get(_st4.GetPrimAtPath(f"{ARM_PATH}/joints/joint_{i}"), "angular")
                    cur = float(drv.GetTargetPositionAttr().Get() or 0.0)
                    drives.append((drv, cur, float(d), f"joint_{i}"))
            if lift_m is not None:
                drv = _UP4.DriveAPI.Get(_st4.GetPrimAtPath(f"{LIFT_JOINT}"), "linear")
                drv.GetStiffnessAttr().Set(max(float(drv.GetStiffnessAttr().Get() or 0.0), 1e6))
                drv.GetDampingAttr().Set(max(float(drv.GetDampingAttr().Get() or 0.0), 1e4))
                cur = float(drv.GetTargetPositionAttr().Get() or 0.0)
                drives.append((drv, cur, float(lift_m), "lift"))
            print("[launch_scene] pose ramp start: " + ", ".join(f"{n} {c:.2f}->{g:.2f}" for _, c, g, n in drives), flush=True)
            # Ramp the drive targets over POSE_RAMP_STEPS frames (about 8 s of sim) instead of jumping: with the
            # team's stiffness (1e8) a jump swings the arm into the lift and can crash PhysX.
            POSE_RAMP_STEPS = 480
            for k in range(1, POSE_RAMP_STEPS + 1):
                f = k / POSE_RAMP_STEPS
                for drv, c, g, _n in drives:
                    drv.GetTargetPositionAttr().Set(c + (g - c) * f)
                app.update()
            for _ in range(120):
                app.update()
            print("[launch_scene] pose targets applied: " + ", ".join(f"{n}={g:.2f}" for _, _c, g, n in drives), flush=True)
        except BaseException as exc:  # noqa: BLE001
            import traceback as _tb
            print(f"[launch_scene] arm/lift pose FAILED: {exc}\n{_tb.format_exc()}", flush=True)

    while app.is_running() and running:
        app.update()
    print("[launch_scene] stopping", flush=True)
    omni.timeline.get_timeline_interface().stop()
    app.update()
finally:
    app.close()
