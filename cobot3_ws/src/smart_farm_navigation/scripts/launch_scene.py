"""Isaac Sim standalone launcher for smart_farm_nav2_01.usd.

Opens the scene, enables the ROS 2 bridge, presses Play, and keeps the
simulation running until SIGTERM/SIGINT.  Run with Isaac Sim's python:

    ~/isaacsim/python.sh launch_scene.py [scene.usd]

The ROS 2 environment (ROS_DISTRO, ROS_DOMAIN_ID, RMW, bridge lib path)
must already be exported by the calling shell; gopi_run.sh does that.
"""

import signal
import sys

DEFAULT_SCENE = "/home/rokey/ROKEY_P3_A1/cobot3_ws/isaacpjt/smart_farm/scenes/smart_farm_nav2_01.usd"
scene = sys.argv[1] if len(sys.argv) > 1 else DEFAULT_SCENE

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
            if path.count("/") > 2:
                continue
            if any(k in path.lower() for k in ("carter", "nova")):
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
            print(f"[launch_scene] spawn pose written to {out}", flush=True)
        else:
            print("[launch_scene] WARNING: no prim with 'carter'/'nova' in its name under /World; AMCL initial pose must be given manually", flush=True)
    except Exception as exc:  # noqa: BLE001
        print(f"[launch_scene] spawn pose record failed: {exc}", flush=True)

    signal.signal(signal.SIGTERM, _request_stop)
    signal.signal(signal.SIGINT, _request_stop)

    omni.timeline.get_timeline_interface().play()
    print("[launch_scene] PLAY", flush=True)
    while app.is_running() and running:
        app.update()
    print("[launch_scene] stopping", flush=True)
    omni.timeline.get_timeline_interface().stop()
    app.update()
finally:
    app.close()
