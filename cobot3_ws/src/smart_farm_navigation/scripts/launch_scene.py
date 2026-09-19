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

app = SimulationApp({"headless": False})

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
