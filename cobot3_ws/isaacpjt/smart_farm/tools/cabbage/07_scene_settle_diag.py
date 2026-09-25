"""No-command settle check of a full smart-farm scene: do trays / heads / the Carter drift on their own?

  D:\\isaacsim\\python.bat 07_scene_settle_diag.py SCENE OUT_JSON [SECONDS]
"""
import sys, json, math, traceback
SCENE, OUT = sys.argv[1], sys.argv[2]
SECS = float(sys.argv[3]) if len(sys.argv) > 3 else 10.0
from isaacsim import SimulationApp
app = SimulationApp({"headless": True})
res = {}
try:
    import numpy as np
    import omni.usd
    from pxr import Usd, UsdPhysics
    from isaacsim.core.api import World
    from omni.physx import get_physx_interface
    from scipy.spatial.transform import Rotation as Rot
    omni.usd.get_context().open_stage(SCENE)
    while omni.usd.get_context().get_stage_loading_status()[2] > 0:
        app.update()
    stage = omni.usd.get_context().get_stage()
    for prim in stage.Traverse():
        if prim.GetTypeName() == "OmniGraph":
            prim.SetActive(False)
    world = World(stage_units_in_meters=1.0, physics_dt=1 / 60, rendering_dt=1 / 60, physics_prim_path="/physicsScene")
    watch = []
    for prim in stage.Traverse():
        p = str(prim.GetPath())
        if prim.HasAPI(UsdPhysics.RigidBodyAPI) and "/Placed/Pallet" in p and not UsdPhysics.RigidBodyAPI(prim).GetKinematicEnabledAttr().Get():
            watch.append(p)
    watch.append("/World/SmartFarm/Placed/LiftRig/Asset/nova_carter_ROS/chassis_link")
    world.reset()
    px = get_physx_interface()
    def pose(p):
        t = px.get_rigidbody_transformation(p)
        return np.array(t["position"]), Rot.from_quat(t["rotation"])
    p0 = {p: pose(p) for p in watch}
    worst = {p: 0.0 for p in watch}
    for i in range(int(SECS * 60)):
        world.step(render=False)
        if i % 30 == 29:
            for p in watch:
                worst[p] = max(worst[p], float(np.linalg.norm(pose(p)[0] - p0[p][0])))
    for p in watch:
        q, r = pose(p)
        yaw = math.degrees((p0[p][1].inv() * r).as_euler("xyz")[2])
        tilt = math.degrees((p0[p][1].inv() * r).magnitude())
        key = p.replace("/World/SmartFarm/Placed/", "")
        res[key] = {"disp_mm": round(float(np.linalg.norm(q - p0[p][0])) * 1000, 2), "max_disp_mm": round(worst[p] * 1000, 2),
                    "rot_deg": round(tilt, 2), "yaw_deg": round(yaw, 2), "start": p0[p][0].round(4).tolist()}
except Exception:
    res["error"] = traceback.format_exc()
open(OUT, "w").write(json.dumps(res, indent=1))
app.close()
