"""Headless PhysX checks for cabbage_pallet_6 (Isaac Sim 5.1).

  D:\\isaacsim\\python.bat 04_physics_tests.py ASSET OUT_JSON MODE [HZ]
    MODE settle    : tray on the ground, 5 s free, every body tracked
    MODE transport : tray kinematic: lift 0.15 m, carry 0.6 m (1.5 m/s2) with 5 deg pitch, side 0.3 m, hard lower
    MODE release   : tray kinematic, gravity reversed: each head must leave its seat straight (no catch / tilt)
"""
import sys, json, math, traceback
ASSET, OUT, MODE = sys.argv[1], sys.argv[2], sys.argv[3]
HZ = float(sys.argv[4]) if len(sys.argv) > 4 else 60.0
from isaacsim import SimulationApp
app = SimulationApp({"headless": True})
res = {"mode": MODE, "hz": HZ}
try:
    import numpy as np
    import omni.usd
    from pxr import Usd, UsdGeom, UsdPhysics, Gf, Sdf
    from isaacsim.core.api import World
    from omni.physx import get_physx_interface
    from scipy.spatial.transform import Rotation as Rot

    DT = 1.0 / HZ
    world = World(stage_units_in_meters=1.0, physics_dt=DT, rendering_dt=DT)
    stage = omni.usd.get_context().get_stage()
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    Z0 = -0.02592913
    if MODE == "settle":
        g = UsdGeom.Cube.Define(stage, "/World/ground"); g.CreateSizeAttr(1.0)
        g.AddTranslateOp().Set(Gf.Vec3d(0, 0, Z0 - 0.05)); g.AddScaleOp().Set(Gf.Vec3f(4, 4, 0.1))
        UsdPhysics.CollisionAPI.Apply(g.GetPrim())
    stage.DefinePrim("/World/asset", "Xform").GetReferences().AddReference(ASSET)
    bodies = [p for p in stage.Traverse() if p.HasAPI(UsdPhysics.RigidBodyAPI)]
    pal = [p for p in bodies if p.GetName() == "Cube_011_001"][0]
    heads = [str(p.GetPath()) for p in bodies if p.GetName().startswith("Cabbage_")]
    pal_path = str(pal.GetPath())
    if MODE in ("transport", "release"):
        UsdPhysics.RigidBodyAPI(pal).GetKinematicEnabledAttr().Set(True)
    if MODE == "release":
        sc = [p for p in stage.Traverse() if p.IsA(UsdPhysics.Scene)]
        if not sc:
            world.reset(); sc = [p for p in stage.Traverse() if p.IsA(UsdPhysics.Scene)]
        UsdPhysics.Scene(sc[0]).GetGravityDirectionAttr().Set(Gf.Vec3f(0, 0, 1))
    if MODE == "transport":
        xf = UsdGeom.Xformable(pal)
        t_op = xf.AddTranslateOp(opSuffix="drive"); o_op = xf.AddOrientOp(opSuffix="drive")
        t_op.Set(Gf.Vec3d(0, 0, 0)); o_op.Set(Gf.Quatf(1, 0, 0, 0))
        for hp in heads:   # never sleep, so nothing is carried by a sleeping island
            hpr = stage.GetPrimAtPath(hp); hpr.AddAppliedSchema("PhysxRigidBodyAPI")
            hpr.CreateAttribute("physxRigidBody:sleepThreshold", Sdf.ValueTypeNames.Float).Set(0.0)
    world.reset()
    px = get_physx_interface()

    def pose(p):
        t = px.get_rigidbody_transformation(p)
        return np.array(t["position"]), Rot.from_quat(t["rotation"])   # x y z w

    names = {b: b.split("/")[-1] for b in [pal_path] + heads}
    if MODE == "settle":
        p0 = {b: pose(b) for b in names}
        st = {b: {"max_disp_mm": 0.0, "max_tilt_deg": 0.0} for b in names}
        for i in range(int(5 * HZ)):
            world.step(render=False)
            for b in names:
                p, r = pose(b)
                st[b]["max_disp_mm"] = max(st[b]["max_disp_mm"], float(np.linalg.norm(p - p0[b][0])) * 1000)
                st[b]["max_tilt_deg"] = max(st[b]["max_tilt_deg"], math.degrees((p0[b][1].inv() * r).magnitude()))
        for b in names:
            p, r = pose(b)
            st[b]["final_disp_mm"] = float(np.linalg.norm(p - p0[b][0])) * 1000
            st[b]["final_dz_mm"] = float(p[2] - p0[b][0][2]) * 1000
            res[names[b]] = {k: round(v, 3) for k, v in st[b].items()}
    elif MODE == "transport":
        def s_curve(t, t0, T):
            u = min(max((t - t0) / T, 0.0), 1.0); return 0.5 - 0.5 * math.cos(math.pi * u)
        A = 1.5; PITCH = 5.0
        TX = math.sqrt(math.pi ** 2 / 2 * 0.6 / A); TY = math.sqrt(math.pi ** 2 / 2 * 0.3 / A)
        t_lift, t_x = 0.5, 1.6; t_y = t_x + TX + 0.2; t_down = t_y + TY + 0.2; T_END = t_down + 1.5
        def pal_pose(t):
            z = 0.15 * s_curve(t, t_lift, 1.0) - 0.15 * s_curve(t, t_down, 0.6)
            x = 0.6 * s_curve(t, t_x, TX); y = 0.3 * s_curve(t, t_y, TY)
            pitch = PITCH * s_curve(t, t_x, 0.6) - PITCH * s_curve(t, t_down, 0.6)
            return np.array([x, y, z]), Rot.from_euler("y", pitch, degrees=True)
        def rel(h, pp, rp):
            p, r = pose(h); return rp.inv().apply(p - pp), math.degrees((rp.inv() * r).magnitude())
        pp, rp = pal_pose(0.0)
        ref = {h: rel(h, pp, rp)[0] for h in heads}
        worst = {h: {"max_rel_disp_mm": 0.0, "max_rel_tilt_deg": 0.0} for h in heads}
        t = 0.0
        while t < T_END:
            t += DT
            pp, rp = pal_pose(t)
            qx, qy, qz, qw = rp.as_quat()
            t_op.Set(Gf.Vec3d(*pp)); o_op.Set(Gf.Quatf(qw, qx, qy, qz))
            world.step(render=False)
            for h in heads:
                d, tl = rel(h, pp, rp)
                worst[h]["max_rel_disp_mm"] = max(worst[h]["max_rel_disp_mm"], float(np.linalg.norm(d - ref[h])) * 1000)
                worst[h]["max_rel_tilt_deg"] = max(worst[h]["max_rel_tilt_deg"], tl)
        for h in heads:
            d, tl = rel(h, pp, rp)
            worst[h]["final_rel_disp_mm"] = float(np.linalg.norm(d - ref[h])) * 1000
            res[names[h]] = {k: round(v, 3) for k, v in worst[h].items()}
        res["profile"] = {"peak_accel_mps2": A, "pitch_deg": PITCH, "lift_m": 0.15, "carry_m": [0.6, 0.3]}
    elif MODE == "release":
        p0 = {h: pose(h)[0] for h in heads}
        out = {names[h]: None for h in heads}
        for i in range(int(1.0 * HZ)):
            world.step(render=False)
            for h in heads:
                p, r = pose(h)
                if out[names[h]] is None and p[2] - p0[h][2] > 0.020:      # past the 8 mm cone + bore
                    out[names[h]] = {"t_s": round((i + 1) * DT, 3),
                                     "lateral_mm": round(float(np.linalg.norm(p[:2] - p0[h][:2])) * 1000, 3),
                                     "tilt_deg": round(math.degrees(r.magnitude()), 3)}
        res.update(out)
except Exception:
    res["error"] = traceback.format_exc()
open(OUT, "w").write(json.dumps(res, indent=1))
app.close()
