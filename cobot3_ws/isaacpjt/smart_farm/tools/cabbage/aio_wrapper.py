"""Run the team's runtime/standalone_app.py unmodified, plus a PhysX-step monitor of every cabbage head.

  D:\\isaacsim\\python.bat aio_wrapper.py <standalone_app.py> [standalone_app args...]
  env CABBAGE_MONITOR_OUT = json path (written every ~1 s of sim time)

The monitor registers itself when standalone_app imports sim_task_node (i.e. after SimulationApp is up) and
records, per head, the displacement / tilt relative to its own tray body (so it is valid while the tray is
carried), plus the Nova Carter chassis track and every tray pose.
"""
import builtins, json, math, os, runpy, sys

OUT = os.environ.get("CABBAGE_MONITOR_OUT", "cabbage_monitor.json")
_orig_import = builtins.__import__
_state = {"installed": False, "sub": None}


def _install():
    import numpy as np
    import omni.physx
    import omni.usd
    from pxr import UsdPhysics

    phys = omni.physx.get_physx_interface()
    S = {"n": 0, "t": 0.0, "heads": None, "ref": {}, "stats": {}, "track": [], "trays": {}}
    CHASSIS = "/World/SmartFarm/Placed/LiftRig/Asset/nova_carter_ROS/chassis_link"

    def rot(q):   # x y z w -> 3x3
        x, y, z, w = q
        return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                         [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                         [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])

    def pose(path):
        t = phys.get_rigidbody_transformation(path)
        if not t or not t.get("ret_val", True):
            return None
        return np.array(t["position"], float), rot(t["rotation"])

    def dump():
        out = {"sim_time_s": round(S["t"], 2), "heads": S["stats"], "trays": S["trays"], "chassis_track": S["track"][-2000:]}
        with open(OUT, "w") as f:
            json.dump(out, f, indent=1)

    def on_step(dt):
        S["n"] += 1
        S["t"] += dt
        if S["n"] % 15:
            return
        stage = omni.usd.get_context().get_stage()
        if S["heads"] is None:
            heads = []
            for p in stage.Traverse():
                if p.GetName().startswith("Cabbage_") and p.HasAPI(UsdPhysics.RigidBodyAPI):
                    path = str(p.GetPath())
                    heads.append((path, path.split("/root_001/")[0] + "/Cube_011_001"))
            S["heads"] = heads
            print(f"[MONITOR] tracking {len(heads)} cabbage heads", flush=True)
        for h, tray in S["heads"]:
            ph, pt = pose(h), pose(tray)
            if ph is None or pt is None:
                continue
            rel = pt[1].T @ (ph[0] - pt[0])
            up = pt[1].T @ ph[1][:, 2]
            if h not in S["ref"]:
                S["ref"][h] = (rel, up)
            r0, u0 = S["ref"][h]
            d = float(np.linalg.norm(rel - r0)) * 1000
            tilt = math.degrees(math.acos(max(-1.0, min(1.0, float(np.dot(up, u0))))))
            key = h.replace("/World/SmartFarm/Placed/", "")
            st = S["stats"].setdefault(key, {"max_rel_disp_mm": 0.0, "max_rel_tilt_deg": 0.0, "t_max": 0.0})
            if d > st["max_rel_disp_mm"]:
                st["max_rel_disp_mm"], st["t_max"] = round(d, 2), round(S["t"], 2)
            st["max_rel_tilt_deg"] = round(max(st["max_rel_tilt_deg"], tilt), 2)
            st["rel_disp_mm"], st["rel_tilt_deg"] = round(d, 2), round(tilt, 2)
        snap_from = float(os.environ.get("CABBAGE_SNAP_FROM", "0") or 0)
        if snap_from and S["t"] >= snap_from and S["n"] % int(float(os.environ.get("CABBAGE_SNAP_EVERY", "1.0")) * 60) == 0:
            try:   # viewport screenshots of a chosen time window (GUI runs only)
                from omni.kit.viewport.utility import get_active_viewport, capture_viewport_to_file
                vp = get_active_viewport()
                look = os.environ.get("CABBAGE_SNAP_CAM", "")        # "ex,ey,ez,tx,ty,tz"
                if vp is not None and look and not S.get("cam"):
                    from pxr import Gf, UsdGeom as G
                    e = [float(v) for v in look.split(",")]
                    cam = G.Camera.Define(stage, "/World/_SnapCam")
                    cam.CreateFocalLengthAttr(18.0)
                    m = Gf.Matrix4d().SetLookAt(Gf.Vec3d(*e[:3]), Gf.Vec3d(*e[3:]), Gf.Vec3d(0, 0, 1)).GetInverse()
                    G.Xformable(cam).ClearXformOpOrder(); G.Xformable(cam).AddTransformOp().Set(m)
                    vp.camera_path = "/World/_SnapCam"; S["cam"] = True
                if vp is not None:
                    capture_viewport_to_file(vp, os.path.join(os.path.dirname(OUT), "snap_%06.1f.png" % S["t"]))
            except Exception as error:  # noqa: BLE001
                print(f"[MONITOR] snapshot failed: {error}", flush=True)
        if S["n"] % 60 == 0:
            c = pose(CHASSIS)
            if c is not None:
                yaw = math.degrees(math.atan2(c[1][1, 0], c[1][0, 0]))
                S["track"].append([round(S["t"], 2), round(float(c[0][0]), 4), round(float(c[0][1]), 4), round(yaw, 2)])
            for _, tray in (S["heads"] or []):
                pt = pose(tray)
                if pt is not None:
                    S["trays"][tray.replace("/World/SmartFarm/Placed/", "")] = [round(float(v), 4) for v in pt[0]]
            dump()

    _state["sub"] = phys.subscribe_physics_step_events(on_step)
    print(f"[MONITOR] cabbage monitor installed -> {OUT}", flush=True)


def _hook(name, globals=None, locals=None, fromlist=(), level=0):
    module = _orig_import(name, globals, locals, fromlist, level)
    if not _state["installed"] and name == "sim_task_node":
        _state["installed"] = True
        try:
            _install()
        except Exception as error:  # noqa: BLE001
            print(f"[MONITOR] install failed: {error}", flush=True)
    return module


builtins.__import__ = _hook
target = os.path.abspath(sys.argv[1])
sys.argv = [target] + sys.argv[2:]
sys.path.insert(0, os.path.dirname(target))     # what `python standalone_app.py` would put first
runpy.run_path(target, run_name="__main__")
