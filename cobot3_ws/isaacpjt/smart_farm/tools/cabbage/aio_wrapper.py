"""Run the team's runtime/standalone_app.py unmodified, plus a PhysX-step monitor of every cabbage head.

  D:\\isaacsim\\python.bat aio_wrapper.py <standalone_app.py> [standalone_app args...]
  env CABBAGE_MONITOR_OUT = json path (written every ~1 s of sim time)

The monitor registers itself when standalone_app imports sim_task_node (i.e. after SimulationApp is up) and
records, per head, the displacement / tilt relative to its own tray body (so it is valid while the tray is
carried), plus the Nova Carter chassis track and every tray pose.
"""
import builtins, json, math, os, runpy, sys

OUT = os.environ.get("CABBAGE_MONITOR_OUT", "cabbage_monitor.json")
VIEW_TRIGGER = r"D:\smartfarm-sim\out\SAVE_VIEW_CAMERA"          # save_view_camera.ps1 [name]
VIEW_SAVED = r"D:\smartfarm-sim\out\saved_view_cameras.json"      # read by 06_make_cabbage_scene.py
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
        out = {"sim_time_s": round(S["t"], 2), "heads": S["stats"], "trays": S["trays"], "chassis_track": S["track"][-2000:],
               "pallet01_track": S.get("tray_track", [])[-2000:]}
        with open(OUT, "w") as f:
            json.dump(out, f, indent=1)

    def capture(stage, spec):
        """CABBAGE_CAPTURE_CAMS = "name=ex,ey,ez,tx,ty,tz;name2=..." -> <out>/cap_<name>_<t>.png|.npz"""
        import omni.replicator.core as rep
        from pxr import Gf, UsdGeom as G
        if "annot" not in S:
            S["annot"] = {}
            for item in spec.split(";"):
                name, look = item.split("=")
                if look.startswith("/"):          # an existing camera prim of the scene (e.g. /World/ProcessCameras/...)
                    cam_path, res = look, (960, 540)
                    if "realsense" in look.lower():
                        res = (640, 640)          # wrist camera: same framing as the YOLO input
                else:
                    e = [float(v) for v in look.split(",")]
                    cam = G.Camera.Define(stage, f"/World/_Cap_{name}")
                    cam.CreateFocalLengthAttr(16.0)
                    cam.CreateClippingRangeAttr(Gf.Vec2f(0.05, 1000.0))
                    m = Gf.Matrix4d().SetLookAt(Gf.Vec3d(*e[:3]), Gf.Vec3d(*e[3:]), Gf.Vec3d(0, 0, 1)).GetInverse()
                    G.Xformable(cam).ClearXformOpOrder(); G.Xformable(cam).AddTransformOp().Set(m)
                    cam_path, res = str(cam.GetPath()), (960, 540)
                rp = rep.create.render_product(cam_path, res)
                a = rep.AnnotatorRegistry.get_annotator("rgb"); a.attach([rp])
                S["annot"][name] = a
            os.makedirs(os.path.join(os.path.dirname(OUT), "captures"), exist_ok=True)
            return
        for name, a in S["annot"].items():
            data = a.get_data()
            if data is None or getattr(data, "size", 0) == 0:
                continue
            base = os.path.join(os.path.dirname(OUT), "captures", "cap_%s_%06.1f" % (name, S["t"]))
            try:
                from PIL import Image
                Image.fromarray(np.asarray(data)[..., :3]).save(base + ".jpg", quality=88)
            except ImportError:
                np.savez_compressed(base + ".npz", rgb=np.asarray(data)[..., :3])

    def on_step(dt):
        S["n"] += 1
        S["t"] += dt
        cams = os.environ.get("CABBAGE_CAPTURE_CAMS", "")
        every = max(1, int(round(float(os.environ.get("CABBAGE_CAPTURE_EVERY", "3.0")) * 60)))
        if cams and S["n"] % every == 0:     # before the 15-step monitor gate (else captures drop to 1 per second)
            try:   # fixed process cameras rendered off-screen (the GUI viewport is left alone)
                capture(omni.usd.get_context().get_stage(), cams)
            except Exception as error:  # noqa: BLE001
                if not S.get("cap_err"):
                    print(f"[MONITOR] capture failed: {error}", flush=True)
                    S["cap_err"] = True
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
        if S["n"] % 60 == 0 and os.path.exists(VIEW_TRIGGER):
            try:   # save the GUI viewport's current view as a camera (save_view_camera.ps1 drops the trigger file)
                from omni.kit.viewport.utility import get_active_viewport
                from pxr import Gf, UsdGeom as G
                vp = get_active_viewport()
                src = stage.GetPrimAtPath(vp.camera_path)
                m = G.Xformable(src).ComputeLocalToWorldTransform(0)
                focal = G.Camera(src).GetFocalLengthAttr().Get() or 18.147
                saved = json.load(open(VIEW_SAVED)) if os.path.exists(VIEW_SAVED) else []
                name = open(VIEW_TRIGGER, encoding="utf-8-sig").read().strip() or f"Cam0_View{len(saved) + 1}"
                saved.append({"name": name, "matrix": [float(m[i][j]) for i in range(4) for j in range(4)], "focal": float(focal),
                              "from": str(vp.camera_path), "eye": [round(float(v), 3) for v in m.ExtractTranslation()]})
                json.dump(saved, open(VIEW_SAVED, "w"), indent=1)
                cam = G.Camera.Define(stage, f"/World/ProcessCameras/{name}")
                cam.CreateFocalLengthAttr(float(focal))
                G.Xformable(cam).ClearXformOpOrder(); G.Xformable(cam).AddTransformOp().Set(m)
                print(f"[MONITOR] viewport view saved as /World/ProcessCameras/{name} -> {VIEW_SAVED}", flush=True)
            except Exception as error:  # noqa: BLE001
                print(f"[MONITOR] view save failed: {error}", flush=True)
            os.remove(VIEW_TRIGGER)
        if S["n"] % 60 == 0:
            c = pose(CHASSIS)
            if c is not None:
                yaw = math.degrees(math.atan2(c[1][1, 0], c[1][0, 0]))
                S["track"].append([round(S["t"], 2), round(float(c[0][0]), 4), round(float(c[0][1]), 4), round(yaw, 2)])
            for _, tray in (S["heads"] or []):
                pt = pose(tray)
                if pt is not None:
                    yaw = math.degrees(math.atan2(pt[1][1, 0], pt[1][0, 0]))
                    key = tray.replace("/World/SmartFarm/Placed/", "")
                    S["trays"][key] = [round(float(v), 4) for v in pt[0]] + [round(yaw, 2)]
                    if key.startswith("Pallet_01"):
                        S.setdefault("tray_track", []).append([round(S["t"], 1)] + S["trays"][key])
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
