"""runtime/standalone_app.py 를 그대로 실행하면서 기록만 덧붙인다 (Isaac 파이썬으로 실행).

  <Isaac>\\python.bat run_with_monitor.py <smart_farm>\\runtime\\standalone_app.py [standalone_app 옵션...]

환경 변수
  CABBAGE_MONITOR_OUT    기록 json 경로 (시뮬레이션 약 1 s 마다 저장). 카터 chassis 궤적(chassis_track: t, x, y, yaw),
                         Pallet_01 궤적, 양배추 포기별 트레이 기준 흔들림
  CABBAGE_CAPTURE_CAMS   "이름=/카메라/prim/경로;이름2=..." 이면 그 카메라를 화면 밖에서 렌더해
                         <기록 폴더>/captures/cap_<이름>_<시뮬레이션 초>.jpg 로 저장 (예: human=/World/Characters/HumanViewCam)
  CABBAGE_CAPTURE_EVERY  캡처 간격(시뮬레이션 초, 기본 3.0)
Windows 창 녹화(gdigrab)로는 Isaac 3D 뷰포트가 갱신되지 않으므로 영상은 이 캡처로 만든다.
기록은 standalone_app 이 sim_task_node 를 import 할 때(SimulationApp 이 뜬 뒤) 물리 스텝 콜백으로 설치된다.
"""
import builtins
import json
import math
import os
import runpy
import sys

OUT = os.environ.get("CABBAGE_MONITOR_OUT", "cabbage_monitor.json")
CHASSIS = "/World/SmartFarm/Placed/LiftRig/Asset/nova_carter_ROS/chassis_link"
_orig_import = builtins.__import__
_state = {"installed": False, "sub": None}


def _install():
    import numpy as np
    import omni.physx
    import omni.usd
    from pxr import UsdPhysics

    phys = omni.physx.get_physx_interface()
    S = {"n": 0, "t": 0.0, "heads": None, "ref": {}, "stats": {}, "track": [], "trays": {}}

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

    def capture(spec):
        import omni.replicator.core as rep

        if "annot" not in S:          # 첫 호출: 카메라마다 render product + rgb annotator
            S["annot"] = {}
            for item in spec.split(";"):
                name, cam_path = item.split("=")
                res = (640, 640) if "realsense" in cam_path.lower() else (960, 540)
                rp = rep.create.render_product(cam_path, res)
                a = rep.AnnotatorRegistry.get_annotator("rgb")
                a.attach([rp])
                S["annot"][name] = a
            os.makedirs(os.path.join(os.path.dirname(OUT), "captures"), exist_ok=True)
            return
        from PIL import Image

        for name, a in S["annot"].items():
            data = a.get_data()
            if data is None or getattr(data, "size", 0) == 0:
                continue
            base = os.path.join(os.path.dirname(OUT), "captures", "cap_%s_%06.1f" % (name, S["t"]))
            Image.fromarray(np.asarray(data)[..., :3]).save(base + ".jpg", quality=88)

    def on_step(dt):
        S["n"] += 1
        S["t"] += dt
        cams = os.environ.get("CABBAGE_CAPTURE_CAMS", "")
        every = max(1, int(round(float(os.environ.get("CABBAGE_CAPTURE_EVERY", "3.0")) * 60)))
        if cams and S["n"] % every == 0:
            try:
                capture(cams)
            except Exception as error:  # noqa: BLE001
                if not S.get("cap_err"):
                    print(f"[MONITOR] capture failed: {error}", flush=True)
                    S["cap_err"] = True
        if S["n"] % 15:
            return
        stage = omni.usd.get_context().get_stage()
        if S["heads"] is None:
            S["heads"] = [(str(p.GetPath()), str(p.GetPath()).split("/root_001/")[0] + "/Cube_011_001")
                          for p in stage.Traverse()
                          if p.GetName().startswith("Cabbage_") and p.HasAPI(UsdPhysics.RigidBodyAPI)]
            print(f"[MONITOR] tracking {len(S['heads'])} cabbage heads", flush=True)
        for h, tray in S["heads"]:
            ph, pt = pose(h), pose(tray)
            if ph is None or pt is None:
                continue
            rel = pt[1].T @ (ph[0] - pt[0])
            up = pt[1].T @ ph[1][:, 2]
            r0, u0 = S["ref"].setdefault(h, (rel, up))
            d = float(np.linalg.norm(rel - r0)) * 1000
            tilt = math.degrees(math.acos(max(-1.0, min(1.0, float(np.dot(up, u0))))))
            st = S["stats"].setdefault(h.replace("/World/SmartFarm/Placed/", ""),
                                       {"max_rel_disp_mm": 0.0, "max_rel_tilt_deg": 0.0, "t_max": 0.0})
            if d > st["max_rel_disp_mm"]:
                st["max_rel_disp_mm"], st["t_max"] = round(d, 2), round(S["t"], 2)
            st["max_rel_tilt_deg"] = round(max(st["max_rel_tilt_deg"], tilt), 2)
        if S["n"] % 60 == 0:
            c = pose(CHASSIS)
            if c is not None:
                yaw = math.degrees(math.atan2(c[1][1, 0], c[1][0, 0]))
                S["track"].append([round(S["t"], 2), round(float(c[0][0]), 4), round(float(c[0][1]), 4), round(yaw, 2)])
            for _, tray in (S["heads"] or []):
                pt = pose(tray)
                if pt is not None:
                    key = tray.replace("/World/SmartFarm/Placed/", "")
                    S["trays"][key] = [round(float(v), 4) for v in pt[0]] + [round(math.degrees(math.atan2(pt[1][1, 0], pt[1][0, 0])), 2)]
                    if key.startswith("Pallet_01"):
                        S.setdefault("tray_track", []).append([round(S["t"], 1)] + S["trays"][key])
            dump()

    _state["sub"] = phys.subscribe_physics_step_events(on_step)
    print(f"[MONITOR] installed -> {OUT}", flush=True)


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
sys.path.insert(0, os.path.dirname(target))     # `python standalone_app.py` 와 같은 import 경로
runpy.run_path(target, run_name="__main__")
