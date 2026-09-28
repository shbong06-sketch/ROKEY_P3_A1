"""runtime/standalone_app.py 를 그대로 실행하면서 기록만 덧붙인다 (Isaac 파이썬으로 실행).

  <Isaac>\\python.bat run_with_monitor.py <smart_farm>\\runtime\\standalone_app.py [standalone_app 옵션...]

환경 변수
  CABBAGE_MONITOR_OUT    기록 json 경로 (시뮬레이션 약 1 s 마다 저장). 카터 chassis 궤적
                         (chassis_track: t_sim, x, y, yaw, roll, pitch, 벽시계),
                         Pallet_01 궤적, 양배추 포기별 트레이 기준 흔들림
  CABBAGE_CAPTURE_CAMS   "이름=/카메라/prim/경로;이름2=..." 이면 그 카메라를 화면 밖에서 렌더해
                         <기록 폴더>/captures/cap_<이름>_<시뮬레이션 초>.jpg 로 저장 (예: human=/World/Characters/HumanViewCam)
  CABBAGE_CAPTURE_EVERY  캡처 간격(시뮬레이션 초, 기본 3.0)
  CABBAGE_HEAD_MASS      양배추 포기 질량(kg) 실험. 비우면 에셋 값(0.3)
  CABBAGE_MIN_MOVE_S     팀 robot_motion 의 구간 최소 보간 시간(s, 팀 값 0.5) 실험. 팀 파일은 그대로, 실행 중 값만 바꿈
Windows 창 녹화(gdigrab)로는 Isaac 3D 뷰포트가 갱신되지 않으므로 영상은 이 캡처로 만든다.
기록은 standalone_app 이 sim_task_node 를 import 할 때(SimulationApp 이 뜬 뒤) 물리 스텝 콜백으로 설치된다.
"""
import builtins
import json
import math
import os
import runpy
import sys
import time

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

    def set_head_mass(stage, when):
        """CABBAGE_HEAD_MASS (kg): 양배추 포기 질량 실험. 에셋 0.3 kg 대신 실물(미니 0.45~0.9, 보통 1.5~4 kg)에 가깝게."""
        m = os.environ.get("CABBAGE_HEAD_MASS")
        if not m or S.get("mass_set"):
            return
        heads = [p for p in stage.Traverse() if p.GetName().startswith("Cabbage_") and p.HasAPI(UsdPhysics.RigidBodyAPI)]
        for p in heads:
            UsdPhysics.MassAPI.Apply(p).CreateMassAttr().Set(float(m))
        if heads:
            S["mass_set"] = True
            print(f"[MONITOR] head mass {m} kg on {len(heads)} heads ({when})", flush=True)

    try:
        set_head_mass(omni.usd.get_context().get_stage(), "install")
    except Exception as error:  # noqa: BLE001
        print(f"[MONITOR] head mass at install failed: {error}", flush=True)

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
               "pallet01_track": S.get("tray_track", [])[-2000:],
               # 1 s 마다 [t_sim, Pallet_01 포기 중 트레이 기준 최대 변위 mm, 최대 기울기 deg (그 1 s 안의 최대),
               #            트레이의 chassis 기준 위치 x y z] - 운반 구간(주행·도킹) 흔들림을 선별 구간과 나눠 보기 위함
               "pallet01_heads_series": S.get("p1_series", [])[-2000:],
               "pallet01_tray_in_chassis_0p25s": S.get("tray_fine", [])[-4000:],     # [t_sim, x, y, z]
               "tilt_max_deg": S.get("tilt_max", 0.0), "tilt_max_t": S.get("tilt_max_t"),
               "tilt_samples_over_0p5deg": S.get("tilt_samples", [])[-4000:]}
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
        set_head_mass(stage, "t=%.1f s" % S["t"])
        c = pose(CHASSIS)
        if c is not None:        # 차체 기울기(롤·피치 중 큰 값) 최대치: 0.25 s 마다 (궤적은 1 s 마다라 짧은 흔들림을 놓친다)
            tilt = max(abs(math.degrees(math.atan2(c[1][2, 1], c[1][2, 2]))),
                       abs(math.degrees(math.asin(max(-1.0, min(1.0, -c[1][2, 0]))))))
            if tilt > 0.5:
                S.setdefault("tilt_samples", []).append([round(S["t"], 2), round(tilt, 3)])
            if tilt > S.get("tilt_max", 0.0):
                S["tilt_max"], S["tilt_max_t"] = round(tilt, 3), round(S["t"], 2)
            p1 = next((pose(tr) for hh, tr in (S["heads"] or []) if "/Pallet_01/" in hh), None)
            if p1 is not None:           # 0.25 s: 트레이의 chassis 기준 위치 (내려놓기 중 포크 위 미끄러짐 추적)
                rel = c[1].T @ (p1[0] - c[0])
                S.setdefault("tray_fine", []).append([round(S["t"], 2)] + [round(float(v), 4) for v in rel])
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
            if h.split("/World/SmartFarm/Placed/")[-1].startswith("Pallet_01"):
                S["win_d"], S["win_t"] = max(S.get("win_d", 0.0), d), max(S.get("win_t", 0.0), tilt)
        if S["n"] % 60 == 0:
            c = pose(CHASSIS)
            p1 = next((pose(tr) for hh, tr in (S["heads"] or []) if "/Pallet_01/" in hh), None)
            if c is not None and p1 is not None:
                rel = c[1].T @ (p1[0] - c[0])
                S.setdefault("p1_series", []).append([round(S["t"], 2), round(S.get("win_d", 0.0), 2), round(S.get("win_t", 0.0), 2)]
                                                     + [round(float(v), 4) for v in rel])
            S["win_d"] = S["win_t"] = 0.0
            if c is not None:
                R = c[1]
                yaw = math.degrees(math.atan2(R[1, 0], R[0, 0]))
                pitch = math.degrees(math.asin(max(-1.0, min(1.0, -R[2, 0]))))
                roll = math.degrees(math.atan2(R[2, 1], R[2, 2]))
                # t_sim, x, y, yaw, roll, pitch, 벽시계(초) — 벽시계는 flow.log 등의 명령 시각을 시뮬레이션 시간으로 바꿀 때 쓴다
                S["track"].append([round(S["t"], 2), round(float(c[0][0]), 4), round(float(c[0][1]), 4), round(yaw, 2),
                                   round(roll, 2), round(pitch, 2), round(time.time(), 2)])
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
    if name == "robot_motion" and os.environ.get("CABBAGE_MIN_MOVE_S") and not _state.get("min_move"):
        # 팀 robot_motion.py 는 고치지 않고 실행 중 값만 바꾼다 (구간 최소 보간 시간, 팀 값 0.5 s)
        _state["min_move"] = True
        module.MIN_MOVE_SECONDS = float(os.environ["CABBAGE_MIN_MOVE_S"])
        print(f"[MONITOR] robot_motion.MIN_MOVE_SECONDS -> {module.MIN_MOVE_SECONDS} s", flush=True)
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
