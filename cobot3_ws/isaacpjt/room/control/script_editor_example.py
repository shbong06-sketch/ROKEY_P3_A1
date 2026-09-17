# =====================================================================================
# Isaac Sim GUI: Window > Script Editor 에 이 파일 내용을 붙여넣고 Run.
#   - mobile_manipulator_demo.usda / m0617_fixed_base_demo.usda 를 연 상태,
#     또는 내 씬에 mobile_manipulator.usda / m0617_fixed_base.usda 를 드래그한 상태에서 사용.
#   - 아래 PARAMETERS 만 수정하면 됨. Play가 꺼져 있으면 자동으로 Play 함.
#   - 다시 실행하면 이전 시퀀스는 자동 해제됨. Stop 누르면 제어 종료.
# =====================================================================================

# ----------------------------- PARAMETERS -----------------------------
ROBOT_ROOT = None      # 예: "/World/mobile_manipulator". None이면 스테이지에서 자동 탐색
PACKAGE_DIR = None     # 예: "/home/rokey/Downloads/robot_sample_v3". None이면 자동 탐색
ARM_SPEED_DEG = 45.0   # 팔 관절 이동 속도 [deg/s] (M0617 공식 한계 100/100/150/225/225/225로 추가 제한)
SEQUENCE = [
    # ("gripper", 각도deg)            -30 닫힘 ~ +30 열림
    # ("arm", [j1,j2,j3,j4,j5,j6])    관절각 deg, joint_1..joint_6 순서
    # ("turn", 각도deg)                폐루프 제자리 회전 (+ = 왼쪽)        [모바일만]
    # ("drive", 거리m)                 폐루프 직진 (음수 = 후진)            [모바일만]
    # ("velocity", v, w, 초)           개루프 속도 명령 v[m/s], w[rad/s]  [모바일만]
    # ("wait", 초)
    ("gripper", 30),
    ("turn", 90),
    ("drive", 1.0),
    ("arm", [90, 20, 70, 0, 90, 0]),
    ("gripper", -30),
    ("arm", [0, 0, 0, 0, 0, 0]),
]
# ----------------------------------------------------------------------

import asyncio, builtins, importlib, math, os, sys
import omni.kit.app, omni.timeline, omni.usd, omni.physx

_stage = omni.usd.get_context().get_stage()


def _find_package_dir():
    for layer in _stage.GetUsedLayers():
        path = layer.realPath or ""
        if path.endswith(("mobile_manipulator.usda", "m0617_fixed_base.usda")):
            # v3.1부터 이 두 usda가 <package>/scenes/ 밑에 있으므로 한 단계 위가 패키지 루트
            return os.path.dirname(os.path.dirname(path))
    return None


def _find_root():
    for prim in _stage.Traverse():
        p = prim.GetPath().pathString
        if _stage.GetPrimAtPath(p + "/m0617").IsValid() and _stage.GetPrimAtPath(p + "/rg6").IsValid():
            return p
    return None


pkg = PACKAGE_DIR or _find_package_dir()
root = ROBOT_ROOT or _find_root()
if not pkg or not root:
    raise RuntimeError(f"robot not found (package_dir={pkg}, root={root}). Set PACKAGE_DIR / ROBOT_ROOT.")
has_base = _stage.GetPrimAtPath(root + "/mir100").IsValid()
sys.path.insert(0, os.path.join(pkg, "control"))
import mm_control
importlib.reload(mm_control)

# release a previous run
_old = getattr(builtins, "_MM_CONTROL_RUN", None)
if _old:
    for s in _old.values():
        try:
            s.unsubscribe() if hasattr(s, "unsubscribe") else None
        except Exception:
            pass
builtins._MM_CONTROL_RUN = {}
print(f"[mm] root={root} mobile_base={has_base} package={pkg}")


async def _main():
    tl = omni.timeline.get_timeline_interface()
    if not tl.is_playing():
        tl.play()
    app = omni.kit.app.get_app()
    for _ in range(10):
        await app.next_update_async()
    ctl = mm_control.MobileManipulatorControl(root, has_base=has_base, arm_speed_deg=ARM_SPEED_DEG)
    ctl.initialize()
    state = {"i": -1, "t": 0.0, "start": None, "done": False}

    def start_item(item):
        kind = item[0]
        state["t"] = 0.0
        state["start"] = ctl.base_pose()
        if kind == "gripper":
            ctl.set_gripper_deg(item[1])
        elif kind == "arm":
            ctl.set_arm_joints_deg(item[1])
        elif kind == "turn":
            ctl.turn_by_deg(item[1])
        elif kind == "drive":
            ctl.drive_distance(item[1])
        elif kind == "velocity":
            ctl.set_base_velocity(item[1], item[2])
        print(f"[mm] start {item}")

    def item_done(item):
        kind = item[0]
        if kind == "gripper":
            return state["t"] > 1.5
        if kind == "arm":
            return ctl.arm_done()
        if kind in ("turn", "drive"):
            return ctl.base_done() and state["t"] > 0.3
        if kind == "velocity":
            if state["t"] >= item[3]:
                ctl.stop_base()
                return True
            return False
        if kind == "wait":
            return state["t"] >= item[1]
        return True

    def report(item):
        x0, y0, a0 = state["start"]; x1, y1, a1 = ctl.base_pose()
        if item[0] == "gripper":
            print(f"[mm]   gripper -> {ctl.gripper_deg():.1f} deg")
        elif item[0] == "arm":
            print(f"[mm]   arm -> {[round(v, 1) for v in ctl.arm_joints_deg()]}")
        elif item[0] in ("turn", "drive", "velocity"):
            h = math.radians(a0)
            print(f"[mm]   base: yaw {(a1 - a0 + 180) % 360 - 180:+.1f} deg, along-heading {(x1-x0)*math.cos(h)+(y1-y0)*math.sin(h):+.3f} m")

    def on_step(dt):
        if state["done"]:
            return
        if not tl.is_playing():
            return
        if state["i"] < 0 or item_done(SEQUENCE[state["i"]]):
            if state["i"] >= 0:
                report(SEQUENCE[state["i"]])
            state["i"] += 1
            if state["i"] >= len(SEQUENCE):
                state["done"] = True
                print("[mm] sequence finished (arm/gripper keep holding their last targets)")
            else:
                start_item(SEQUENCE[state["i"]])
        state["t"] += dt
        ctl.step(dt)

    builtins._MM_CONTROL_RUN["step"] = omni.physx.get_physx_interface().subscribe_physics_step_events(on_step)

    def on_timeline(e):
        if e.type == int(omni.timeline.TimelineEventType.STOP):
            for s in builtins._MM_CONTROL_RUN.values():
                try:
                    s.unsubscribe()
                except Exception:
                    pass
            builtins._MM_CONTROL_RUN = {}
            print("[mm] timeline stopped -> control released")

    builtins._MM_CONTROL_RUN["timeline"] = tl.get_timeline_event_stream().create_subscription_to_pop(on_timeline)


asyncio.ensure_future(_main())
