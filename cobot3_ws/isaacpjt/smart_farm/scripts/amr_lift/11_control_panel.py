"""
카터 주행 + 리프트 승강 + M0609 6축을 **사람이 직접 조작**하는 패널.

Isaac Sim 5.1 GUI 에서 고쳐진 파일을 연 뒤
Window > Script Editor 에 이 파일 내용을 붙여넣고 Ctrl+Enter → 창이 뜬다.
Play 를 누른 상태에서 조작한다. (정지 상태에서 값을 바꿔 두면 Play 할 때 반영된다)

조작
  [카터]  전진/후진/좌회전/우회전/정지, 속도 슬라이더 (rad/s)
  [리프트] 하한·상한(안전 범위 안에서), 속도(m/s), 목표 높이 슬라이더, ▲▼, 정지
  [팔]    joint_1 ~ joint_6 슬라이더 (도), 0 자세로, 드라이브 강화

원리
  전부 **USD 드라이브 목표값**만 바꾼다. 아티큘레이션 래퍼를 쓰지 않으므로
  Play 전/후 아무 때나 동작하고, 다른 스크립트(test.py 등)와도 충돌하지 않는다.
    카터 바퀴 : **아티큘레이션 API** (ArticulationAction 의 joint_velocities).
                USD 의 targetVelocity 속성은 Play 중에 PhysX 로 반영되지 않는다
                (실측: USD 로 2.14 rad/s 를 써도 실제 0.03 rad/s, 액션으로는 2.13).
                그래서 바퀴만 Play 중에 아티큘레이션으로 명령한다.
    리프트    : 위치 드라이브(targetPosition, m) — 속도만큼 목표를 끌어 준다
    팔 6축    : 위치 드라이브(targetPosition, **도 단위**)

스크립트에서 쓰려면
    rig.lift_to(0.2)      rig.lift_speed(0.05)
    rig.arm_deg([0,-30,60,0,30,0])
    rig.drive(0.5, 0.0)   # 전진 0.5 m/s, 회전 0 rad/s
    rig.stop()
"""

import math

import omni.ui as ui
import omni.usd
import omni.timeline
import omni.physx
from pxr import Usd, UsdPhysics


LIFT_JOINT = "lift_prismatic_joint"
ARM_JOINTS = ["joint_%d" % i for i in range(1, 7)]
WHEEL_JOINTS = ["joint_wheel_left", "joint_wheel_right"]

WHEEL_RADIUS = 0.14        # m (Nova Carter 구동륜)
WHEEL_BASE = 0.413         # m (좌우 바퀴 간격)

ARM_STRONG_STIFFNESS = 1.0e8   # 'Stiffen arm' 버튼이 넣는 값 (test.py 와 동일)
ARM_STRONG_DAMPING = 1.0e4

# Isaac 기본 UI 폰트에는 한글 글리프가 없어서 한글 라벨이 "??" 로 깨진다.
# 그래서 창 안의 글자는 영문으로 둔다.
# 한글로 보고 싶으면 한글 TTF 경로를 넣으면 된다. 예)
#   sudo apt install fonts-nanum
#   UI_FONT = "/usr/share/fonts/truetype/nanum/NanumGothic.ttf"
# (.ttc 묶음 폰트(Noto Sans CJK)는 omni.ui 가 못 읽는 경우가 많아 .ttf 를 쓸 것)
UI_FONT = None
UI_FONT_SIZE = 14


def _find(stage, name, type_name=None):
    for prim in stage.Traverse():
        if prim.GetName() == name and (type_name is None or prim.GetTypeName() == type_name):
            return prim
    return None


class Rig:
    def __init__(self):
        self.stage = omni.usd.get_context().get_stage()
        joint = _find(self.stage, LIFT_JOINT, "PhysicsPrismaticJoint")
        if joint is None:
            raise RuntimeError(
                "%s not found. Open the file that contains the lift." % LIFT_JOINT)
        self.lift_joint = joint
        self.pj = UsdPhysics.PrismaticJoint(joint)
        self.lift_drive = UsdPhysics.DriveAPI.Get(joint, "linear") or \
            UsdPhysics.DriveAPI.Apply(joint, "linear")

        self.safe_lower = self._attr(joint, "lift:safeLower",
                                     self.pj.GetLowerLimitAttr().Get() or 0.0)
        self.safe_upper = self._attr(joint, "lift:safeUpper",
                                     self.pj.GetUpperLimitAttr().Get() or 0.0)
        self.lower = max(self.safe_lower, float(self.pj.GetLowerLimitAttr().Get() or 0.0))
        self.upper = min(self.safe_upper, float(self.pj.GetUpperLimitAttr().Get() or 0.0))
        self.speed = self._attr(joint, "lift:defaultSpeed", 0.08)

        self.arm_drives = {}
        for name in ARM_JOINTS:
            p = _find(self.stage, name, "PhysicsRevoluteJoint")
            if p is not None:
                self.arm_drives[name] = (p, UsdPhysics.DriveAPI.Get(p, "angular"))
        self.wheel_drives = []
        for name in WHEEL_JOINTS:
            p = _find(self.stage, name, "PhysicsRevoluteJoint")
            if p is not None:
                self.wheel_drives.append((name, UsdPhysics.DriveAPI.Get(p, "angular")))

        self._arti = None
        self._wheel_idx = None
        self._wheel_cmd = (0.0, 0.0)      # (left, right) rad/s
        self.command = float(self.lift_drive.GetTargetPositionAttr().Get() or 0.0)
        self.command = min(max(self.command, self.lower), self.upper)
        self.goal = self.command
        self._moving = False
        self._write(self.command, 0.0)

        self.timeline = omni.timeline.get_timeline_interface()
        self._sub = omni.physx.get_physx_interface().subscribe_physics_step_events(self._on_step)
        self._tl_sub = self.timeline.get_timeline_event_stream().create_subscription_to_pop(
            self._on_timeline)

    # ── 내부 ──────────────────────────────────────────────
    @staticmethod
    def _attr(prim, name, fallback):
        a = prim.GetAttribute(name)
        return float(a.Get()) if a and a.Get() is not None else float(fallback)

    def _write(self, pos, vel):
        self.lift_drive.GetTargetPositionAttr().Set(float(pos))
        self.lift_drive.GetTargetVelocityAttr().Set(float(vel))

    def _ensure_articulation(self):
        """Play 중에만 아티큘레이션을 잡을 수 있다 (물리 컨텍스트가 있어야 바인딩된다)."""
        if self._arti is not None:
            return self._arti
        if not self.timeline.is_playing():
            return None
        try:
            from isaacsim.core.prims import SingleArticulation

            root = None
            for prim in self.stage.Traverse():
                if prim.HasAPI(UsdPhysics.ArticulationRootAPI):
                    root = prim
                    break
            if root is None:
                return None
            a = SingleArticulation(prim_path=str(root.GetPath()), name="panel_rig")
            a.initialize()
            names = list(a.dof_names)
            self._wheel_idx = [names.index(n) for n in WHEEL_JOINTS if n in names]
            self._arti = a
            return a
        except Exception:
            return None

    def _apply_wheels(self):
        a = self._ensure_articulation()
        if a is None or not self._wheel_idx:
            return
        import numpy as np
        from isaacsim.core.utils.types import ArticulationAction

        a.apply_action(ArticulationAction(
            joint_velocities=np.array(self._wheel_cmd[:len(self._wheel_idx)]),
            joint_indices=np.array(self._wheel_idx),
        ))

    def _on_step(self, dt):
        if any(abs(v) > 1e-9 for v in self._wheel_cmd) or self._arti is not None:
            self._apply_wheels()
        delta = self.goal - self.command
        if abs(delta) < 1e-6:
            if self._moving:
                self._moving = False
                self._write(self.command, 0.0)
            return
        move = min(abs(delta), self.speed * dt)
        self.command += move if delta > 0 else -move
        self._moving = True
        self._write(self.command, self.speed if delta > 0 else -self.speed)

    def _on_timeline(self, event):
        if not self.timeline.is_playing():
            # Stop/Pause 하면 아티큘레이션 핸들이 무효가 된다. 다음 Play 때 다시 잡는다.
            self._arti = None
            self._wheel_idx = None
            self._wheel_cmd = (0.0, 0.0)
            self._moving = False
            self.goal = self.command
            self._write(self.command, 0.0)

    # ── 리프트 ────────────────────────────────────────────
    def lift_to(self, height):
        self.goal = min(max(float(height), self.lower), self.upper)
        if not self.timeline.is_playing():
            self.command = self.goal
            self._write(self.command, 0.0)
        return self.goal

    def lift_speed(self, v):
        self.speed = max(0.001, float(v))
        return self.speed

    def lift_range(self, lower, upper):
        lower = max(self.safe_lower, float(lower))
        upper = min(self.safe_upper, float(upper))
        if upper < lower:
            upper = lower
        self.lower, self.upper = lower, upper
        self.pj.GetLowerLimitAttr().Set(lower)
        self.pj.GetUpperLimitAttr().Set(upper)
        self.lift_to(self.goal)
        return lower, upper

    def lift_stop(self):
        self.goal = self.command
        self._moving = False
        self._write(self.command, 0.0)
        return self.command

    # ── 팔 ────────────────────────────────────────────────
    def arm_deg(self, degrees):
        """joint_1~6 목표 각도(도). USD 각도 드라이브는 도 단위다."""
        for name, deg in zip(ARM_JOINTS, degrees):
            item = self.arm_drives.get(name)
            if item and item[1]:
                item[1].GetTargetPositionAttr().Set(float(deg))

    def arm_joint_deg(self, name, deg):
        item = self.arm_drives.get(name)
        if item and item[1]:
            item[1].GetTargetPositionAttr().Set(float(deg))

    def arm_strengthen(self, stiffness=ARM_STRONG_STIFFNESS, damping=ARM_STRONG_DAMPING):
        n = 0
        for name, (prim, drive) in self.arm_drives.items():
            if drive:
                drive.GetStiffnessAttr().Set(float(stiffness))
                drive.GetDampingAttr().Set(float(damping))
                n += 1
        return n

    # ── 카터 주행 ─────────────────────────────────────────
    def drive(self, linear_mps, angular_rps):
        """전진 속도(m/s)와 회전 속도(rad/s) → 좌우 바퀴 각속도(rad/s)."""
        left = (linear_mps - angular_rps * WHEEL_BASE / 2.0) / WHEEL_RADIUS
        right = (linear_mps + angular_rps * WHEEL_BASE / 2.0) / WHEEL_RADIUS
        self._wheel_cmd = (left, right)
        # Play 전에 눌러 둘 수도 있으므로 USD 쪽에도 같이 적어 둔다(초기값 용도).
        for (name, drive), val in zip(self.wheel_drives, (left, right)):
            if drive:
                drive.GetTargetVelocityAttr().Set(float(val))
        self._apply_wheels()
        return left, right

    def stop(self):
        self.drive(0.0, 0.0)
        return self.lift_stop()

    def destroy(self):
        try:
            self.stop()
        except Exception:
            pass
        self._sub = None
        self._tl_sub = None


class Panel:
    def __init__(self, rig):
        self.r = rig
        self.step = 0.05
        self.window = ui.Window("AMR + Lift + M0609 control", width=470, height=640)
        self.style = {}
        if UI_FONT:
            self.style = {"": {"font": UI_FONT, "font_size": UI_FONT_SIZE}}
        self._build()

    def _build(self):
        r = self.r
        with self.window.frame:
            with ui.ScrollingFrame(style=self.style):
                with ui.VStack(spacing=6, height=0):
                    ui.Label("-- Carter drive --")
                    with ui.HStack(spacing=6, height=24):
                        ui.Label("speed m/s", width=90)
                        self.v_model = ui.SimpleFloatModel(0.3)
                        ui.FloatSlider(self.v_model, min=0.05, max=1.5, step=0.05, format="%.2f")
                    with ui.HStack(spacing=6, height=24):
                        ui.Label("turn rad/s", width=90)
                        self.w_model = ui.SimpleFloatModel(0.6)
                        ui.FloatSlider(self.w_model, min=0.1, max=2.0, step=0.1, format="%.1f")
                    with ui.HStack(spacing=6, height=28):
                        ui.Button("Forward", clicked_fn=lambda: self._drive(1, 0))
                        ui.Button("Back", clicked_fn=lambda: self._drive(-1, 0))
                        ui.Button("Left", clicked_fn=lambda: self._drive(0, 1))
                        ui.Button("Right", clicked_fn=lambda: self._drive(0, -1))
                        ui.Button("STOP", clicked_fn=lambda: self._drive(0, 0))

                    ui.Spacer(height=8)
                    ui.Label("-- Lift --")
                    ui.Label("safe range  %.4f ~ %.4f m  (before mast contact)"
                             % (r.safe_lower, r.safe_upper))
                    with ui.HStack(spacing=6, height=24):
                        ui.Label("min / max", width=90)
                        self.lo_model = ui.SimpleFloatModel(r.lower)
                        self.hi_model = ui.SimpleFloatModel(r.upper)
                        ui.FloatField(self.lo_model)
                        ui.FloatField(self.hi_model)
                        ui.Button("Apply", width=60, clicked_fn=self._apply_range)
                    with ui.HStack(spacing=6, height=24):
                        ui.Label("speed m/s", width=90)
                        self.sp_model = ui.SimpleFloatModel(r.speed)
                        self.sp_model.add_value_changed_fn(
                            lambda m: r.lift_speed(m.get_value_as_float()))
                        ui.FloatSlider(self.sp_model, min=0.005, max=0.5, step=0.005, format="%.3f")
                    with ui.HStack(spacing=6, height=26):
                        ui.Label("height m", width=90)
                        self.pos_model = ui.SimpleFloatModel(r.goal)
                        self.pos_model.add_value_changed_fn(self._on_lift)
                        self.slider = ui.FloatSlider(self.pos_model, min=r.lower, max=r.upper,
                                                     step=0.001, format="%.3f")
                    with ui.HStack(spacing=6, height=28):
                        ui.Button("Down", clicked_fn=self._down)
                        ui.Button("Up", clicked_fn=self._up)
                        ui.Button("Bottom", clicked_fn=lambda: self._set_lift(r.lower))
                        ui.Button("Top", clicked_fn=lambda: self._set_lift(r.upper))
                        ui.Button("Hold", clicked_fn=self._lift_stop)

                    ui.Spacer(height=8)
                    ui.Label("-- M0609 arm (deg) --")
                    self.arm_models = {}
                    for name in ARM_JOINTS:
                        if name not in r.arm_drives:
                            continue
                        with ui.HStack(spacing=6, height=22):
                            ui.Label(name, width=90)
                            m = ui.SimpleFloatModel(0.0)
                            self.arm_models[name] = m
                            m.add_value_changed_fn(
                                lambda mm, n=name: r.arm_joint_deg(n, mm.get_value_as_float()))
                            lo, hi = (-150.0, 150.0) if name == "joint_3" else (-180.0, 180.0)
                            ui.FloatSlider(m, min=lo, max=hi, step=1.0, format="%.0f")
                    with ui.HStack(spacing=6, height=28):
                        ui.Button("Arm home (0 deg)", clicked_fn=self._arm_zero)
                        ui.Button("Stiffen arm drives", clicked_fn=self._arm_strong)

                    ui.Spacer(height=8)
                    self.status = ui.Label("Ready. Press Play, then use the buttons.")

    # ── 콜백 ──────────────────────────────────────────────
    def _drive(self, fwd, turn):
        v = self.v_model.get_value_as_float() * fwd
        w = self.w_model.get_value_as_float() * turn
        left, right = self.r.drive(v, w)
        self.status.text = "drive v=%.2f m/s  w=%.2f rad/s  (wheels %.2f / %.2f rad/s)" % (
            v, w, left, right)

    def _apply_range(self):
        lo, hi = self.r.lift_range(self.lo_model.get_value_as_float(),
                                   self.hi_model.get_value_as_float())
        self.lo_model.set_value(lo)
        self.hi_model.set_value(hi)
        self.slider.min, self.slider.max = lo, hi
        self.pos_model.set_value(self.r.goal)
        self.status.text = "lift range %.3f ~ %.3f m" % (lo, hi)

    def _on_lift(self, model):
        g = self.r.lift_to(model.get_value_as_float())
        self.status.text = "lift target %.3f m  (speed %.3f m/s)" % (g, self.r.speed)

    def _set_lift(self, v):
        self.pos_model.set_value(v)

    def _up(self):
        self._set_lift(min(self.r.upper, self.pos_model.get_value_as_float() + self.step))

    def _down(self):
        self._set_lift(max(self.r.lower, self.pos_model.get_value_as_float() - self.step))

    def _lift_stop(self):
        here = self.r.lift_stop()
        self.pos_model.set_value(here)
        self.status.text = "lift hold at %.3f m" % here

    def _arm_zero(self):
        for m in self.arm_models.values():
            m.set_value(0.0)
        self.status.text = "arm -> 0 deg"

    def _arm_strong(self):
        n = self.r.arm_strengthen()
        self.status.text = "stiffened %d arm drives (k=%g, c=%g)" % (
            n, ARM_STRONG_STIFFNESS, ARM_STRONG_DAMPING)


try:
    rig.destroy()      # noqa: F821  (다시 실행할 때 이전 것 정리)
except Exception:
    pass

rig = Rig()
panel = Panel(rig)
print("control panel ready: lift %.4f ~ %.4f m, arm %d joints, wheels %d"
      % (rig.lower, rig.upper, len(rig.arm_drives), len(rig.wheel_drives)))
