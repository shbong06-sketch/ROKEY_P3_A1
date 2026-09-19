"""
리프트 조작 패널 (이동 범위 + 속도까지 사용자가 정함).

이 파일 하나만 있으면 됩니다. 다른 PC 에서도 Isaac Sim 5.1 의
Window > Script Editor 에 붙여넣고 Ctrl+Enter 하면 창이 뜹니다.
우리 저장소나 다른 스크립트를 import 하지 않습니다.

패널에서 정하는 것
  하한 / 상한 (m)  : 실제 조인트 리미트에 그대로 들어갑니다.
                     안전 범위(= 캐리지가 마스트 폴리곤에 닿기 직전)를 넘겨
                     입력하면 자동으로 잘립니다.
  속도 (m/s)       : 목표 높이를 이 속도로 끌고 갑니다. 물리 엔진이 그 목표를
                     따라가므로, 위에 얹힌 무게·마찰이 그대로 반영됩니다.
  스텝 (m)         : ▲ ▼ 버튼 한 번에 움직일 거리.

안전 범위는 리그를 만들 때 계산해서 USD 안에 넣어 둡니다
  lift:safeLower / lift:safeUpper / lift:defaultSpeed
없으면 현재 조인트 리미트를 안전 범위로 씁니다.

스크립트에서 쓰려면
    lift.set_goal(0.20)      # 0.20 m 로 (설정한 속도로) 이동
    lift.set_speed(0.05)
    lift.set_range(0.0, 0.25)
    lift.stop()              # 지금 위치에서 멈춤
"""

import omni.ui as ui
import omni.usd
import omni.timeline
import omni.physx
from pxr import UsdPhysics


JOINT_NAME = "lift_prismatic_joint"


def _find_joint():
    stage = omni.usd.get_context().get_stage()
    for prim in stage.Traverse():
        if prim.GetName() == JOINT_NAME and prim.GetTypeName() == "PhysicsPrismaticJoint":
            return prim
    raise RuntimeError(
        "%s 를 못 찾았습니다. 리프트가 들어 있는 장면을 먼저 여세요." % JOINT_NAME
    )


class LiftController:
    def __init__(self):
        self.joint = _find_joint()
        self.pj = UsdPhysics.PrismaticJoint(self.joint)
        self.drive = UsdPhysics.DriveAPI.Get(self.joint, "linear")
        if not self.drive:
            self.drive = UsdPhysics.DriveAPI.Apply(self.joint, "linear")

        lo = self.pj.GetLowerLimitAttr().Get()
        hi = self.pj.GetUpperLimitAttr().Get()
        self.safe_lower = self._attr("lift:safeLower", lo if lo is not None else 0.0)
        self.safe_upper = self._attr("lift:safeUpper", hi if hi is not None else 0.0)

        self.lower = max(self.safe_lower, lo if lo is not None else self.safe_lower)
        self.upper = min(self.safe_upper, hi if hi is not None else self.safe_upper)
        self.speed = self._attr("lift:defaultSpeed", 0.08)

        self.command = float(self.drive.GetTargetPositionAttr().Get() or 0.0)
        self.command = min(max(self.command, self.lower), self.upper)
        self.goal = self.command
        self._write(self.command)

        self._moving = False
        self.timeline = omni.timeline.get_timeline_interface()
        self._sub = omni.physx.get_physx_interface().subscribe_physics_step_events(
            self._on_step
        )
        self._timeline_sub = (
            self.timeline.get_timeline_event_stream().create_subscription_to_pop(
                self._on_timeline
            )
        )

    # ── 내부 ──────────────────────────────────────────────
    def _attr(self, name, fallback):
        a = self.joint.GetAttribute(name)
        if a and a.Get() is not None:
            return float(a.Get())
        return float(fallback)

    def _write(self, value, velocity=0.0):
        self.drive.GetTargetPositionAttr().Set(float(value))
        # 이동 중에는 목표 속도를 같이 줍니다(피드포워드). 이게 없으면 등속 구간에서
        # 8 mm 쯤 뒤처집니다. 멈출 때는 반드시 0 으로 되돌립니다.
        self.drive.GetTargetVelocityAttr().Set(float(velocity))

    def _on_step(self, dt):
        """물리 한 스텝마다 목표를 speed 만큼만 끌고 갑니다 (= 속도 제어)."""
        delta = self.goal - self.command
        if abs(delta) < 1e-6:
            if self._moving:
                self._moving = False
                self._write(self.command, 0.0)
            return
        step = self.speed * dt
        self.command += step if delta > 0 else -step
        if (delta > 0 and self.command > self.goal) or (delta < 0 and self.command < self.goal):
            self.command = self.goal
        self._moving = True
        self._write(self.command, self.speed if delta > 0 else -self.speed)

    def _on_timeline(self, event):
        """Pause / Stop 순간에 목표 속도를 0 으로 되돌립니다 (재생 재개 시 밀림 방지)."""
        if not self.timeline.is_playing():
            self._moving = False
            self.goal = self.command
            self._write(self.command, 0.0)

    # ── 바깥에서 쓰는 것 ──────────────────────────────────
    def set_range(self, lower, upper):
        lower = max(self.safe_lower, float(lower))
        upper = min(self.safe_upper, float(upper))
        if upper < lower:
            upper = lower
        self.lower, self.upper = lower, upper
        self.pj.GetLowerLimitAttr().Set(lower)
        self.pj.GetUpperLimitAttr().Set(upper)
        self.set_goal(self.goal)
        return lower, upper

    def set_speed(self, speed):
        self.speed = max(0.001, float(speed))
        return self.speed

    def set_goal(self, height):
        self.goal = min(max(float(height), self.lower), self.upper)
        if not self.timeline.is_playing():
            # 정지 상태에서는 램프가 돌지 않으므로 바로 씁니다.
            self.command = self.goal
            self._write(self.command)
        return self.goal

    def stop(self):
        self.goal = self.command
        self._moving = False
        self._write(self.command, 0.0)
        return self.command

    def actual(self):
        """실제 높이(m). Play 중에만 읽힙니다. 못 읽으면 None."""
        try:
            from isaacsim.core.prims import SingleArticulation

            stage = omni.usd.get_context().get_stage()
            root = None
            for prim in stage.Traverse():
                if prim.HasAPI(UsdPhysics.ArticulationRootAPI):
                    root = prim
                    break
            if root is None:
                return None
            arti = SingleArticulation(prim_path=str(root.GetPath()), name="lift_read")
            arti.initialize()
            names = list(arti.dof_names)
            if JOINT_NAME not in names:
                return None
            return float(arti.get_joint_positions()[names.index(JOINT_NAME)])
        except Exception:
            return None

    def destroy(self):
        try:
            self._write(self.command, 0.0)
        except Exception:
            pass
        self._sub = None
        self._timeline_sub = None


class LiftPanel:
    def __init__(self, controller):
        self.c = controller
        self.step = 0.05
        self.window = ui.Window("리프트 조작", width=420, height=300)
        self._build()

    def _build(self):
        c = self.c
        with self.window.frame:
            with ui.VStack(spacing=6, height=0):
                ui.Label("안전 범위 (폴리곤에 닿기 전까지)  %.4f ~ %.4f m"
                         % (c.safe_lower, c.safe_upper))

                with ui.HStack(spacing=6, height=24):
                    ui.Label("하한 / 상한", width=90)
                    self.lo_model = ui.SimpleFloatModel(c.lower)
                    self.hi_model = ui.SimpleFloatModel(c.upper)
                    ui.FloatField(self.lo_model)
                    ui.FloatField(self.hi_model)
                    ui.Button("적용", width=50, clicked_fn=self._apply_range)

                with ui.HStack(spacing=6, height=24):
                    ui.Label("속도 m/s", width=90)
                    self.sp_model = ui.SimpleFloatModel(c.speed)
                    self.sp_model.add_value_changed_fn(
                        lambda m: c.set_speed(m.get_value_as_float())
                    )
                    ui.FloatSlider(self.sp_model, min=0.005, max=0.50, step=0.005,
                                   format="%.3f")

                with ui.HStack(spacing=6, height=24):
                    ui.Label("스텝 m", width=90)
                    self.st_model = ui.SimpleFloatModel(self.step)
                    self.st_model.add_value_changed_fn(self._on_step_changed)
                    ui.FloatSlider(self.st_model, min=0.005, max=0.20, step=0.005,
                                   format="%.3f")

                ui.Spacer(height=4)
                with ui.HStack(spacing=6, height=26):
                    ui.Label("목표 높이", width=90)
                    self.pos_model = ui.SimpleFloatModel(c.goal)
                    self.pos_model.add_value_changed_fn(self._on_slider)
                    self.slider = ui.FloatSlider(self.pos_model, min=c.lower, max=c.upper,
                                                 step=0.001, format="%.3f")

                with ui.HStack(spacing=6, height=28):
                    ui.Button("▼ 내리기", clicked_fn=self._down)
                    ui.Button("▲ 올리기", clicked_fn=self._up)
                    ui.Button("정지", clicked_fn=self._stop)

                with ui.HStack(spacing=6, height=28):
                    ui.Button("맨 아래", clicked_fn=lambda: self._set(c.lower))
                    ui.Button("맨 위", clicked_fn=lambda: self._set(c.upper))
                    ui.Button("실제 높이 읽기", clicked_fn=self._read)

                self.status = ui.Label("목표 %.3f m / 속도 %.3f m/s" % (c.goal, c.speed))

    # ── 콜백 ──────────────────────────────────────────────
    def _apply_range(self):
        lo, hi = self.c.set_range(self.lo_model.get_value_as_float(),
                                  self.hi_model.get_value_as_float())
        self.lo_model.set_value(lo)
        self.hi_model.set_value(hi)
        self.slider.model.set_value(self.c.goal)
        self.slider.min = lo
        self.slider.max = hi
        self.status.text = "가동 범위 %.3f ~ %.3f m 적용" % (lo, hi)

    def _on_step_changed(self, model):
        self.step = max(0.001, model.get_value_as_float())

    def _on_slider(self, model):
        g = self.c.set_goal(model.get_value_as_float())
        self.status.text = "목표 %.3f m / 속도 %.3f m/s" % (g, self.c.speed)

    def _set(self, value):
        self.pos_model.set_value(value)

    def _up(self):
        self._set(min(self.c.upper, self.pos_model.get_value_as_float() + self.step))

    def _down(self):
        self._set(max(self.c.lower, self.pos_model.get_value_as_float() - self.step))

    def _stop(self):
        here = self.c.stop()
        self.pos_model.set_value(here)
        self.status.text = "정지 (%.3f m)" % here

    def _read(self):
        a = self.c.actual()
        if a is None:
            self.status.text = "실제 높이는 Play 중에만 읽힙니다."
        else:
            self.status.text = "목표 %.3f m / 실제 %.3f m" % (self.c.goal, a)


# 전역으로 잡아 두지 않으면 창과 물리 콜백이 바로 사라집니다.
try:
    lift.destroy()          # noqa: F821  (다시 실행할 때 이전 것 정리)
except Exception:
    pass

lift = LiftController()
lift_panel = LiftPanel(lift)
print("리프트 조작 패널: 안전 범위 %.4f ~ %.4f m, 현재 가동 범위 %.4f ~ %.4f m, 속도 %.3f m/s"
      % (lift.safe_lower, lift.safe_upper, lift.lower, lift.upper, lift.speed))
