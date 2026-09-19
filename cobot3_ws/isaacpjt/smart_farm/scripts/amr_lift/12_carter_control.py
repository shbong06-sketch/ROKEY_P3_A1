"""
Nova Carter 주행 제어만 떼어낸 모듈.

리프트·팔과 독립적으로 쓸 수 있다. 같은 아티큘레이션 안에 있어도
바퀴 DOF 인덱스만 건드리므로 다른 관절 명령과 섞이지 않는다.

쓰는 법
  A) GUI : 파일 열고 Play → Script Editor 에 붙여넣기 (작은 창이 뜬다)
             carter.drive(0.3, 0.0)     # 전진 0.3 m/s
             carter.turn(0.6)           # 좌회전 0.6 rad/s
             carter.stop()
  B) 스탠드얼론 : 아래 example() 참고. World 를 직접 만들고 매 스텝 돌린다.

왜 USD 속성이 아니라 아티큘레이션으로 명령하나
  바퀴 조인트의 `drive:angular:physics:targetVelocity` 를 USD 로 바꿔도
  Play 중에는 PhysX 로 반영되지 않는다 (실측: 2.14 rad/s 를 써도 실제 0.03).
  ArticulationAction 의 joint_velocities 로 줘야 실제로 돈다 (2.13 rad/s).
  리프트·팔의 targetPosition 은 USD 로도 반영되는 것과 다르다.

카터 제원 (nova_carter.usd 에서 실측)
  구동륜 반지름 0.14 m      (wheel_left 원점 z = 0.14)
  좌우 간격    0.4132 m     (wheel_left y = +0.2066, wheel_right y = -0.2066)
  바퀴 Drive   stiffness 0 / damping 1e6  = 속도 제어 전용
  캐스터 4축은 자유 회전 (명령하지 않는다)
"""

import math

import omni.usd
import omni.timeline
import omni.physx
from pxr import UsdPhysics


WHEEL_JOINTS = ["joint_wheel_left", "joint_wheel_right"]
WHEEL_RADIUS = 0.14        # m
WHEEL_BASE = 0.4132        # m (좌우 구동륜 간격)

MAX_LINEAR = 1.5           # m/s  (안전 상한, 명령은 여기서 잘린다)
MAX_ANGULAR = 2.0          # rad/s

# 실측 보정값
#   직진 : 명령의 98% 로 나온다 (0.30 m/s x 3s = 0.90 m 명령 -> 실제 0.882 m)
#   회전 : 명령의 약 70% 밖에 안 돈다. 바퀴는 명령대로 돌지만(잰 w=0.553 vs 0.4 명령)
#          캐스터 4개의 저항과 타이어 슬립 때문에 차체 yaw 는 덜 돈다.
#          정확한 각도가 필요하면 turn_by() 같은 폐루프를 쓸 것.
TURN_EFFICIENCY = 0.70     # 참고용(열린 루프 추정에 쓰고 싶으면 w / 이 값으로 명령)

SHOW_UI = True             # Script Editor 에서 실행할 때 작은 창을 띄운다


class CarterDriver:
    """바퀴 2축만 다루는 얇은 래퍼."""

    def __init__(self, articulation=None, stage=None):
        self.stage = stage or omni.usd.get_context().get_stage()
        self.timeline = omni.timeline.get_timeline_interface()
        self._arti = articulation
        self._idx = None
        self._cmd = (0.0, 0.0)          # (left, right) rad/s
        self._goal = None               # 폐루프 목표 (move/turn)
        self._sub = None
        if articulation is None:
            # GUI 모드: Play 될 때마다 아티큘레이션을 다시 잡고 매 스텝 명령을 유지한다
            self._sub = omni.physx.get_physx_interface().subscribe_physics_step_events(
                self._on_step)
            self._tl_sub = self.timeline.get_timeline_event_stream().create_subscription_to_pop(
                self._on_timeline)
        else:
            self._bind_indices()

    # ── 내부 ──────────────────────────────────────────────
    def _bind_indices(self):
        names = list(self._arti.dof_names)
        self._idx = [names.index(n) for n in WHEEL_JOINTS if n in names]
        return self._idx

    def _ensure(self):
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
            a = SingleArticulation(prim_path=str(root.GetPath()), name="carter_driver")
            a.initialize()
            self._arti = a
            self._bind_indices()
            return a
        except Exception:
            return None

    def _apply(self):
        a = self._ensure()
        if a is None or not self._idx:
            return False
        import numpy as np
        from isaacsim.core.utils.types import ArticulationAction

        a.apply_action(ArticulationAction(
            joint_velocities=np.array(self._cmd[:len(self._idx)]),
            joint_indices=np.array(self._idx),
        ))
        self._check_goal()
        return True

    def _on_step(self, dt):
        self._apply()

    # ── 폐루프(목표까지 가고 스스로 멈춤) ─────────────────
    def move_by(self, distance_m, v=0.3, tol=0.01):
        """앞(+)/뒤(-)로 distance_m 만큼 가고 스스로 멈춘다."""
        p = self.pose()
        if p is None:
            return False
        self._goal = ("move", p[0], p[1], abs(float(distance_m)), float(tol))
        self.drive(v if distance_m >= 0 else -abs(v), 0.0)
        return True

    def turn_by(self, degrees, w=0.6, tol_deg=1.0):
        """좌(+)/우(-)로 degrees 만큼 제자리 회전하고 스스로 멈춘다.

        슬립이 있어도 각도가 맞는다 (실제 yaw 를 보고 멈추기 때문).
        """
        p = self.pose()
        if p is None:
            return False
        target = p[2] + math.radians(float(degrees))
        self._goal = ("turn", target, math.radians(float(tol_deg)))
        self.turn(abs(w) if degrees >= 0 else -abs(w))
        return True

    def goal_done(self):
        return self._goal is None

    def _check_goal(self):
        if self._goal is None:
            return
        p = self.pose()
        if p is None:
            return
        kind = self._goal[0]
        if kind == "move":
            _, x0, y0, dist, tol = self._goal
            if math.hypot(p[0] - x0, p[1] - y0) >= dist - tol:
                self._goal = None
                self.stop()
        elif kind == "turn":
            _, target, tol = self._goal
            err = (target - p[2] + math.pi) % (2 * math.pi) - math.pi
            if abs(err) <= tol:
                self._goal = None
                self.stop()

    def _on_timeline(self, event):
        if not self.timeline.is_playing():
            # Stop/Pause 하면 핸들이 무효가 된다. 다음 Play 때 다시 잡는다.
            self._arti = None
            self._idx = None
            self._cmd = (0.0, 0.0)

    # ── 명령 ──────────────────────────────────────────────
    def drive(self, linear_mps, angular_rps=0.0):
        """전진 속도(m/s), 회전 속도(rad/s, 좌회전 +) → 좌우 바퀴 각속도."""
        v = max(-MAX_LINEAR, min(MAX_LINEAR, float(linear_mps)))
        w = max(-MAX_ANGULAR, min(MAX_ANGULAR, float(angular_rps)))
        left = (v - w * WHEEL_BASE / 2.0) / WHEEL_RADIUS
        right = (v + w * WHEEL_BASE / 2.0) / WHEEL_RADIUS
        self._cmd = (left, right)
        self._apply()
        return left, right

    def forward(self, v=0.3):
        return self.drive(v, 0.0)

    def back(self, v=0.3):
        return self.drive(-abs(v), 0.0)

    def turn(self, w=0.6):
        """양수 = 좌회전, 음수 = 우회전 (제자리 회전)."""
        return self.drive(0.0, w)

    def stop(self):
        self._goal = None
        return self.drive(0.0, 0.0)

    def wheel_velocities(self):
        """실제 바퀴 각속도 (rad/s). Play 중에만 읽힌다."""
        a = self._ensure()
        if a is None or not self._idx:
            return None
        q = a.get_joint_velocities()
        return [float(q[i]) for i in self._idx]

    def measured(self):
        """바퀴 각속도에서 되돌린 실제 (전진 m/s, 회전 rad/s)."""
        wv = self.wheel_velocities()
        if not wv or len(wv) < 2:
            return None
        left, right = wv[0] * WHEEL_RADIUS, wv[1] * WHEEL_RADIUS
        return ((left + right) / 2.0, (right - left) / WHEEL_BASE)

    def pose(self):
        """섀시 (x, y, yaw[rad]). Play 중에만 읽힌다."""
        a = self._ensure()
        if a is None:
            return None
        pos, quat = a.get_world_pose()
        w, x, y, z = [float(v) for v in quat]
        yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
        return float(pos[0]), float(pos[1]), yaw

    def destroy(self):
        try:
            self.stop()
        except Exception:
            pass
        self._sub = None
        self._tl_sub = None


# ── 스탠드얼론 예제 ──────────────────────────────────────
def example(rig_usd, seconds=4.0, v=0.3, w=0.0):
    """~/isaacsim/python.sh 로 돌릴 때의 최소 형태."""
    import numpy as np
    from isaacsim.core.api import World
    from isaacsim.core.prims import SingleArticulation

    omni.usd.get_context().open_stage(rig_usd)
    world = World(stage_units_in_meters=1.0, physics_dt=1.0 / 120.0)
    stage = omni.usd.get_context().get_stage()
    root = None
    for prim in stage.Traverse():
        if prim.HasAPI(UsdPhysics.ArticulationRootAPI):
            root = prim
            break
    robot = world.scene.add(SingleArticulation(prim_path=str(root.GetPath()), name="carter"))
    world.reset()

    carter = CarterDriver(articulation=robot, stage=stage)
    carter.drive(v, w)
    for _ in range(int(seconds * 120)):
        carter._apply()            # 속도 명령을 매 스텝 유지
        world.step(render=True)
    print("실제 속도(v, w) =", carter.measured(), " 위치 =", carter.pose())
    carter.stop()
    return carter


# ── Script Editor 용 작은 창 ─────────────────────────────
class CarterPanel:
    def __init__(self, driver):
        import omni.ui as ui

        self.ui = ui
        self.d = driver
        self.window = ui.Window("Carter drive", width=380, height=170)
        with self.window.frame:
            with ui.VStack(spacing=6, height=0):
                with ui.HStack(spacing=6, height=24):
                    ui.Label("speed m/s", width=90)
                    self.v = ui.SimpleFloatModel(0.3)
                    ui.FloatSlider(self.v, min=0.05, max=MAX_LINEAR, step=0.05, format="%.2f")
                with ui.HStack(spacing=6, height=24):
                    ui.Label("turn rad/s", width=90)
                    self.w = ui.SimpleFloatModel(0.6)
                    ui.FloatSlider(self.w, min=0.1, max=MAX_ANGULAR, step=0.1, format="%.1f")
                with ui.HStack(spacing=6, height=28):
                    ui.Button("Forward", clicked_fn=lambda: self._go(1, 0))
                    ui.Button("Back", clicked_fn=lambda: self._go(-1, 0))
                    ui.Button("Left", clicked_fn=lambda: self._go(0, 1))
                    ui.Button("Right", clicked_fn=lambda: self._go(0, -1))
                    ui.Button("STOP", clicked_fn=lambda: self._go(0, 0))
                self.status = ui.Label("Press Play, then drive.")

    def _go(self, fwd, turn):
        v = self.v.get_value_as_float() * fwd
        w = self.w.get_value_as_float() * turn
        left, right = self.d.drive(v, w)
        m = self.d.measured()
        self.status.text = ("cmd v=%.2f w=%.2f -> wheels %.2f / %.2f rad/s%s"
                            % (v, w, left, right,
                               "" if m is None else "  (measured v=%.2f w=%.2f)" % m))


if __name__ == "__main__":
    # Script Editor 에서 붙여넣어 실행하는 경우
    try:
        carter.destroy()      # noqa: F821
    except Exception:
        pass
    carter = CarterDriver()
    if SHOW_UI:
        carter_panel = CarterPanel(carter)
    print("carter driver ready. carter.drive(0.3, 0.0) / carter.turn(0.6) / carter.stop()")
