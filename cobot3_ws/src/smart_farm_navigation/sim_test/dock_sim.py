"""도킹 상태기계(DockLogic)를 ROS 없이 평면 운동학 위에서 돌리는 오프라인 모의.

실측 bag(2026-09-23 22·24차)에서 읽은 카터 특성을 플랜트에 넣는다:
  * 각속도는 명령을 즉시 따르지 않고 초당 `ang_accel` 만큼만 변한다
  * 제자리 회전(v≈0)은 명령의 `inplace_gain` 배만 나온다 (22차 0.12, 24차 0.75)
  * 이동 중 조향은 명령의 `moving_gain` 배 (0.6~0.8)
  * 선속도는 시정수 0.3 s 로 빠르게 따른다
  * 면 측정(/scan)은 0.5 s 늦고 3.3 Hz 로만 갱신된다, odom 은 7 Hz 즉시
플랜트 편차·초기 자세를 격자로 돌려 전부 도킹되는지 본다.

    python3 dock_sim.py            # 격자 전체
    python3 dock_sim.py -v         # 케이스마다 단계 로그
"""
import argparse
import itertools
import math
import os
import random
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
from smart_farm_navigation.feeder_dock import DockLogic, DockParams, wrap  # noqa: E402

# v011 장면: TurnTable 앞면 y −3.60, x −2.76~−1.61, 법선 +y (면 -> 로봇)
FACE_C = (-2.185, -3.60)
FACE_N = (0.0, 1.0)
FACE_LEN = 1.15
REAR_X = 0.607            # base_link 에서 차체 뒤끝까지


class Plant:
    """카터 대역. 각속도는 명령×gain 을 목표로 시정수 tau_w 의 1차 지연으로 따라가되
    초당 ang_accel 을 넘지 못한다. 제자리(v≈0)에서는 gain 이 다르다(마찰·캐스터)."""

    def __init__(self, x, y, yaw_deg, ang_accel, inplace_gain, moving_gain, tau_w=1.0, tau_inplace=None,
                 pivot_x=-0.25, v0=0.0, w0=0.0):
        self.x, self.y, self.yaw = x, y, math.radians(yaw_deg)
        self.v, self.w = v0, w0        # 시작 순간 Nav2 감속이 아직 남아 있을 수 있다 (24차: v -0.21, w +0.34)
        self.pivot_x = pivot_x         # 회전 중심이 base_link 뒤에 있어 제자리 회전에도 차체가 옆으로 밀린다 (24차 AMCL 0.16 m)
        self.ang_accel, self.inplace_gain, self.moving_gain, self.tau_w = ang_accel, inplace_gain, moving_gain, tau_w
        self.tau_inplace = tau_w if tau_inplace is None else tau_inplace   # 제자리 회전은 마찰 때문에 더 굼뜨다
        self.max_w = 0.0

    def step(self, v_cmd, w_cmd, dt):
        self.v += (v_cmd - self.v) * min(1.0, dt / 0.3)
        moving = abs(self.v) > 0.03
        gain = self.moving_gain if moving else self.inplace_gain
        tau = self.tau_w if moving else self.tau_inplace
        target = w_cmd * gain
        if abs(target) < abs(self.w) and target * self.w >= 0 or target * self.w < 0:
            # 멈추는 쪽은 마찰이 지배해 지연 없이 초당 1.4×ang_accel 로 준다 (24차: 올라갈 때 0.05~0.09, 내려올 때 0.14)
            lim = 1.4 * self.ang_accel * dt
            dw = max(-lim, min(lim, target - self.w))
        else:
            lim = self.ang_accel * dt
            dw = max(-lim, min(lim, (target - self.w) * min(1.0, dt / tau)))
        self.w += dw
        # 회전 중심(pivot) 을 기준으로 돌린다: base_link 는 pivot 둘레를 도는 만큼 옆으로 밀린다
        px = self.x + self.pivot_x * math.cos(self.yaw); py = self.y + self.pivot_x * math.sin(self.yaw)
        self.yaw = wrap(self.yaw + self.w * dt)
        self.x = px - self.pivot_x * math.cos(self.yaw) + self.v * math.cos(self.yaw) * dt
        self.y = py - self.pivot_x * math.sin(self.yaw) + self.v * math.sin(self.yaw) * dt
        self.max_w = max(self.max_w, abs(self.w))

    def face(self):
        """노드의 detect_face 가 돌려주는 것과 같은 (dist, yaw_err, lat, cx, cy, n, length)."""
        c, s = math.cos(-self.yaw), math.sin(-self.yaw)
        dx, dy = FACE_C[0] - self.x, FACE_C[1] - self.y
        cx, cy = c * dx - s * dy, s * dx + c * dy
        nx, ny = c * FACE_N[0] - s * FACE_N[1], s * FACE_N[0] + c * FACE_N[1]
        if nx < math.cos(math.radians(60)):        # 법선이 뒤쪽 ±60도 밖이면 면으로 인정하지 않는다
            return None
        yaw_err = wrap(math.atan2(ny, nx))
        dist = abs(nx * cx + ny * cy)
        return (dist, yaw_err, cy, cx, cy, 40, FACE_LEN)

    def rear_clearance(self):
        """차체 뒤끝과 면 사이 거리. 음수면 면을 뚫은 것이다."""
        f = self.face()
        return None if f is None else f[0] - REAR_X


# (ang_accel, inplace_gain, moving_gain, tau_w). 22·24차 bag 에서 읽은 범위를 넓게 덮는다.
PLANTS = [
    (0.10, 0.75, 0.7, 1.0),   # 24차: 제자리 회전 3 s 뒤 0.26, 이동 중 0.7 배
    (0.10, 0.12, 0.6, 1.0),   # 22차: 제자리 회전이 거의 안 됨
    (0.06, 0.5, 0.7, 2.0),    # 더 둔함
    (0.06, 0.5, 0.7, 3.0),    # 가장 둔함
    (0.3, 1.0, 1.0, 0.3),     # 이상적 (합성 로봇 기본)
]

# 24차까지 쓰던 이득. 관성·지연 보정 없음, 정렬 1.5도, 후진 조향 상한 0.20.
OLD_PARAMS = DockParams(reverse_speed_mps=0.15, turn_speed_radps=0.35, reverse_w_max=0.20, align_tol_deg=1.5,
                        ang_decel_radps2=1e6, meas_lag_s=0.0, settle_w_radps=1e6, blend_dist_m=0.5)


def run_case(x, y, yaw_deg, ang_accel, inplace_gain, moving_gain, tau_w=1.0, tau_inplace=None,
             params=None, verbose=False, jam_at=None, noise_deg=1.0, v0=-0.1, w0=0.2):
    p = params or DockParams()
    logic = DockLogic(p)
    plant = Plant(x, y, yaw_deg, ang_accel, inplace_gain, moving_gain, tau_w, tau_inplace, v0=v0, w0=w0)
    rng = random.Random(int(x * 100 + y * 10 + yaw_deg))
    noise = math.radians(noise_deg)
    dt = 0.05
    now = 0.0
    scan_period, scan_lag = 0.3, p.meas_lag_s
    face_hist = []            # (t, face) — 지연을 흉내낸다
    face, face_t = None, -1e9
    odom = (0.0, 0.0, 0.0)
    ext_cmd_t = 0.4           # Nav2 감속 명령이 시작 후 0.4 s 까지 남아 있다 (24차 bag)
    logic.start("sim", now)
    min_clear = 9.0
    jammed = False
    while now < p.timeout_s + 5:
        if int(now / scan_period) != int((now - dt) / scan_period):
            f = plant.face()
            if f is not None:           # 방향 측정에 ±noise 를 섞는다 (합친 점군의 흔들림)
                e = rng.uniform(-noise, noise)
                f = (f[0], f[1] + e, f[2], f[3], f[4], f[5], f[6])
            face_hist.append((now, f))
        while face_hist and face_hist[0][0] <= now - scan_lag:
            _, face = face_hist.pop(0)
            if face is not None:
                face_t = now
        if jam_at is not None and now >= jam_at:
            jammed = True
        v, w = logic.update(now, face, now - face_t, plant.w, plant.v, odom, now - ext_cmd_t)
        for ph, detail in logic.events:
            if verbose:
                print(f"    {now:6.1f}s [{ph}] {detail}")
        logic.events.clear()
        if not jammed:
            plant.step(v, w, dt)
        odom = (plant.x, plant.y, plant.yaw)
        clear = plant.rear_clearance()
        if clear is not None:
            min_clear = min(min_clear, clear)
        if logic.result is not None:
            break
        now += dt
    return logic.result, min_clear, plant.max_w, now


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("-v", action="store_true")
    ap.add_argument("--fail-only", action="store_true")
    ap.add_argument("--old", action="store_true", help="24차까지의 이득으로 돌린다 (실측 실패가 재현되는지 확인용)")
    args = ap.parse_args()

    yaws = [70, 80, 90, 100, 110]                 # FEEDER_APPROACH 도착 방향 (Nav2 허용 0.5 rad)
    xs = [-2.39, -2.19, -1.99]                    # 좌우 0.2 m
    ys = [-1.35, -1.55, -1.85]                    # 앞뒤
    plants = PLANTS
    params = OLD_PARAMS if args.old else DockParams()
    rows = []
    for (x, y, yaw), (aa, ig, mg, tw) in itertools.product(itertools.product(xs, ys, yaws), plants):
        res, clear, max_w, t = run_case(x, y, yaw, aa, ig, mg, tw, params=params, verbose=args.v)
        ok = (res is not None and res["status"] == "SUCCEEDED"
              and abs(res["face_dist_m"] - DockParams().standoff_m) <= 0.05
              and abs(res["yaw_err_deg"]) <= 3.0 and abs(res["lat_m"]) <= 0.06 and clear > 0.0)
        rows.append((ok, x, y, yaw, aa, ig, mg, res, clear, max_w, t))
        if not ok or not args.fail_only:
            tag = "통과" if ok else "실패"
            r = res or {}
            print(f"{tag} start({x:+.2f},{y:+.2f},{yaw:3d}) plant(a={aa},ig={ig},mg={mg},tau={tw}) -> "
                  f"{r.get('status')}/{r.get('reason')} d={r.get('face_dist_m')} yaw={r.get('yaw_err_deg')} "
                  f"lat={r.get('lat_m')} clear={clear:.2f} max_w={max_w:.2f} t={t:.0f}s")
    n_ok = sum(1 for r in rows if r[0])
    # 방향(3도)·거리(±0.05)·충돌 없음만 본 수. 횡 오차는 팔 쪽 허용치가 확정되지 않아 따로 센다.
    n_heading = sum(1 for r in rows if r[7] is not None and r[7]["status"] == "SUCCEEDED"
                    and abs(r[7]["yaw_err_deg"]) <= 3.0 and abs(r[7]["face_dist_m"] - DockParams().standoff_m) <= 0.05 and r[8] > 0.0)
    worst_lat = max((abs(r[7]["lat_m"]) for r in rows if r[7] is not None and r[7]["status"] == "SUCCEEDED"), default=0.0)
    print(f"\n방향·거리·충돌 기준 {n_heading}/{len(rows)} 통과, 횡 오차 0.06 m 까지 포함하면 {n_ok}/{len(rows)} (최대 횡 오차 {worst_lat:.3f} m)")
    return 0 if n_heading == len(rows) else 1


if __name__ == "__main__":
    sys.exit(main())
