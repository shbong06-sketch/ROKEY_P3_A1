"""Slide / document figures for the 2026-09-27 experiments (cull-failure cause, carry speed, asset vs real).

  <Isaac>\\kit\\python\\python.exe make_figures.py [OUT]     (needs PIL; OUT default D:\\smartfarm-sim\\out\\figures_2026-09-27)

Data: out\\webdb_2026-09-27\\smartfarm_20260927.sqlite (export_webdb.py) + run folders (captures, station frames, monitor.json).
Charts 1600x900 PNG (16:9), frame sheets 1920 wide JPG. captions.csv + README.md list every file.
"""
import csv
import json
import math
import random
import re
import sqlite3
import sys
from pathlib import Path

from PIL import Image, ImageDraw, ImageFont

sys.path.insert(0, str(Path(__file__).parent))
from analyze import load, wall_to_sim  # noqa: E402

OUT = Path(sys.argv[1] if len(sys.argv) > 1 else r"D:\smartfarm-sim\out\figures_2026-09-27")
ROOT = Path(r"D:\smartfarm-sim\out")
SPEED, GRASP = ROOT / "exp_20260927_speed", ROOT / "exp_20260927_grasp"
DB = sqlite3.connect(ROOT / "webdb_2026-09-27" / "smartfarm_20260927.sqlite")
FONT, FONTB = "C:/Windows/Fonts/malgun.ttf", "C:/Windows/Fonts/malgunbd.ttf"
f = lambda s, b=False: ImageFont.truetype(FONTB if b else FONT, s)  # noqa: E731
INK, SUB, GRID, LIGHT = (24, 34, 30), (86, 99, 93), (228, 232, 229), (150, 156, 152)
BLUE, ORANGE, GREEN, RED, PURPLE = (42, 111, 184), (212, 114, 31), (46, 143, 94), (179, 54, 44), (122, 91, 176)
CAPS = []


def save(img, sub, name, title, caption, source):
    p = OUT / sub / name
    p.parent.mkdir(parents=True, exist_ok=True)
    img.save(p, quality=90) if name.endswith(".jpg") else img.save(p)
    CAPS.append(dict(file=f"{sub}/{name}", title=title, caption=caption, source=source))
    print("saved", p.relative_to(OUT))


def canvas(title, sub=None, W=1600, H=900):
    img = Image.new("RGB", (W, H), "white")
    d = ImageDraw.Draw(img)
    d.text((60, 36), title, font=f(34, True), fill=INK)
    if sub:
        d.text((60, 86), sub, font=f(20), fill=SUB)
    return img, d


def axes(d, box, xr, yr, xt, yt, xlab, ylab, xfmt=str, yfmt=str):
    L, T, R, B = box
    X = lambda v: L + (v - xr[0]) / (xr[1] - xr[0]) * (R - L)  # noqa: E731
    Y = lambda v: B - (v - yr[0]) / (yr[1] - yr[0]) * (B - T)  # noqa: E731
    for v in yt:
        d.line([(L, Y(v)), (R, Y(v))], fill=GRID, width=2)
        d.text((L - 14, Y(v)), yfmt(v), font=f(20), fill=SUB, anchor="rm")
    for v in xt:
        d.text((X(v), B + 16), xfmt(v), font=f(20), fill=SUB, anchor="mt")
    d.line([(L, B), (R, B)], fill=LIGHT, width=2)
    d.text(((L + R) / 2, B + 52), xlab, font=f(22), fill=SUB, anchor="mt")
    if ylab:
        d.text((L, T - 36), ylab, font=f(20), fill=SUB, anchor="lm")
    return X, Y


def legend(d, x, y, items):
    for i, (c, t) in enumerate(items):
        d.rounded_rectangle([x, y + i * 36, x + 26, y + i * 36 + 18], 3, fill=c)
        d.text((x + 36, y + i * 36 + 9), t, font=f(21), fill=INK, anchor="lm")


def fit(im, w, h):
    im = im.convert("RGB")
    r = min(w / im.width, h / im.height)
    return im.resize((int(im.width * r), int(im.height * r)), Image.LANCZOS)


def sheet(title, tiles, cols, tile_w, tile_h, sub=None):
    """tiles: [(path, label, colour)] -> labelled grid"""
    pad, head, cap = 24, (130 if sub else 96), 54
    rows = math.ceil(len(tiles) / cols)
    W, H = pad + cols * (tile_w + pad), head + rows * (tile_h + cap + pad)
    img = Image.new("RGB", (W, H), "white")
    d = ImageDraw.Draw(img)
    d.text((pad + 8, 28), title, font=f(34, True), fill=INK)
    if sub:
        d.text((pad + 8, 80), sub, font=f(20), fill=SUB)
    for i, (path, label, col) in enumerate(tiles):
        r, c = divmod(i, cols)
        x, y = pad + c * (tile_w + pad), head + r * (tile_h + cap + pad)
        im = fit(Image.open(path), tile_w, tile_h)
        img.paste(im, (x + (tile_w - im.width) // 2, y))
        d.rectangle([x, y + tile_h, x + tile_w, y + tile_h + cap], fill=col or (245, 247, 245))
        d.text((x + 14, y + tile_h + cap / 2), label, font=f(22, True), fill="white" if col else INK, anchor="lm")
    return img


def frame(run, t, cam="navplace"):
    """nearest capture at or after t (captures every 0.5 s)"""
    caps = sorted((float(m.group(1)), p) for p in (SPEED / run / "captures").glob(f"cap_{cam}_*.jpg")
                  if (m := re.search(r"_(\d+\.\d)\.jpg$", p.name)))
    return min(caps, key=lambda c: abs(c[0] - t))


def events(run):
    m = load(SPEED / run / "monitor.json", {})
    fn = wall_to_sim(m.get("chassis_track", []))
    return {t.split(" TASK")[0][:40]: fn(w) for w, t in load(SPEED / run / "flow.json", {}).get("events", [])}, m


# ================= A. cull failure cause =================
def a1_grip():
    img, d = canvas("RG2 쥐는 힘에 따른 선별 성공률", "스테이션 단독 · 정답 라벨 · 조건당 선별 6건 (10 N·m / 0.5 kg 은 3건)")
    X, Y = axes(d, (140, 170, 1300, 760), (3, 17), (0, 1), [4, 6, 8, 10, 12, 14, 16], [0, .25, .5, .75, 1],
                "finger_joint 최대 토크 (N·m)", "선별 성공률", yfmt=lambda v: f"{int(v * 100)}%")
    d.line([(X(8), 170), (X(8), 760)], fill=LIGHT, width=2)
    d.text((X(8) + 8, 180), "기존 기본값 8", font=f(20), fill=SUB)
    d.line([(X(12), 170), (X(12), 760)], fill=GREEN, width=2)
    d.text((X(12) + 8, 180), "권고 12", font=f(20, True), fill=GREEN)
    cols = {0.3: BLUE, 0.5: ORANGE, 0.7: GREEN}
    q = DB.execute("SELECT head_mass_kg, grip_force_nm, n, ok FROM v_grasp_by_condition WHERE source='grasp_exp' "
                   "ORDER BY head_mass_kg, grip_force_nm").fetchall()
    for m, c in cols.items():
        pts = [(g, ok / n, ok, n) for mm, g, n, ok in q if mm == m]
        d.line([(X(g), Y(s)) for g, s, _, _ in pts], fill=c, width=5)
        for g, s, ok, n in pts:
            d.ellipse([X(g) - 10, Y(s) - 10, X(g) + 10, Y(s) + 10], fill=c, outline="white", width=3)
    legend(d, 1340, 200, [(BLUE, "포기 0.3 kg (에셋)"), (ORANGE, "포기 0.5 kg"), (GREEN, "포기 0.7 kg")])
    save(img, "A_선별실패원인", "A1_쥐는힘_성공률.png", "RG2 쥐는 힘에 따른 선별 성공률",
         "8 N·m 는 0.3 kg 에서 경계(5~6 N·m) 바로 위, 0.5 kg 에서 1/6. 12 N·m 에서 0.3·0.5·0.7 kg 모두 6/6.",
         "grasp_trial / v_grasp_by_condition (grasp_exp 93건)")


def a2_finger():
    img, d = canvas("들어 올리기 직전 손가락 각도 — 실패의 지문",
                    "LIFT 단계 시작 시 finger_joint (rad). 덜 닫힌 채 들면 미끄러진다 (lift_slip = 경유점에서 손가락 1.18 = 빈손)")
    rows = [("성공 (9/27 스테이션, 전 조건)", "source='grasp_exp' AND in_box", GREEN),
            ("미끄러짐 (9/27, 약한 힘·무거운 포기)", "source='grasp_exp' AND outcome='lift_slip'", ORANGE),
            ("성공 (9/26 전체 흐름, 8 N·m)", "source='0926_flow' AND in_box", BLUE),
            ("실패 (9/26 전체 흐름, 3건)", "source='0926_flow' AND NOT in_box", RED)]
    X, _ = axes(d, (560, 170, 1520, 760), (0.34, 0.41), (0, 1), [0.34, 0.35, 0.36, 0.37, 0.38, 0.39, 0.40, 0.41], [],
                "LIFT 시점 손가락 각도 (rad)", None, xfmt=lambda v: f"{v:.2f}")
    d.rectangle([X(0.355), 170, X(0.371), 760], fill=(252, 238, 236))
    d.text((X(0.363), 178), "9/26 실패 3건 구간", font=f(20, True), fill=RED, anchor="mt")
    rnd = random.Random(3)
    for i, (lab, cond, c) in enumerate(rows):
        yc = 250 + i * 140
        d.text((60, yc), lab, font=f(23, True), fill=INK, anchor="lm")
        vals = [v for (v,) in DB.execute(f"SELECT finger_at_lift_rad FROM grasp_trial WHERE {cond} AND finger_at_lift_rad < 0.45")]
        for v in vals:
            if 0.34 <= v <= 0.41:
                y = yc + rnd.uniform(-34, 34)
                d.ellipse([X(v) - 8, y - 8, X(v) + 8, y + 8], fill=c, outline="white", width=2)
        d.text((1540, yc), f"{len(vals)}건", font=f(20), fill=SUB, anchor="rm")
    save(img, "A_선별실패원인", "A2_손가락각도_실패지문.png", "들어 올리기 직전 손가락 각도",
         "9/26 전체 흐름 실패 3건(0.356~0.367 rad)은 성공 대부분(0.38 이상)보다 덜 닫힌 채 들어 올렸다. 다만 같은 구간의 성공도 3건 있고, "
         "9/27 미끄러짐은 0.34~0.39 로 넓게 퍼져 있어 각도만으로 실패를 가를 수는 없다. 공통 원인은 쥐는 힘 부족(A1).",
         "grasp_trial.finger_at_lift_rad (0.45 rad 이상 이상치 1건 제외)")


def a3_frames():
    g = GRASP
    tiles = [(g / "f8_m0.5" / "Pallet_Test_0_yolo.jpg", "8 N·m · 0.5 kg — 검사: 불량 6", None),
             (g / "f8_m0.5" / "Pallet_Test_recheck_2_yolo.jpg", "8 N·m — 재검사: 5개 남음 (미끄러짐)", RED),
             (g / "f12_m0.5" / "Pallet_Test_0_yolo.jpg", "12 N·m · 0.5 kg — 검사: 불량 6", None),
             (g / "f12_m0.5" / "Pallet_Test_recheck_0_yolo.jpg", "12 N·m — 재검사: 모두 제거", GREEN)]
    img = sheet("같은 트레이, 쥐는 힘만 다르게 (포기 0.5 kg)", tiles, 4, 440, 440,
                "비전 손목 카메라 · 선별 전(검사) / 선별 후(재검사). 불량 패턴 B,Y,B,Y,B,Y")
    save(img, "A_선별실패원인", "A3_8Nm_vs_12Nm_재검사.jpg", "8 N·m vs 12 N·m 재검사 화면",
         "0.5 kg 포기 6개를 모두 버려야 하는 패턴. 8 N·m 는 5개가 미끄러져 트레이에 남고, 12 N·m 는 모두 제거된다.",
         r"exp_20260927_grasp\f8_m0.5, f12_m0.5")


def a4_flow():
    tiles = [(ROOT / "exp_20260926" / "e0_default_v0.6" / "station" / "Pallet_01_recheck_0_yolo.jpg",
              "9/26 · 8 N·m — 재검사: 노랑·갈색 남음", RED),
             (GRASP / "flow_f12_v0.6_r2" / "station" / "Pallet_01_recheck_0_yolo.jpg", "9/27 · 12 N·m — 재검사: 불량 모두 제거", GREEN)]
    img = sheet("전체 흐름(수확 → 주행 → 컨베이어 → 검사 → 선별)에서 확인", tiles, 2, 640, 640,
                "9/26 실패 회차(e0_default_v0.6, SLOT 03·05 미끄러짐) vs 9/27 12 N·m 회차(유효 3회 선별 9/9). 9/26 화면의 라벨은 교체 전 로메인 모델")
    save(img, "A_선별실패원인", "A4_전체흐름_재검사_전후.jpg", "전체 흐름 재검사 화면 (9/26 실패 vs 9/27 12 N·m)",
         "9/26 은 들어 올리다 미끄러진 노랑·갈색 포기가 남았다. 12 N·m 로 올린 뒤 유효 3회 모두 불량 3개씩 제거됐다.",
         r"exp_20260926\e0_default_v0.6\station, exp_20260927_grasp\flow_f12_v0.6_r2\station")


def a5_des():
    img, d = canvas("설비 구성 DES — 로봇 대수별 처리량", "불량률 10 % · 주행 0.3 m/s · 선별 성공률 92.3 % · 8 h x 20회 평균")
    X, Y = axes(d, (140, 170, 1300, 760), (0.8, 5.2), (0, 140), [1, 2, 3, 4, 5], [0, 35, 70, 105, 140],
                "수확 로봇 대수", "트레이/h", xfmt=lambda v: f"{v}대")
    d.line([(X(1), Y(25.62)), (X(5), Y(128.1))], fill=LIGHT, width=3)
    for st, c in ((1, ORANGE), (2, BLUE)):
        pts = DB.execute("SELECT robots, trays_per_h, station_util FROM des_scenario WHERE drive_cond='default_v0.3' "
                         "AND defect_rate=0.1 AND q_name='q_flow_0926' AND policy='none' AND stations=? ORDER BY robots",
                         (st,)).fetchall()
        d.line([(X(r), Y(t)) for r, t, _ in pts], fill=c, width=5)
        for r, t, u in pts:
            d.ellipse([X(r) - 10, Y(t) - 10, X(r) + 10, Y(t) + 10], fill=c, outline="white", width=3)
            if st == 1 and r >= 3:
                d.text((X(r) + 14, Y(t) + 10), f"가동률 {u * 100:.0f}%", font=f(19), fill=ORANGE)
    legend(d, 1340, 200, [(ORANGE, "스테이션 1곳"), (BLUE, "스테이션 2곳"), (LIGHT, "로봇만의 한계")])
    save(img, "A_선별실패원인", "A5_DES_처리량.png", "설비 구성 DES 처리량",
         "불량률 10 % 에서 스테이션 1곳은 로봇 3대까지 따라간다. 4대부터 스테이션 가동률 94 % 로 병목.",
         "des_scenario (default_v0.3, q_flow_0926, policy none)")


def a6_retry():
    img, d = canvas("재검사 후 1회 재시도 정책 — 놓친 불량", "로봇 3대 · 스테이션 1곳 · 주행 0.3 m/s · 선별 1회 성공률 92.3 % (9/26 전체 흐름)")
    X, Y = axes(d, (140, 170, 1300, 760), (0, 3), (0, 100), [], [0, 25, 50, 75, 100], "불량률", "놓친 불량 / 1,000트레이")
    for i, dr in enumerate((0.05, 0.1, 0.2)):
        for j, (pol, c) in enumerate((("none", RED), ("retry", GREEN))):
            v, = DB.execute("SELECT escaped_per_1000_trays FROM des_scenario WHERE drive_cond='default_v0.3' AND robots=3 "
                            "AND stations=1 AND q_name='q_flow_0926' AND defect_rate=? AND policy=?", (dr, pol)).fetchone()
            x0 = X(i + 0.22 + j * 0.3)
            d.rectangle([x0, Y(v), x0 + X(0.26) - X(0), Y(0)], fill=c)
            d.text((x0 + (X(0.26) - X(0)) / 2, Y(v) - 10), f"{v:.1f}", font=f(22, True), fill=c, anchor="mb")
        d.text((X(i + 0.5), 776), f"{int(dr * 100)} %", font=f(22), fill=SUB, anchor="mt")
    legend(d, 1340, 200, [(RED, "재시도 없음 (현재)"), (GREEN, "1회 재시도")])
    d.text((1340, 300), "처리량 75.4 → 74.3 트레이/h", font=f(20), fill=SUB)
    d.text((1340, 330), "(불량률 10 %, -1.5 %)", font=f(20), fill=SUB)
    save(img, "A_선별실패원인", "A6_재시도정책_놓친불량.png", "재시도 정책의 놓친 불량 감소",
         "재검사에서 남은 불량을 한 번 더 집으면 놓친 불량이 92 % 줄고 처리량은 1.5 % 준다. 스테이션 코드에는 아직 없는 정책.",
         "des_scenario (robots 3, stations 1)")


# ================= B. carry speed =================
def b1_carry():
    rows = DB.execute("""SELECT c.run_id, c.max_speed_mps, c.accel_mps2, c.dock_args, c.nav_ok, c.place_ok, c.carry_s, c.dock_s,
                        c.dock_retries, c.head_disp_mm FROM carry_result c WHERE c.head_mass_kg = 0.3
                        ORDER BY c.max_speed_mps, c.accel_mps2, c.dock_args, c.run_id""").fetchall()
    img, d = canvas("운반 시간 (주행 + 도킹) — 회차별", "포기 0.3 kg · Nav2 이동 명령 → 도킹 완료 · 도킹 d1 = 후진 0.15 · 미세 접근 0.08 m/s · 대기 1 s",
                    H=170 + len(rows) * 40 + 110)
    L, R = 420, 1330
    X = lambda v: L + v / 80 * (R - L)  # noqa: E731
    B = 170 + len(rows) * 40
    for v in (0, 20, 40, 60, 80):
        d.line([(X(v), 160), (X(v), B)], fill=GRID, width=2)
        d.text((X(v), B + 14), f"{v} s", font=f(20), fill=SUB, anchor="mt")
    base = [r[6] for r in rows if r[1] == 0.3]
    bm = sum(base) / len(base)
    d.line([(X(bm), 150), (X(bm), B)], fill=LIGHT, width=3)
    d.text((X(bm) + 8, 140), f"팀 설정 평균 {bm:.1f} s", font=f(19), fill=SUB)
    for i, (run, v, a, dock, nav_ok, place_ok, cs, ds, retries, disp) in enumerate(rows):
        y = 172 + i * 40
        dn = "팀 값" if not dock else ("d2" if "0.20" in dock else "d1")
        rec = v == 0.6 and dn == "d1"
        d.text((60, y + 15), f"{v} m/s · {a} m/s² · 도킹 {dn}", font=f(21, rec), fill=GREEN if rec else INK, anchor="lm")
        if nav_ok and place_ok:
            dr = cs - (ds or 0)
            d.rectangle([X(0), y + 3, X(dr), y + 28], fill=BLUE)
            d.rectangle([X(dr), y + 3, X(cs), y + 28], fill=ORANGE)
            d.text((X(cs) + 8, y + 15), f"{cs:.1f}" + (f"  재도킹 {retries}" if retries else ""), font=f(20), fill=INK, anchor="lm")
        else:
            d.rectangle([X(0), y + 3, X(min(cs, 80)), y + 28], fill=RED)
            why = "내려놓기 실패" if nav_ok else ("주행 실패 · 포기 낙하" if disp > 100 else "주행 실패")
            d.text((X(min(cs, 80)) + 8, y + 15), why, font=f(20, True), fill=RED, anchor="lm")
    legend(d, 1380, 180, [(BLUE, "주행 (Nav2)"), (ORANGE, "도킹"), (RED, "실패")])
    save(img, "B_운반속도", "B1_운반시간_회차별.png", "운반 시간 회차별",
         "0.6 m/s + 도킹 d1 이 17.3~19.6 s 로 가장 빠르다(팀 설정 30.5 s). 0.8 m/s 이상은 재도킹·주행 실패로 오히려 느리다.",
         "carry_result (head_mass 0.3)")


def b2_breakdown():
    img, d = canvas("운반 시간은 어디에 쓰이나", "조건별 평균 (각 2회) · 주행 = 운반 - 도킹")
    conds = [("팀 설정\n0.3 m/s", "v03"), ("최고속도만\n0.6 m/s", "v06"), ("추천\n0.6 m/s + 도킹 d1", "v06d1")]
    X, Y = axes(d, (160, 170, 1300, 760), (0, 3), (0, 35), [], [0, 10, 20, 30], "", "초 (시뮬레이션)")
    for i, (lab, cond) in enumerate(conds):
        rs = DB.execute("SELECT c.carry_s, c.dock_s FROM carry_result c JOIN run r USING(run_id) WHERE r.condition=? "
                        "AND c.nav_ok AND c.dock_s IS NOT NULL", (cond,)).fetchall()
        cs, ds = sum(r[0] for r in rs) / len(rs), sum(r[1] for r in rs) / len(rs)
        x0, w = X(i + 0.25), X(0.5) - X(0)
        d.rectangle([x0, Y(cs - ds), x0 + w, Y(0)], fill=BLUE)
        d.rectangle([x0, Y(cs), x0 + w, Y(cs - ds)], fill=ORANGE)
        d.text((x0 + w / 2, Y(cs) - 12), f"{cs:.1f} s", font=f(28, True), fill=INK, anchor="mb")
        d.text((x0 + w / 2, (Y(cs) + Y(cs - ds)) / 2), f"도킹 {ds:.1f}", font=f(21, True), fill="white", anchor="mm")
        d.text((x0 + w / 2, (Y(cs - ds) + Y(0)) / 2), f"주행 {cs - ds:.1f}", font=f(21, True), fill="white", anchor="mm")
        d.multiline_text((x0 + w / 2, 780), lab, font=f(22, cond == "v06d1"), fill=GREEN if cond == "v06d1" else INK,
                         anchor="ma", align="center")
    legend(d, 1340, 200, [(BLUE, "주행 (Nav2)"), (ORANGE, "도킹 (feeder_dock)")])
    save(img, "B_운반속도", "B2_시간구성_주행vs도킹.png", "운반 시간 구성",
         "팀 설정에서 도킹이 60 % 를 차지한다. 최고속도만 두 배로 해도 24 s, 도킹 속도까지 올려야 18.5 s.",
         "carry_result (v03, v06, v06d1 각 2회)")


def b3_sway():
    img, d = canvas("운반 중 포기 흔들림 (트레이 기준 최대 변위)", "Pallet_01 포기 6개 중 최대값, 1 s 창 · 가로축 = Nav2 이동 명령 후 시간")
    X, Y = axes(d, (140, 170, 1300, 760), (-2, 48), (0, 16), [0, 10, 20, 30, 40], [0, 4, 8, 12, 16],
                "이동 명령 후 (s)", "변위 mm", xfmt=lambda v: f"{v}")
    d.rectangle([140, Y(16), 1300, Y(10)], fill=(252, 238, 236))
    d.line([(140, Y(10)), (1300, Y(10))], fill=RED, width=2)
    d.text((150, Y(10) - 6), "자리 깊이 약 10 mm", font=f(20, True), fill=RED, anchor="lb")
    for run, c, lab in (("v06d1_r2", GREEN, "0.6 m/s + d1 (추천)"), ("v08_r1", ORANGE, "0.8 m/s (제자리 회전)"),
                        ("v10a06_r1", RED, "1.0 m/s · 0.6 m/s² (랙 충돌)")):
        ev, m = events(run)
        t0 = ev["navigation command FEEDER_DOCK"]
        pts = [(s[0] - t0, s[1]) for s in m["pallet01_heads_series"] if -2 <= s[0] - t0 <= 48]
        seg = [(X(t), Y(min(v, 15.8))) for t, v in pts]
        d.line(seg, fill=c, width=5)
        big = [(t, v) for t, v in pts if v > 16]
        if big:
            t, v = big[0]
            d.text((X(t) + 10, Y(15.8) + 26), f"포기 낙하 ({max(v for _, v in big):,.0f} mm)", font=f(21, True), fill=c)
    legend(d, 1340, 200, [(GREEN, "0.6 m/s + d1 (추천)"), (ORANGE, "0.8 m/s 제자리 회전"), (RED, "1.0 m/s 랙 충돌")])
    save(img, "B_운반속도", "B3_포기흔들림_시계열.png", "운반 중 포기 흔들림 시계열",
         "추천 설정은 운반 내내 1 mm 이하. 0.8 m/s 는 회전 중 9 mm(자리 깊이 직전), 1.0 m/s 는 회전 후 랙에 부딪혀 포기가 떨어졌다.",
         "head_sway_sample (v06d1_r2, v08_r1, v10a06_r1)")


def b4_track():
    img, d = canvas("차체 궤적 (위에서 본 바닥 좌표)", "수확 위치 → FEEDER 접근점 → 벨트 앞 도킹 · 1 s 간격")
    S = 130                                                   # px per m, same on both axes
    X = lambda x: 700 + (x + 2.0) * S  # noqa: E731
    Y = lambda y: 190 + (1.3 - y) * S  # noqa: E731
    d.line([(X(-3.4), Y(-3.58)), (X(-0.9), Y(-3.58))], fill=LIGHT, width=10)
    d.text((X(-0.8), Y(-3.58)), "컨베이어 면 (y -3.58 m)", font=f(20), fill=SUB, anchor="lm")
    d.ellipse([X(-2.19) - 9, Y(-1.55) - 9, X(-2.19) + 9, Y(-1.55) + 9], outline=INK, width=3)
    d.text((X(-3.5), Y(-1.55)), "FEEDER 접근점 →", font=f(20, True), fill=INK, anchor="rm")
    d.text((X(-0.42) + 18, Y(1.01)), "← 수확 후 출발", font=f(20, True), fill=INK, anchor="lm")
    for run, c in (("v06d1_r2", GREEN), ("v08_r1", ORANGE), ("v10a06_r1", RED)):
        ev, m = events(run)
        t0 = ev["navigation command FEEDER_DOCK"]
        t1 = max(ev.values()) + 1
        pts = [(X(r[1]), Y(r[2])) for r in m["chassis_track"] if t0 - 1 <= r[0] <= t1]
        d.line(pts, fill=c, width=5, joint="curve")
        d.ellipse([pts[-1][0] - 10, pts[-1][1] - 10, pts[-1][0] + 10, pts[-1][1] + 10], fill=c)
    for i in range(-4, 1):
        d.text((X(i), 856), f"x {i} m", font=f(18), fill=SUB, anchor="mt")
    legend(d, 1080, 200, [(GREEN, "0.6 m/s + d1 (성공)"), (ORANGE, "0.8 m/s (회전, 도킹 실패)"), (RED, "1.0 m/s (랙 충돌)")])
    save(img, "B_운반속도", "B4_차체궤적.png", "차체 궤적 비교",
         "추천 설정은 접근점을 거쳐 곧게 도킹한다. 0.8·1.0 m/s 는 접근점을 지나쳐 되돌아오며 크게 돈다.",
         "run 폴더 monitor.json chassis_track")


def b5_sequences():
    ev, _ = events("v06d1_r2")
    t0 = ev["navigation command FEEDER_DOCK"]
    t1 = next(v for k, v in ev.items() if k.startswith("navigation result"))
    tp = max(ev.values())
    seq = [(t0 + 1, "출발"), (t0 + 5, "주행 0.6 m/s"), (t1 - 6, "도킹 후진"), (tp, "내려놓기 완료")]
    tiles = [(frame("v06d1_r2", t)[1], f"{lab} · t {frame('v06d1_r2', t)[0]:.1f} s", GREEN if i == 3 else None)
             for i, (t, lab) in enumerate(seq)]
    save(sheet("추천 설정 한 사이클: 0.6 m/s + 도킹 d1 (운반 17.3 s)", tiles, 4, 460, 259,
               "공정 카메라 Cam2 · v06d1_r2 · 포기 흔들림 최대 0.85 mm"),
         "B_운반속도", "B5_성공_연속장면.jpg", "추천 설정 연속 장면", "출발 → 주행 → 도킹 → 내려놓기. 포기 6개가 자리를 지킨다.",
         r"exp_20260927_speed\v06d1_r2\captures")
    tiles = [(frame("v10a06_r1", t)[1], lab, RED if t >= 104 else None)
             for t, lab in ((96, "t 96 s · 접근점 지나침"), (100, "t 100 s · 제자리 회전 (포기 8.5 mm)"),
                            (104.5, "t 104.5 s · 랙 충돌"), (109, "t 109 s · 포기 낙하"))]
    save(sheet("실패: 1.0 m/s · 0.6 m/s² — 회전 후 랙 충돌", tiles, 4, 460, 259,
               "v10a06_r1 · 차체 피치 11.7° · 포기 6개 모두 떨어짐 (변위 0.6~1.2 m, 뒤집힘 168°)"),
         "B_운반속도", "B6_실패_랙충돌_연속장면.jpg", "1.0 m/s 랙 충돌 연속 장면",
         "목표점을 지나친 뒤 되돌아오며 발산해 랙에 부딪혔다.", r"exp_20260927_speed\v10a06_r1\captures")
    tiles = [(frame("v08_r1", t)[1], lab, ORANGE if 116 <= t <= 118 else None)
             for t, lab in ((106, "t 106 s · 접근점 지나침"), (110, "t 110 s · 되돌아옴"), (117, "t 117 s · 회전 (포기 9 mm)"),
                            (122, "t 122 s · 멈춤, 도킹 면 못 찾음"))]
    save(sheet("실패: 0.8 m/s — 제자리 회전으로 포기가 들림", tiles, 4, 460, 259,
               "v08_r1 · 포기 9.0 mm · 12.2° (자리 깊이 약 10 mm), 트레이 포크 위 12.6 mm 밀림"),
         "B_운반속도", "B7_실패_제자리회전_연속장면.jpg", "0.8 m/s 제자리 회전 연속 장면",
         "떨어지지는 않았지만 자리 깊이 직전까지 들렸고, 도킹 시작 위치가 벗어나 도킹에 실패했다.",
         r"exp_20260927_speed\v08_r1\captures")
    tiles = [(frame("v06d1_m1.5_r2", t)[1], lab, ORANGE if t > 110 else None)
             for t, lab in ((100, "t 100 s · 도킹 완료 (주행 중 1.9 mm)"), (105, "t 105 s · 내려놓기 하강 (7 mm)"),
                            (111, "t 111 s · 포기 14 mm · 24°"), (120, "t 120 s · 트레이 위에 남음"))]
    save(sheet("포기 1.5 kg (실물 무게 쪽) — 주행은 안전, 내려놓을 때 밀림", tiles, 4, 460, 259,
               "v06d1_m1.5_r2 · Cabbage_05 15.2 mm, Cabbage_03 10.7 mm (자리 깊이 초과), 떨어지지는 않음"),
         "B_운반속도", "B8_1.5kg_내려놓기.jpg", "1.5 kg 포기 내려놓기",
         "무거운 포기에서는 주행보다 내려놓기 팔 동작(구간당 0.5 s)이 위험 요인이다.",
         r"exp_20260927_speed\v06d1_m1.5_r2\captures")


# ================= C. asset vs real =================
def c_asset():
    img, d = canvas("양배추 트레이 에셋 vs 실물 규격 (위에서 본 같은 축척)",
                    "그림 전체 1 px = 1.25 mm · 시뮬 트레이 552 x 252 mm, 칸 간격 150 x 189 mm")
    P = 0.8                                                   # px per mm, whole figure
    TRAY = (29, 107, 86)

    def tray(x, y, head_d, col, fill=None):
        d.rectangle([x, y, x + 552 * P, y + 252 * P], outline=TRAY, width=4)
        for a in (-189, 0, 189):
            for b in (-75, 75):
                cx, cy, r = x + (276 + a) * P, y + (126 + b) * P, head_d / 2 * P
                d.ellipse([cx - r, cy - r, cx + r, cy + r], fill=fill, outline=col, width=3)
    ox, oy = 80, 190                                          # left: sim tray + real container outlines
    d.rectangle([ox, oy, ox + 550 * P, oy + 366 * P], outline=(170, 150, 210), width=3)
    d.rectangle([ox, oy, ox + 540 * P, oy + 280 * P], outline=PURPLE, width=3)
    tray(ox, oy, 88, TRAY, (205, 232, 214))
    lx = ox + 552 * P + 18
    d.text((lx, oy + 126 * P), "시뮬 트레이 552 x 252\n포기 88 mm", font=f(21, True), fill=TRAY, anchor="lm")
    d.text((lx, oy + 280 * P), "육묘 트레이 540 x 280", font=f(21), fill=PURPLE, anchor="lm")
    d.text((lx, oy + 366 * P), "표준 플라스틱 상자 550 x 366", font=f(21), fill=(150, 130, 200), anchor="lm")
    rx = 1000                                                 # right: same pitch, real head sizes
    d.text((rx, 200), "미니 양배추 약 12 cm: 칸에 들어감", font=f(22, True), fill=BLUE, anchor="lb")
    tray(rx, 245, 120, BLUE)
    d.text((rx, 520), "일반 양배추 약 20 cm: 이웃 칸과 겹침", font=f(22, True), fill=RED, anchor="lb")
    tray(rx, 590, 200, RED)
    d.text((80, 820), "RG2 완전 개방 패드 간격 99.6 mm (Isaac 실측) — 일반 양배추는 집을 수도 없다", font=f(20), fill=SUB)
    save(img, "C_에셋실물비교", "C1_트레이_실물규격_축척.png", "트레이와 포기 크기 비교",
         "트레이 외곽은 육묘 트레이·표준 상자와 비슷하다. 칸 간격 150 mm 에는 미니 양배추(약 12 cm)까지 들어가고 일반 양배추(20 cm)는 겹친다.",
         "build_info.json, 범농, 국가법령정보센터 별표 2")
    render = ROOT / "notion_import_2026-09-27" / "images" / "asset_tray.jpg"
    if render.exists():
        im = fit(Image.open(render), 960, 640)
        save(im, "C_에셋실물비교", "C2_양배추트레이_렌더.jpg", "양배추 트레이 에셋 렌더",
             "cabbage_pallet_6 (랙용, 포기 6개). 포기 85~90 mm, 0.3 kg — 실측 밀도의 약 2배.", "scratchpad cmp_trays.jpg")


for fn in (a1_grip, a2_finger, a3_frames, a4_flow, a5_des, a6_retry, b1_carry, b2_breakdown, b3_sway, b4_track,
           b5_sequences, c_asset):
    fn()
with open(OUT / "captions.csv", "w", newline="", encoding="utf-8-sig") as fh:
    w = csv.DictWriter(fh, fieldnames=["file", "title", "caption", "source"])
    w.writeheader()
    w.writerows(CAPS)
lines = ["# 2026-09-27 실험 그림 모음", "", "슬라이드·문서용. 차트 1600x900 PNG, 장면 모음 JPG. 수치는 시뮬레이션 측정.",
         "다시 만들기: `D:\\isaacsim\\kit\\python\\python.exe D:\\smartfarm-sim\\scripts\\experiments\\make_figures.py`", ""]
for c in CAPS:
    lines += [f"## {c['title']}", "", f"![{c['title']}]({c['file']})", "", c["caption"], "", f"출처: {c['source']}", ""]
(OUT / "README.md").write_text("\n".join(lines), encoding="utf-8")
print(len(CAPS), "figures")
