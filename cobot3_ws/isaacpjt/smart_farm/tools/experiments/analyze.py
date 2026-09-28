"""사업용 시뮬레이션 실험 분석.

  python analyze.py gate ROOT   -> 실험 0 속도 판정, 'PASS=0.3,0.45' 출력
  python analyze.py all  ROOT   -> ROOT/results/ 에 runs.csv, station_runs.csv, summary.json, des_inputs.json,
                                   throughput.csv, report.md

시간은 모두 시뮬레이션 시간(초). 흐름 명령(flow.json)은 벽시계로 기록되므로 monitor.json 의 chassis_track
(t_sim, x, y, yaw, roll, pitch, 벽시계)으로 벽시계 -> 시뮬레이션 시간을 보간한다.
데이터 등급: S = 시뮬레이션 측정(코드 설정값의 결과), A = 가정, D = 문헌·데이터시트.
"""
import csv
import json
import math
import re
import statistics as st
import sys
from pathlib import Path

# 사전 고정 판정 기준
CRIT_TILT_DEG = 3.0          # 주행 중 차체 기울기
CRIT_FAIL_RATE = 0.05        # 선별 버림 실패율
DEFECT_RATES = [0.05, 0.10, 0.20]   # A: 불량률 가정
SLOTS = 6


def load(p, default=None):
    try:
        return json.loads(Path(p).read_text(encoding="utf-8-sig"))
    except Exception:
        return default


def wall_to_sim(track):
    pts = [(r[6], r[0]) for r in track if len(r) >= 7]
    if len(pts) < 2:
        return None

    def f(w):
        if w <= pts[0][0]:
            return pts[0][1] - (pts[0][0] - w)          # 앞쪽은 1:1 로 외삽 (READY 전)
        for (w0, s0), (w1, s1) in zip(pts, pts[1:]):
            if w0 <= w <= w1:
                return s0 + (s1 - s0) * (w - w0) / max(w1 - w0, 1e-6)
        return pts[-1][1]
    return f


def rng(v):
    v = [x for x in v if x is not None]
    if not v:
        return None
    return {"n": len(v), "mean": round(st.mean(v), 2), "min": round(min(v), 2), "max": round(max(v), 2),
            "sd": round(st.stdev(v), 2) if len(v) > 1 else 0.0}


def wilson(k, n, z=1.96):
    if n == 0:
        return None
    p = k / n
    d = 1 + z * z / n
    c = (p + z * z / (2 * n)) / d
    h = z * math.sqrt(p * (1 - p) / n + z * z / (4 * n * n)) / d
    return {"k": k, "n": n, "rate": round(p, 3), "ci95": [round(max(0, c - h), 3), round(min(1, c + h), 3)]}


def station_phases(events, t_place_done=None):
    ev = {}
    culls = []
    for e in events or []:
        name, t = e["event"], e["t"]
        if name.startswith("CULL_DONE"):
            culls.append(t)
        else:
            ev.setdefault(name, t)
    out = {}
    a, p, i = ev.get("ARRIVED_PUSH_START"), ev.get("PUSH_DONE_INSPECT_MOVE"), ev.get("INSPECTION_DONE")
    r, b = ev.get("RECHECK_DONE"), ev.get("PUSH_BACK_DONE_RELEASED")
    if a is not None and t_place_done is not None and 0 < a - t_place_done < 600:
        out["conveyor_s"] = round(a - t_place_done, 2)
    if a is not None and p is not None:
        out["push_s"] = round(p - a, 2)
    if p is not None and i is not None:
        out["inspect_s"] = round(i - p, 2)
    if i is not None:
        prev, per = i, []
        for t in culls:
            per.append(round(t - prev, 2))
            prev = t
        out["cull_total_s"] = round((culls[-1] - i) if culls else 0.0, 2)
        out["cull_per_head_s"] = per
        end_cull = culls[-1] if culls else i
        if r is not None:
            out["recheck_s"] = round(r - end_cull, 2)
        if b is not None:
            out["pushback_s"] = round(b - (r if r is not None else end_cull), 2)
    if a is not None and b is not None:
        out["station_total_s"] = round(b - a, 2)
    return out


def allinone_run(d):
    meta = load(d / "run_meta.json", {})
    done = load(d / "done.json", {})
    flow = load(d / "flow.json", {})
    mon = load(d / "monitor.json", {})
    row = {"run": d.name, "exp": meta.get("exp"), "route": meta.get("route"), "speed": meta.get("speed"),
           "human": meta.get("human"), "rep": meta.get("rep"), "finished": done.get("finished"), "timeout": done.get("timeout")}
    track = mon.get("chassis_track", [])
    f = wall_to_sim(track)
    evs = flow.get("events", [])

    def t_of(pattern, nth=0):
        hits = [w for w, txt in evs if re.search(pattern, txt)]
        return f(hits[nth]) if f and len(hits) > nth else None

    pc, pr = t_of(r"sim command PICK_HARVEST"), t_of(r"sim result", 0)
    nc, nr = t_of(r"navigation command"), t_of(r"navigation result")
    lc, lr = t_of(r"sim command PLACE_INSPECT"), t_of(r"sim result", 1)
    row["pick_ok"] = (flow.get("pick") or {}).get("status") == "SUCCEEDED"
    row["nav_ok"] = (flow.get("nav") or {}).get("status") == "SUCCEEDED"
    row["place_ok"] = (flow.get("place") or {}).get("status") == "SUCCEEDED"
    row["pick_s"] = round(pr - pc, 2) if pc is not None and pr is not None else None
    row["nav_s"] = round(nr - nc, 2) if nc is not None and nr is not None else None
    row["place_s"] = round(lr - lc, 2) if lc is not None and lr is not None else None
    # 주행 구간 거리·기울기
    if nc is not None and nr is not None:
        seg = [r for r in track if nc <= r[0] <= nr]
        dist = sum(math.hypot(b[1] - a[1], b[2] - a[2]) for a, b in zip(seg, seg[1:]))
        row["nav_dist_m"] = round(dist, 2)
        row["nav_eff_speed_mps"] = round(dist / row["nav_s"], 3) if row["nav_s"] else None
        moving = [r[0] for a, r in zip(seg, seg[1:]) if math.hypot(r[1] - a[1], r[2] - a[2]) > 0.02]
        row["drive_s"] = round(moving[-1] - moving[0] + 1.0, 1) if moving else None
        tilts = [s[1] for s in mon.get("tilt_samples_over_0p5deg", []) if nc <= s[0] <= nr]
        row["tilt_nav_deg"] = round(max(tilts), 2) if tilts else 0.5
    row["tilt_all_deg"] = mon.get("tilt_max_deg")
    # 스테이션
    ev = load(d / "station" / "station_events.json", [])
    res = load(d / "station" / "station_results.json", [])
    row.update(station_phases(ev, lr))
    if res:
        r0 = res[0]
        insp = r0.get("inspection", [])
        row["heads_seen"] = sum(1 for x in insp if x.get("label") not in (None, "", "empty", "missing", "unknown"))
        row["vision_correct"] = sum(1 for x in insp if x.get("label") == x.get("truth"))
        culls = r0.get("culls", [])
        row["culls"] = len(culls)
        row["cull_fail"] = sum(1 for c in culls if not c.get("in_box"))
    log = (d / "isaac.log").read_text(encoding="utf-8", errors="ignore") if (d / "isaac.log").exists() else ""
    row["cull_fail_log"] = log.count("버림 실패")
    # 사람
    he = load(d / "station" / "human_events.json", {})
    if he:
        e = {x["event"]: x for x in he.get("events", [])}
        if "STAND_IN_PATH" in e:
            row["human_gap_m"] = e["STAND_IN_PATH"].get("robot_gap")
        if "WALK_IN" in e and "ROBOT_RESUMED" in e:
            row["human_block_s"] = round(e["ROBOT_RESUMED"]["t"] - e["WALK_IN"]["t"], 2)
    parts = [row.get(k) for k in ("pick_s", "nav_s", "place_s", "conveyor_s", "station_total_s")]
    row["cycle_to_discharge_s"] = round(sum(parts), 2) if all(p is not None for p in parts) else None
    return row


def station_run(d):
    meta = load(d / "run_meta.json", {})
    res = load(d / "result.json", {})
    ev = load(d / "station_events.json", [])
    pat = meta.get("pattern", "")
    row = {"run": d.name, "pattern": pat, "defects": sum(1 for c in pat.split(",") if c.strip().upper() in ("Y", "B")),
           "exception": bool(res.get("exception")), "sim_seconds": res.get("sim_seconds")}
    row.update(station_phases(ev))
    st_res = res.get("station") or []
    if st_res:
        r0 = st_res[0]
        culls = r0.get("culls", [])
        row["culls"] = len(culls)
        row["cull_fail"] = sum(1 for c in culls if not c.get("in_box"))
        insp = r0.get("inspection", [])
        row["vision_correct"] = sum(1 for x in insp if x.get("label") == x.get("truth"))
        rc = r0.get("recheck", [])
        row["left_defects_after"] = sum(1 for x in rc if x.get("label") in ("yellow", "brown"))
    return row


def gate(root):
    ok = []
    for d in sorted(root.glob("e0_default_v*")):
        r = allinone_run(d)
        good = (r.get("nav_ok") and r.get("place_ok") and (r.get("tilt_nav_deg") or 99) <= CRIT_TILT_DEG
                and (r.get("heads_seen") or 0) == SLOTS)
        print(f"{d.name}: nav_ok={r.get('nav_ok')} place_ok={r.get('place_ok')} tilt_nav={r.get('tilt_nav_deg')} "
              f"heads_seen={r.get('heads_seen')} -> {'PASS' if good else 'FAIL'}")
        if good:
            ok.append(str(r["speed"]))
    print("PASS=" + ",".join(ok))


def write_csv(path, rows):
    keys = []
    for r in rows:
        for k in r:
            if k not in keys:
                keys.append(k)
    with open(path, "w", newline="", encoding="utf-8-sig") as f:
        w = csv.DictWriter(f, fieldnames=keys)
        w.writeheader()
        for r in rows:
            w.writerow({k: (json.dumps(v) if isinstance(v, list) else v) for k, v in r.items()})


def all_(root):
    out = root / "results"
    out.mkdir(exist_ok=True)
    runs = [allinone_run(d) for d in sorted(root.glob("e[012]_*")) if d.is_dir()]
    sruns = [station_run(d) for d in sorted(root.glob("e3_*")) if d.is_dir()]
    write_csv(out / "runs.csv", runs)
    write_csv(out / "station_runs.csv", sruns)

    good = [r for r in runs if r.get("nav_ok") and r.get("place_ok")]
    groups = {}
    for r in good:
        groups.setdefault(f"{r['route']}_v{r['speed']}{'_human' if r.get('human') else ''}", []).append(r)
    metrics = ["pick_s", "nav_s", "drive_s", "nav_dist_m", "nav_eff_speed_mps", "place_s", "conveyor_s", "push_s", "inspect_s",
               "cull_total_s", "recheck_s", "pushback_s", "station_total_s", "cycle_to_discharge_s", "tilt_nav_deg",
               "tilt_all_deg", "human_gap_m", "human_block_s"]
    cond = {k: {m: rng([r.get(m) for r in v]) for m in metrics if any(r.get(m) is not None for r in v)} for k, v in groups.items()}

    per_head = [t for r in sruns for t in (r.get("cull_per_head_s") or [])] + \
               [t for r in good for t in (r.get("cull_per_head_s") or [])]
    ncull = sum(r.get("culls") or 0 for r in sruns + good)
    nfail = sum(r.get("cull_fail") or 0 for r in sruns + good)
    fail = wilson(nfail, ncull)
    fixed = {m: rng([r.get(m) for r in sruns + good]) for m in ("push_s", "inspect_s", "pushback_s")}
    recheck = rng([r.get("recheck_s") for r in sruns + good if (r.get("culls") or 0) > 0])
    by_def = {}
    for r in sruns:
        by_def.setdefault(r["defects"], []).append(r.get("station_total_s"))
    by_def = {k: rng(v) for k, v in sorted(by_def.items())}
    conveyor = rng([r.get("conveyor_s") for r in good])
    vision_acc = wilson(sum(r.get("vision_correct") or 0 for r in sruns + good),
                        SLOTS * len([r for r in sruns + good if r.get("vision_correct") is not None]))

    judg = {
        "속도별 주행 판정(기울기 <= 3도, 도착·놓기 성공, 포기 6개 유지)": {
            k: bool(v.get("tilt_nav_deg") and v["tilt_nav_deg"]["max"] <= CRIT_TILT_DEG) for k, v in cond.items()},
        "선별 버림 실패율 5% 초과(개선 과제)": (fail or {}).get("rate", 0) > CRIT_FAIL_RATE if fail else None,
        "실패·중단 회차": [r["run"] for r in runs if not (r.get("nav_ok") and r.get("place_ok") and r.get("finished"))],
    }
    summary = {"conditions": cond, "station": {"cull_per_head_s": rng(per_head), "cull_fail": fail, "fixed_steps": fixed,
                                               "recheck_s": recheck, "station_total_by_defects": by_def, "conveyor_s": conveyor,
                                               "vision_accuracy_sim_images": vision_acc},
               "judgments": judg, "criteria": {"tilt_deg": CRIT_TILT_DEG, "cull_fail_rate": CRIT_FAIL_RATE}}
    (out / "summary.json").write_text(json.dumps(summary, ensure_ascii=False, indent=1), encoding="utf-8")

    # DES 입력 + 처리량
    ph = rng(per_head) or {"mean": None, "min": None, "max": None}
    des = {"note": "시뮬레이션 산출값(Isaac Sim 5.1, 설정 파라미터 기준). 실측 아님. 값은 [최소, 평균, 최대].",
           "grades": {"S": "시뮬레이션 측정", "A": "가정", "D": "문헌"},
           "robot": {}, "station": {
               "conveyor_s": {"S": [conveyor["min"], conveyor["mean"], conveyor["max"]] if conveyor else None},
               "fixed_s(push+inspect+pushback)": {"S": [round(sum(fixed[m]["min"] for m in fixed if fixed[m]), 2),
                                                         round(sum(fixed[m]["mean"] for m in fixed if fixed[m]), 2),
                                                         round(sum(fixed[m]["max"] for m in fixed if fixed[m]), 2)]},
               "cull_per_head_s": {"S": [ph["min"], ph["mean"], ph["max"]]},
               "recheck_s_if_any_defect": {"S": [recheck["min"], recheck["mean"], recheck["max"]] if recheck else None},
               "cull_fail_rate": {"S": fail},
               "defect_rate": {"A": DEFECT_RATES}},
           "return_leg": {"A": "복귀 시간 = 가는 길 주행시간(nav_s)과 같다고 가정 (복귀 주행 미시험)"}}
    for k, v in cond.items():
        des["robot"][k] = {m: {"S": [v[m]["min"], v[m]["mean"], v[m]["max"]]} for m in ("pick_s", "nav_s", "place_s") if m in v}
    (out / "des_inputs.json").write_text(json.dumps(des, ensure_ascii=False, indent=1), encoding="utf-8")

    rows = []
    fx = sum(fixed[m]["mean"] for m in fixed if fixed[m])
    for k, v in cond.items():
        if not all(m in v for m in ("pick_s", "nav_s", "place_s")) or "human" in k:
            continue
        robot = v["pick_s"]["mean"] + 2 * v["nav_s"]["mean"] + v["place_s"]["mean"]        # 복귀 = 가는 길 (A)
        for p in DEFECT_RATES:
            exp_def = SLOTS * p
            p_any = 1 - (1 - p) ** SLOTS
            station = fx + exp_def * (ph["mean"] or 0) + p_any * (recheck["mean"] if recheck else 0)
            for n in (1, 2, 3):
                thr = min(n * 3600 / robot, 3600 / station)
                rows.append({"condition": k, "defect_rate": p, "robots": n, "robot_cycle_s": round(robot, 1),
                             "station_cycle_s": round(station, 1), "trays_per_hour": round(thr, 1),
                             "bottleneck": "robot" if n * 3600 / robot < 3600 / station else "station"})
    write_csv(out / "throughput.csv", rows)

    # 보고서
    L = ["# 사업용 시뮬레이션 실험 결과 (자동 생성)", "",
         "모든 시간은 Isaac Sim 5.1 시뮬레이션 시간(초)이며 **코드에 설정된 속도·동작 파라미터의 결과(S 등급)** 다. 실측이 아니다.", "",
         "## 조건별 공정 시간 (평균 [최소~최대])", "",
         "| 조건 | n | 수확 | 주행(도킹 포함) | 주행거리 m | 유효속도 m/s | 놓기 | 컨베이어 | 스테이션 | 배출까지 | 주행 기울기 최대 ° |",
         "|---|---|---|---|---|---|---|---|---|---|---|"]

    def fm(v, m):
        x = v.get(m)
        return f"{x['mean']} [{x['min']}~{x['max']}]" if x else "-"
    for k, v in sorted(cond.items()):
        n = max((x["n"] for x in v.values() if isinstance(x, dict) and "n" in x), default=0)
        L.append(f"| {k} | {n} | {fm(v,'pick_s')} | {fm(v,'nav_s')} | {fm(v,'nav_dist_m')} | {fm(v,'nav_eff_speed_mps')} | "
                 f"{fm(v,'place_s')} | {fm(v,'conveyor_s')} | {fm(v,'station_total_s')} | {fm(v,'cycle_to_discharge_s')} | "
                 f"{(v.get('tilt_nav_deg') or {}).get('max', '-')} |")
    L += ["", "## 비전 스테이션", "",
          f"- 포기당 선별 시간: {ph}",
          f"- 선별 버림 실패: {fail}",
          f"- 불량 수별 스테이션 전체 시간: {by_def}",
          f"- 고정 단계(밀기·검사·되밀기): {fixed}",
          f"- 비전 판정 일치율(합성 이미지 기준, 실제 작물 정확도 아님): {vision_acc}", "",
          "## 처리량 (로봇 대수 × 불량률, 복귀 = 가는 길 가정)", "", "| 조건 | 불량률 | 로봇 | 로봇 사이클 s | 스테이션 사이클 s | 트레이/시간 | 병목 |", "|---|---|---|---|---|---|---|"]
    for r in rows:
        L.append(f"| {r['condition']} | {int(r['defect_rate']*100)}% | {r['robots']} | {r['robot_cycle_s']} | {r['station_cycle_s']} | "
                 f"{r['trays_per_hour']} | {r['bottleneck']} |")
    L += ["", "## 판정", "", "```", json.dumps(judg, ensure_ascii=False, indent=1), "```", "",
          "## 사업적으로 말할 수 있는 것 / 없는 것", "",
          "- 말할 수 있음: 공정 흐름 성립, 병목 위치, 설계 선택(속도·경로·사람 대응·로봇 대수)의 상대 비교, 불량률에 따른 스테이션 부하 변화.",
          "- 말할 수 없음: 고객 견적·처리량 보장·인건비 절감 수치(실물 보정 전), 안전 인증, 실제 작물 비전 정확도.",
          "- 실물 보정 항목: 수확 동작 시간, 컨베이어 속도, 실제 불량률, 포기 파지 성공률, 주행 최고속도·가감속, 사람 정지 반응 시간."]
    (out / "report.md").write_text("\n".join(L), encoding="utf-8")
    print(f"results -> {out} (allinone {len(runs)}, station {len(sruns)})")


if __name__ == "__main__":
    mode, root = sys.argv[1], Path(sys.argv[2])
    gate(root) if mode == "gate" else all_(root)
