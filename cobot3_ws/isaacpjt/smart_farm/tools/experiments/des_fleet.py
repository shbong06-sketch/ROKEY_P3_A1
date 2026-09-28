"""Fleet / station sizing DES fed by the 2026-09-26 simulation DB (+ grasp success from the 09-27 experiment).

  python des_fleet.py OUT_CSV [--q 0.97,0.99,1.0] [--hours 8] [--reps 20] [--inputs des_inputs.json] [--conds a,b]
  (표준 라이브러리만 사용. --inputs 가 없으면 스크립트 옆 des_inputs.json, 없으면 D:\\smartfarm-sim 경로)

Model (grades: S = simulation measurement, A = assumption)
  robot loop     pick (S) -> nav to FEEDER (S) -> place (S) -> return nav (A: = nav) -> pick ...
  conveyor       15.0 s transit (S), buffer of 3 trays between robots and stations (A); a robot waits at the
                 feeder while the buffer is full
  station        FIFO, service = 18.8 s + 32.4 s x culls (S fit, 17 standalone + 13 flow runs);
                 defects per tray ~ Binomial(6, d) (A: d = 5/10/20 %)
  cull           each attempt succeeds with q; policy 'none' = a failed head stays (escaped defect),
                 policy 'retry' = one more attempt (+32.4 s)
Times: triangular(min, mode, max) from des_inputs.json [min, mean, max] (mode chosen so the mean matches).
"""
import csv
import heapq
import json
import random
import statistics as st
import sys

from pathlib import Path

_local = Path(__file__).with_name("des_inputs.json")
_inputs = (sys.argv[sys.argv.index("--inputs") + 1] if "--inputs" in sys.argv
           else _local if _local.exists() else r"D:\smartfarm-sim\out\exp_20260926\results\des_inputs.json")
DES = json.load(open(_inputs, encoding="utf-8"))
BASE_S, PER_CULL, CONVEYOR_S, BUFFER = 18.8, 32.4, 15.0, 3


def tri(rng, lo, mean, hi):
    if hi - lo < 1e-6:
        return mean
    mode = min(max(3 * mean - lo - hi, lo), hi)
    return rng.triangular(lo, hi, mode)


def sample(rng, cond, key):
    lo, mean, hi = DES["robot"][cond][key]["S"]
    return tri(rng, lo, mean, hi)


def run(cond, robots, stations, d, q, policy, hours, seed):
    rng = random.Random(seed)
    T = hours * 3600.0
    ev = []                      # (time, seq, kind, data)
    seq = [0]

    def push(t, kind, data=None):
        seq[0] += 1
        heapq.heappush(ev, (t, seq[0], kind, data))

    queue, busy, buffer_used = [], 0, 0
    waiting_robots = []
    done_trays, heads_culled, escaped, station_busy_s, robot_wait_s = 0, 0, 0, 0.0, 0.0
    wait_start = {}
    for r in range(robots):
        push(rng.uniform(0, 5), "pick", r)

    def start_station(t):
        nonlocal busy, station_busy_s, heads_culled, escaped
        while busy < stations and queue:
            queue.pop(0)
            busy += 1
            k = sum(1 for _ in range(6) if rng.random() < d)
            dur, esc = BASE_S, 0
            for _ in range(k):
                dur += PER_CULL
                ok = rng.random() < q
                if not ok and policy == "retry":
                    dur += PER_CULL
                    ok = rng.random() < q
                esc += 0 if ok else 1
            heads_culled += k - esc
            escaped += esc
            station_busy_s += min(dur, max(0.0, T - t))
            push(t + dur, "station_done", None)

    while ev:
        t, _, kind, data = heapq.heappop(ev)
        if t > T:
            break
        if kind == "pick":
            push(t + sample(rng, cond, "pick_s") + sample(rng, cond, "nav_s"), "at_feeder", data)
        elif kind == "at_feeder":
            if buffer_used >= BUFFER:
                waiting_robots.append(data)
                wait_start[data] = t
                continue
            buffer_used += 1
            push(t + sample(rng, cond, "place_s"), "placed", data)
        elif kind == "placed":
            push(t + CONVEYOR_S, "tray_at_station", None)
            push(t + sample(rng, cond, "nav_s"), "pick", data)          # return trip (A)
        elif kind == "tray_at_station":
            queue.append(t)
            start_station(t)
        elif kind == "station_done":
            busy -= 1
            buffer_used -= 1
            done_trays += 1
            if waiting_robots:
                r = waiting_robots.pop(0)
                robot_wait_s += t - wait_start.pop(r)
                buffer_used += 1
                push(t + sample(rng, cond, "place_s"), "placed", r)
            start_station(t)
    return dict(trays_per_h=done_trays / hours, station_util=station_busy_s / (T * stations),
                robot_wait_frac=robot_wait_s / (T * robots), escaped_per_1000_trays=1000 * escaped / max(1, done_trays),
                culled=heads_culled)


def main():
    out = sys.argv[1]
    q_list = [float(v) for v in sys.argv[sys.argv.index("--q") + 1].split(",")] if "--q" in sys.argv else []
    hours = float(sys.argv[sys.argv.index("--hours") + 1]) if "--hours" in sys.argv else 8.0
    reps = int(sys.argv[sys.argv.index("--reps") + 1]) if "--reps" in sys.argv else 20
    qs = {"q_flow_0926": 36 / 39}
    for q in q_list:
        qs[f"q_{q}"] = q
    rows = []
    conds = (sys.argv[sys.argv.index("--conds") + 1].split(",") if "--conds" in sys.argv
             else ("default_v0.3", "default_v0.6"))
    for cond in conds:
        for robots in (1, 2, 3, 4, 5):
            for stations in (1, 2):
                for d in (0.05, 0.10, 0.20):
                    for qname, q in qs.items():
                        for policy in ("none", "retry"):
                            res = [run(cond, robots, stations, d, q, policy, hours, s) for s in range(reps)]
                            row = dict(cond=cond, robots=robots, stations=stations, defect_rate=d, q_name=qname, q=round(q, 4),
                                       policy=policy)
                            for k in res[0]:
                                vals = [r[k] for r in res]
                                row[k] = round(st.mean(vals), 3)
                                row[k + "_sd"] = round(st.pstdev(vals), 3)
                            rows.append(row)
    with open(out, "w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        w.writeheader()
        w.writerows(rows)
    print(f"{len(rows)} scenarios x {reps} reps -> {out}")


if __name__ == "__main__":
    main()
