#!/usr/bin/env python3
"""팀 nav2_params.yaml 을 읽어 시험용 복사본을 만든다 (팀 파일은 건드리지 않는다).

  python3 make_nav2_test_params.py SRC.yaml DST.yaml [--human] [--lane BT.xml]      (start_nav2_stack.sh 가 부른다)

--human  사람 돌발상황 (사람이 경로에 있으면 정지 -> 비키면 재출발)
  - 카터 윤곽에 뒤로 튀어나온 수확 트레이를 넣는다: 뒤끝 -0.607 -> -1.10 m
    (주행 중 트레이 중심은 chassis 뒤 0.80 m, 높이 1.02 m. 2026-09-25 실측)
  - collision_monitor HumanStop (stop): 후진 -0.62 ~ -1.90 m (트레이 끝에서 0.8 m), 전진 0.15 ~ 1.20 m, 폭 ±0.38 m.
    속도 0 근처(정지 유지)에는 반경 1.25 m 팔각형(트레이 회전 원 + 여유). 폭 ±0.38 은 랙 통로 팔레트 가장자리(중심에서 0.44 m) 안쪽.
    -Lane 을 켜면 FEEDER 도킹 구역에서 컨베이어가 이 영역에 들어오므로 lane_planner.py 가 HumanStop/HumanSlow 를 끈다(안전 영역 전환).
    (-Human 만이면 Nav2 가 FEEDER_APPROACH(y -1.55)에서 멈추고 영역 끝이 y -3.45 라 컨베이어 면 y -3.58 에 닿지 않는다)
  - collision_monitor HumanSlow (slowdown 50 %): 후진 -0.62 ~ -2.60 m, 전진 0.15 ~ 1.90 m, 폭 ±0.38 m
  - 장애물 지도 inf_is_valid (사람 자국 지우기), RPP use_collision_detection off, movement_time_allowance 30 s
--lane BT.xml  바닥 노란 차선 주행: bt_navigator 기본 BT 를 lane_route_bt.xml 로 (lane_planner.py 와 함께)
"""
import sys

import yaml

src, dst = sys.argv[1], sys.argv[2]
human = "--human" in sys.argv
lane_bt = sys.argv[sys.argv.index("--lane") + 1] if "--lane" in sys.argv else None
p = yaml.safe_load(open(src))
notes = []

if human:
    FOOT = "[[0.14, 0.25], [0.14, -0.25], [-1.10, -0.25], [-1.10, 0.25]]"
    for cm in ("local_costmap", "global_costmap"):
        p[cm][cm]["ros__parameters"]["footprint"] = FOOT

    def box(x_near, x_far, w=0.38):
        return f"[[{x_near}, {w}], [{x_near}, {-w}], [{x_far}, {-w}], [{x_far}, {w}]]"

    def vpoly(action, back, front, both, extra):
        return {"type": "velocity_polygon", "action_type": action, "min_points": 6, "visualize": True, "enabled": True,
                "holonomic": False, "velocity_polygons": ["rotation", "backward", "forward", "stopped"],
                "rotation": {"points": both, "linear_min": -0.03, "linear_max": 0.03, "theta_min": -10.0, "theta_max": 10.0},
                "backward": {"points": back, "linear_min": -1.0, "linear_max": -0.03, "theta_min": -10.0, "theta_max": 10.0},
                "forward": {"points": front, "linear_min": 0.03, "linear_max": 1.0, "theta_min": -10.0, "theta_max": 10.0},
                "stopped": {"points": both, "linear_min": -1.0, "linear_max": 1.0, "theta_min": -10.0, "theta_max": 10.0},
                **extra}

    # 제자리 회전·정지 유지: 트레이가 휩쓰는 원(반경 1.10 m) + 여유 = 반경 1.25 m 팔각형.
    # 앞뒤 긴 상자로 하면 모서리 회전 때 상자가 돌면서 옆 1 m 에 선 사람에 걸려 회전이 30 s 막힘(2026-09-25 시험)
    import math as _m
    ROT = "[" + ", ".join(f"[{1.25 * _m.cos(a * _m.pi / 4 + _m.pi / 8):.3f}, {1.25 * _m.sin(a * _m.pi / 4 + _m.pi / 8):.3f}]"
                          for a in range(8)) + "]"

    cmp = p["collision_monitor"]["ros__parameters"]
    cmp["polygons"] = list(cmp["polygons"]) + ["HumanStop", "HumanSlow"]
    cmp["HumanStop"] = vpoly("stop", box(-0.62, -1.90), box(1.20, 0.15), ROT,
                             {"polygon_pub_topic": "human_stop_polygon"})
    cmp["HumanSlow"] = vpoly("slowdown", box(-0.62, -2.60), box(1.90, 0.15), ROT,
                             {"polygon_pub_topic": "human_slow_polygon", "slowdown_ratio": 0.5})

    for cm in ("local_costmap", "global_costmap"):
        cp = p[cm][cm]["ros__parameters"]
        for layer in cp["plugins"]:
            names = cp.get(layer, {}).get("observation_sources")
            for s in (names.split() if isinstance(names, str) else []):
                cp[layer][s]["inf_is_valid"] = True
    p["controller_server"]["ros__parameters"]["FollowPath"]["use_collision_detection"] = False
    pc = p["controller_server"]["ros__parameters"].setdefault("progress_checker", {})
    pc["movement_time_allowance"] = max(float(pc.get("movement_time_allowance", 10.0)), 30.0)
    notes.append("human (footprint -1.10, HumanStop -1.90, HumanSlow -2.60)")

if lane_bt:
    p["bt_navigator"]["ros__parameters"]["default_nav_to_pose_bt_xml"] = lane_bt
    # 도착 판정기는 팀 것 하나만 둔다. 둘 이상이면 판정기 이름을 안 주는 FollowPath 가 전부 거부된다(2026-09-25 시험).
    notes.append(f"lane BT {lane_bt}")

yaml.safe_dump(p, open(dst, "w"), allow_unicode=True, sort_keys=False)
print(f"nav2 params -> {dst}: " + ", ".join(notes or ["unchanged"]))
