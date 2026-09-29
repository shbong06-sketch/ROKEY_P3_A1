"""실측 전에 내피에서 한 번 돌리는 도킹 회귀 시험.

Isaac 대신 `fake_robot.py` 를 띄우고 현장과 같은 launch 로 Nav2·도킹을 올린 뒤,
시나리오마다 도킹을 시작시켜 결과를 기댓값과 대조한다. 실측에서 한 번이라도
막혔던 상황은 모두 시나리오로 남겨 두어, 고친 것이 다시 깨지면 여기서 걸리게 한다.

    python3 dock_regression.py             # 전체
    python3 dock_regression.py -k jam      # 이름에 jam 이 들어간 것만
    python3 dock_regression.py --list      # 목록만 보기

실패한 시나리오가 하나라도 있으면 종료 코드가 1 이다.
"""
import argparse
import json
import os
import signal
import subprocess
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String

HERE = os.path.dirname(os.path.abspath(__file__))
FAKE = os.path.join(HERE, "fake_robot.py")
RTF = 0.33                  # 고피 실측에서 관찰된 실시간 배율
APPROACH = (-2.19, -1.55, 90.0)     # FEEDER_APPROACH
CORRIDOR = (-0.421, 1.006, 90.0)    # 초기 위치

# (이름, 시작 자세, fake_robot 추가 인자, 기대 status, 기대 reason, 설명)
# 차체 응답은 fake_robot 기본값이 실측형(24차)이다. --ideal 은 즉시 응답, --gain-inplace 0.12 는 22차형.
SCENARIOS = [
    ("normal",      APPROACH,               [],                        "SUCCEEDED", "NONE", "접근 자세에서 정상 도킹 (실측형 차체)"),
    ("yaw_pos_20",  (-2.19, -1.55, 110.0),  [],                        "SUCCEEDED", "NONE", "도착 방향이 +20도 틀어짐"),
    ("yaw_neg_30",  (-2.19, -1.55, 60.0),   [],                        "SUCCEEDED", "NONE", "도착 방향이 −30도 틀어짐 (24차 실측 시작각)"),
    ("lat_left",    (-2.39, -1.55, 90.0),   [],                        "SUCCEEDED", "NONE", "접근 지점이 0.2 m 왼쪽"),
    ("lat_right",   (-1.99, -1.55, 90.0),   [],                        "SUCCEEDED", "NONE", "접근 지점이 0.2 m 오른쪽"),
    ("far",         (-2.19, -1.15, 90.0),   [],                        "SUCCEEDED", "NONE", "접근 지점이 0.4 m 뒤"),
    ("weak_turn",   (-2.19, -1.55, 105.0),  ["--gain-inplace=0.12"],   "SUCCEEDED", "NONE", "제자리 회전이 거의 안 되는 차체 (22차)"),
    ("ideal",       (-2.19, -1.55, 75.0),   ["--ideal"],               "SUCCEEDED", "NONE", "명령을 즉시 따르는 이상적 차체"),
    ("jam",         APPROACH,               ["--jam=8"],               "FAILED", "STALLED_REVERSE", "후진 중 구조물에 끼임"),
    ("no_face",     CORRIDOR,               [],                        "FAILED", "FACE_NOT_FOUND", "면이 안 보이는 곳에서 시작"),
]

TOL_DIST = 0.05     # 면까지 거리 허용 오차
TOL_YAW = 3.0       # 방향 허용 오차(도). 도킹 노드의 square_tol_deg 와 같다
TOL_LAT = 0.08      # 좌우 허용 오차. 노드가 받아들이는 기준은 lat_tol_m 0.06 이고, 마지막 CREEP 에서
#                     조금 더 움직이므로 여기서는 0.02 m 여유를 둔다. 이 값을 넘으면 실제로 고장이다.


class Probe(Node):
    """준비 상태를 보고 도킹을 시작시킨 뒤 결과 한 건을 받아 온다."""

    def __init__(self, run_id):
        super().__init__("dock_regression_probe")
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.run_id = run_id
        self.result = None
        self.scan_seen = False
        self.create_subscription(LaserScan, "/scan", self._on_scan, qos_profile_sensor_data)   # /scan 은 BEST_EFFORT
        self.create_subscription(String, "/feeder_dock/result", self._on_result, latched)
        self.start_pub = self.create_publisher(String, "/feeder_dock/start", 10)

    def _on_scan(self, _):
        self.scan_seen = True

    def _on_result(self, m):
        try:
            data = json.loads(m.data)
        except ValueError:
            return
        if data.get("run_id") == self.run_id:      # 래치된 지난 결과를 받지 않는다
            self.result = data

    def wait(self, cond, timeout, step=0.2):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=step)
            if cond():
                return True
        return False


def launch(cmd, log_path):
    log = open(log_path, "w")
    return subprocess.Popen(cmd, stdout=log, stderr=subprocess.STDOUT,
                            start_new_session=True), log


def shutdown(procs):
    for p, _ in procs:
        try:
            os.killpg(p.pid, signal.SIGINT)
        except ProcessLookupError:
            pass
    time.sleep(5)
    for p, log in procs:
        try:
            os.killpg(p.pid, signal.SIGTERM)
        except ProcessLookupError:
            pass
        log.close()


def judge(expect_status, expect_reason, data, standoff):
    """기댓값과 대조해 (통과 여부, 사유) 를 돌려준다."""
    if data is None:
        return False, "결과 없음 (시간 초과)"
    status, reason = data.get("status"), data.get("reason")
    if status != expect_status or reason != expect_reason:
        return False, f"{status}/{reason} (기대 {expect_status}/{expect_reason})"
    if status != "SUCCEEDED":
        return True, reason
    dist, yaw, lat = data.get("face_dist_m"), data.get("yaw_err_deg"), data.get("lat_m")
    if dist is None or yaw is None or lat is None:
        return False, "성공인데 자세 값이 비어 있음"
    if abs(dist - standoff) > TOL_DIST:
        return False, f"면까지 {dist:.3f} m (기대 {standoff:.2f}±{TOL_DIST})"
    if abs(yaw) > TOL_YAW:
        return False, f"방향 오차 {yaw:.2f}도 (허용 {TOL_YAW})"
    if abs(lat) > TOL_LAT:
        return False, f"좌우 {lat:.3f} m (허용 {TOL_LAT})"
    return True, f"면 {dist:.3f} m, 방향 {yaw:+.2f}도, 좌우 {lat:+.3f} m"


def read_standoff():
    """도킹 노드가 실제로 쓰는 standoff 를 읽는다. 기본값이 바뀌어도 시험이 따라간다."""
    try:
        out = subprocess.run(["ros2", "param", "get", "/feeder_dock", "standoff_m"],
                             capture_output=True, text=True, timeout=20).stdout
        return float(out.strip().split()[-1])
    except (ValueError, IndexError, subprocess.SubprocessError):
        return 0.85


def run_one(name, start, extra, expect_status, expect_reason, log_dir, ready_timeout, dock_timeout):
    tag = f"{name}"
    fake_cmd = [sys.executable, FAKE, f"--rtf={RTF}",
                "--start=" + ",".join(str(v) for v in start)] + list(extra)
    nav_cmd = ["ros2", "launch", "smart_farm_navigation", "nav2.launch.py",
               "use_rviz:=false", "record:=false",
               f"initial_x:={start[0]}", f"initial_y:={start[1]}",
               f"initial_yaw_deg:={start[2]}"]

    procs = [launch(fake_cmd, os.path.join(log_dir, f"{tag}_fake.log"))]
    time.sleep(2)
    procs.append(launch(nav_cmd, os.path.join(log_dir, f"{tag}_nav2.log")))

    rclpy.init()
    probe = Probe(tag)
    try:
        if not probe.wait(lambda: probe.scan_seen, ready_timeout):
            return False, "준비 실패 (/scan 이 오지 않음)"
        time.sleep(5)                       # 자세 추정이 자리를 잡을 시간
        standoff = read_standoff()
        for _ in range(3):                  # 구독자가 붙기 전 발행을 놓치지 않게
            probe.start_pub.publish(String(data=tag))
            probe.wait(lambda: probe.result is not None, 2.0)
            if probe.result is not None:
                break
        probe.wait(lambda: probe.result is not None, dock_timeout)
        return judge(expect_status, expect_reason, probe.result, standoff)
    finally:
        probe.destroy_node()
        rclpy.shutdown()
        shutdown(procs)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("-k", default="", help="이름에 이 글자가 들어간 시나리오만")
    ap.add_argument("--list", action="store_true")
    ap.add_argument("--log-dir", default=os.path.expanduser("~/.ros/smart_farm_navigation/regression"))
    ap.add_argument("--ready-timeout", type=float, default=120.0)
    ap.add_argument("--dock-timeout", type=float, default=300.0)
    args = ap.parse_args()

    picked = [s for s in SCENARIOS if args.k in s[0]]
    if args.list or not picked:
        for name, start, extra, st, rs, desc in SCENARIOS:
            print(f"  {name:12s} {desc}  → {st}/{rs}")
        return 0 if picked or args.list else 1

    os.makedirs(args.log_dir, exist_ok=True)
    print(f"기록: {args.log_dir}\n")
    rows = []
    for i, (name, start, extra, st, rs, desc) in enumerate(picked, 1):
        print(f"[{i}/{len(picked)}] {name} — {desc} ... ", end="", flush=True)
        began = time.monotonic()
        ok, detail = run_one(name, start, extra, st, rs, args.log_dir,
                             args.ready_timeout, args.dock_timeout)
        print(f"{'통과' if ok else '실패'} ({time.monotonic() - began:.0f}s)")
        rows.append((name, ok, detail))
        time.sleep(3)

    print("\n" + "=" * 78)
    for name, ok, detail in rows:
        print(f"  {'통과' if ok else '실패'}  {name:12s} {detail}")
    failed = [n for n, ok, _ in rows if not ok]
    print("=" * 78)
    print(f"{len(rows) - len(failed)}/{len(rows)} 통과" + (f" — 실패: {', '.join(failed)}" if failed else ""))
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
