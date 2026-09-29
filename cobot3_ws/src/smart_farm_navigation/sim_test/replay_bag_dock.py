"""실측 bag 의 `/scan` 을 도킹 노드와 똑같은 검출기에 다시 넣어, 그때 제어기가 무엇을 보았는지 본다.

도킹이 이상하게 움직였을 때 "센서가 잘못 보았는가, 제어가 잘못 판단했는가" 를 가르는 도구다.
같은 `detect_face` 를 쓰므로 노드가 그 순간 본 값과 같고, 옆에 실제 `/cmd_vel` 과 odom 각속도를
나란히 찍어 준다. 24차 진단이 이 방법으로 끝났다.

    python3 replay_bag_dock.py ~/.ros/smart_farm_navigation/bags/nav2_20260923_2154
    python3 replay_bag_dock.py <bag> --from 291 --to 306      # bag 기록 시각 구간만

읽는 항목: /scan, /cmd_vel, /chassis/odom, /feeder_dock/status
"""
import argparse
import math
import os
import sys

from rclpy.serialization import deserialize_message
from rosbag2_py import ConverterOptions, SequentialReader, StorageOptions
from rosidl_runtime_py.utilities import get_message

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
from smart_farm_navigation.feeder_dock import DockParams, detect_face, wrap  # noqa: E402

SEARCH_X = [-3.4, -0.5]
SEARCH_Y = [-1.3, 1.3]


def read_bag(path):
    reader = SequentialReader()
    reader.open(StorageOptions(uri=path, storage_id="mcap"), ConverterOptions("", ""))
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    while reader.has_next():
        topic, data, stamp = reader.read_next()
        if topic in ("/scan", "/cmd_vel", "/chassis/odom", "/feeder_dock/status"):
            yield topic, deserialize_message(data, get_message(types[topic])), stamp * 1e-9


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("bag")
    ap.add_argument("--from", dest="t_from", type=float, default=0.0, help="시작 시각(초, bag 기록 시각). 0 이면 처음부터")
    ap.add_argument("--to", dest="t_to", type=float, default=0.0, help="끝 시각(초, bag 기록 시각). 0 이면 끝까지")
    args = ap.parse_args()

    p = DockParams()
    cmd_v = cmd_w = odom_wz = 0.0
    print("    t    면거리  방향오차  횡    |G|   G방위  점수  길이 | 실제 cmd_v cmd_w  odom_wz")
    for topic, msg, stamp in read_bag(args.bag):
        t = stamp                      # --from/--to 와 같은 눈금(bag 기록 시각)으로 찍는다
        if args.t_from and stamp < args.t_from:
            continue
        if args.t_to and stamp > args.t_to:
            break

        if topic == "/cmd_vel":
            cmd_v, cmd_w = msg.linear.x, msg.angular.z
            continue
        if topic == "/chassis/odom":
            odom_wz = msg.twist.twist.angular.z
            continue
        if topic == "/feeder_dock/status":
            print(f"{t:8.2f} --- {msg.data[:100]}")
            continue

        face = detect_face(msg.ranges, msg.angle_min, msg.angle_increment,
                           SEARCH_X, SEARCH_Y, (p.face_min_len_m, p.face_max_len_m),
                           seed=int(msg.header.stamp.nanosec) & 0xFFFF)
        if face is None:
            print(f"{t:8.2f} 면 검출 실패                                        "
                  f"| {cmd_v:+.3f} {cmd_w:+.3f} {odom_wz:+.3f}")
            continue
        dist, yaw_err, lat, cx, cy, n, length = face
        nx, ny = math.cos(yaw_err), math.sin(yaw_err)
        gx, gy = cx + nx * p.standoff_m, cy + ny * p.standoff_m
        g_dist = math.hypot(gx, gy)
        bearing = wrap(math.atan2(gy, gx) - math.pi)
        print(f"{t:8.2f} {dist:6.2f}  {math.degrees(yaw_err):+7.1f} {lat:+.3f} {g_dist:5.2f} "
              f"{math.degrees(bearing):+6.1f} {n:4d} {length:5.2f} "
              f"| {cmd_v:+.3f} {cmd_w:+.3f} {odom_wz:+.3f}")


if __name__ == "__main__":
    main()
