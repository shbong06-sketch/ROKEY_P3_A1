"""STEP 1 -- URDF 단독 검증. Isaac Sim 불필요, 순수 파이썬으로 아무 데서나 실행 가능.

base_link -> joint_1 -> link_1 -> ... -> joint_6 -> link_6 체인이 안 끊겼는지,
각 조인트의 type/axis/limit이 정상 범위인지, urdf가 참조하는 mesh 파일이 실제로
존재하는지(= package:// 경로가 진짜 resolve되는지)를 확인한다.

실행: python3 test_urdf.py  (lula/scripts/ 안에서, 또는 아무 위치에서나 --urdf로 경로 지정)
Lula를 붙이기 전에 이 스크립트가 FAIL을 내면 Lula로 넘어가면 안 된다.
"""
import argparse
import os
import re
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
DEFAULT_URDF = os.path.join(HERE, "..", "dsr_description2", "urdf", "m0617.urdf")

EXPECTED_CHAIN = [
    ("base_link", "joint_1", "link_1"),
    ("link_1", "joint_2", "link_2"),
    ("link_2", "joint_3", "link_3"),
    ("link_3", "joint_4", "link_4"),
    ("link_4", "joint_5", "link_5"),
    ("link_5", "joint_6", "link_6"),
]


def parse_joints(urdf_text: str):
    joints = {}
    for m in re.finditer(
        r'<joint name="(joint_\d)" type="(\w+)">\s*'
        r'<parent link="([^"]+)"\s*/>\s*'
        r'<child link="([^"]+)"\s*/>\s*'
        r'<origin[^/]*/>\s*'
        r'<axis xyz="([^"]+)"\s*/>\s*'
        r'<limit effort="[^"]*" lower="([^"]*)" upper="([^"]*)" velocity="([^"]*)"',
        urdf_text,
    ):
        name, jtype, parent, child, axis, lower, upper, vel = m.groups()
        joints[name] = dict(
            type=jtype, parent=parent, child=child,
            axis=tuple(round(float(x)) for x in axis.split()),
            lower_deg=round(float(lower) * 180 / 3.141592653589793, 2),
            upper_deg=round(float(upper) * 180 / 3.141592653589793, 2),
        )
    return joints


def check_mesh_paths(urdf_dir: str, urdf_text: str):
    missing = []
    checked = 0
    for m in re.finditer(r'filename="package://dsr_description2/([^"]+)"', urdf_text):
        rel = m.group(1)
        checked += 1
        # urdf_dir = .../dsr_description2/urdf -> package root는 한 단계 위
        pkg_root = os.path.dirname(urdf_dir)
        if not os.path.isfile(os.path.join(pkg_root, rel)):
            missing.append(rel)
    return checked, missing


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--urdf", default=DEFAULT_URDF)
    args = ap.parse_args()

    urdf_path = os.path.abspath(args.urdf)
    print(f"[test_urdf] 대상: {urdf_path}")
    if not os.path.isfile(urdf_path):
        print("FAIL: URDF 파일이 없음")
        sys.exit(1)

    text = open(urdf_path).read()
    joints = parse_joints(text)

    ok = True

    # 1) 체인 연결성
    print("\n-- 체인 연결성 --")
    for parent, jname, child in EXPECTED_CHAIN:
        j = joints.get(jname)
        if j is None:
            print(f"FAIL: {jname} 파싱 안 됨 (정규식이 이 URDF 포맷과 안 맞을 수 있음)")
            ok = False
            continue
        good = (j["parent"] == parent and j["child"] == child)
        print(f"  {jname}: {j['parent']}->{j['child']} (기대 {parent}->{child}) {'OK' if good else 'FAIL'}")
        ok &= good

    # 2) 타입/축/리미트 정상 범위
    print("\n-- 타입/축/리미트 --")
    for jname, j in joints.items():
        type_ok = j["type"] == "revolute"
        axis_ok = j["axis"] in [(1, 0, 0), (0, 1, 0), (0, 0, 1)]
        limit_ok = j["lower_deg"] < j["upper_deg"]
        line_ok = type_ok and axis_ok and limit_ok
        print(f"  {jname}: type={j['type']} axis={j['axis']} "
              f"limit=[{j['lower_deg']}, {j['upper_deg']}]deg {'OK' if line_ok else 'FAIL'}")
        ok &= line_ok

    # 3) mesh 경로 resolve
    print("\n-- mesh 경로 resolve --")
    checked, missing = check_mesh_paths(os.path.dirname(urdf_path), text)
    print(f"  {checked}건 체크, 누락 {len(missing)}건")
    for r in missing:
        print("   MISSING:", r)
    ok &= (len(missing) == 0)

    print("\n===", "ALL PASS -- Lula로 넘어가도 됨" if ok else "FAIL 있음 -- Lula 붙이기 전에 위부터 고칠 것", "===")
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
