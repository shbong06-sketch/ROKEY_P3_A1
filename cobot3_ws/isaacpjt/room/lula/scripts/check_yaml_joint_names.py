"""robot_description.yaml의 cspace: 조인트 이름이 URDF와 실제로 일치하는지 확인.
Isaac Sim 불필요 -- 순수 텍스트/정규식 파싱, 아무 데서나 실행 가능.

배경: Lula의 robot_description.yaml 포맷은 cspace: 리스트가 URDF의 <joint name="..">와
문자 그대로 일치해야 한다고 파일 포맷 자체 주석에 명시돼 있다(NVIDIA 공식 예시 확인함).
Robot Description Editor는 Stage에 이미 올라온 USD Articulation을 보고 조인트 목록을 만드는데,
우리 USD DOF 이름은 "joint_1_joint"처럼 URDF("joint_1")와 접미사가 다르다. Editor가 USD 쪽
이름을 그대로 cspace에 써버리면 Lula가 URDF에서 그 이름을 못 찾아 로드 자체가 실패한다.

사용법:
  python3 check_yaml_joint_names.py --yaml <robot_description.yaml 경로>
  (--urdf 생략 시 lula/dsr_description2/urdf/m0617.urdf를 기본으로 씀)

미스매치가 "_joint 접미사 차이"처럼 단순 1:1 치환으로 고칠 수 있는 패턴이면
--fix 옵션으로 그 자리에서 바로 고쳐준다(원본은 .bak로 백업).
"""
import argparse
import os
import re
import shutil
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
DEFAULT_URDF = os.path.join(HERE, "..", "dsr_description2", "urdf", "m0617.urdf")


def parse_urdf_joint_names(urdf_path: str):
    text = open(urdf_path).read()
    return [m.group(1) for m in re.finditer(r'<joint name="(joint_\d)" type="revolute">', text)]


def parse_yaml_cspace(yaml_path: str):
    """cspace: 아래 '- xxx' 형태의 리스트 항목만 뽑는다. 범용 YAML 파서 없이도
    Lula robot_description.yaml의 cspace 블록은 이 단순 포맷을 따른다."""
    text = open(yaml_path).read()
    m = re.search(r'^cspace:\s*\n((?:\s*-\s*\S+\s*\n?)+)', text, re.MULTILINE)
    if not m:
        return [], text
    block = m.group(1)
    names = [line.strip().lstrip("-").strip() for line in block.strip().splitlines()]
    return names, text


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--yaml", required=True)
    ap.add_argument("--urdf", default=DEFAULT_URDF)
    ap.add_argument("--fix", action="store_true", help="단순 1:1 치환으로 고칠 수 있으면 그 자리에서 수정")
    args = ap.parse_args()

    urdf_names = parse_urdf_joint_names(os.path.abspath(args.urdf))
    print(f"[URDF] {args.urdf} 의 조인트: {urdf_names}")
    if len(urdf_names) != 6:
        print(f"FAIL: URDF에서 6개 조인트를 못 찾음 (찾은 개수: {len(urdf_names)}) -- URDF 경로 확인")
        sys.exit(1)

    yaml_path = os.path.abspath(args.yaml)
    if not os.path.isfile(yaml_path):
        print(f"FAIL: {yaml_path} 없음")
        sys.exit(1)
    cspace_names, full_text = parse_yaml_cspace(yaml_path)
    print(f"[YAML] {args.yaml} 의 cspace: {cspace_names}")

    if not cspace_names:
        print("FAIL: yaml에서 cspace: 블록을 못 찾음 -- Editor에서 export가 제대로 됐는지 확인")
        sys.exit(1)

    if cspace_names == urdf_names:
        print("\n=== PASS -- cspace 이름이 URDF와 완전히 일치함, 바로 LulaKinematicsSolver 써도 됨 ===")
        sys.exit(0)

    # 단순 접미사 차이인지 확인 (예: joint_1_joint -> joint_1)
    suffix_fixable = (
        len(cspace_names) == len(urdf_names)
        and all(c.startswith(u) for c, u in zip(cspace_names, urdf_names))
    )

    print("\n=== MISMATCH ===")
    for i, (c, u) in enumerate(zip(cspace_names, urdf_names), 1):
        mark = "OK" if c == u else "DIFF"
        print(f"  [{i}] yaml='{c}'  urdf='{u}'  {mark}")

    if not suffix_fixable:
        print("\n단순 접미사 패턴이 아님 -- 자동 수정 대상 아님, yaml을 직접 열어서 확인 필요")
        sys.exit(1)

    print("\n단순 접미사 차이로 보임 (예: 'joint_1_joint' -> 'joint_1') -- 1:1 치환으로 고칠 수 있음")
    if not args.fix:
        print("--fix 옵션 없이 실행함, 실제 파일은 안 건드림. 고치려면 --fix 붙여서 재실행.")
        sys.exit(1)

    backup = yaml_path + ".bak"
    shutil.copy2(yaml_path, backup)
    new_text = full_text
    for c, u in zip(cspace_names, urdf_names):
        # cspace 블록 안 "- c" 형태만 치환 (다른 곳에 같은 문자열이 있어도 안 건드리도록 앞의 '- ' 포함해서 매치)
        new_text = new_text.replace(f"- {c}\n", f"- {u}\n", 1)
    with open(yaml_path, "w") as f:
        f.write(new_text)
    print(f"\n수정 완료. 원본은 {backup}에 백업됨. 다시 이 스크립트를 --fix 없이 실행해서 PASS 확인할 것.")


if __name__ == "__main__":
    main()
