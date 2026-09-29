from pathlib import Path
from PIL import Image
import re
import shutil


ROOT = Path(
    "/home/rokey/ROKEY_P3_A1/cobot3_ws/isaacpjt/smart_farm/scenes/"
    "Collected_smartfarm_v006_lite"
).resolve()

TEXTURE_PATTERN = re.compile(r'texture_2d\(\s*"([^"]+)"')


def make_placeholder(path: Path) -> None:
    name = path.stem.lower()

    if "normal" in name or name.endswith("_n"):
        image = Image.new("RGB", (4, 4), (128, 128, 255))
    elif "orm" in name:
        # R=AO, G=roughness, B=metallic
        image = Image.new("RGB", (4, 4), (255, 170, 0))
    elif "rough" in name:
        image = Image.new("L", (4, 4), 170)
    elif "alpha" in name or "opacity" in name or "opasity" in name:
        image = Image.new("L", (4, 4), 255)
    elif "rubber" in name or "belt" in name or "black" in name:
        image = Image.new("RGB", (4, 4), (35, 35, 35))
    elif "glass" in name:
        image = Image.new("RGB", (4, 4), (170, 185, 195))
    else:
        image = Image.new("RGB", (4, 4), (140, 140, 140))

    path.parent.mkdir(parents=True, exist_ok=True)
    image.save(path, format="PNG", optimize=True, compress_level=9)


if not ROOT.is_dir():
    raise RuntimeError(f"lite 폴더가 없습니다: {ROOT}")

# 같은 이름으로 이미 수집된 텍스처를 우선 재사용합니다.
existing_by_name: dict[str, list[Path]] = {}
for png_path in ROOT.rglob("*.png"):
    existing_by_name.setdefault(png_path.name, []).append(png_path.resolve())

missing: list[tuple[Path, Path]] = []
for mdl_path in ROOT.rglob("*.mdl"):
    mdl_text = mdl_path.read_text(encoding="utf-8", errors="ignore")
    for reference in TEXTURE_PATTERN.findall(mdl_text):
        target = (mdl_path.parent / reference).resolve()
        if ROOT not in target.parents:
            raise RuntimeError(f"lite 폴더 밖을 가리키는 경로입니다: {target}")
        if not target.is_file():
            missing.append((mdl_path, target))

print(f"누락된 MDL 텍스처: {len(missing)}개")

for mdl_path, target in missing:
    candidates = [
        candidate
        for candidate in existing_by_name.get(target.name, [])
        if candidate != target
    ]

    target.parent.mkdir(parents=True, exist_ok=True)

    if candidates:
        shutil.copy2(candidates[0], target)
        action = f"기존 파일 복사: {candidates[0].relative_to(ROOT)}"
    else:
        make_placeholder(target)
        action = "4x4 대체 텍스처 생성"

    print(f"[FIXED] {target.relative_to(ROOT)}")
    print(f"        {action}")
    print(f"        referenced by {mdl_path.relative_to(ROOT)}")

# 실제로 모든 로컬 MDL 텍스처 참조가 해결됐는지 재검증합니다.
unresolved: list[Path] = []
for mdl_path in ROOT.rglob("*.mdl"):
    mdl_text = mdl_path.read_text(encoding="utf-8", errors="ignore")
    for reference in TEXTURE_PATTERN.findall(mdl_text):
        target = (mdl_path.parent / reference).resolve()
        if not target.is_file():
            unresolved.append(target)

print("\n===== 검증 결과 =====")
print(f"복구한 파일: {len(missing)}개")
print(f"남은 누락:   {len(unresolved)}개")

if unresolved:
    for path in unresolved:
        print(f"[UNRESOLVED] {path}")
    raise RuntimeError("해결되지 않은 MDL 텍스처가 있습니다.")

print("완료: Isaac Sim을 다시 실행하고 lite USD를 여세요.")
