from pathlib import Path
from PIL import Image
import os


ROOT = Path(
    "/home/rokey/ROKEY_P3_A1/cobot3_ws/isaacpjt/smart_farm/scenes/"
    "Collected_smartfarm_v006_lite"
)


def texture_kind(path: Path) -> str:
    name = path.stem.lower()
    if "normal" in name or name.endswith("_n"):
        return "normal"
    if "orm" in name:
        return "orm"
    if "rough" in name:
        return "roughness"
    if "metallic" in name or "metalness" in name:
        return "metallic"
    if any(word in name for word in ("alpha", "opacity", "opasity")):
        return "alpha"
    if "emissive" in name:
        return "emissive"
    return "color"


def simple_color(path: Path) -> tuple[int, int, int]:
    name = path.stem.lower()
    colors = {
        "orange": (210, 90, 20),
        "yellow": (220, 190, 30),
        "green": (50, 130, 60),
        "blue": (45, 90, 170),
        "red": (170, 45, 40),
        "black": (35, 35, 35),
        "white": (205, 205, 205),
        "steel": (120, 125, 130),
        "metal": (125, 125, 125),
        "iron": (105, 105, 105),
        "aluminium": (155, 155, 155),
        "aluminum": (155, 155, 155),
    }
    for word, color in colors.items():
        if word in name:
            return color
    return (140, 140, 140)


def save_png_atomic(image: Image.Image, path: Path) -> None:
    temporary = path.with_suffix(path.suffix + ".tmp")
    image.save(temporary, format="PNG", optimize=True, compress_level=9)
    os.replace(temporary, path)


def simplify(path: Path) -> tuple[int, int, str]:
    old_size = path.stat().st_size
    kind = texture_kind(path)

    with Image.open(path) as source:
        if kind == "normal":
            output = Image.new("RGB", (4, 4), (128, 128, 255))
        elif kind == "orm":
            output = Image.new("RGB", (4, 4), (255, 170, 0))
        elif kind == "roughness":
            output = Image.new("L", (4, 4), 170)
        elif kind == "metallic":
            output = Image.new("L", (4, 4), 0)
        elif kind == "emissive":
            output = Image.new("RGB", (4, 4), (0, 0, 0))
        elif kind == "alpha":
            output = source.convert("L")
            output.thumbnail((256, 256), Image.Resampling.LANCZOS)
            output = output.copy()
        elif "A" in source.getbands():
            # 투명 실루엣은 유지하고 RGB만 단색화합니다.
            alpha = source.getchannel("A")
            alpha.thumbnail((256, 256), Image.Resampling.LANCZOS)
            output = Image.new("RGBA", alpha.size, simple_color(path) + (255,))
            output.putalpha(alpha)
        else:
            output = Image.new("RGB", (8, 8), simple_color(path))

    save_png_atomic(output, path)
    return old_size, path.stat().st_size, kind


if not ROOT.is_dir():
    raise RuntimeError(f"lite 폴더가 없습니다: {ROOT}")

png_files = sorted(ROOT.rglob("*.png"))
if len(png_files) != 94:
    raise RuntimeError(
        f"예상한 PNG 94개가 아니라 {len(png_files)}개입니다. ROOT를 확인하세요."
    )

before = 0
after = 0
failed = []

for number, png_path in enumerate(png_files, 1):
    try:
        old_size, new_size, kind = simplify(png_path)
        before += old_size
        after += new_size
        print(
            f"[{number:02d}/94] {kind:9s} "
            f"{old_size / 1048576:7.2f} -> {new_size / 1024:7.1f} KiB  "
            f"{png_path.name}"
        )
    except Exception as error:
        failed.append((png_path, error))
        print(f"[FAILED] {png_path}: {error}")

print("\n===== 실제 변환 결과 =====")
print(f"변환 전: {before / 1048576:.1f} MiB")
print(f"변환 후: {after / 1048576:.1f} MiB")
print(f"절감량:   {(before - after) / 1048576:.1f} MiB")
print(f"실패:     {len(failed)}개")

if failed:
    raise RuntimeError("일부 PNG 변환에 실패했습니다. 위의 [FAILED] 항목을 확인하세요.")

print("완료: Isaac Sim을 종료한 뒤 lite USD를 다시 여세요.")
