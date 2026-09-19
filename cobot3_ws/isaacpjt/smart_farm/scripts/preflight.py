import os
import sys


def preflight_check():
    report = {"python_version": sys.version.split()[0], "checks": {}}

    def try_import(mod_name):
        try:
            __import__(mod_name)
            return True, None
        except ImportError as e:
            return False, str(e)

    for mod in ["pxr", "pxr.Usd", "pxr.UsdGeom", "pxr.UsdPhysics", "pxr.UsdShade"]:
        report["checks"][mod] = try_import(mod)

    report["checks"]["PhysxSchema"] = try_import("pxr.PhysxSchema")
    report["checks"]["omni (Isaac Sim/Kit)"] = try_import("omni")
    report["env_ISAAC_PATH"] = os.environ.get("ISAAC_PATH") or os.environ.get("EXP_PATH")
    return report


def run_preflight_or_raise(require_physx: bool = False) -> dict:
    report = preflight_check()

    print("=" * 60)
    print("사전 점검: Python 환경")
    print("=" * 60)
    print(f"Python 버전: {report['python_version']}")
    print(f"실행 파일: {sys.executable}")

    core_mods = ["pxr", "pxr.Usd", "pxr.UsdGeom", "pxr.UsdPhysics", "pxr.UsdShade"]
    missing_core = []
    for mod in core_mods:
        ok, err = report["checks"][mod]
        print(f"  [{'OK' if ok else '실패'}] {mod}" + (f" -> {err}" if not ok else ""))
        if not ok:
            missing_core.append(mod)

    if missing_core:
        print("-" * 60)
        print(f"[오류] 필수 모듈 누락: {missing_core}")
        print("  usd-core가 설치된 venv를 activate했는지, 또는 Isaac Sim의 python 환경인지 확인하세요.")
        raise RuntimeError("필수 pxr 모듈 누락 — 실행 환경을 확인하세요.")

    physx_ok, physx_err = report["checks"]["PhysxSchema"]
    print(f"  [{'OK' if physx_ok else '경고'}] PhysxSchema" + (f" -> {physx_err}" if not physx_ok else ""))
    if not physx_ok:
        print("    -> Isaac Sim 환경이 아닙니다. convexDecomposition 세부 튜닝은 건너뜁니다.")
        print("    -> 단일 convexHull 적용에는 영향 없습니다 (이 컴퓨터에서의 검증 목적엔 문제 없음).")
        if require_physx:
            raise RuntimeError("PhysxSchema가 필요한 옵션이 설정됐지만 사용할 수 없습니다.")

    omni_ok, omni_err = report["checks"]["omni (Isaac Sim/Kit)"]
    print(f"  [{'OK' if omni_ok else '정보'}] omni (Isaac Sim/Kit)" + (f" -> {omni_err}" if not omni_ok else ""))
    if not omni_ok:
        print("    -> Kit 프로세스 밖입니다. USD 파일 편집 자체는 정상 동작합니다.")

    print("=" * 60)
    return report


if __name__ == "__main__":
    run_preflight_or_raise(require_physx=False)
