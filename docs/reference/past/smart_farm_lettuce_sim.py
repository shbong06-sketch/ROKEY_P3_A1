"""
스마트팜 상추 생육확인 + 층옮김 시뮬레이션
=========================================
확정 사항 반영:
  1. 매니퓰레이터: Doosan M0609 + OnRobot RG2 (승강축 없음, B안 - 팔 리치 내에서 층간 해결)
     자산 출처: https://github.com/rebi00dev/Smartwarehouse (USD/m0609_rg2_final.usd)
     -> LICENSE 확인 완료, 그리퍼 조인트명 확인 완료
  2. 상추 성장단계: 1차는 크기/색상 기반 placeholder (구형 프리미티브)
     추후 팀 모델링/서드파티 SimReady 자산으로 교체 예정 (LETTUCE_ASSET_SOURCE 참고)
  3. SimsFactory(robot_move.py) 구조 재사용: reparent_with_world_pose, grasp_frame attach/detach,
     _detect_color 패턴을 _detect_maturity로 치환

실행 전 필수 확인 (3번 항목, 로딩 테스트 미완료 상태):
  - SMARTWAREHOUSE_ROOT 환경변수 또는 기본 경로(~/Smartwarehouse)에 저장소가 클론되어 있어야 함
  - 최초 실행 시 art.dof_names를 반드시 출력해 조인트 인식 확인할 것 (diagnose_articulation 참고)
"""

from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": False})

import os
import time
import random
import numpy as np
import cv2
from pxr import UsdLux, UsdGeom, Gf, UsdPhysics

from isaacsim.core.api.world import World
from isaacsim.core.api.objects import DynamicSphere
from isaacsim.core.prims import Articulation
from isaacsim.core.utils.stage import add_reference_to_stage
from isaacsim.core.utils.nucleus import get_assets_root_path

# =========================================================================
# 0. 전역 설정 (SimsFactory의 P1_*/P2_* 전역 상수 패턴)
# =========================================================================

ASSETS_ROOT = get_assets_root_path()

# --- 1. M0609+RG2 자산 경로 (Smartwarehouse 저장소, 로컬 클론 기준) ---
SMARTWAREHOUSE_REPO_ROOT = os.environ.get(
    "SMARTWAREHOUSE_ROOT",
    os.path.expanduser("~/Smartwarehouse"),
)
M0609_USD_PATH = os.path.join(SMARTWAREHOUSE_REPO_ROOT, "USD", "m0609_rg2_final.usd")

# 확인 완료(2번): 실제 USD 파일 안의 조인트명으로 교체
ARM_JOINT_NAMES = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]
GRIPPER_JOINT_NAME = "rg2_finger_joint"  # 확인된 그리퍼 조인트명으로 실제 값 교체

# --- 2. 베드/재배대 레이아웃 (승강 없이 팔 리치 안에서 해결 = B안) ---
BED_ROW_COUNT = 1              # 1개 row로 시작, 스윕 실험 시 확장
SLOTS_PER_ROW = 10             # 슬롯 수 — 6번 밀도 스윕에서 가변
SLOT_SPACING = 0.6             # 슬롯 간 간격(m)
MANIPULATOR_BASE_POS = (0.0, 0.0, 1.0)   # 성장층/수확대기층 중간 높이

GROWING_LAYER_Z = 0.8
READY_LAYER_Z = 1.2

# 관절각 프리셋 (실제 튜닝값으로 교체 필요 — 캡스톤 grasp pose 참고)
POSE_READY = [0, -20, 90, 0, 90, 0]
POSE_SCAN = [0, -10, 80, 0, 90, 0]
POSE_PICK = [10, 0, 70, 0, 100, 0]
POSE_LIFT_TO_READY = [10, -30, 60, 0, 80, 0]
POSE_PLACE_READY = [20, -20, 50, 0, 70, 0]

# --- 3. 상추 성장단계 (placeholder — LETTUCE_ASSET_SOURCE로 전환 관리) ---
LETTUCE_ASSET_SOURCE = "placeholder"   # "placeholder" | "team_model" | "third_party_simready"

STAGE_SCALE = {"seedling": 0.10, "growing": 0.18, "mature": 0.28}
STAGE_COLOR = {
    "seedling": (0.65, 0.90, 0.45),
    "growing": (0.35, 0.75, 0.25),
    "mature": (0.12, 0.55, 0.15),
}
STAGE_WEIGHTS = {"seedling": 0.4, "growing": 0.4, "mature": 0.2}

# 판별 임계값 (SimsFactory _detect_color의 200픽셀 매직넘버와 동일 성격 — 튜닝 필요)
MATURE_AREA_THRESHOLD = 1500
GROWING_AREA_THRESHOLD = 600


# =========================================================================
# 1. 좌표/USD 유틸리티 (SimsFactory 공통 헬퍼 재사용)
# =========================================================================

def prim_exists(stage, path):
    prim = stage.GetPrimAtPath(path)
    return prim.IsValid()


def get_world_matrix(stage, path):
    prim = stage.GetPrimAtPath(path)
    xform = UsdGeom.Xformable(prim)
    return xform.ComputeLocalToWorldTransform(0)


def move_prim(src, dst):
    import omni.kit.commands
    omni.kit.commands.execute("MovePrim", path_from=src, path_to=dst)


def reparent_with_world_pose(stage, src_path, dst_parent_path):
    """월드 포즈를 유지한 채 재부모화 (robot_move.py 핵심 패턴)"""
    world_m = get_world_matrix(stage, src_path)
    name = src_path.split("/")[-1]
    new_path = f"{dst_parent_path}/{name}"
    move_prim(src_path, new_path)

    parent_world_m = get_world_matrix(stage, dst_parent_path)
    local_m = world_m * parent_world_m.GetInverse()

    prim = stage.GetPrimAtPath(new_path)
    xf = UsdGeom.Xformable(prim)
    xf.ClearXformOpOrder()
    xf.AddTransformOp().Set(local_m)
    return new_path


def diagnose_articulation(art: Articulation):
    """3번(로딩 확인불가) 대응 — 실행 환경에서 가장 먼저 돌려볼 진단 함수"""
    print("=" * 60)
    print("[진단] 인식된 관절 목록:", art.dof_names)
    print("[진단] 관절 수:", art.num_dof)
    missing = [n for n in ARM_JOINT_NAMES + [GRIPPER_JOINT_NAME] if n not in art.dof_names]
    if missing:
        print("[경고] 다음 조인트명이 인식되지 않았습니다 — 실제 USD 조인트명과 대조 필요:", missing)
    else:
        print("[정상] 팔 6축 + 그리퍼 조인트 모두 인식됨")
    print("=" * 60)


# =========================================================================
# 2. 상추 스폰 (성장단계 placeholder)
# =========================================================================

def spawn_lettuce(stage_key, prim_path, position):
    DynamicSphere(
        prim_path=prim_path,
        position=np.array(position),
        radius=STAGE_SCALE[stage_key],
        color=np.array(STAGE_COLOR[stage_key]),
    )
    return stage_key  # ground-truth 라벨로 반환 (지표 검증용)


def populate_growing_layer(bed_row_idx, slot_count):
    """성장층 슬롯에 무작위 성장단계로 상추 스폰. ground-truth 딕셔너리 반환"""
    ground_truth = {}
    for i in range(slot_count):
        stage_key = random.choices(
            list(STAGE_WEIGHTS.keys()), weights=list(STAGE_WEIGHTS.values())
        )[0]
        slot_path = f"/World/Bed_Row_{bed_row_idx}/Layer_Growing/Lettuce_{i}"
        position = (i * SLOT_SPACING, 1.5, GROWING_LAYER_Z)
        spawn_lettuce(stage_key, slot_path, position)
        ground_truth[slot_path] = stage_key
    return ground_truth


# =========================================================================
# 3. 생육 판별 (SimsFactory _detect_color 대응)
# =========================================================================

def get_camera_image(camera_sensor):
    """카메라 센서에서 RGB 이미지 획득 (isaacsim.sensor.Camera 사용 가정)"""
    frame = camera_sensor.get_rgba()[:, :, :3]
    if frame.max() <= 1.0:
        frame = (frame * 255).astype(np.uint8)
    return cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)


def detect_maturity(img):
    """녹색 영역 크기 기반 성숙도 판별 (색상 진하기+면적 조합)"""
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    green_low = np.array([35, 60, 40])
    green_high = np.array([85, 255, 255])
    mask = cv2.inRange(hsv, green_low, green_high)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
    area = cv2.countNonZero(mask)

    if area > MATURE_AREA_THRESHOLD:
        return "mature"
    elif area > GROWING_AREA_THRESHOLD:
        return "growing"
    return "seedling"


# =========================================================================
# 4. 상태머신 (SimsFactory amr2shelf 구조 재사용, 승강 없이 관절각만)
# =========================================================================

class LettuceTransfer:
    def __init__(self, world, stage, art: Articulation, bed_row_idx, camera_sensor):
        self.world = world
        self.stage = stage
        self.art = art
        self.bed_row_idx = bed_row_idx
        self.camera_sensor = camera_sensor
        self.arm_indices = [art.dof_names.index(n) for n in ARM_JOINT_NAMES]
        self.gripper_index = art.dof_names.index(GRIPPER_JOINT_NAME)
        self._grasp_frame_path = None

    def _apply_arm(self, joint_deg, gripper_open=True):
        from isaacsim.core.utils.types import ArticulationAction
        full_indices = self.arm_indices + [self.gripper_index]
        gripper_val = 0.0 if gripper_open else 0.6  # 실제 개폐 range로 교체 필요
        positions = np.deg2rad(joint_deg).tolist() + [gripper_val]
        action = ArticulationAction(
            joint_positions=np.array(positions),
            joint_indices=np.array(full_indices),
        )
        self.art.apply_action(action)

    def hold(self, seconds, joint_deg=None, gripper_open=True):
        t0 = time.time()
        while simulation_app.is_running() and (time.time() - t0) < seconds:
            if joint_deg is not None:
                self._apply_arm(joint_deg, gripper_open)
            self.world.step(render=True)

    def _ensure_grasp_frame(self):
        path = f"/World/Bed_Row_{self.bed_row_idx}/Manipulator/grasp_frame"
        if not prim_exists(self.stage, path):
            self.stage.DefinePrim(path, "Xform")
        self._grasp_frame_path = path
        return path

    def attach(self, lettuce_path):
        grasp_frame = self._ensure_grasp_frame()
        new_path = reparent_with_world_pose(self.stage, lettuce_path, grasp_frame)
        return new_path

    def detach(self, attached_path, target_layer_path):
        new_path = reparent_with_world_pose(self.stage, attached_path, target_layer_path)
        return new_path

    def process_slot(self, slot_path):
        """SCAN -> JUDGE -> (PICK -> MOVE -> PLACE) | SKIP"""
        t0 = time.time()

        self._apply_arm(POSE_SCAN)
        self.hold(0.5, POSE_SCAN)
        img = get_camera_image(self.camera_sensor)
        maturity = detect_maturity(img)

        if maturity != "mature":
            self.hold(0.2, POSE_READY)
            return {"result": "skip", "maturity": maturity, "cycle_time": time.time() - t0}

        self.hold(0.5, POSE_PICK, gripper_open=True)
        attached_path = self.attach(slot_path)
        self.hold(0.5, POSE_PICK, gripper_open=False)

        self.hold(0.6, POSE_LIFT_TO_READY, gripper_open=False)
        self.hold(0.6, POSE_PLACE_READY, gripper_open=False)

        ready_layer_path = f"/World/Bed_Row_{self.bed_row_idx}/Layer_Ready"
        self.detach(attached_path, ready_layer_path)
        self.hold(0.3, POSE_PLACE_READY, gripper_open=True)

        self.hold(0.3, POSE_READY)
        return {"result": "moved", "maturity": maturity, "cycle_time": time.time() - t0}


# =========================================================================
# 5. 지표 수집 + 밀도 스윕 실험
# =========================================================================

def run_bed_experiment(world, stage, art, bed_row_idx, slot_count, camera_sensor):
    ground_truth = populate_growing_layer(bed_row_idx, slot_count)
    world.reset()
    for _ in range(10):
        world.step(render=True)

    transfer = LettuceTransfer(world, stage, art, bed_row_idx, camera_sensor)

    metrics = {
        "slot_count": slot_count,
        "processed": 0,
        "moved": 0,
        "skipped": 0,
        "false_positive": 0,   # 미성숙인데 mature로 오판
        "false_negative": 0,   # 성숙인데 미성숙으로 오판
        "cycle_times": [],
    }

    for slot_path, true_stage in ground_truth.items():
        result = transfer.process_slot(slot_path)
        metrics["processed"] += 1
        metrics["cycle_times"].append(result["cycle_time"])

        if result["result"] == "moved":
            metrics["moved"] += 1
            if true_stage != "mature":
                metrics["false_positive"] += 1
        else:
            metrics["skipped"] += 1
            if true_stage == "mature":
                metrics["false_negative"] += 1

    n = max(metrics["processed"], 1)
    metrics["avg_cycle_time"] = sum(metrics["cycle_times"]) / n
    metrics["accuracy"] = 1 - (metrics["false_positive"] + metrics["false_negative"]) / n
    return metrics


def run_density_sweep(world, stage, art, bed_row_idx, camera_sensor, slot_counts=(10, 20, 40, 80)):
    results = {}
    for count in slot_counts:
        print(f"[스윕] 슬롯 수 {count} 실험 시작...")
        results[count] = run_bed_experiment(world, stage, art, bed_row_idx, count, camera_sensor)
        print(f"[스윕] 슬롯 수 {count} 결과: {results[count]}")
    return results


# =========================================================================
# 6. 환경 구성 (베드, 조명, 로봇 배치)
# =========================================================================

def build_environment(world, stage):
    world.scene.add_default_ground_plane()

    dome = UsdLux.DomeLight.Define(stage, "/World/DomeLight")
    dome.CreateIntensityAttr(800)

    for row in range(1, BED_ROW_COUNT + 1):
        for layer_name, z in [("Layer_Growing", GROWING_LAYER_Z), ("Layer_Ready", READY_LAYER_Z)]:
            layer_path = f"/World/Bed_Row_{row}/{layer_name}"
            stage.DefinePrim(layer_path, "Xform")
            xf = UsdGeom.Xformable(stage.GetPrimAtPath(layer_path))
            xf.AddTranslateOp().Set(Gf.Vec3d(0, 1.5 if layer_name == "Layer_Growing" else 3.0, z))

        # 재배광
        light_path = f"/World/Bed_Row_{row}/GrowLight"
        light = UsdLux.RectLight.Define(stage, light_path)
        light.CreateWidthAttr(SLOTS_PER_ROW * SLOT_SPACING)
        light.CreateHeightAttr(1.0)
        light.CreateIntensityAttr(3000)
        UsdGeom.Xformable(stage.GetPrimAtPath(light_path)).AddTranslateOp().Set(
            Gf.Vec3d(SLOTS_PER_ROW * SLOT_SPACING / 2, 1.5, GROWING_LAYER_Z + 1.0)
        )


def load_manipulator(stage, bed_row_idx):
    if not os.path.exists(M0609_USD_PATH):
        raise FileNotFoundError(
            f"M0609 USD를 찾을 수 없습니다: {M0609_USD_PATH}\n"
            f"Smartwarehouse 저장소를 클론했는지, SMARTWAREHOUSE_ROOT 환경변수가 맞는지 확인하세요.\n"
            f"참고: https://github.com/rebi00dev/Smartwarehouse"
        )
    prim_path = f"/World/Bed_Row_{bed_row_idx}/Manipulator"
    add_reference_to_stage(usd_path=M0609_USD_PATH, prim_path=prim_path)
    xf = UsdGeom.Xformable(stage.GetPrimAtPath(prim_path))
    xf.AddTranslateOp().Set(Gf.Vec3d(*MANIPULATOR_BASE_POS))
    return prim_path


def setup_scan_camera(stage, bed_row_idx):
    """그리퍼에 탑재 가정 — 실제로는 M0609 USD 내 그리퍼 prim 하위에 배치 권장"""
    from isaacsim.sensors.camera import Camera
    cam_path = f"/World/Bed_Row_{bed_row_idx}/Manipulator/ScanCam"
    camera = Camera(prim_path=cam_path, resolution=(320, 240))
    camera.initialize()
    return camera


# =========================================================================
# 7. main
# =========================================================================

def main():
    world = World()
    stage = world.stage

    build_environment(world, stage)
    manipulator_path = load_manipulator(stage, bed_row_idx=1)

    world.reset()
    for _ in range(30):
        world.step(render=True)

    art = Articulation(prim_paths_expr=manipulator_path)
    world.scene.add(art)
    world.reset()

    # === 3번(로딩 확인불가) 대응 — 실행 즉시 진단부터 ===
    diagnose_articulation(art)

    camera_sensor = setup_scan_camera(stage, bed_row_idx=1)

    # 단일 실험 (목표1)
    single_result = run_bed_experiment(world, stage, art, bed_row_idx=1,
                                        slot_count=SLOTS_PER_ROW, camera_sensor=camera_sensor)
    print("\n[단일 실험 결과]", single_result)

    # 밀도 스윕 (목표2) — 필요 시 주석 해제
    # sweep_result = run_density_sweep(world, stage, art, bed_row_idx=1, camera_sensor=camera_sensor)
    # print("\n[스윕 결과]", sweep_result)

    output_dir = "/mnt/user-data/outputs"
    os.makedirs(output_dir, exist_ok=True)
    stage.Export(os.path.join(output_dir, "smart_farm_lettuce_env.usd"))

    while simulation_app.is_running():
        world.step(render=True)

    simulation_app.close()


if __name__ == "__main__":
    main()
