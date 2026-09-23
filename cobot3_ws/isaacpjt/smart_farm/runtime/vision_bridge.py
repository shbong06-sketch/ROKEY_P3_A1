"""비전이 낸 픽셀 좌표를 로봇 base 기준 3D 좌표로 바꾼다.

검사 노드는 `/inspection/detections_2d` 로 **픽셀** 중심점을 낸다.
`cull_motion.build_cull_plan` 은 **로봇 base 기준 미터** 를 요구한다.
그 사이를 잇는 것이 이 파일이다. 문서상 Sim Task Executor 의 몫이다.

변환 절차
    1. 픽셀 (u, v) 와 그 자리의 깊이 d 를 받는다
       (깊이가 없으면 알려진 높이의 수평면과 시선의 교점을 쓴다)
    2. 카메라 내부 파라미터로 카메라 좌표계 3D 점을 만든다
    3. 카메라 월드 자세를 곱해 월드 좌표로 바꾼다
    4. 로봇 base 를 빼서 base 기준으로 바꾼다

pxr 가 필요하므로 Isaac Python 안에서 쓴다 (호스트 ROS 노드에서는 import
불가). 카메라·base 자세는 stage 에서 읽는 것이 기본이지만, 재생 중 USD 가
물리 결과를 따라가지 않을 수 있으므로 호출자가 행렬을 직접 넘길 수 있다.

행렬 규약: Gf 행벡터 (p_world = p_local * M).
깊이 규약: 광축 방향 거리(distance_to_image_plane), 미터.
"""

from __future__ import annotations

import json
import math

import numpy as np
from pxr import Gf, UsdGeom


class CameraModel:
    """USD 카메라와 RenderProduct 해상도로 핀홀 내부 파라미터를 만든다.

    Isaac 은 정사각 픽셀을 가정하므로 fy 를 fx 에 맞춘다. 수직 조리개
    값은 렌더에 쓰이지 않는다고 보고 무시한다.
    """

    def __init__(self, stage, camera_path, render_product_path):
        cam = UsdGeom.Camera(stage.GetPrimAtPath(camera_path))
        if not cam:
            raise RuntimeError(f"카메라를 찾지 못했습니다: {camera_path}")
        product = stage.GetPrimAtPath(render_product_path)
        if not product:
            raise RuntimeError(f"RenderProduct 없음: {render_product_path}")

        self.width = int(product.GetAttribute("inputs:width").Get())
        self.height = int(product.GetAttribute("inputs:height").Get())
        focal = float(cam.GetFocalLengthAttr().Get())
        aperture = float(cam.GetHorizontalApertureAttr().Get())

        # USD 초점거리·조리개는 같은 단위라 비율만 쓴다.
        self.fx = self.width * focal / aperture
        self.fy = self.fx
        self.cx = self.width / 2.0
        self.cy = self.height / 2.0
        self.camera_path = camera_path

        # 조리개 오프셋이 있으면 주점이 중심에서 벗어난다. 지금은 반영하지
        # 않고 경고만 한다 (RealSense 에셋은 0 이어야 정상).
        offsets = []
        for getter in (cam.GetHorizontalApertureOffsetAttr,
                       cam.GetVerticalApertureOffsetAttr):
            attr = getter()
            offsets.append(float(attr.Get() or 0.0) if attr else 0.0)
        self.aperture_offsets = tuple(offsets)
        if any(abs(o) > 1e-6 for o in offsets):
            print(f"[브리지][경고] 조리개 오프셋 {offsets} — 주점을 화면 "
                  f"중심으로 가정했으므로 좌표가 어긋날 수 있습니다.",
                  flush=True)

    def fov_deg(self):
        return (2 * math.degrees(math.atan(self.width / (2 * self.fx))),
                2 * math.degrees(math.atan(self.height / (2 * self.fy))))

    def ray(self, u, v):
        """픽셀 -> 카메라 좌표계 단위 방향. USD 카메라는 -Z 를 본다."""
        x = (u - self.cx) / self.fx
        y = -(v - self.cy) / self.fy      # 이미지 v 는 아래로, USD Y 는 위로
        direction = np.array([x, y, -1.0])
        return direction / np.linalg.norm(direction)

    def point_at(self, u, v, depth):
        """픽셀과 깊이(광축 방향 거리) -> 카메라 좌표계 3D 점."""
        x = (u - self.cx) / self.fx * depth
        y = -(v - self.cy) / self.fy * depth
        return np.array([x, y, -depth])


# ── 자세 행렬 ─────────────────────────────────────────
def prim_world_matrix(stage, path):
    """USD 에서 읽은 월드 행렬 (스케일 제거). 재생 중이면 낡았을 수 있다."""
    prim = stage.GetPrimAtPath(path)
    if not prim:
        raise RuntimeError(f"prim 없음: {path}")
    xf = UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(0)
    return Gf.Matrix4d(xf).RemoveScaleShear()


def camera_to_world(stage, camera_path, point_camera, matrix=None):
    """카메라 좌표계 점을 월드로 옮긴다. matrix 가 있으면 그것을 쓴다."""
    m = matrix if matrix is not None else prim_world_matrix(stage, camera_path)
    world = m.Transform(Gf.Vec3d(*map(float, point_camera)))
    return np.array([world[0], world[1], world[2]])


def base_from_world(stage, arm_root_path, point_world, matrix=None):
    """월드 점을 로봇 base 기준으로 바꾼다. matrix 는 base->world."""
    m = matrix if matrix is not None else prim_world_matrix(stage,
                                                            arm_root_path)
    local = m.GetInverse().Transform(Gf.Vec3d(*map(float, point_world)))
    return np.array([local[0], local[1], local[2]])


def ray_plane_point(model, cam_matrix, u, v, plane_z):
    """시선과 수평면 z=plane_z 의 교점 (월드). 없으면 None.

    깊이 영상이 없을 때의 대안. 트레이처럼 높이를 아는 대상에만 쓴다.
    """
    eye = cam_matrix.ExtractTranslation()
    d_cam = model.ray(u, v)
    # 방향 벡터는 회전만 적용한다.
    d = cam_matrix.TransformDir(Gf.Vec3d(*map(float, d_cam)))
    if abs(d[2]) < 1e-9:
        return None
    t = (plane_z - eye[2]) / d[2]
    if t <= 0:
        return None
    return np.array([eye[0] + t * d[0], eye[1] + t * d[1], plane_z])


# ── 검출 메시지 ───────────────────────────────────────
def _confidence(item):
    for key in ("confidence", "conf", "score"):
        if key in item:
            try:
                return float(item[key])
            except (TypeError, ValueError):
                pass
    return -1.0


def parse_detections(payload):
    """`/inspection/detections_2d` 의 JSON 문자열을 dict 로.

    슬롯이 비어 있는 검출(`slot_id` == '')은 버린다. ROI 밖이라 어느
    슬롯에도 속하지 않은 것이다. 한 슬롯에 둘 이상이면 신뢰도가 높은
    쪽을 남기고 그 사실을 `duplicates` 에 기록한다.
    """
    data = json.loads(payload)
    by_slot, duplicates = {}, []
    for item in data.get("detections", []):
        slot = item.get("slot_id") or ""
        if not slot:
            continue
        if slot in by_slot:
            duplicates.append(slot)
            if _confidence(item) <= _confidence(by_slot[slot]):
                continue
        by_slot[slot] = item
    return {
        "task_id": data.get("task_id"),
        "command_id": data.get("command_id"),
        "stamp": data.get("stamp") or data.get("timestamp"),
        "image_width": data.get("image_width"),
        "image_height": data.get("image_height"),
        "slots": by_slot,
        "duplicates": sorted(set(duplicates)),
    }


def extract_center(item):
    """검출 항목에서 픽셀 중심 (u, v) 를 꺼낸다. 못 찾으면 None.

    필드 이름이 확정되지 않아 흔한 형태를 모두 받는다.
      center / center_px / pixel : [u, v] 또는 {"u","v"} / {"x","y"}
      u, v / cx, cy / center_x, center_y
      bbox / xyxy : [x1, y1, x2, y2] (중심 계산)
    """
    for key in ("center", "center_px", "pixel", "centroid"):
        value = item.get(key)
        if isinstance(value, (list, tuple)) and len(value) >= 2:
            return float(value[0]), float(value[1])
        if isinstance(value, dict):
            for a, b in (("u", "v"), ("x", "y")):
                if a in value and b in value:
                    return float(value[a]), float(value[b])
    for a, b in (("u", "v"), ("cx", "cy"), ("center_x", "center_y")):
        if a in item and b in item:
            return float(item[a]), float(item[b])
    for key in ("bbox", "xyxy", "box"):
        value = item.get(key)
        if isinstance(value, (list, tuple)) and len(value) >= 4:
            x1, y1, x2, y2 = (float(v) for v in value[:4])
            return (x1 + x2) / 2.0, (y1 + y2) / 2.0
    return None


def check_resolution(parsed, model):
    """검출 메시지 해상도와 카메라 해상도가 다르면 예외."""
    w, h = parsed.get("image_width"), parsed.get("image_height")
    if w is None or h is None:
        return False          # 정보 없음 — 호출자가 경고한다
    if int(w) != model.width or int(h) != model.height:
        raise RuntimeError(f"해상도 불일치: 검출 {w}x{h} vs 카메라 "
                           f"{model.width}x{model.height}")
    return True


# ── 깊이 영상 ─────────────────────────────────────────
def decode_depth(msg):
    """sensor_msgs/Image 형태의 객체를 미터 단위 float32 배열로.

    rclpy 를 import 하지 않도록 속성만 읽는다. 32FC1(m), 16UC1(mm) 지원.
    """
    enc = str(msg.encoding).upper()
    h, w, step = int(msg.height), int(msg.width), int(msg.step)
    raw = bytes(msg.data)
    big = bool(getattr(msg, "is_bigendian", 0))
    if enc == "32FC1":
        dt = np.dtype(">f4" if big else "<f4")
        arr = np.frombuffer(raw, dt).reshape(h, step // 4)[:, :w]
        return arr.astype(np.float32)
    if enc in ("16UC1", "MONO16"):
        dt = np.dtype(">u2" if big else "<u2")
        arr = np.frombuffer(raw, dt).reshape(h, step // 2)[:, :w]
        out = arr.astype(np.float32) / 1000.0
        out[arr == 65535] = 0.0          # 흔한 '측정 없음' 표시값
        return out
    raise RuntimeError(f"지원하지 않는 깊이 encoding: {msg.encoding}")


def depth_at(depth_image, u, v, window=2):
    """(u, v) 주변의 유효 깊이 중앙값. 구멍과 튀는 값을 견딘다."""
    height, width = depth_image.shape[:2]
    cu, cv = int(round(u)), int(round(v))
    u0, u1 = max(0, cu - window), min(width, cu + window + 1)
    v0, v1 = max(0, cv - window), min(height, cv + window + 1)
    patch = depth_image[v0:v1, u0:u1].astype(float).ravel()
    valid = patch[np.isfinite(patch) & (patch > 0.0)]
    if valid.size == 0:
        return None
    return float(np.median(valid))


# ── 한 번에 ───────────────────────────────────────────
def pixel_to_base(model, cam_matrix, base_matrix, u, v,
                  depth_image=None, plane_z=None):
    """픽셀 하나를 base 좌표로. 결과 dict, 실패하면 None.

    깊이 영상이 있으면 그것을, 없거나 구멍이면 plane_z 교점을 쓴다.
    깊이 영상 해상도가 카메라와 다르면 픽셀을 비율로 옮겨 읽는다.
    """
    world, source, depth = None, None, None
    if depth_image is not None:
        dh, dw = depth_image.shape[:2]
        du, dv = u * dw / model.width, v * dh / model.height
        depth = depth_at(depth_image, du, dv)
        if depth is not None:
            p_cam = model.point_at(u, v, depth)
            world = camera_to_world(None, None, p_cam, matrix=cam_matrix)
            source = "depth"
    if world is None and plane_z is not None:
        world = ray_plane_point(model, cam_matrix, u, v, plane_z)
        source = "plane" if world is not None else None
    if world is None:
        return None
    base = base_from_world(None, None, world, matrix=base_matrix)
    return {"pixel": [round(u, 1), round(v, 1)],
            "source": source,
            "depth_m": None if depth is None else round(depth, 4),
            "xyz_world": [round(float(c), 4) for c in world],
            "xyz_base": [round(float(c), 4) for c in base]}
