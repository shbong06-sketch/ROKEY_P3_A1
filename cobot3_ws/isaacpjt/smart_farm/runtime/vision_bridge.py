"""비전이 낸 픽셀 좌표를 로봇 base 기준 3D 좌표로 바꾼다.

검사 노드는 `/inspection/detections_2d` 로 **픽셀** 중심점을 낸다.
`cull_motion.build_cull_plan` 은 **로봇 base 기준 미터** 를 요구한다.
그 사이를 잇는 것이 이 파일이다. 문서상 Sim Task Executor 의 몫이다.

변환 절차
    1. 픽셀 (u, v) 와 그 자리의 깊이 d 를 받는다
    2. 카메라 내부 파라미터로 카메라 좌표계 3D 점을 만든다
    3. 카메라 월드 자세를 곱해 월드 좌표로 바꾼다
    4. 로봇 base 를 빼서 base 기준으로 바꾼다

Isaac 안에서 쓰는 것을 전제로 한다 (stage 에서 카메라·base 자세를 읽음).
"""

from __future__ import annotations

import json
import math

import numpy as np
from pxr import Gf, UsdGeom


class CameraModel:
    """USD 카메라와 RenderProduct 해상도로 핀홀 내부 파라미터를 만든다.

    `/camera_info` 를 구독해도 되지만, 씬에서 직접 읽으면 ROS 왕복이 없고
    렌더 해상도와 항상 일치한다. Isaac 은 정사각 픽셀을 가정하므로
    fy 를 fx 에 맞춘다 (브리지가 찍는 경고와 같은 처리).
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


def camera_to_world(stage, camera_path, point_camera):
    """카메라 좌표계 점을 월드로 옮긴다."""
    xf = UsdGeom.Xformable(
        stage.GetPrimAtPath(camera_path)).ComputeLocalToWorldTransform(0)
    matrix = Gf.Matrix4d(xf).RemoveScaleShear()
    world = matrix.Transform(Gf.Vec3d(*point_camera))
    return np.array([world[0], world[1], world[2]])


def base_from_world(stage, arm_root_path, point_world):
    """월드 점을 로봇 base 기준으로 바꾼다."""
    xf = UsdGeom.Xformable(
        stage.GetPrimAtPath(arm_root_path)).ComputeLocalToWorldTransform(0)
    inverse = Gf.Matrix4d(xf).RemoveScaleShear().GetInverse()
    local = inverse.Transform(Gf.Vec3d(*point_world))
    return np.array([local[0], local[1], local[2]])


def parse_detections(payload):
    """`/inspection/detections_2d` 의 JSON 문자열을 dict 로.

    슬롯이 비어 있는 검출(`slot_id` == '')은 버린다. ROI 밖이라 어느
    슬롯에도 속하지 않은 것이다.
    """
    data = json.loads(payload)
    by_slot = {}
    for item in data.get("detections", []):
        slot = item.get("slot_id") or ""
        if not slot:
            continue
        by_slot[slot] = item
    return {
        "task_id": data.get("task_id"),
        "command_id": data.get("command_id"),
        "image_width": data.get("image_width"),
        "image_height": data.get("image_height"),
        "slots": by_slot,
    }


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
