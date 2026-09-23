from collections.abc import Callable
from dataclasses import dataclass
import json
from typing import Any

import numpy as np


@dataclass(frozen=True)
class ProjectionConfig:
    inner_bbox_ratio: float = 0.25   # bbox 내부에서 depth값 활용 범위 - 실험을 통해 최적값 계산 필요
    min_depth: float = 0.28         # realsense의 안정 depth 측정 최소 거리
    max_depth: float = 5.0
    min_valid_pixels: int = 4


@dataclass(frozen=True)
class ProjectedDetection:
    task_id: str
    pallet_id: str
    slot_id: str
    class_name: str
    pixel_uv: tuple[float, float]
    depth: float
    position_camera: tuple[float, float, float]
    position_base: tuple[float, float, float]


class CameraToRobotBase:
    def __init__(
        self,
        camera: Any,
        robot_base: Any,
        config: ProjectionConfig | None = None,
        quaternion_to_matrix: Callable[[np.ndarray], np.ndarray] | None = None,
    ) -> None:
        self._camera = camera
        self._robot_base = robot_base
        self._config = config or ProjectionConfig()
        self._quaternion_to_matrix = (
            quaternion_to_matrix or self._isaac_quaternion_to_matrix
        )

        self._pending_json: str | None = None
        self._cache: dict[
            tuple[str, str, str],
            ProjectedDetection,
        ] = {}

    @staticmethod
    def _isaac_quaternion_to_matrix(
        quaternion: np.ndarray,
    ) -> np.ndarray:
        from isaacsim.core.utils.rotations import quat_to_rot_matrix

        return quat_to_rot_matrix(quaternion)

    def accept_detection_message(self, message: Any) -> None:
        """ROS callback에서는 문자열만 보관한다."""
        self._pending_json = message.data

    def get(
        self,
        task_id: str,
        pallet_id: str,
        slot_id: str,
    ) -> ProjectedDetection | None:
        return self._cache.get((task_id, pallet_id, slot_id))
    
    def update(self) -> tuple[ProjectedDetection, ...]:
        if self._pending_json is None:
            return ()

        raw_json = self._pending_json
        payload = json.loads(raw_json)

        depth_image = np.asarray(self._camera.get_depth(), dtype=float)
        if depth_image.ndim == 3 and depth_image.shape[-1] == 1:
            depth_image = depth_image[..., 0]
        if depth_image.ndim != 2:
            raise ValueError("camera depth image must be two-dimensional")

        expected_shape = (
            int(payload["image_height"]),
            int(payload["image_width"]),
        )
        if depth_image.shape != expected_shape:
            raise ValueError(
                f"RGB/depth resolution mismatch: "
                f"detection={expected_shape}, depth={depth_image.shape}"
            )

        base_position, base_quaternion = self._robot_base.get_world_pose()
        base_position = np.asarray(base_position, dtype=float)
        world_from_base = np.asarray(
            self._quaternion_to_matrix(
                np.asarray(base_quaternion, dtype=float)
            ),
            dtype=float,
        )

        projected: list[ProjectedDetection] = []
        new_cache: dict[
            tuple[str, str, str],
            ProjectedDetection,
        ] = {}

        for detection in payload["detections"]:
            slot_id = str(detection["slot_id"])
            if not slot_id:
                continue

            bbox = (
                float(detection["bbox_x_min"]),
                float(detection["bbox_y_min"]),
                float(detection["bbox_x_max"]),
                float(detection["bbox_y_max"]),
            )
            pixel_uv = (
                float(detection["center_u"]),
                float(detection["center_v"]),
            )
            depth = sample_bbox_depth(depth_image, bbox, self._config)

            image_coordinates = np.asarray([pixel_uv], dtype=float)
            depths = np.asarray([depth], dtype=float)

            camera_point = np.asarray(
                self._camera.get_camera_points_from_image_coords(
                    image_coordinates,
                    depths,
                ),
                dtype=float,
            )[0]
            world_point = np.asarray(
                self._camera.get_world_points_from_image_coords(
                    image_coordinates,
                    depths,
                ),
                dtype=float,
            )[0]

            base_point = world_from_base.T @ (
                world_point - base_position
            )

            result = ProjectedDetection(
                task_id=str(payload["task_id"]),
                pallet_id=str(payload["pallet_id"]),
                slot_id=slot_id,
                class_name=str(detection["class_name"]),
                pixel_uv=pixel_uv,
                depth=depth,
                position_camera=tuple(camera_point.tolist()),
                position_base=tuple(base_point.tolist()),
            )

            key = (result.task_id, result.pallet_id, result.slot_id)
            new_cache[key] = result
            projected.append(result)

        self._cache.update(new_cache)
        self._pending_json = None
        return tuple(projected)


def sample_bbox_depth(
    depth_image: np.ndarray,
    bbox_xyxy: tuple[float, float, float, float],
    config: ProjectionConfig,
) -> float:
    x_min, y_min, x_max, y_max = bbox_xyxy

    if not np.isfinite(bbox_xyxy).all():
        raise ValueError("bbox contains a non-finite value")
    if x_min >= x_max or y_min >= y_max:
        raise ValueError("bbox has no area")

    # 안정적인 depth값 계산을 위해 중앙 25% 영역의 중앙값 사용
    height, width = depth_image.shape
    center_x = (x_min + x_max) / 2.0
    center_y = (y_min + y_max) / 2.0
    half_width = (x_max - x_min) * config.inner_bbox_ratio / 2.0
    half_height = (y_max - y_min) * config.inner_bbox_ratio / 2.0

    left = max(0, int(np.floor(center_x - half_width)))
    right = min(width, int(np.ceil(center_x + half_width)))
    top = max(0, int(np.floor(center_y - half_height)))
    bottom = min(height, int(np.ceil(center_y + half_height)))

    region = depth_image[top:bottom, left:right]
    valid = region[
        np.isfinite(region)
        & (region >= config.min_depth)
        & (region <= config.max_depth)
    ]

    if valid.size < config.min_valid_pixels:
        raise ValueError("not enough valid depth pixels")

    return float(np.median(valid))