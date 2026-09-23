import json
from types import SimpleNamespace

import numpy as np
import pytest

from runtime.camera_to_robot_base import CameraToRobotBase, ProjectionConfig


CAMERA_WORLD_POSITION = np.array([10.0, 20.0, 30.0])
CAMERA_WORLD_ROTATION = np.array(
    [
        [0.0, 0.0, 1.0],
        [0.0, 1.0, 0.0],
        [-1.0, 0.0, 0.0],
    ]
)
BASE_WORLD_POSITION = np.array([10.0, 20.0, 30.0])
BASE_WORLD_ROTATION = np.array(
    [
        [0.0, -1.0, 0.0],
        [1.0, 0.0, 0.0],
        [0.0, 0.0, 1.0],
    ]
)
BASE_WORLD_QUATERNION = np.array([np.sqrt(0.5), 0.0, 0.0, np.sqrt(0.5)])


class FakePinholeCamera:
    def __init__(self, depth_image: np.ndarray) -> None:
        self._depth_image = depth_image

    def get_depth(self) -> np.ndarray:
        return self._depth_image

    def get_camera_points_from_image_coords(
        self,
        image_coordinates: np.ndarray,
        depths: np.ndarray,
    ) -> np.ndarray:
        u = image_coordinates[:, 0]
        v = image_coordinates[:, 1]
        x = (u - 6.0) * depths / 2.0
        y = (v - 10.0) * depths / 4.0
        return np.column_stack((x, y, depths))

    def get_world_points_from_image_coords(
        self,
        image_coordinates: np.ndarray,
        depths: np.ndarray,
    ) -> np.ndarray:
        camera_points = self.get_camera_points_from_image_coords(
            image_coordinates,
            depths,
        )
        return (
            camera_points @ CAMERA_WORLD_ROTATION.T
            + CAMERA_WORLD_POSITION
        )


class FakeRobotBase:
    def get_world_pose(self) -> tuple[np.ndarray, np.ndarray]:
        return BASE_WORLD_POSITION, BASE_WORLD_QUATERNION


def fake_quaternion_to_matrix(quaternion: np.ndarray) -> np.ndarray:
    np.testing.assert_allclose(quaternion, BASE_WORLD_QUATERNION)
    return BASE_WORLD_ROTATION


def test_projects_arbitrary_depths_from_camera_to_robot_base() -> None:
    depth_image = np.full((16, 16), 20.0)
    depth_image[6:10, 6:10] = np.array(
        [
            [0.0, np.nan, 4.0, 4.0],
            [4.0, 4.0, 4.0, 4.0],
            [6.0, 6.0, 6.0, 6.0],
            [np.inf, 6.0, 20.0, 6.0],
        ]
    )
    converter = CameraToRobotBase(
        camera=FakePinholeCamera(depth_image),
        robot_base=FakeRobotBase(),
        config=ProjectionConfig(
            inner_bbox_ratio=0.25,
            min_depth=0.1,
            max_depth=10.0,
            min_valid_pixels=10,
        ),
        quaternion_to_matrix=fake_quaternion_to_matrix,
    )
    payload = {
        "task_id": "task-17",
        "pallet_id": "pallet-3",
        "image_width": 16,
        "image_height": 16,
        "detections": [
            {
                "slot_id": "B2",
                "class_name": "lettuce_brown",
                "center_u": 8.0,
                "center_v": 8.0,
                "bbox_x_min": 2.0,
                "bbox_y_min": 2.0,
                "bbox_x_max": 14.0,
                "bbox_y_max": 14.0,
            }
        ],
    }
    converter.accept_detection_message(
        SimpleNamespace(data=json.dumps(payload))
    )

    results = converter.update()

    assert len(results) == 1
    result = results[0]
    assert result.depth == 5.0
    assert result.pixel_uv == (8.0, 8.0)
    np.testing.assert_allclose(result.position_camera, [5.0, -2.5, 5.0])
    np.testing.assert_allclose(result.position_base, [-2.5, -5.0, -5.0])
    assert converter.get("task-17", "pallet-3", "B2") == result
    assert converter.update() == ()


def test_retries_all_detections_without_partial_cache_after_depth_failure() -> None:
    depth_image = np.full((16, 16), np.nan)
    depth_image[3:5, 3:5] = 1.0
    camera = FakePinholeCamera(depth_image)
    converter = CameraToRobotBase(
        camera=camera,
        robot_base=FakeRobotBase(),
        config=ProjectionConfig(
            inner_bbox_ratio=0.25,
            min_depth=0.1,
            max_depth=10.0,
            min_valid_pixels=4,
        ),
        quaternion_to_matrix=fake_quaternion_to_matrix,
    )
    payload = {
        "task_id": "task-retry",
        "pallet_id": "pallet-retry",
        "image_width": 16,
        "image_height": 16,
        "detections": [
            {
                "slot_id": "A1",
                "class_name": "lettuce_brown",
                "center_u": 4.0,
                "center_v": 4.0,
                "bbox_x_min": 0.0,
                "bbox_y_min": 0.0,
                "bbox_x_max": 8.0,
                "bbox_y_max": 8.0,
            },
            {
                "slot_id": "B2",
                "class_name": "lettuce_brown",
                "center_u": 12.0,
                "center_v": 12.0,
                "bbox_x_min": 8.0,
                "bbox_y_min": 8.0,
                "bbox_x_max": 16.0,
                "bbox_y_max": 16.0,
            },
        ],
    }
    converter.accept_detection_message(
        SimpleNamespace(data=json.dumps(payload))
    )

    with pytest.raises(ValueError, match="not enough valid depth pixels"):
        converter.update()

    assert converter.get("task-retry", "pallet-retry", "A1") is None
    camera._depth_image[11:13, 11:13] = 2.0

    results = converter.update()

    assert [result.slot_id for result in results] == ["A1", "B2"]
    assert converter.get("task-retry", "pallet-retry", "A1") is not None
    assert converter.get("task-retry", "pallet-retry", "B2") is not None
    assert converter.update() == ()
