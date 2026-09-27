"""픽셀 추론과 독립적인 팔레트 자세 기반 Cull 목표 검증."""

import numpy as np
import pytest

from cull_target_geometry import head_inside_box, pick_target_world, seat_offset_local


def test_slot_target_follows_tray_translation_and_rotation():
    first = np.eye(4)
    first[:3, 3] = (-0.7, -7.0, 0.8)
    head = np.array([-0.6, -7.05, 0.85])
    offset = seat_offset_local(first, head)
    moved = np.eye(4)
    moved[:3, :3] = np.array([[0.0, -1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, 1.0]])
    moved[:3, 3] = (1.0, 2.0, 0.9)
    target = pick_target_world(moved, offset, 0.09)
    assert target == pytest.approx((1.05, 2.10, 0.99))


def test_invalid_geometry_fails_before_motion():
    bad = np.eye(4)
    bad[0, 0] = float("nan")
    with pytest.raises(ValueError, match="finite"):
        seat_offset_local(bad, (0.0, 0.0, 0.0))
    with pytest.raises(ValueError, match="finite"):
        pick_target_world(np.eye(4), (0.0, 0.0, 0.0), float("nan"))


def test_only_head_inside_all_three_box_dimensions_counts_as_dropped():
    lower = (-1.0, -1.0, 0.0)
    upper = (1.0, 1.0, 0.5)
    assert head_inside_box((0.0, 0.0, 0.2), lower, upper)
    assert not head_inside_box((0.0, 0.0, -0.1), lower, upper)
    assert not head_inside_box((1.1, 0.0, 0.2), lower, upper)
