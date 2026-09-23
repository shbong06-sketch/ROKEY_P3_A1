import sys
from pathlib import Path

import pytest


sys.path.insert(0, str(Path(__file__).parents[1] / "scripts"))

from cull_motion import CullConfig, build_cull_plan  # noqa: E402


def test_builds_pick_and_fixed_place_plan_in_base_frame():
    plan = build_cull_plan(
        (0.40, 0.10, 0.05),
        CullConfig(
            place_position_base=(0.30, -0.25, 0.08),
            approach_clearance=0.10,
            transit_clearance=0.20,
            pick_z_offset=0.01,
            place_z_offset=0.02,
        ),
    )

    assert [step.name for step in plan] == [
        "OPEN",
        "PICK_APPROACH",
        "PICK_DESCEND",
        "GRASP",
        "LIFT",
        "PLACE_APPROACH",
        "PLACE_DESCEND",
        "RELEASE",
        "RETREAT",
    ]
    assert plan[2].position_base == pytest.approx((0.40, 0.10, 0.06))
    assert plan[3].gripper == "close"
    assert plan[4].position_base == pytest.approx((0.40, 0.10, 0.30))
    assert plan[5].position_base == pytest.approx((0.30, -0.25, 0.30))
    assert plan[6].position_base == pytest.approx((0.30, -0.25, 0.10))
    assert plan[7].gripper == "open"


@pytest.mark.parametrize(
    "position",
    ((1.0, 2.0), (1.0, 2.0, float("nan")), ("x", 2.0, 3.0)),
)
def test_rejects_invalid_detection_position(position):
    config = CullConfig(place_position_base=(0.3, -0.2, 0.1))

    with pytest.raises(ValueError, match="detected_position_base"):
        build_cull_plan(position, config)
