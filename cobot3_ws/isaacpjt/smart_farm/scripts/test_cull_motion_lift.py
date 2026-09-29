"""Cull 계획이 LIFT 후 실제 포기 상승을 확인하는지 검증."""

import numpy as np
import pytest

from cull_motion import CullMotion, CullPickConfig, CullStep


def test_full_cull_plan_stops_when_head_did_not_rise():
    motion = object.__new__(CullMotion)
    motion.config = CullPickConfig()
    motion._plan = (CullStep("LIFT", (0.0, 0.0, 0.2)), CullStep("VIA", (0.1, 0.0, 0.2)))
    motion._index = 0
    motion._get_target_world_pose = lambda: ((0.0, 0.0, 0.01), None)
    motion._initial_target_position = np.zeros(3)
    motion._done = False
    motion._failed = False
    motion._error = None
    with pytest.raises(RuntimeError, match="물리 Pick 실패"):
        motion._complete_step()
    assert motion.is_failed is True
    assert motion._index == 0


def test_full_cull_plan_continues_after_confirmed_lift():
    motion = object.__new__(CullMotion)
    motion.config = CullPickConfig()
    motion._plan = (CullStep("LIFT", (0.0, 0.0, 0.2)), CullStep("VIA", (0.1, 0.0, 0.2)))
    motion._index = 0
    motion._get_target_world_pose = lambda: ((0.0, 0.0, 0.08), None)
    motion._initial_target_position = np.zeros(3)
    motion._done = False
    motion._failed = False
    motion._error = None
    motion._complete_step()
    assert motion._index == 1
    assert motion.final_target_rise == pytest.approx(0.08)
