"""yellow/brown 배출 경로와 기존 DROP_SPREAD 순환을 검증한다."""

import pytest

from cull_bin_routing import next_drop_position, validate_bin_routing


BOXES = ("SortBin_1", "SortBin_2")
SPREAD = (0.0, -0.15, 0.15, -0.075, 0.075)


def test_consecutive_and_mixed_colours_use_separate_bins_and_spread_counts():
    counts = {}
    requests = ("yellow", "yellow", "brown", "yellow", "brown")
    assert [next_drop_position(colour, BOXES, counts, SPREAD) for colour in requests] == [
        (0, 0.0), (0, -0.15), (1, 0.0), (0, 0.15), (1, -0.15),
    ]


def test_sixth_same_colour_wraps_to_first_spread_position():
    counts = {}
    positions = [next_drop_position("yellow", BOXES, counts, SPREAD) for _ in range(6)]
    assert positions[-1] == (0, 0.0)


def test_invalid_colour_or_missing_bin_fails_before_cull():
    with pytest.raises(RuntimeError, match="unsupported"):
        validate_bin_routing(("green",), BOXES)
    with pytest.raises(RuntimeError, match="SortBin_2"):
        validate_bin_routing(("yellow", "brown"), ("SortBin_1",))
