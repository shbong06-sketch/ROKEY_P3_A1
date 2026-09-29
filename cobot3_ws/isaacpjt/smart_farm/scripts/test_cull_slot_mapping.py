"""영상 슬롯과 v014 물리 Prim의 연결을 검증한다."""

import pytest

from cull_slot_mapping import select_cull_heads


class Head:
    def __init__(self, name):
        self.name = name

    def GetName(self):
        return self.name


def test_each_vision_slot_selects_its_physical_cabbage():
    heads = [Head(f"Cabbage_{index:02d}") for index in range(1, 7)]
    selected = select_cull_heads(heads, [f"SLOT_{index:02d}" for index in range(1, 7)])
    assert [(slot, head.GetName()) for slot, head in selected] == [
        ("SLOT_01", "Cabbage_06"),
        ("SLOT_02", "Cabbage_04"),
        ("SLOT_03", "Cabbage_02"),
        ("SLOT_04", "Cabbage_05"),
        ("SLOT_05", "Cabbage_03"),
        ("SLOT_06", "Cabbage_01"),
    ]
    defects = select_cull_heads(heads, ["SLOT_03", "SLOT_04", "SLOT_05"])
    assert [(slot, head.GetName()) for slot, head in defects] == [
        ("SLOT_03", "Cabbage_02"),
        ("SLOT_04", "Cabbage_05"),
        ("SLOT_05", "Cabbage_03"),
    ]


def test_missing_or_duplicate_prim_fails_before_motion():
    heads = [Head(f"Cabbage_{index:02d}") for index in range(1, 7)]
    with pytest.raises(RuntimeError, match="do not match"):
        select_cull_heads(heads[:-1], ["SLOT_03"])
    with pytest.raises(RuntimeError, match="do not match"):
        select_cull_heads(heads[:-1] + [heads[0]], ["SLOT_03"])
    with pytest.raises(RuntimeError, match="duplicates"):
        select_cull_heads(heads, ["SLOT_03", "SLOT_03"])
