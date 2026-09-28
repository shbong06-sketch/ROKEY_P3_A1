"""검사 색상별 배출 상자와 상자 안 투하 순서를 정한다."""


COLOUR_TO_BIN = {
    "yellow": "SortBin_1",
    "brown": "SortBin_2",
}


def validate_bin_routing(colours, box_names):
    """Cull 시작 전에 모든 대상 색상과 실제 상자를 확인한다."""
    if len(box_names) != len(set(box_names)):
        raise RuntimeError("Cull sorting bins are duplicated")
    for colour in colours:
        box_name = COLOUR_TO_BIN.get(colour)
        if box_name is None:
            raise RuntimeError(f"Cull target colour is unsupported: {colour}")
        if box_name not in box_names:
            raise RuntimeError(f"Cull sorting bin is missing: {box_name}")


def next_drop_position(colour, box_names, drop_counts, spread):
    """(상자 인덱스, 월드 y 방향 분산값)을 반환하고 해당 상자 횟수를 증가시킨다."""
    validate_bin_routing((colour,), box_names)
    if not spread:
        raise RuntimeError("Cull drop spread is empty")
    box_name = COLOUR_TO_BIN[colour]
    use = drop_counts.get(box_name, 0)
    drop_counts[box_name] = use + 1
    return box_names.index(box_name), spread[use % len(spread)]
