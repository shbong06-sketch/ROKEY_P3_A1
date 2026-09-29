"""v014 장면의 양배추 Prim 이름을 영상 슬롯에 연결한다."""


# 영상 ROI 번호와 USD Prim 번호가 서로 다르다. 명령과 결과에는 영상 슬롯을 사용한다.
SLOT_TO_CABBAGE = {
    "SLOT_01": "Cabbage_06",
    "SLOT_02": "Cabbage_04",
    "SLOT_03": "Cabbage_02",
    "SLOT_04": "Cabbage_05",
    "SLOT_05": "Cabbage_03",
    "SLOT_06": "Cabbage_01",
}


def select_cull_heads(heads, target_slots):
    """요청된 영상 슬롯 순서대로 (slot_id, 물리 Prim)을 반환한다."""
    by_name = {head.GetName(): head for head in heads}
    if len(by_name) != len(heads) or set(by_name) != set(SLOT_TO_CABBAGE.values()):
        raise RuntimeError("Cull pallet cabbage Prims do not match the slot mapping")
    if len(target_slots) != len(set(target_slots)):
        raise RuntimeError("Cull target slots contain duplicates")
    if any(slot not in SLOT_TO_CABBAGE for slot in target_slots):
        raise RuntimeError("Cull target slot is unknown")
    return [(slot, by_name[SLOT_TO_CABBAGE[slot]]) for slot in target_slots]
