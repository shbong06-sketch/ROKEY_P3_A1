"""기록 파일(JSONL)에서 지난 사이클을 읽어 옵니다.

대시보드는 켜져 있는 동안의 사이클만 기억합니다. 껐다 켜면 사라지고,
대시보드를 나중에 띄운 날의 사이클은 아예 못 봅니다.
cycle_recorder 가 이미 디스크에 남기고 있으니, 그걸 읽어 채워 넣습니다.

읽는 것은 cycle_summary 줄 하나뿐입니다. 사건 줄까지 읽으면 파일이 커질수록
시작이 느려지는데, 화면에 필요한 값은 요약에 다 있습니다.
"""

import json
from pathlib import Path


SUMMARY = "cycle_summary"


def load_summaries(directory, limit=20):
    """최근 사이클 요약을 새 것부터 돌려줍니다. 없거나 깨졌으면 빈 목록입니다.

    기록은 시연 중에도 계속 쌓이므로, 읽다가 실패해도 대시보드는 떠야 합니다.
    그래서 어떤 오류든 삼키고 읽은 만큼만 씁니다.
    """
    root = Path(directory).expanduser()
    if not root.is_dir():
        return []

    found = []
    # 파일 이름이 cycle_<날짜>_<시각>.jsonl 이라 이름순이 곧 시간순입니다.
    for path in sorted(root.glob("cycle_*.jsonl"), reverse=True):
        try:
            text = path.read_text(encoding="utf-8")
        except OSError:
            continue
        for line in text.splitlines():
            if SUMMARY not in line:      # 대부분은 사건 줄입니다. 먼저 걸러 냅니다.
                continue
            try:
                record = json.loads(line)
            except json.JSONDecodeError:
                continue                 # 기록 중에 끊긴 마지막 줄일 수 있습니다
            if record.get("kind") != SUMMARY:
                continue
            found.append(_to_entry(record, path.name))
        if len(found) >= limit:
            break

    found.sort(key=lambda entry: entry["wall"], reverse=True)
    return found[:limit]


def _to_entry(record, filename):
    """요약 줄을 화면이 쓰는 이력 항목으로 바꿉니다.

    Board 가 실시간으로 만드는 항목과 같은 모양이어야 화면에서 섞어 쓸 수 있습니다.
    """
    stages = record.get("state_seconds") or []
    slowest = max(stages, key=lambda s: s.get("seconds", 0), default=None)
    return {
        "task_id": record.get("task_id", ""),
        "status": record.get("status", ""),
        "reason": record.get("reason", ""),
        "seconds": record.get("total_seconds"),
        "defects": list(record.get("defect_slots", [])),
        "culled": list(record.get("culled_slots", [])),
        "inspected": bool(record.get("defect_slots") is not None
                          and any(s.get("state") == "INSPECT" for s in stages)),
        "wall": record.get("wall", ""),
        "source": filename,
        "slowest": (None if slowest is None
                    else {"state": slowest.get("state", ""),
                          "seconds": slowest.get("seconds", 0)}),
    }
