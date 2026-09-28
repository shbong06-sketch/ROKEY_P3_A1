"""트레이 자세와 포기 칸 배치로 Cull 파지 목표를 계산한다."""

import numpy as np


def seat_offset_local(tray_world_matrix, head_world_position):
    """검사 전 포기 위치를 트레이의 로컬 좌표로 보관한다."""
    matrix = np.asarray(tray_world_matrix, dtype=float)
    head = np.asarray(head_world_position, dtype=float)
    if matrix.shape != (4, 4) or head.shape != (3,):
        raise ValueError("tray matrix and head position have invalid dimensions")
    if not np.all(np.isfinite(matrix)) or not np.all(np.isfinite(head)):
        raise ValueError("tray matrix and head position must be finite")
    return np.linalg.solve(matrix[:3, :3], head - matrix[:3, 3])


def pick_target_world(tray_world_matrix, offset_local, aim_above_tray):
    """현재 트레이 자세를 반영한 월드 XY와 트레이 평면의 목표 Z를 반환한다."""
    matrix = np.asarray(tray_world_matrix, dtype=float)
    offset = np.asarray(offset_local, dtype=float)
    if matrix.shape != (4, 4) or offset.shape != (3,):
        raise ValueError("tray matrix and slot offset have invalid dimensions")
    if (not np.all(np.isfinite(matrix)) or not np.all(np.isfinite(offset))
            or not np.isfinite(aim_above_tray)):
        raise ValueError("Cull geometry must be finite")
    target = matrix[:3, 3] + matrix[:3, :3] @ offset
    target[2] = matrix[2, 3] + float(aim_above_tray)
    return target


def head_inside_box(head_position, lower_bound, upper_bound):
    """물리 안정화 뒤 머리 기준점의 XY가 상자 안이고 윗면보다 낮은지 확인한다."""
    head = np.asarray(head_position, dtype=float)
    lower = np.asarray(lower_bound, dtype=float)
    upper = np.asarray(upper_bound, dtype=float)
    if any(item.shape != (3,) for item in (head, lower, upper)):
        raise ValueError("Cull box bounds require xyz positions")
    if not all(np.all(np.isfinite(item)) for item in (head, lower, upper)):
        raise ValueError("Cull box bounds must be finite")
    return bool(np.all(lower[:2] <= head[:2]) and np.all(head[:2] <= upper[:2])
                and head[2] <= upper[2])
