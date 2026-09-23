"""
공정 Topic 목록과 공통 변환

기록 노드와 대시보드가 같은 목록을 봐야 합니다. 한쪽만 Topic 을 추가하면
기록에는 남는데 화면에는 안 뜨는 식으로 조용히 어긋납니다.
"""

from smart_farm_interfaces.msg import CycleStatus, ExecutorStatus, TaskCommand, TaskResult


# 설계 문서 14장: command/result 와 status 모두 RELIABLE, KEEP_LAST, depth 10
QOS_DEPTH = 10

# 타입이 있는 Topic. (경로, 메시지 타입, 기록 종류)
TYPED_TOPICS = (
    ("/cycle/status", CycleStatus, "cycle"),
    ("/navigation/command", TaskCommand, "command"),
    ("/navigation/result", TaskResult, "result"),
    ("/navigation/status", ExecutorStatus, "status"),
    ("/inspection/command", TaskCommand, "command"),
    ("/inspection/result", TaskResult, "result"),
    ("/inspection/status", ExecutorStatus, "status"),
)

# Sim Task 는 std_msgs/String 에 JSON 을 담습니다 (설계 문서 3장).
# 필드 의미는 위 타입들과 같습니다.
JSON_TOPICS = (
    ("/sim_task/command", "command"),
    ("/sim_task/result", "result"),
    ("/sim_task/status", "status"),
)

# 사이클이 끝났다고 보는 status 값 (CycleStatus 계약)
CYCLE_END = ("SUCCEEDED", "FAILED", "TIMEOUT", "RESET_REQUIRED")

# 공정 단계. task_manager 상태 머신과 같은 순서입니다.
CYCLE_STATES = (
    "PREFLIGHT",
    "TRANSFER",
    "PICK_HARVEST",
    "NAVIGATION",
    "PLACE_INSPECT",
    "INSPECT",
    "CULL",
    "CONVEYOR_OUT",
)

EXECUTORS = ("sim_task", "navigation", "inspection")

# 팔레트 한 장의 식물 슬롯.
# 6구입니다. docs/01-architecture.md 2장, docs/02-interfaces.md 13장,
# inspection_executor_node.SLOT_IDS, state_machine 모두 SLOT_01~SLOT_06 입니다.
# 8구로 두면 대시보드가 SLOT_07·SLOT_08 을 그리는데 거기엔 상태가 오지 않습니다.
SLOTS = tuple(f"SLOT_{n:02d}" for n in range(1, 7))


def fields(message):
    """ROS 메시지를 그대로 dict 로 폅니다. 필드가 늘어도 코드를 고칠 일이 없습니다."""
    record = {}
    for name in message.get_fields_and_field_types():
        value = getattr(message, name)
        record[name] = list(value) if isinstance(value, (list, tuple)) else value
    return record


# 공통 reason 어휘 (설계 문서 6장). 화면에서 뜻을 풀어 주려고 둡니다.
# 코드만 보고는 어디를 봐야 할지 알기 어렵습니다.
REASON_HINTS = {
    "NONE": "정상",
    "INVALID_COMMAND": "명령 형식이 계약과 다릅니다",
    "INVALID_ID": "task_id 또는 command_id 가 맞지 않습니다",
    "BUSY": "이미 다른 작업을 수행 중입니다",
    "NOT_READY": "수신자가 아직 준비되지 않았습니다",
    "SIM_NOT_READY": "Sim Task Executor 가 준비되지 않았습니다",
    "NAV_NOT_READY": "Navigation Node 가 준비되지 않았습니다",
    "INSPECTION_NOT_READY": "Inspection Node 가 준비되지 않았습니다",
    "BASE_NOT_STOPPED": "AMR 이 정지하지 않은 상태에서 팔을 쓰려 했습니다",
    "DOCKING_ERROR": "도킹 위치·자세가 허용 범위를 벗어났습니다",
    "LIFT_FAILED": "리프트가 목표 높이에 도달하지 못했습니다",
    "MOTION_FAILED": "팔 동작이 실패했습니다",
    "PICK_VERIFY_FAILED": "집기는 했지만 상승·인출·미끄러짐 확인에 실패했습니다",
    "PLACE_VERIFY_FAILED": "내려놓기 후 안착 확인에 실패했습니다",
    "TRANSPORT_NOT_SAFE": "운송 자세·높이가 이동 조건을 만족하지 않습니다",
    "NAV_FAILED": "Nav2 주행이 실패했습니다",
    "IMAGE_TIMEOUT": "검사 영상이 오지 않았습니다",
    "INSPECTION_FAILED": "검사 추론이 실패했습니다",
    "UNKNOWN_SLOT": "판정하지 못한 슬롯이 있습니다",
    "CULL_FAILED": "솎아내기가 실패했습니다",
    "CONVEYOR_FAILED": "컨베이어 배출이 실패했습니다",
    "RESULT_TIMEOUT": "결과를 기다리다 제한 시간을 넘겼습니다",
    # /start_cycle 이 거절할 때 쓰는 사유. task_manager 는 프로세스당 사이클
    # 한 번만 돌리므로, 끝난 뒤에 다시 누르면 종료 상태가 그대로 사유가 됩니다.
    "SUCCEEDED": "이미 끝난 사이클입니다 — task_manager 를 재시작해야 다시 시작할 수 있습니다",
    "FAILED": "실패로 끝난 사이클입니다 — task_manager 를 재시작해야 다시 시작할 수 있습니다",
    "NOT_IDLE": "사이클이 진행 중입니다",
    "RESET_REQUIRED": "장면과 논리 상태를 함께 초기화해야 합니다",
}

# 사이클이 정상 종료가 아닌 경우
BAD_STATUS = ("FAILED", "TIMEOUT", "RESET_REQUIRED")
