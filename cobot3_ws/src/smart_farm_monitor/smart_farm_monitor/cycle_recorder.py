"""
시연 관찰·기록 노드

아키텍처 설계도의 OBS["사용자·기록"] 자리를 채웁니다. 공정 사이클과 각 executor의
명령·상태·결과를 하나도 빠뜨리지 않고 JSONL 파일로 남깁니다.

왜 필요한가
    status Topic 은 VOLATILE 입니다. 지금 받는 사람이 없으면 그 순간 사라지고,
    나중에 붙은 관찰자는 과거를 볼 수 없습니다. 대시보드를 만들든 안 만들든,
    시연 기록이 남으려면 누군가는 계속 듣고 있어야 합니다.

쓰는 법
    ros2 run smart_farm_monitor cycle_recorder
    ros2 run smart_farm_monitor cycle_recorder --ros-args -p output_dir:=~/records

남는 것
    records/cycle_20260922_154012.jsonl      한 줄 = 한 사건
    마지막에 cycle_summary 한 줄 (단계별 소요 시간)

기록 형식 (JSON Lines)
    {"t": 12.345, "wall": "...", "kind": "status", "topic": "/sim_task/status",
     "executor": "sim_task", "state": "RUNNING", "task_id": "...", "phase": "..."}

    t    : /clock 기준 경과 초. use_sim_time 을 쓰면 시뮬레이션 시간입니다.
    wall : 실제 시각. 시뮬레이션이 멈춰도 흐릅니다.
"""

import json
import os
from datetime import datetime
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile

from std_msgs.msg import String

from smart_farm_monitor.topics import (
    CYCLE_END,
    JSON_TOPICS,
    QOS_DEPTH,
    TYPED_TOPICS,
    fields,
)


class CycleRecorder(Node):
    """모든 공정 Topic 을 듣고 JSONL 로 남깁니다. 아무것도 발행하지 않습니다."""

    def __init__(self):
        super().__init__("cycle_recorder")

        self.declare_parameter("output_dir", "records")
        directory = Path(os.path.expanduser(
            str(self.get_parameter("output_dir").value)))
        directory.mkdir(parents=True, exist_ok=True)

        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self._path = directory / f"cycle_{stamp}.jsonl"
        self._file = self._path.open("a", encoding="utf-8")

        self._start_ns = None        # 첫 사건 시각. t 의 기준점입니다.
        self._cycle_start_ns = None  # 사이클이 수락된 시각. 소요시간의 기준점입니다.
        self._state_since = None     # 지금 공정 단계에 들어선 시각
        self._state = None
        self._durations = []         # [(단계, 초)]
        self._defects = []           # 검사에서 불량으로 나온 슬롯
        self._unknowns = []          # 판정하지 못한 슬롯
        self._culled = []            # 솎아내기를 마친 슬롯

        qos = QoSProfile(depth=QOS_DEPTH)
        for topic, message_type, kind in TYPED_TOPICS:
            self.create_subscription(
                message_type, topic,
                lambda message, t=topic, k=kind: self._on_typed(message, t, k),
                qos,
            )
        for topic, kind in JSON_TOPICS:
            self.create_subscription(
                String, topic,
                lambda message, t=topic, k=kind: self._on_json(message, t, k),
                qos,
            )

        listening = len(TYPED_TOPICS) + len(JSON_TOPICS)
        self.get_logger().info(f"기록 시작: {self._path}  (Topic {listening}개)")

    # ── 수신 ────────────────────────────────────────
    def _on_typed(self, message, topic, kind):
        self._write(kind, topic, fields(message))

    def _on_json(self, message, topic, kind):
        """Sim Task 의 String JSON. 깨진 payload 도 버리지 않고 원문을 남깁니다."""
        try:
            payload = json.loads(message.data)
        except (json.JSONDecodeError, TypeError):
            payload = {"raw": message.data, "parse_error": True}
        self._write(kind, topic, payload)

    # ── 기록 ────────────────────────────────────────
    def _write(self, kind, topic, payload):
        now_ns = self.get_clock().now().nanoseconds
        if self._start_ns is None:
            self._start_ns = now_ns

        record = {
            "t": round((now_ns - self._start_ns) / 1e9, 3),
            "wall": datetime.now().isoformat(timespec="milliseconds"),
            "kind": kind,
            "topic": topic,
        }
        record.update(payload)
        self._emit(record)

        if kind == "cycle":
            self._track_cycle(record, now_ns)
        elif kind == "result":
            self._track_slots(record)

    def _track_slots(self, record):
        """검사·솎아내기 결과를 모아 둡니다. 사이클 요약에 같이 남기려고요."""
        operation = record.get("operation", "")
        if operation == "INSPECT":
            self._defects = list(record.get("defect_slots", []))
            self._unknowns = list(record.get("unknown_slots", []))
        elif operation == "CULL":
            self._culled = list(record.get("completed_units", []))

    def _track_cycle(self, record, now_ns):
        """공정 단계가 바뀔 때마다 머문 시간을 재고, 끝나면 요약 한 줄을 남깁니다."""
        state = record.get("state", "")

        # 사이클 소요시간은 '수락된 순간' 부터 잽니다. recorder 시작 기준으로 재면
        # launch 후 시작 요청까지 기다린 시간이 통째로 섞여 들어갑니다.
        if record.get("status", "") == "ACCEPTED" and self._cycle_start_ns is None:
            self._cycle_start_ns = now_ns
            self._durations = []

        if state and state != self._state:
            # 사이클 전(IDLE)에 머문 시간은 소요시간이 아닙니다.
            if (self._state is not None and self._state_since is not None
                    and self._cycle_start_ns is not None):
                self._durations.append(
                    (self._state, round((now_ns - self._state_since) / 1e9, 3)))
            self._state = state
            self._state_since = now_ns

        if record.get("status", "") in CYCLE_END:
            if (self._state is not None and self._state_since is not None
                    and self._cycle_start_ns is not None):
                self._durations.append(
                    (self._state, round((now_ns - self._state_since) / 1e9, 3)))
            elapsed = (round((now_ns - self._cycle_start_ns) / 1e9, 3)
                       if self._cycle_start_ns is not None else None)
            self._emit({
                "t": record["t"],
                "wall": record["wall"],
                "kind": "cycle_summary",
                "topic": "",
                "task_id": record.get("task_id", ""),
                "scenario_id": record.get("scenario_id", ""),
                "status": record.get("status", ""),
                "reason": record.get("reason", ""),
                "total_seconds": elapsed,     # 수락 시각부터 종료까지
                "defect_slots": list(self._defects),
                "unknown_slots": list(self._unknowns),
                "culled_slots": list(self._culled),
                "state_seconds": [
                    {"state": name, "seconds": seconds}
                    for name, seconds in self._durations
                ],
            })
            self.get_logger().info(
                f"사이클 종료 {record.get('status', '')} — "
                f"{elapsed}초, 단계 {len(self._durations)}개 기록")
            self._cycle_start_ns = None
            self._state = None
            self._state_since = None
            self._durations = []
            self._defects = []
            self._unknowns = []
            self._culled = []

    def _emit(self, record):
        """한 줄 쓰고 바로 flush 합니다. 중간에 죽어도 거기까지는 남습니다."""
        self._file.write(json.dumps(record, ensure_ascii=False) + "\n")
        self._file.flush()

    def destroy_node(self):
        self._file.close()
        self.get_logger().info(f"기록 종료: {self._path}")
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CycleRecorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
