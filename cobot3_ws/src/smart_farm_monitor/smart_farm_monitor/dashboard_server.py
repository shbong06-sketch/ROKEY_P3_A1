"""
시연 대시보드 서버

공정 Topic 을 듣고, 브라우저에 실시간으로 그려 줍니다. 시연 중에 터미널
여러 개를 번갈아 보지 않아도 되게 하는 것이 목적입니다.

    ros2 run smart_farm_monitor dashboard_server
    브라우저에서 http://localhost:8080

rosbridge 를 쓰지 않습니다. 이 노드가 직접 페이지를 주고 Server-Sent Events 로
상태를 흘려보냅니다. 설치할 것도, 인터넷에서 받아올 것도 없습니다.

보여 주는 것
    · 공정 8단계 중 지금 어디인지, 각 단계가 몇 초 걸렸는지
    · executor 3개의 상태·phase·마지막 소식이 몇 초 전인지
    · 멈췄다면 어느 단계에서 무슨 reason 인지
"""

import json
import queue
import threading
from datetime import datetime
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from rclpy.qos import QoSProfile

from std_msgs.msg import String

from smart_farm_interfaces.srv import StartCycle

from smart_farm_monitor.records import load_summaries
from smart_farm_monitor.topics import (
    BAD_STATUS,
    CYCLE_END,
    CYCLE_STATES,
    EXECUTORS,
    JSON_TOPICS,
    QOS_DEPTH,
    REASON_HINTS,
    SLOTS,
    TYPED_TOPICS,
    fields,
)


class Board:
    """화면에 보여 줄 현재 상태. ROS 스레드와 웹 스레드가 같이 씁니다."""

    def __init__(self, scenario_id):
        self._lock = threading.Lock()
        self._clients = []          # SSE 로 연결된 브라우저들
        self.scenario_id = scenario_id
        self.history = []           # 끝난 사이클 기록. reset 으로 지우지 않습니다.
        self.manager_ready = False  # /start_cycle 서비스가 보이는가 (연결 상태)
        self.cycle_seen = False     # /cycle/status 를 한 번이라도 받았는가
        self.reset()

    def reset(self):
        with self._lock:
            self.cycle = {"task_id": "", "scenario_id": "", "state": "IDLE",
                          "status": "", "reason": ""}
            self.state_seconds = {}      # 단계 → 걸린 초
            self.state_since = None      # 지금 단계에 들어선 시각
            self.cycle_start = None
            self.executors = {
                name: {"state": "", "phase": "", "detail": "",
                       "operation": "", "last_seen": None}
                for name in EXECUTORS
            }
            self.log = []                # 최근 사건 (뒤에서부터 보여 줍니다)
            self.last_command = {}       # executor → 마지막으로 받은 명령
            self.last_result = {}        # executor → 마지막으로 돌려준 결과
            self.defects = []            # 검사에서 불량으로 나온 슬롯
            self.unknowns = []           # 판정하지 못한 슬롯
            self.culled = []             # 솎아내기를 마친 슬롯
            self.inspected = False       # 이번 사이클에서 검사를 마쳤는가

    # ── 갱신 ────────────────────────────────────────
    def load_past(self, entries):
        """기록 파일에서 읽은 지난 사이클을 이력에 채웁니다 (시작할 때 한 번).

        이번에 돌린 사이클이 이미 있으면 건너뜁니다. 같은 사이클을 recorder 도
        쓰고 대시보드도 기억하기 때문입니다.
        """
        with self._lock:
            known = {entry["task_id"] for entry in self.history}
            for entry in entries:
                if entry["task_id"] and entry["task_id"] in known:
                    continue
                self.history.append(entry)
            self.history.sort(key=lambda entry: entry.get("wall", ""), reverse=True)
            del self.history[20:]

    def set_manager_ready(self, ready):
        with self._lock:
            self.manager_ready = bool(ready)

    def update_cycle(self, payload, now):
        self.cycle_seen = True
        with self._lock:
            state = payload.get("state", "")
            if payload.get("status", "") == "ACCEPTED" and self.cycle_start is None:
                self.cycle_start = now
                self.state_seconds = {}
            if state and state != self.cycle["state"]:
                if self.state_since is not None and self.cycle_start is not None:
                    self.state_seconds[self.cycle["state"]] = round(now - self.state_since, 2)
                self.state_since = now
            self.cycle.update({k: payload.get(k, "") for k in self.cycle})
            if payload.get("status", "") in CYCLE_END:
                if self.state_since is not None and self.cycle_start is not None:
                    self.state_seconds[state] = round(now - self.state_since, 2)
                stages = [{"state": name, "seconds": seconds}
                          for name, seconds in self.state_seconds.items()]
                slowest = max(stages, key=lambda item: item["seconds"], default=None)
                self.history.insert(0, {
                    "task_id": self.cycle.get("task_id", ""),
                    "status": payload.get("status", ""),
                    "reason": payload.get("reason", ""),
                    "seconds": (None if self.cycle_start is None
                                else round(now - self.cycle_start, 1)),
                    "defects": list(self.defects),
                    "culled": list(self.culled),
                    "inspected": self.inspected,
                    "wall": datetime.now().isoformat(timespec="milliseconds"),
                    "source": "live",
                    "slowest": slowest,
                })
                del self.history[20:]        # 최근 20 사이클만
                self.cycle_start = None
                self.state_since = None

    def update_executor(self, payload, now):
        name = payload.get("executor", "")
        if name not in self.executors:
            return
        with self._lock:
            self.executors[name].update({
                "state": payload.get("state", ""),
                "phase": payload.get("phase", ""),
                "detail": payload.get("detail", ""),
                "operation": payload.get("operation", ""),
                "last_seen": now,
            })

    def remember(self, kind, topic, payload):
        """실패했을 때 '무엇을 시켰고 무엇이 돌아왔는지' 를 보여 주려고 남깁니다."""
        name = topic.strip("/").split("/")[0]
        with self._lock:
            if kind == "command":
                self.last_command[name] = payload
            elif kind == "result":
                self.last_result[name] = payload
                operation = payload.get("operation", "")
                if operation == "INSPECT":
                    self.defects = list(payload.get("defect_slots", []))
                    self.unknowns = list(payload.get("unknown_slots", []))
                    self.inspected = True
                elif operation == "CULL":
                    self.culled = list(payload.get("completed_units", []))

    def add_log(self, kind, topic, payload):
        with self._lock:
            self.log.append({
                "wall": datetime.now().strftime("%H:%M:%S"),
                "kind": kind,
                "topic": topic,
                "text": self._summarize(kind, payload),
            })
            del self.log[:-40]          # 최근 40건만 들고 있습니다

    @staticmethod
    def _summarize(kind, payload):
        if kind == "command":
            return (f"{payload.get('operation', '')} → "
                    f"{payload.get('destination', '') or payload.get('pallet_id', '')}")
        if kind == "result":
            reason = payload.get("reason", "")
            tail = "" if reason in ("", "NONE") else f" ({reason})"
            return f"{payload.get('operation', '')} {payload.get('status', '')}{tail}"
        return f"{payload.get('state', '')} {payload.get('status', '')}".strip()

    def _start_state(self):
        """지금 '시연 시작' 을 누를 수 있는가. 누르기 전에 알려 주려고 봅니다.

        누른 뒤에 거절 사유를 보는 것과, 누르기 전에 아는 것은 다릅니다.
        특히 task_manager 는 프로세스당 한 사이클만 돌립니다 — state_machine.start()
        가 IDLE 에서만 받고, 노드가 reset() 을 부르지 않기 때문입니다.
        시연 중에 두 번째로 눌렀다가 당황하지 않게 미리 적어 둡니다.
        """
        if not self.manager_ready:
            return False, "down", "task_manager 가 떠 있지 않습니다"
        state = self.cycle.get("state", "")
        if self.cycle.get("status", "") in CYCLE_END or state in ("COMPLETE", "ERROR"):
            return (False, "done",
                    "이 사이클은 끝났습니다 — 다시 하려면 task_manager 를 재시작하세요")
        if state in ("", "IDLE"):
            if not self.cycle_seen:
                # /cycle/status 는 전이할 때만 옵니다. 대시보드를 나중에 켜면
                # 그 사이 소식을 못 받아 IDLE 로 보입니다. 단정하지 않습니다.
                return True, "unknown", "task_manager 상태를 아직 받지 못했습니다"
            return True, "ready", "시작할 수 있습니다"
        return False, "busy", f"진행 중 — {state}"

    # ── 화면에 줄 형태 ───────────────────────────────
    def snapshot(self, now):
        with self._lock:
            executors = {}
            for name, info in self.executors.items():
                ago = None if info["last_seen"] is None else round(now - info["last_seen"], 1)
                executors[name] = dict(info, ago=ago)
                executors[name].pop("last_seen")
            elapsed = None if self.cycle_start is None else round(now - self.cycle_start, 1)
            current = dict(self.state_seconds)
            if self.state_since is not None and self.cycle_start is not None:
                current[self.cycle["state"]] = round(now - self.state_since, 2)
            failed = self.cycle.get("status", "") in BAD_STATUS
            can_start, start_code, start_text = self._start_state()
            return {
                "start": {"can": can_start, "code": start_code, "text": start_text},
                "cycle": dict(self.cycle),
                "failed": failed,
                "reason_hint": REASON_HINTS.get(self.cycle.get("reason", ""), ""),
                "last_command": dict(self.last_command),
                "last_result": dict(self.last_result),
                "slots": SLOTS,
                "inspection": {
                    "done": self.inspected,
                    "defects": list(self.defects),
                    "unknowns": list(self.unknowns),
                    "culled": list(self.culled),
                },
                "history": list(self.history),
                "elapsed": elapsed,
                "states": CYCLE_STATES,
                "state_seconds": current,
                "executors": executors,
                "log": list(reversed(self.log)),
            }

    # ── 브라우저 연결 ────────────────────────────────
    def subscribe(self):
        stream = queue.Queue(maxsize=32)
        with self._lock:
            self._clients.append(stream)
        return stream

    def unsubscribe(self, stream):
        with self._lock:
            if stream in self._clients:
                self._clients.remove(stream)

    def broadcast(self, snapshot):
        line = f"data: {json.dumps(snapshot, ensure_ascii=False)}\n\n"
        with self._lock:
            targets = list(self._clients)
        for stream in targets:
            try:
                stream.put_nowait(line)
            except queue.Full:
                pass        # 느린 브라우저 때문에 노드가 밀리지 않게 그냥 버립니다


class StartRequest:
    """브라우저가 누른 '시작' 을 ROS 스레드로 넘기는 편지."""

    def __init__(self, scenario_id):
        self.scenario_id = scenario_id
        self.done = threading.Event()
        self.reply = None


def make_handler(board, page_path, starts):
    class Handler(BaseHTTPRequestHandler):
        def do_POST(self):
            if not self.path.startswith("/start"):
                self._send(404, "text/plain; charset=utf-8", b"not found")
                return
            # ROS 서비스 호출은 ROS 스레드에서만 합니다. 여기서는 부탁만 넣고 기다립니다.
            request = StartRequest(board.scenario_id)
            starts.put(request)
            ok = request.done.wait(timeout=5.0)
            reply = request.reply if ok else {
                "accepted": False, "task_id": "",
                "reason": "NO_RESPONSE", "hint": "task_manager 가 응답하지 않습니다",
            }
            self._send(200, "application/json; charset=utf-8",
                       json.dumps(reply, ensure_ascii=False).encode("utf-8"))

        def do_GET(self):
            if self.path.startswith("/events"):
                self._events()
            elif self.path in ("/", "/index.html"):
                # 요청마다 읽습니다. 화면을 고치는 동안 노드를 껐다 켜지 않아도
                # 새로고침만으로 확인할 수 있습니다. 파일 하나라 비용은 무시할 만합니다.
                try:
                    body = page_path.read_text(encoding="utf-8")
                except OSError as error:
                    self._send(500, "text/plain; charset=utf-8",
                               f"화면 파일을 읽지 못했습니다: {error}".encode("utf-8"))
                    return
                self._send(200, "text/html; charset=utf-8", body.encode("utf-8"))
            else:
                self._send(404, "text/plain; charset=utf-8", b"not found")

        def _send(self, code, content_type, body):
            self.send_response(code)
            self.send_header("Content-Type", content_type)
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        def _events(self):
            self.send_response(200)
            self.send_header("Content-Type", "text/event-stream; charset=utf-8")
            self.send_header("Cache-Control", "no-cache")
            self.end_headers()
            stream = board.subscribe()
            try:
                while True:
                    self.wfile.write(stream.get().encode("utf-8"))
                    self.wfile.flush()
            except (BrokenPipeError, ConnectionResetError):
                pass
            finally:
                board.unsubscribe(stream)

        def log_message(self, *args):
            pass        # 접속 로그로 터미널을 채우지 않습니다

    return Handler


class DashboardServer(Node):
    """Topic 을 듣고 브라우저로 흘려보냅니다. 아무것도 발행하지 않습니다."""

    def __init__(self):
        super().__init__("dashboard_server")
        self.declare_parameter("port", 8080)
        self.declare_parameter("scenario_id", "DEMO_HARVEST_01")
        # cycle_recorder 의 output_dir 과 같은 곳을 봅니다. 지난 사이클을 읽어 옵니다.
        self.declare_parameter("records_dir", "records")
        port = int(self.get_parameter("port").value)
        scenario_id = str(self.get_parameter("scenario_id").value)
        records_dir = str(self.get_parameter("records_dir").value)

        self._board = Board(scenario_id)
        past = load_summaries(records_dir)
        self._board.load_past(past)
        self.get_logger().info(
            f"지난 사이클 {len(past)}개를 {records_dir} 에서 읽었습니다"
            if past else f"{records_dir} 에 지난 기록이 없습니다")
        self._starts = queue.Queue()
        self._start_client = self.create_client(StartCycle, "/start_cycle")
        self._started = self.get_clock().now().nanoseconds / 1e9

        qos = QoSProfile(depth=QOS_DEPTH)
        for topic, message_type, kind in TYPED_TOPICS:
            self.create_subscription(
                message_type, topic,
                lambda message, t=topic, k=kind: self._on(k, t, fields(message)),
                qos,
            )
        for topic, kind in JSON_TOPICS:
            self.create_subscription(
                String, topic,
                lambda message, t=topic, k=kind: self._on_json(k, t, message),
                qos,
            )

        page_path = (Path(get_package_share_directory("smart_farm_monitor"))
                     / "web" / "index.html")
        if not page_path.is_file():
            raise FileNotFoundError(f"화면 파일이 없습니다: {page_path}")
        self._http = ThreadingHTTPServer(
            ("0.0.0.0", port),
            make_handler(self._board, page_path, self._starts))
        threading.Thread(target=self._http.serve_forever, daemon=True).start()

        # 화면은 4 Hz 로 갱신합니다. Topic 이 올 때마다 보내면 브라우저가 밀립니다.
        self.create_timer(0.25, self._tick)
        self.get_logger().info(f"대시보드: http://localhost:{port}")

    def _now(self):
        return self.get_clock().now().nanoseconds / 1e9 - self._started

    def _on(self, kind, topic, payload):
        now = self._now()
        if kind == "cycle":
            self._board.update_cycle(payload, now)
            self._board.add_log(kind, topic, payload)
        elif kind == "status":
            self._board.update_executor(payload, now)
        else:
            self._board.remember(kind, topic, payload)
            self._board.add_log(kind, topic, payload)

    def _on_json(self, kind, topic, message):
        try:
            payload = json.loads(message.data)
        except (json.JSONDecodeError, TypeError):
            return
        self._on(kind, topic, payload)

    def _tick(self):
        self._board.set_manager_ready(self._start_client.service_is_ready())
        self._drain_starts()
        self._board.broadcast(self._board.snapshot(self._now()))

    def _drain_starts(self):
        """브라우저가 누른 '시작' 을 ROS 서비스 호출로 바꿉니다."""
        while True:
            try:
                request = self._starts.get_nowait()
            except queue.Empty:
                return
            if not self._start_client.service_is_ready():
                request.reply = {
                    "accepted": False, "task_id": "", "reason": "NOT_READY",
                    "hint": "task_manager 의 /start_cycle 이 아직 없습니다",
                }
                request.done.set()
                continue
            message = StartCycle.Request()
            message.scenario_id = request.scenario_id
            future = self._start_client.call_async(message)
            future.add_done_callback(
                lambda done, r=request: self._start_replied(done, r))

    @staticmethod
    def _start_replied(future, request):
        try:
            response = future.result()
        except Exception as error:                      # noqa: BLE001
            request.reply = {"accepted": False, "task_id": "",
                             "reason": "CALL_FAILED", "hint": str(error)}
        else:
            request.reply = {
                "accepted": bool(response.accepted),
                "task_id": response.task_id,
                "reason": response.reason,
                "hint": REASON_HINTS.get(response.reason, ""),
            }
        request.done.set()

    def destroy_node(self):
        self._http.shutdown()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = DashboardServer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
