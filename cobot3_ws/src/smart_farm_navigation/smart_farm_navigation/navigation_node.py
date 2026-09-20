"""Navigation Node (/cmd_vel 판): 인터페이스 설계의 command/result 계약을 따르는 주행 지시 수신 노드.

  /navigation/command  (std_msgs/String, UTF-8 JSON)  <- Task Manager / 통합 스크립트
      {"command_id": "...", "task_id": "...", "operation": "NAVIGATION", "destination": "INSPECTION_DOCK"}
  /navigation/result   (std_msgs/String, JSON)        -> 요청 ID 를 그대로 되돌림
      {"command_id", "task_id", "operation", "status": SUCCEEDED|FAILED|TIMEOUT, "phase", "reason", "reached_station"}
  /navigation/status   (std_msgs/String, JSON, TRANSIENT_LOCAL) state READY|BUSY|ERROR

명령을 받으면 config/destinations.yaml 에 적힌 launch 를 자식 프로세스로 띄우고(path_runner_smooth 등),
종료 코드로 결과를 만든다. 한 번에 하나만 실행하며 BUSY 중 새 명령은 FAILED/BUSY 로 응답한다.
smart_farm_interfaces 에 TaskCommand/TaskResult 메시지가 생기면 이 노드의 파서만 바꾸면 된다.
"""

import json
import os
import subprocess
import time

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

REQUIRED = ("command_id", "task_id", "destination")


class NavigationNode(Node):
    def __init__(self) -> None:
        super().__init__("navigation_node")
        share = get_package_share_directory("smart_farm_navigation")
        self.declare_parameter("destinations_file", os.path.join(share, "config", "destinations.yaml"))
        self.declare_parameter("timeout_s", 180.0)
        self.declare_parameter("executor", "navigation")
        self.dest = yaml.safe_load(open(self.get_parameter("destinations_file").value))["destinations"]
        self.timeout = float(self.get_parameter("timeout_s").value)
        self.executor_name = self.get_parameter("executor").value
        self.share = share

        cmd_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        status_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(String, "/navigation/command", self._on_command, cmd_qos)
        self.result_pub = self.create_publisher(String, "/navigation/result", cmd_qos)
        self.status_pub = self.create_publisher(String, "/navigation/status", status_qos)

        self.active = None          # dict of the running command
        self.proc = None
        self.started_at = 0.0
        self.done_ids = {}          # command_id -> result json (재수신 시 재발행)
        self.create_timer(0.2, self._poll)
        self._status("READY", "", "", "IDLE", "waiting for /navigation/command")
        self.get_logger().info(f"navigation_node ready; destinations: {list(self.dest)}")

    # ---------- publish helpers ----------
    def _status(self, state, task_id, command_id, phase, detail) -> None:
        self.status_pub.publish(String(data=json.dumps({
            "executor": self.executor_name, "state": state, "task_id": task_id, "command_id": command_id,
            "operation": "NAVIGATION", "phase": phase, "detail": detail}, ensure_ascii=False)))

    def _result(self, cmd, status, reason, phase, reached="") -> None:
        payload = {"command_id": cmd.get("command_id", ""), "task_id": cmd.get("task_id", ""),
                   "operation": "NAVIGATION", "status": status, "phase": phase, "reason": reason,
                   "safe_to_navigate": False, "reached_station": reached}
        self.done_ids[payload["command_id"]] = payload
        self.result_pub.publish(String(data=json.dumps(payload, ensure_ascii=False)))
        self.get_logger().info(f"result {status}/{reason} for {payload['command_id']} (phase {phase})")

    # ---------- command handling ----------
    def _on_command(self, msg: String) -> None:
        try:
            cmd = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().error(f"invalid JSON on /navigation/command: {msg.data[:120]}")
            return
        if not all(k in cmd for k in REQUIRED):
            self._result(cmd, "FAILED", "INVALID_COMMAND", "VALIDATE"); return
        if cmd.get("operation", "NAVIGATION") != "NAVIGATION":
            self._result(cmd, "FAILED", "INVALID_COMMAND", "VALIDATE"); return
        cid = cmd["command_id"]
        if cid in self.done_ids:                       # 완료된 명령 재수신: 재실행하지 않고 결과 재발행
            self.result_pub.publish(String(data=json.dumps(self.done_ids[cid], ensure_ascii=False))); return
        if self.active is not None:
            self._result(cmd, "FAILED", "BUSY", "VALIDATE"); return
        dest = self.dest.get(cmd["destination"])
        if dest is None:
            self._result(cmd, "FAILED", "INVALID_COMMAND", "VALIDATE"); return
        params = os.path.join(self.share, "config", dest["params"])
        argv = ["ros2", "launch", "smart_farm_navigation", dest["launch"], "auto_start:=true", f"params_file:={params}"]
        self.get_logger().info(f"command {cid}: {cmd['destination']} -> {' '.join(argv)}")
        self.proc = subprocess.Popen(argv)
        self.active = cmd
        self.started_at = time.monotonic()
        self._status("BUSY", cmd["task_id"], cid, "DRIVING", cmd["destination"])

    def _poll(self) -> None:
        if self.active is None:
            return
        rc = self.proc.poll()
        if rc is None:
            if time.monotonic() - self.started_at > self.timeout:
                self.proc.terminate()
                try:
                    self.proc.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    self.proc.kill()
                self._finish("TIMEOUT", "RESULT_TIMEOUT", "DRIVING")
            return
        if rc == 0:
            self._finish("SUCCEEDED", "NONE", "ARRIVED", self.active["destination"])
        elif rc == 2:
            self._finish("FAILED", "NAV_FAILED", "DRIVING")
        else:
            self._finish("FAILED", "NAV_FAILED", "LAUNCH")

    def _finish(self, status, reason, phase, reached="") -> None:
        cmd, self.active, self.proc = self.active, None, None
        self._result(cmd, status, reason, phase, reached)
        self._status("READY", "", "", "IDLE", f"last {status}")

    def shutdown(self) -> None:
        if self.proc is not None and self.proc.poll() is None:
            self.proc.terminate()
            try:
                self.proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                self.proc.kill()


def main() -> None:
    rclpy.init()
    node = NavigationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
