"""Sim Task가 검출을 보관해도 물리 명령을 시작하지 않는지 확인한다."""

import json
from pathlib import Path
import sys

import rclpy
from std_msgs.msg import String

sys.path.insert(0, str(Path(__file__).resolve().parent))
from sim_task_node import SimTaskCommand, SimTaskNode
from test_inspection_detection_store import context, sample_detections


def message(payload):
    result = String()
    result.data = json.dumps(payload)
    return result


def test_inspection_detections_are_stored_without_starting_cull():
    owns_context = not rclpy.ok()
    if owns_context:
        rclpy.init()
    node = SimTaskNode(supported_operations={"PREPARE_INSPECT"})
    try:
        node.mark_inspection_prepared(SimTaskCommand(
            task_id="TASK-001",
            command_id="TASK-001-CMD-006",
            operation="PREPARE_INSPECT",
            pallet_id="PALLET_001",
        ))
        node._inspection_context_callback(message(context()))
        node._inspection_detections_callback(message(sample_detections()))

        assert node.inspection_data.data is not None
        assert node.inspection_data.data["inspection_command_id"] == "TASK-001-CMD-007"
        assert node.active_command is None
        assert node._queued_command is None
    finally:
        node.destroy_node()
        if owns_context:
            rclpy.shutdown()


def test_recheck_detections_do_not_replace_initial_cull_data():
    owns_context = not rclpy.ok()
    if owns_context:
        rclpy.init()
    node = SimTaskNode(supported_operations={"PREPARE_INSPECT"})
    try:
        node.mark_inspection_prepared(SimTaskCommand(
            task_id="TASK-001", command_id="TASK-001-CMD-006",
            operation="PREPARE_INSPECT", pallet_id="PALLET_001",
        ))
        node._inspection_context_callback(message(context()))
        node._inspection_detections_callback(message(sample_detections()))
        recheck = sample_detections(command_id="TASK-001-CMD-009")
        recheck["operation"] = "RECHECK"
        recheck["valid_for_cull"] = False
        node._inspection_detections_callback(message(recheck))
        assert node.inspection_data.data["inspection_command_id"] == "TASK-001-CMD-007"
    finally:
        node.destroy_node()
        if owns_context:
            rclpy.shutdown()
