"""Real-Nav2 flow, issued like the team's guidance (guidance2_25 section 8), from WSL:
PICK_HARVEST (sim) -> NAVIGATION FEEDER_DOCK (navigation_node: Nav2 NavigateToPose + feeder_dock) -> PLACE_INSPECT (sim).

  python3 run_flow.py OUT_JSON [--skip-place]
"""
import json, sys, time
import rclpy
from std_msgs.msg import String
from smart_farm_interfaces.msg import TaskCommand, TaskResult

OUT = sys.argv[1]
TASK = "TASK-CABBAGE-%s" % time.strftime("%H%M%S")
log = {"task": TASK, "events": []}


def ev(text):
    log["events"].append([round(time.time(), 1), text])
    print("[flow]", text, flush=True)


rclpy.init()
node = rclpy.create_node("cabbage_flow")
sim_pub = node.create_publisher(String, "/sim_task/command", 10)
nav_pub = node.create_publisher(TaskCommand, "/navigation/command", 10)
sim_results, nav_results, status = {}, {}, {}
node.create_subscription(String, "/sim_task/result", lambda m: sim_results.__setitem__(json.loads(m.data)["command_id"], json.loads(m.data)), 10)
node.create_subscription(String, "/sim_task/status", lambda m: status.update(json.loads(m.data)), 10)
node.create_subscription(TaskResult, "/navigation/result", lambda m: nav_results.__setitem__(m.command_id, m), 10)


def spin(sec):
    end = time.time() + sec
    while time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.05)


def sim_command(cid, body, timeout):
    msg = String(); msg.data = json.dumps({"task_id": TASK, "command_id": cid, **body})
    ev(f"sim command {body['operation']} {cid}")
    t0 = time.time(); last = 0
    while time.time() - t0 < timeout:
        if cid not in sim_results and status.get("command_id") != cid and time.time() - last > 5:
            sim_pub.publish(msg); last = time.time()
        spin(0.2)
        if cid in sim_results:
            r = sim_results[cid]; ev(f"sim result {r.get('status')} phase={r.get('phase')} reason={r.get('reason')}")
            return r
    raise RuntimeError(f"no sim result for {cid}")


def nav_command(cid, destination, timeout):
    msg = TaskCommand(task_id=TASK, command_id=cid, operation="NAVIGATION", destination=destination)
    ev(f"navigation command {destination} {cid}")
    t0 = time.time()
    while nav_pub.get_subscription_count() == 0 and time.time() - t0 < 30:
        spin(0.5)
    nav_pub.publish(msg)
    while time.time() - t0 < timeout:
        spin(0.2)
        if cid in nav_results:
            r = nav_results[cid]
            ev(f"navigation result {r.status} reason={r.reason} phase={r.phase} reached={r.reached_station}")
            return {"status": r.status, "reason": r.reason, "phase": r.phase, "reached_station": r.reached_station}
    raise RuntimeError(f"no navigation result for {cid}")


try:
    ev("waiting for sim_task READY")
    while status.get("state") != "READY":
        spin(0.5)
    log["pick"] = sim_command(TASK + "-CMD-001", {"operation": "PICK_HARVEST", "recipe_id": "HARVEST_RACK_L1",
                              "pallet_id": "PALLET_001", "source": "RACK_L1", "destination": "CARRY"}, 900)
    if log["pick"].get("status") == "SUCCEEDED":
        spin(2)
        log["nav"] = nav_command(TASK + "-CMD-002", "FEEDER_DOCK", 1200)
        if log["nav"]["status"] == "SUCCEEDED" and "--skip-place" not in sys.argv:
            spin(3)
            log["place"] = sim_command(TASK + "-CMD-003", {"operation": "PLACE_INSPECT", "recipe_id": "PLACE_AT_INSPECTION",
                                       "pallet_id": "PALLET_001", "source": "CARRY", "destination": "INSPECT_STATION"}, 900)
except Exception as e:  # noqa: BLE001
    log["error"] = repr(e); ev("error " + repr(e))
finally:
    json.dump(log, open(OUT, "w"), indent=1)
    node.destroy_node(); rclpy.shutdown()
