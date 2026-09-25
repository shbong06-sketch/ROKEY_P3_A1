#!/usr/bin/env bash
# WSL side of the real Nav2 test: relay server, team Nav2 (nav2.launch.py) and navigation_node, logs under $OUT.
#   bash start_nav2_stack.sh OUT_DIR [rviz true|false]
OUT=${1:-/mnt/d/smartfarm-sim/out/nav2_real}
RVIZ=${2:-false}
mkdir -p "$OUT"
source ~/nav2_env.sh
pkill -f ros_tcp_relay.py; pkill -f nav2.launch.py; pkill -f navigation_node; pkill -f component_container; sleep 1
nohup python3 /mnt/d/smartfarm-sim/scripts/wsl/ros_tcp_relay.py wsl > "$OUT/relay_wsl.log" 2>&1 &
echo "relay pid $!"
# wait until Isaac's clock arrives through the relay before starting Nav2 (AMCL / costmaps need /clock + tf)
for i in $(seq 1 900); do
  if grep -q "/clock=" "$OUT/relay_wsl.log" 2>/dev/null; then break; fi
  sleep 1
done
grep -m1 "connected" "$OUT/relay_wsl.log"
nohup ros2 launch smart_farm_navigation nav2.launch.py use_rviz:=$RVIZ record:=false > "$OUT/nav2.log" 2>&1 &
echo "nav2 pid $!"
for i in $(seq 1 180); do
  if grep -q "Managed nodes are active" "$OUT/nav2.log"; then echo "nav2 active after ${i}s"; break; fi
  sleep 1
done
if [ -n "$STANDOFF" ]; then
  # guidance2_25 section 9: re-run feeder_dock alone with another standoff_m (params are read only at start)
  pkill -f "lib/smart_farm_navigation/feeder_dock"; sleep 1
  nohup ros2 run smart_farm_navigation feeder_dock --ros-args -p use_sim_time:=true -p standoff_m:=$STANDOFF > "$OUT/feeder_dock.log" 2>&1 &
  echo "feeder_dock restarted with standoff_m=$STANDOFF"
fi
nohup ros2 launch smart_farm_navigation navigation_node.launch.py > "$OUT/navnode.log" 2>&1 &
echo "navigation_node pid $!"
sleep 8
grep -m2 -E "ready|destinations" "$OUT/navnode.log"
