#!/usr/bin/env bash
# WSL 쪽: 중계기(ros_tcp_relay.py wsl) -> 팀 Nav2(nav2.launch.py) -> feeder_dock -> navigation_node. 로그는 OUT 아래.
#   bash start_nav2_stack.sh OUT_DIR [rviz true|false]      (run_allinone.ps1 이 부른다)
# 환경 변수: STANDOFF (feeder_dock standoff_m), HUMAN=1 (사람 돌발상황), LANE=1 (바닥 차선 주행)
# 전제: ~/nav2_env.sh 가 ROS 2 Jazzy + 팀 워크스페이스(cobot3_ws/install)를 source 한다.
HERE=$(cd "$(dirname "$0")" && pwd)
OUT=${1:?OUT_DIR}
RVIZ=${2:-false}
mkdir -p "$OUT"
source ~/nav2_env.sh
pkill -f ros_tcp_relay.py; pkill -f nav2.launch.py; pkill -f navigation_node; pkill -f component_container; pkill -f lane_planner.py; sleep 1
nohup python3 "$HERE/ros_tcp_relay.py" wsl > "$OUT/relay_wsl.log" 2>&1 &
echo "relay pid $!"
# Isaac 의 /clock 이 중계기로 들어올 때까지 기다린 뒤 Nav2 를 띄운다 (AMCL·costmap 이 /clock + tf 필요)
for i in $(seq 1 900); do
  if grep -q "/clock=" "$OUT/relay_wsl.log" 2>/dev/null; then break; fi
  sleep 1
done
grep -m1 "connected" "$OUT/relay_wsl.log"
EXTRA=""
if [ "$HUMAN" = "1" ] || [ "$LANE" = "1" ]; then
  # 팀 nav2_params.yaml 을 읽어 시험용 복사본을 만든다 (팀 파일은 그대로)
  SRC=$(ros2 pkg prefix smart_farm_navigation)/share/smart_farm_navigation/config/nav2_params.yaml
  FLAGS=""
  [ "$HUMAN" = "1" ] && FLAGS="$FLAGS --human"
  if [ "$LANE" = "1" ]; then
    cp "$HERE/lane_route_bt.xml" "$OUT/lane_route_bt.xml"
    FLAGS="$FLAGS --lane $OUT/lane_route_bt.xml"
    nohup python3 "$HERE/lane_planner.py" --ros-args -p use_sim_time:=true > "$OUT/lane_planner.log" 2>&1 &
    echo "lane_planner pid $!"
  fi
  python3 "$HERE/make_nav2_test_params.py" "$SRC" "$OUT/nav2_params_test.yaml" $FLAGS
  EXTRA="params_file:=$OUT/nav2_params_test.yaml"
fi
nohup ros2 launch smart_farm_navigation nav2.launch.py use_rviz:=$RVIZ record:=false $EXTRA > "$OUT/nav2.log" 2>&1 &
echo "nav2 pid $!"
for i in $(seq 1 180); do
  if grep -q "Managed nodes are active" "$OUT/nav2.log"; then echo "nav2 active after ${i}s"; break; fi
  sleep 1
done
if [ "$HUMAN" = "1" ]; then   # 정지 영역 기록 (action_type 0=해제 1=정지 2=감속, polygon_name)
  nohup ros2 topic echo /collision_monitor_state nav2_msgs/msg/CollisionMonitorState > "$OUT/collision_state.log" 2>&1 &
fi
if [ -n "$STANDOFF" ]; then
  # feeder_dock 을 standoff_m 을 바꿔 다시 띄운다 (파라미터는 시작 때만 읽음)
  pkill -f "lib/smart_farm_navigation/feeder_dock"; sleep 1
  nohup ros2 run smart_farm_navigation feeder_dock --ros-args -p use_sim_time:=true -p standoff_m:=$STANDOFF > "$OUT/feeder_dock.log" 2>&1 &
  echo "feeder_dock restarted with standoff_m=$STANDOFF"
fi
nohup ros2 launch smart_farm_navigation navigation_node.launch.py > "$OUT/navnode.log" 2>&1 &
echo "navigation_node pid $!"
sleep 8
grep -m2 -E "ready|destinations" "$OUT/navnode.log"
