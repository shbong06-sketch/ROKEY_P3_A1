#!/bin/bash
# 녹화용 ROS 2 통신 화면: xterm 하나에 tmux 6칸, 토픽마다 타입을 주고 echo (나중에 생기는 토픽도 보인다).
# xterm 은 한글 글꼴이 없어 칸 제목은 영어로 쓴다. record_drive.sh 가 부른다.
S=comm
tmux kill-session -t $S 2>/dev/null
E="source ~/nav2_env.sh >/dev/null 2>&1; clear; printf '\033[1;34m%s\033[0m\n' "
tmux new-session -d -s $S -x 230 -y 62
tmux send-keys -t $S "$E '/sim_task/command  (Nav2 PC -> Isaac)'; ros2 topic echo /sim_task/command std_msgs/msg/String" C-m
tmux split-window -h -t $S
tmux send-keys -t $S "$E '/sim_task/result  (Isaac -> Nav2 PC)'; ros2 topic echo /sim_task/result std_msgs/msg/String" C-m
tmux select-pane -t $S:0.0; tmux split-window -v -t $S
tmux send-keys -t $S "$E '/navigation/command  (task -> Nav2)'; ros2 topic echo /navigation/command smart_farm_interfaces/msg/TaskCommand" C-m
tmux select-pane -t $S:0.2; tmux split-window -v -t $S
tmux send-keys -t $S "$E '/navigation/result  (Nav2 -> task)'; ros2 topic echo /navigation/result smart_farm_interfaces/msg/TaskResult" C-m
tmux select-pane -t $S:0.1; tmux split-window -v -t $S
tmux send-keys -t $S "$E '/cmd_vel  (Nav2 -> Nova Carter) linear.x / angular.z'; ros2 topic echo /cmd_vel geometry_msgs/msg/Twist --field linear.x" C-m
tmux select-pane -t $S:0.4; tmux split-window -v -t $S
if [ "$HUMAN" = "1" ]; then   # 사람 돌발상황: 정지 영역 상태 (action_type 1 = stop, polygon_name HumanStop)
tmux send-keys -t $S "$E '/collision_monitor_state  (human stop: action_type 1=STOP 2=SLOW 0=clear)'; ros2 topic echo /collision_monitor_state nav2_msgs/msg/CollisionMonitorState" C-m
else
tmux send-keys -t $S "$E '/chassis/odom  (Isaac -> Nav2) position'; ros2 topic echo /chassis/odom nav_msgs/msg/Odometry --field pose.pose.position" C-m
fi
tmux select-layout -t $S tiled
export DISPLAY=:0
setsid nohup xterm -T "ROS2 comm" -geometry 230x62+10+10 -fa Monospace -fs 8 -bg white -fg black -e tmux attach -t $S >/tmp/xterm_comm.log 2>&1 &
sleep 4
echo started
