#!/bin/bash
# 주행 녹화 (WSL): RViz 창 + ROS2 통신 화면(tmux 6칸, comm_panes.sh)을 x11grab 으로 녹화하고, 통신 화면은 1초마다 사진도 남긴다.
#   [HUMAN=1] bash record_drive.sh OUT_DIR [SECONDS=1500]
#   -> OUT_DIR/nav2_rviz.mp4, ros2_comm.mp4, comm_frames/000001.png ...  (끝내기: pkill -INT -f 'ffmpeg -y'; pkill -f snap_loop)
# HUMAN=1 이면 odom 칸 대신 /collision_monitor_state (사람 정지 상태)를 보인다.
# 필요: ffmpeg, xdotool, tmux, xterm (sudo apt install ffmpeg xdotool tmux xterm). 창이 1920x1080 화면 안에 있어야 녹화된다.
HERE=$(cd "$(dirname "$0")" && pwd)
O=${1:?OUT_DIR}; SECS=${2:-1500}
export DISPLAY=:0
mkdir -p "$O"
bash "$HERE/comm_panes.sh"
r=$(xdotool search --name 'nav2_smartfarm.rviz.* - RViz$' | head -1)
[ -n "$r" ] && { xdotool windowsize "$r" 1100 780; xdotool windowmove "$r" 800 280; }
sleep 2
rec() {  # window-id name
  setsid nohup ffmpeg -y -loglevel error -f x11grab -framerate 6 -window_id "$1" -i :0 -t "$SECS" \
    -vf "scale=trunc(iw/2)*2:trunc(ih/2)*2" -c:v libx264 -preset veryfast -crf 26 -g 12 -pix_fmt yuv420p \
    -movflags +frag_keyframe+empty_moov "$O/$2.mp4" >"$O/ff_$2.log" 2>&1 &
}
c=$(xdotool search --name '^ROS2 comm$' | head -1)
[ -n "$r" ] && rec "$r" nav2_rviz
rec "$c" ros2_comm
setsid nohup bash "$HERE/snap_loop.sh" "$O" comm_frames '^ROS2 comm$' "$SECS" >/tmp/snap_comm.log 2>&1 &
sleep 20
ls -la "$O"/*.mp4; echo "comm frames: $(ls "$O/comm_frames" | wc -l)"
