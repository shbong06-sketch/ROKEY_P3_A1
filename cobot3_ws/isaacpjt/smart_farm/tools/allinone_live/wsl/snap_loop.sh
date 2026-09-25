#!/bin/bash
# one frame per second of an X window into OUT/NAME/%06d.png (for windows whose continuous x11grab stalls)
#   snap_loop.sh OUT NAME WINDOW_NAME_REGEX SECONDS
OUT=$1; NAME=$2; PAT=$3; SECS=${4:-1500}
export DISPLAY=:0
mkdir -p "$OUT/$NAME"
w=$(xdotool search --name "$PAT" | head -1)
[ -z "$w" ] && { echo "not found: $PAT"; exit 1; }
for i in $(seq -f '%06g' 1 "$SECS"); do
  ffmpeg -y -loglevel quiet -f x11grab -window_id "$w" -i :0 -frames:v 1 "$OUT/$NAME/$i.png"
  sleep 0.85
done
