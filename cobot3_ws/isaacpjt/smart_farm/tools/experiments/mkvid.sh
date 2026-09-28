#!/bin/bash
# usage: mkvid.sh RUN_DIR (wsl path)
cd "$1/captures" || exit 1
for n in harvest navplace; do
  ls cap_${n}_*.jpg >/dev/null 2>&1 || continue
  ffmpeg -y -loglevel error -framerate 8 -pattern_type glob -i "cap_${n}_*.jpg" -vf "scale=960:-2,format=yuv420p" -c:v libx264 -crf 26 "../video_${n}.mp4" && echo "ok $n"
done
