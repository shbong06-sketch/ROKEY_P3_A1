#!/usr/bin/env bash
# [navigation 2026-09-26] 실측 녹화 장비.
#
# 화면(가상 디스플레이)을 셋으로 나눠 띄우고 각각 따로 녹화한다.
#   :99  Isaac Sim 뷰포트   -> isaac.mp4
#   :98  RViz2             -> rviz.mp4
#   :97  터미널(xterm+tmux) -> terminal.mp4
#
# 왜 나누는가: 가상 디스플레이에는 창 관리자가 없어서 한 화면에 두 창을 띄우면
# 서로 겹치고 화면 밖으로 잘린다(2026-09-26 확인). 화면을 나누면 그 문제가 없다.
#
# 쓰는 법
#   scripts/record_rig.sh start     화면 3개 + tmux 세션 + 녹화기 3개를 띄운다
#   scripts/record_rig.sh fit       Isaac·RViz2 를 띄운 뒤 창을 화면에 맞춘다 (창 관리자가 없어서 필요)
#   scripts/record_rig.sh status    지금 무엇이 돌고 있는지 본다
#   scripts/record_rig.sh stop      녹화를 끝내고(파일 마무리) 화면과 tmux 를 정리한다
#
# start 뒤에 사용자는 SSH 터미널에서 다음 한 줄로 같은 tmux 에 붙는다.
#   tmux attach -t farm
# 거기서 친 명령이 :97 화면의 xterm 에도 그대로 보이므로 terminal.mp4 에 녹화된다.
#
# 디스플레이 번호나 세션 이름을 바꾸려면 환경변수로 준다.
#   DISP_ISAAC=89 DISP_RVIZ=88 DISP_TERM=87 SES=test scripts/record_rig.sh start

set -u

PROJ=/home/rokey/ROKEY_P3_A1/cobot3_ws/src/smart_farm_navigation
MEDIA="$PROJ/results/media_log"        # git 제외 경로
CURRENT="$MEDIA/current"               # 지금 녹화 중인 폴더를 가리키는 링크

DISP_ISAAC=${DISP_ISAAC:-99}
DISP_RVIZ=${DISP_RVIZ:-98}
DISP_TERM=${DISP_TERM:-97}
SES=${SES:-farm}

SIZE_ISAAC=${SIZE_ISAAC:-1440x900}     # Isaac 창 기본 크기와 같게 두어 검은 여백을 없앴다
SIZE_RVIZ=${SIZE_RVIZ:-1680x1050}
SIZE_TERM=${SIZE_TERM:-1600x900}
XTERM_GEOM=${XTERM_GEOM:-200x50}       # tmux 는 붙어 있는 클라이언트 중 가장 작은 것에 맞춰진다
XTERM_FONT=${XTERM_FONT:-13}
# [2026-09-26] 한글이 모두 빈칸으로 찍히던 문제. 이 기기에 한글 글꼴이 하나도 없었다.
#   sudo apt-get install -y fonts-nanum-coding  로 넣고 고정폭 한글 글꼴을 직접 지정한다.
XTERM_FONT_FAMILY=${XTERM_FONT_FAMILY:-NanumGothicCoding}
FPS=${FPS:-10}                         # 렌더가 초당 7 회 수준이라 10 이면 충분하다


start_display() {   # $1 디스플레이 번호, $2 해상도
    if [ -e "/tmp/.X11-unix/X$1" ]; then
        echo "  [실패] :$1 는 이미 쓰이고 있다."
        echo "         앞서 띄워 둔 Isaac 이 그 화면을 쓰고 있을 수 있다. 확인: ps -ef | grep Xvfb"
        echo "         그 Isaac 을 끄거나(PID 를 찾아 kill. pkill -f 는 다른 셸까지 죽이므로 쓰지 않는다)"
        echo "         다른 번호를 준다: DISP_ISAAC=89 DISP_RVIZ=88 DISP_TERM=87 $0 start"
        return 1
    fi
    Xvfb ":$1" -screen 0 "$2x24" -nolisten tcp >"$RUN/xvfb_$1.log" 2>&1 &
    echo $! > "$RUN/xvfb_$1.pid"
    sleep 1
    echo "  화면 :$1 ($2) 시작"
}

start_recorder() {  # $1 디스플레이 번호, $2 해상도, $3 파일이름
    ffmpeg -loglevel warning -nostdin -y \
        -f x11grab -framerate "$FPS" -video_size "$2" -i ":$1" \
        -c:v libx264 -preset veryfast -crf 26 -pix_fmt yuv420p \
        "$RUN/$3.mp4" >"$RUN/ffmpeg_$3.log" 2>&1 &
    echo $! > "$RUN/ffmpeg_$3.pid"
    echo "  녹화 :$1 -> $3.mp4"
}

fit_display() {     # $1 디스플레이 번호, $2 해상도 — 그 화면의 창을 좌상단에 화면 크기로 맞춘다
    local w=${2%x*} h=${2#*x} n=0
    for wid in $(DISPLAY=":$1" xdotool search --onlyvisible --name "." 2>/dev/null); do
        # 1x1 짜리 보조 창(Qt selection owner 등)은 건너뛴다
        local geo; geo=$(DISPLAY=":$1" xdotool getwindowgeometry "$wid" 2>/dev/null | grep Geometry)
        case "$geo" in *" 1x1"*|"") continue;; esac
        DISPLAY=":$1" xdotool windowmove "$wid" 0 0 2>/dev/null
        DISPLAY=":$1" xdotool windowsize "$wid" "$w" "$h" 2>/dev/null
        n=$((n+1))
    done
    echo "  :$1 의 창 ${n}개를 0,0 / $2 로 맞췄다"
}

stop_pid() {        # $1 pid 파일, $2 보낼 신호
    [ -f "$1" ] || return 0
    local pid; pid=$(cat "$1")
    if kill -0 "$pid" 2>/dev/null; then
        kill "$2" "$pid" 2>/dev/null
    fi
    rm -f "$1"
}


case "${1:-}" in

start)
    if [ -L "$CURRENT" ]; then
        echo "이미 녹화 중인 것 같다: $(readlink "$CURRENT")"
        echo "먼저 'scripts/record_rig.sh stop' 을 실행한다."
        exit 1
    fi
    RUN="$MEDIA/run_$(date +%Y%m%d_%H%M)"
    mkdir -p "$RUN"
    ln -sfn "$RUN" "$CURRENT"

    # 시작 도중 실패하면 흔적을 지우고 나간다. 안 그러면 다음 start 가 "이미 녹화 중" 으로 막힌다.
    abort() { rm -f "$CURRENT"; echo "시작하지 못했다. 위 메시지를 보고 정리한 뒤 다시 실행한다."; exit 1; }

    echo "[1/4] 가상 화면 3개를 띄운다"
    start_display "$DISP_ISAAC" "$SIZE_ISAAC" || abort
    start_display "$DISP_RVIZ"  "$SIZE_RVIZ"  || abort
    start_display "$DISP_TERM"  "$SIZE_TERM"  || abort

    echo "[2/4] :$DISP_TERM 에 터미널(xterm + tmux '$SES')을 띄운다"
    tmux has-session -t "$SES" 2>/dev/null || tmux new-session -d -s "$SES"
    tmux set-option -t "$SES" -g history-limit 100000 >/dev/null
    DISPLAY=":$DISP_TERM" xterm -geometry "$XTERM_GEOM" \
        -fa "$XTERM_FONT_FAMILY" -fw "$XTERM_FONT_FAMILY" -fs "$XTERM_FONT" -bg black -fg white \
        -e tmux attach -t "$SES" >"$RUN/xterm.log" 2>&1 &
    echo $! > "$RUN/xterm.pid"
    sleep 2

    echo "[3/4] 녹화기 3개를 띄운다"
    start_recorder "$DISP_ISAAC" "$SIZE_ISAAC" isaac
    start_recorder "$DISP_RVIZ"  "$SIZE_RVIZ"  rviz
    start_recorder "$DISP_TERM"  "$SIZE_TERM"  terminal
    sleep 2

    echo "[4/4] 세 영상을 나중에 맞추기 위한 시각 표시를 남긴다"
    STAMP=$(date -u +%Y-%m-%dT%H:%M:%SZ)
    echo "$STAMP" > "$RUN/start_time.txt"
    tmux send-keys -t "$SES" "clear; echo '=== 녹화 시작 $STAMP (UTC) ==='" C-m

    cat <<EOF

녹화를 시작했다. 저장 위치: $RUN

이제 SSH 터미널에서 아래 한 줄로 같은 tmux 에 붙는다. 거기서 친 명령이
:$DISP_TERM 화면에도 그대로 나타나 terminal.mp4 에 녹화된다.

    tmux attach -t $SES

각 창(tmux 판)에서 쓸 화면 지정:
    Isaac 을 띄우는 판     export DISPLAY=:$DISP_ISAAC   (xvfb-run 을 쓰지 않는다)
    Nav2/RViz2 를 띄우는 판 export DISPLAY=:$DISP_RVIZ    (nav2.launch.py 를 use_rviz:=true 로)
    나머지 판(명령·결과)   화면이 필요 없다

끝낼 때: scripts/record_rig.sh stop
EOF
    ;;

fit)
    # Isaac 과 RViz2 를 띄운 뒤 한 번 실행한다. 가상 화면에는 창 관리자가 없어
    # 창이 저장된 위치 그대로 떠서 화면 밖으로 잘리는 일이 있다(2026-09-26 RViz2 가 그랬다).
    echo "창을 화면에 맞춘다"
    fit_display "$DISP_ISAAC" "$SIZE_ISAAC"
    fit_display "$DISP_RVIZ"  "$SIZE_RVIZ"
    ;;

status)
    if [ ! -L "$CURRENT" ]; then echo "녹화 중이 아니다."; exit 0; fi
    RUN=$(readlink "$CURRENT")   # -f 를 쓰지 않는다. 쓰면 심볼릭 링크가 풀려 소문자 실경로가 나온다
    echo "녹화 폴더: $RUN  (시작 $(cat "$RUN/start_time.txt" 2>/dev/null))"
    for f in "$RUN"/*.pid; do
        [ -e "$f" ] || continue
        pid=$(cat "$f")
        kill -0 "$pid" 2>/dev/null && state="돌고 있음" || state="죽었음"
        echo "  $(basename "$f" .pid): pid $pid ($state)"
    done
    ls -lh "$RUN"/*.mp4 2>/dev/null
    ;;

stop)
    if [ ! -L "$CURRENT" ]; then echo "녹화 중이 아니다."; exit 0; fi
    RUN=$(readlink "$CURRENT")   # -f 를 쓰지 않는다. 쓰면 심볼릭 링크가 풀려 소문자 실경로가 나온다

    echo "[1/3] 녹화기를 멈춘다 (파일 마무리)"
    for n in isaac rviz terminal; do stop_pid "$RUN/ffmpeg_$n.pid" -INT; done
    sleep 3

    echo "[2/3] 터미널 창을 닫는다 (tmux 세션 '$SES' 는 남겨 둔다)"
    stop_pid "$RUN/xterm.pid" -TERM

    echo "[3/3] 가상 화면을 닫는다"
    for d in "$DISP_ISAAC" "$DISP_RVIZ" "$DISP_TERM"; do stop_pid "$RUN/xvfb_$d.pid" -TERM; done

    rm -f "$CURRENT"
    echo
    echo "결과: $RUN"
    for f in "$RUN"/*.mp4; do
        [ -e "$f" ] || continue
        dur=$(ffprobe -v error -show_entries format=duration -of csv=p=0 "$f" 2>/dev/null)
        printf "  %-14s %8s  %s초\n" "$(basename "$f")" "$(du -h "$f" | cut -f1)" "${dur%.*}"
    done
    echo
    echo "내려받기(로컬 PC 에서):"
    echo "  gcloud compute scp --recurse <인스턴스>:$RUN ."
    ;;

*)
    echo "사용법: $0 {start|fit|status|stop}"
    exit 1
    ;;
esac
