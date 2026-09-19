#!/usr/bin/env bash
# smart_farm_nav2_01.usd 점검·탈출 실행 스크립트 (고피 전용, 한 번 실행으로 전 과정 자동화)
#
# 사용법
#   bash gopi_run.sh              빌드 → Isaac Sim 자동 실행 → 통신 점검 → escape_controller 대기 실행(이동 없음)
#   bash gopi_run.sh --escape     위 과정 + escape_controller 탈출 동작 실행 (auto_start:=true)
# 옵션
#   --no-launch    Isaac Sim을 자동 실행하지 않음. 이미 Play 중인 GUI를 사용할 때.
#   --keep-isaac   스크립트가 띄운 Isaac Sim을 종료하지 않고 남겨 둠.
#
# 모든 출력은 results/run_<시각>.txt 에 저장되고 feature/navigation 에 자동 커밋·푸시된다.
set -u

ROOT=/home/rokey/ROKEY_P3_A1
WS=$ROOT/cobot3_ws
PKG=$WS/src/smart_farm_navigation
SCENE=$WS/isaacpjt/smart_farm/scenes/smart_farm_nav2_01.usd
ISAAC_PY=$HOME/isaacsim/python.sh
BRIDGE_LIB=$HOME/isaacsim/exts/isaacsim.ros2.bridge/jazzy/lib
RESULTS=$PKG/results
STAMP=$(date +%Y%m%d_%H%M%S)
LOG=$RESULTS/run_${STAMP}.txt
ISAAC_LOG=$RESULTS/isaac_${STAMP}.txt

ESCAPE=0; LAUNCH=1; KEEP=0
for a in "$@"; do
  case "$a" in
    --escape) ESCAPE=1 ;;
    --no-launch) LAUNCH=0 ;;
    --keep-isaac) KEEP=1 ;;
    *) echo "unknown argument: $a"; exit 64 ;;
  esac
done

mkdir -p "$RESULTS"
exec > >(tee -a "$LOG") 2>&1
echo "== gopi_run.sh $STAMP  args: $*  host: $(hostname)  commit: $(git -C "$ROOT" rev-parse --short HEAD) =="

ISAAC_PID=""

finish() {
  local rc=$1
  if [[ -n "$ISAAC_PID" && $KEEP -eq 0 ]]; then
    echo "== stopping Isaac Sim launched by this script (pgid $ISAAC_PID) =="
    kill -TERM -- "-$ISAAC_PID" 2>/dev/null
    for _ in $(seq 1 12); do kill -0 "$ISAAC_PID" 2>/dev/null || break; sleep 5; done
    kill -KILL -- "-$ISAAC_PID" 2>/dev/null
  fi
  echo "== RESULT rc=$rc  log: $LOG =="
  cd "$ROOT" || exit "$rc"
  git add "$RESULTS" >/dev/null 2>&1
  git commit -q -m "run ${STAMP} rc=${rc} escape=${ESCAPE}" && git push origin feature/navigation 2>&1 | tail -1
  exit "$rc"
}

have_odom() { timeout 6 ros2 topic echo /chassis/odom --once --field header.stamp >/dev/null 2>&1; }

# ---------- 1. ROS 2 환경 ----------
# shellcheck disable=SC1091
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=101
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
[[ -f "$HOME/.ros/fastdds_whitelist.xml" ]] && export FASTRTPS_DEFAULT_PROFILES_FILE="$HOME/.ros/fastdds_whitelist.xml"
if [[ ":${LD_LIBRARY_PATH:-}:" != *":$BRIDGE_LIB:"* ]]; then
  export LD_LIBRARY_PATH="${LD_LIBRARY_PATH:-}:$BRIDGE_LIB"
fi
echo "ROS_DISTRO=$ROS_DISTRO ROS_DOMAIN_ID=$ROS_DOMAIN_ID RMW=$RMW_IMPLEMENTATION"

# ---------- 2. 빌드 (항상 실제 경로 $WS 에서, 패키지 산출물 초기화 후) ----------
cd "$WS" || finish 1
rm -rf "$WS/build/smart_farm_navigation" "$WS/install/smart_farm_navigation"
echo "== colcon build =="
colcon build --packages-select smart_farm_navigation || finish 1
# shellcheck disable=SC1091
source "$WS/install/setup.bash"
PREFIX=$(ros2 pkg prefix smart_farm_navigation) || finish 1
PARAMS="$PREFIX/share/smart_farm_navigation/config/escape_controller.yaml"
echo "pkg prefix: $PREFIX"
[[ -r "$PARAMS" ]] || { echo "params file missing: $PARAMS"; finish 1; }

# ---------- 3. Isaac Sim ----------
if have_odom; then
  echo "== Isaac Sim already publishing /chassis/odom; using the running instance =="
elif [[ $LAUNCH -eq 1 ]]; then
  if pgrep -f "isaacsim.exp|isaac-sim.sh" >/dev/null; then
    echo "!! Isaac Sim is running but /chassis/odom is not published (scene not open or not playing)."
    echo "!! Press Play in that window, or close it and re-run this script."
    finish 3
  fi
  echo "== launching Isaac Sim standalone (log: $ISAAC_LOG) =="
  setsid bash "$ISAAC_PY" "$PKG/scripts/launch_scene.py" "$SCENE" >"$ISAAC_LOG" 2>&1 &
  ISAAC_PID=$!
  for i in $(seq 1 60); do
    have_odom && break
    kill -0 "$ISAAC_PID" 2>/dev/null || { echo "!! Isaac Sim exited early"; tail -40 "$ISAAC_LOG"; finish 3; }
    sleep 5
  done
  have_odom || { echo "!! no /chassis/odom after 300 s"; tail -40 "$ISAAC_LOG"; finish 3; }
  echo "== Isaac Sim is playing =="
else
  echo "!! --no-launch given but /chassis/odom is not published"; finish 3
fi

# ---------- 4. 통신 점검 ----------
echo "== topics =="; ros2 topic list | sort
echo "== /cmd_vel =="; ros2 topic info /cmd_vel -v | grep -E "Publisher count|Subscription count|Node name"
echo "== odom drift over 15 s (no command) =="
ros2 topic echo /chassis/odom --once --field pose.pose.position
sleep 15
ros2 topic echo /chassis/odom --once --field pose.pose.position

# ---------- 5. escape_controller ----------
if [[ $ESCAPE -eq 1 ]]; then
  echo "== ESCAPE RUN (auto_start:=true, cap 180 s) =="
  timeout -s INT --preserve-status 180 ros2 run smart_farm_navigation escape_controller \
    --ros-args --params-file "$PARAMS" -p auto_start:=true
  RC=$?
else
  echo "== WAIT RUN (no motion; SIGINT after 10 s) =="
  timeout -s INT --preserve-status 10 ros2 run smart_farm_navigation escape_controller \
    --ros-args --params-file "$PARAMS"
  RC=$?
fi
echo "escape_controller exit code: $RC"
echo "== final odom pose =="
ros2 topic echo /chassis/odom --once --field pose.pose
finish "$RC"
