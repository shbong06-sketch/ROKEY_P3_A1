#!/usr/bin/env bash
# ==============================================================================
# 🚀 Smart Farm MiR100 올인원 통합 실행 스크립트 (One-Line Complete Runner)
# ==============================================================================
# [동작 순서]:
# 1. FastDDS 및 ROS 2 Jazzy, isaac_ros 통신 환경 자동 구성
# 2. 올바른 워크스페이스(~/ROKEY_P3_A1/cobot3_ws) 자동 소싱
# 3. Isaac Sim 실행 + Collected_260916_AMR_test.usd 자동 로드 + ActionGraph 주입 + 자동 Play
# 4. /clock 토픽 발행 감지 후 Nav2 & RViz2 자동 기동
# 5. Ctrl + C 입력 시 모든 프로세스 일괄 정상 종료
#
# [실행 방법]:
#   bash ~/ROKEY_P3_A1/cobot3_ws/src/smart_farm_navigation/scripts/run_all.sh
# ==============================================================================

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
HOME_DIR="$HOME"

echo "=================================================================="
echo "🌱 [1/4] ROS 2 및 통신 환경변수 구성..."
echo "=================================================================="
export ROS_DISTRO=jazzy
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE="$HOME_DIR/.ros/fastdds_whitelist.xml"
export ROS_DOMAIN_ID=103

# Isaac Sim ROS 2 Bridge 라이브러리 경로 등록
ISAAC_ROS_LIB="$HOME_DIR/isaacsim/exts/isaacsim.ros2.bridge/jazzy/lib"
if [ -d "$ISAAC_ROS_LIB" ]; then
    export LD_LIBRARY_PATH="$LD_LIBRARY_PATH:$ISAAC_ROS_LIB"
    echo "  - Isaac Sim ROS2 Bridge 라이브러리 등록 완료: $ISAAC_ROS_LIB"
fi

# 1. ROS 2 기본 환경 소싱
source /opt/ros/jazzy/setup.bash

# 2. 프로젝트 워크스페이스 소싱 (ROKEY_P3_A1 최우선)
WS_SETUP=""
if [ -f "$HOME_DIR/ROKEY_P3_A1/cobot3_ws/install/setup.bash" ]; then
    WS_SETUP="$HOME_DIR/ROKEY_P3_A1/cobot3_ws/install/setup.bash"
elif [ -f "$HOME_DIR/cobot3_ws/install/setup.bash" ]; then
    WS_SETUP="$HOME_DIR/cobot3_ws/install/setup.bash"
fi

if [ -n "$WS_SETUP" ]; then
    source "$WS_SETUP"
    echo "  - 워크스페이스 소싱 완료: $WS_SETUP"
else
    echo "❌ [에러] 워크스페이스 setup.bash 를 찾을 수 없습니다. colcon build 를 먼저 수행해주세요."
    exit 1
fi

# USD 스테이지 경로 확인 (Collected_260916_AMR_test)
USD_CANDIDATES=(
    "$HOME_DIR/ROKEY_P3_A1/cobot3_ws/src/smart_farm_navigation/Collected_260916_AMR_test/260916_AMR_test.usd"
    "$HOME_DIR/cobot3_ws/src/smart_farm_navigation/Collected_260916_AMR_test/260916_AMR_test.usd"
    "/home/rokey/ROKEY_P3_A1/cobot3_ws/src/smart_farm_navigation/Collected_260916_AMR_test/260916_AMR_test.usd"
)

USD_PATH=""
for p in "${USD_CANDIDATES[@]}"; do
    if [ -f "$p" ]; then
        USD_PATH="$p"
        break
    fi
done

if [ -z "$USD_PATH" ]; then
    echo "❌ [에러] Collected_260916_AMR_test/260916_AMR_test.usd 파일을 찾을 수 없습니다."
    exit 1
fi
echo "  - 로드할 USD: $USD_PATH"

echo "=================================================================="
echo "🎮 [2/4] Isaac Sim 시작 및 스테이지 자동 로드 / 자동 Play..."
echo "=================================================================="
ISAAC_SH="$HOME_DIR/isaacsim/isaac-sim.sh"
STAGE_LOADER="$SCRIPT_DIR/open_and_setup_stage.py"

if [ ! -f "$ISAAC_SH" ]; then
    echo "❌ [에러] Isaac Sim 실행 스크립트($ISAAC_SH)를 찾을 수 없습니다."
    exit 1
fi

# Omniverse Kit 표준 방식으로 스테이지 오픈 및 자동 Play 실행
"$ISAAC_SH" \
    --/isaac/startup/ros_bridge_extension=isaacsim.ros2.bridge \
    --exec "$STAGE_LOADER --path $USD_PATH --start-on-play" &
ISAAC_PID=$!

echo "  - Isaac Sim 프로세스 구동됨 (PID: $ISAAC_PID)"

# 종료 처리 핸들러 등록
cleanup() {
    echo -e "\n🛑 종료 신호 수신. 실행 중인 Isaac Sim 및 ROS 노드를 정리합니다..."
    if [ -n "$ISAAC_PID" ]; then
        kill -SIGINT "$ISAAC_PID" 2>/dev/null || true
    fi
    exit 0
}
trap cleanup SIGINT SIGTERM

echo "=================================================================="
echo "⏳ [3/4] Isaac Sim 시뮬레이션 및 /clock 토픽 발행 대기 중..."
echo "=================================================================="
# 최대 45초 동안 /clock 토픽 유입 대기
MAX_WAIT=45
WAIT_COUNT=0
CLOCK_DETECTED=0

while [ $WAIT_COUNT -lt $MAX_WAIT ]; do
    if ros2 topic list 2>/dev/null | grep -q "^/clock$"; then
        CLOCK_DETECTED=1
        echo "  - ✅ /clock 토픽 감지 완료! (소요 시간: 약 ${WAIT_COUNT}초)"
        break
    fi
    sleep 2
    WAIT_COUNT=$((WAIT_COUNT + 2))
    echo "  - 대기 중... (${WAIT_COUNT}s / ${MAX_WAIT}s)"
done

if [ $CLOCK_DETECTED -eq 0 ]; then
    echo "⚠️ [주의] /clock 토픽 대기 시간이 초과되었으나, Nav2 기동을 계속 진행합니다."
fi

echo "=================================================================="
echo "🧭 [4/4] Nav2 Navigation 및 RViz2 자동 실행..."
echo "=================================================================="
MAP_YAML="$HOME_DIR/ROKEY_P3_A1/cobot3_ws/src/smart_farm_navigation/maps/260916_AMR_test_yaml.yaml"
if [ ! -f "$MAP_YAML" ]; then
    MAP_YAML="$HOME_DIR/cobot3_ws/src/smart_farm_navigation/maps/260916_AMR_test_yaml.yaml"
fi

ros2 launch mir100_navigation mir_navigation.launch.py map:="$MAP_YAML"
