#!/usr/bin/env bash
# ==============================================================================
# 🚀 Smart Farm MiR100 올인원 실행 스크립트 [안정화 4차]
# ==============================================================================
set -e

HOME_DIR="$HOME"
ISAAC_SH="$HOME_DIR/isaacsim/isaac-sim.sh"
USD_PATH="$HOME_DIR/ROKEY_P3_A1/cobot3_ws/src/smart_farm_navigation/Collected_260916_AMR_test/260916_AMR_test.usd"

if [ ! -f "$USD_PATH" ]; then
    USD_PATH="$HOME_DIR/cobot3_ws/src/smart_farm_navigation/Collected_260916_AMR_test/260916_AMR_test.usd"
fi

# [중요] 기존 환경에 설정된 ROS_DOMAIN_ID를 절대 변경하지 않고 100% 보존
echo "=================================================================="
echo "🌱 [1/3] 통신 환경 확인 (현재 ROS_DOMAIN_ID: ${ROS_DOMAIN_ID:-설정안됨})..."
echo "=================================================================="
export ROS_DISTRO=jazzy
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE="$HOME_DIR/.ros/fastdds_whitelist.xml"

ISAAC_ROS_LIB="$HOME_DIR/isaacsim/exts/isaacsim.ros2.bridge/jazzy/lib"

# 이미 실행 중인 Isaac Sim이 있는지 확인
if pgrep -f "isaac-sim" > /dev/null 2>&1; then
    echo "  - 이미 실행 중인 Isaac Sim 프로세스를 감지했습니다. 기존 창을 그대로 사용합니다."
else
    echo "  - Isaac Sim 구동 중 (USD: $USD_PATH)..."
    env -u PYTHONPATH -u AMENT_PREFIX_PATH -u COLCON_PREFIX_PATH -u CMAKE_PREFIX_PATH \
        ROS_DISTRO=jazzy \
        ROS_DOMAIN_ID="$ROS_DOMAIN_ID" \
        RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
        FASTRTPS_DEFAULT_PROFILES_FILE="$HOME_DIR/.ros/fastdds_whitelist.xml" \
        LD_LIBRARY_PATH="$ISAAC_ROS_LIB:$LD_LIBRARY_PATH" \
        "$ISAAC_SH" --/app/file/open="$USD_PATH" &
    ISAAC_PID=$!
    echo "  - Isaac Sim PID: $ISAAC_PID"
fi

cleanup() {
    echo -e "\n🛑 종료 신호 수신. 프로세스를 정리합니다..."
    if [ -n "$ISAAC_PID" ]; then
        kill -SIGINT "$ISAAC_PID" 2>/dev/null || true
    fi
    exit 0
}
trap cleanup SIGINT SIGTERM

echo "=================================================================="
echo "⏳ [2/3] 아이작 심 시뮬레이션 활성화 대기 중..."
echo "=================================================================="
echo "👉 아이작 심 창에서 Play(▶) 버튼을 눌러주세요."

# ROS 2 환경 소싱 (Nav2 실행용)
source /opt/ros/jazzy/setup.bash

WS_SETUP="$HOME_DIR/ROKEY_P3_A1/cobot3_ws/install/setup.bash"
if [ ! -f "$WS_SETUP" ]; then
    WS_SETUP="$HOME_DIR/cobot3_ws/install/setup.bash"
fi
source "$WS_SETUP"

# /clock 토픽 유입 대기 (최대 60초)
MAX_WAIT=60
WAIT_COUNT=0
READY=0

while [ $WAIT_COUNT -lt $MAX_WAIT ]; do
    if ros2 topic list 2>/dev/null | grep -qE "^/clock$|^/tf$|^/odom$"; then
        READY=1
        echo "  - ✅ 아이작 심 통신 연결 확인 완료!"
        break
    fi
    sleep 2
    WAIT_COUNT=$((WAIT_COUNT + 2))
    echo "  - 아이작 심 Play 대기 중... (${WAIT_COUNT}s / ${MAX_WAIT}s)"
done

if [ $READY -eq 0 ]; then
    echo "⚠️ [안내] 토픽 자동 감지 대기 종료. Nav2 기동을 진행합니다."
fi

echo "=================================================================="
echo "🧭 [3/3] Nav2 Navigation 및 RViz2 실행..."
echo "=================================================================="
MAP_YAML="$HOME_DIR/ROKEY_P3_A1/cobot3_ws/src/smart_farm_navigation/maps/260916_AMR_test_yaml.yaml"
if [ ! -f "$MAP_YAML" ]; then
    MAP_YAML="$HOME_DIR/cobot3_ws/src/smart_farm_navigation/maps/260916_AMR_test_yaml.yaml"
fi

ros2 launch mir100_navigation mir_navigation.launch.py map:="$MAP_YAML"
