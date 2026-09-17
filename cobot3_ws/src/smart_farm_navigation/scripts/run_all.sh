#!/usr/bin/env bash
# ==============================================================================
# 🚀 Smart Farm MiR100 올인원 통합 실행 스크립트 (One-Line Runner)
# ==============================================================================
# Isaac Sim (USD 로드 및 Play) + Nav2 + RViz2 를 한 번에 실행합니다.
# [실행 방법]:
#   bash ~/ROKEY_P3_A1/cobot3_ws/src/smart_farm_navigation/scripts/run_all.sh
# ==============================================================================

set -e

echo "=================================================================="
echo "🌱 [1/3] ROS 2 및 통신 환경변수 설정..."
echo "=================================================================="
export ROS_DISTRO=jazzy
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE=$HOME/.ros/fastdds_whitelist.xml

# Isaac Sim ROS 2 Bridge 라이브러리 경로 등록
ISAAC_ROS_LIB="$HOME/isaacsim/exts/isaacsim.ros2.bridge/jazzy/lib"
if [ -d "$ISAAC_ROS_LIB" ]; then
    export LD_LIBRARY_PATH="$LD_LIBRARY_PATH:$ISAAC_ROS_LIB"
    echo "  - Isaac Sim ROS2 Bridge 라이브러리 등록 완료"
fi

# ROS 2 및 워크스페이스 소싱
source /opt/ros/jazzy/setup.bash

if [ -f "$HOME/ROKEY_P3_A1/cobot3_ws/install/setup.bash" ]; then
    source "$HOME/ROKEY_P3_A1/cobot3_ws/install/setup.bash"
elif [ -f "$HOME/cobot3_ws/install/setup.bash" ]; then
    source "$HOME/cobot3_ws/install/setup.bash"
fi

# USD 스테이지 경로 탐색 (Collected_260916_AMR_test 우선)
USD_CANDIDATES=(
    "$HOME/ROKEY_P3_A1/cobot3_ws/src/smart_farm_navigation/Collected_260916_AMR_test/260916_AMR_test.usd"
    "$HOME/cobot3_ws/src/smart_farm_navigation/Collected_260916_AMR_test/260916_AMR_test.usd"
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
    echo "❌ [에러] Collected_260916_AMR_test.usd 파일을 찾을 수 없습니다."
    exit 1
fi

echo "  - 로드할 USD: $USD_PATH"

echo "=================================================================="
echo "🎮 [2/3] Isaac Sim 실행 중 (스테이지 자동 로드 및 시뮬레이션 시작)..."
echo "=================================================================="
ISAAC_SH="$HOME/isaacsim/isaac-sim.sh"
if [ -f "$ISAAC_SH" ]; then
    # Isaac Sim 백그라운드 실행
    "$ISAAC_SH" "$USD_PATH" --play-sim-on-start &
    ISAAC_PID=$!
    echo "  - Isaac Sim 프로세스 시작됨 (PID: $ISAAC_PID)"
else
    echo "⚠️ [경고] $ISAAC_SH 를 찾을 수 없습니다. 아이작 심을 수동 실행해주세요."
fi

# Ctrl+C 입력 시 Isaac Sim 함께 종료
cleanup() {
    echo -e "\n🛑 종료 시그널 수신. 모든 프로세스를 정리합니다..."
    if [ -n "$ISAAC_PID" ]; then
        kill -SIGINT "$ISAAC_PID" 2>/dev/null || true
    fi
    exit 0
}
trap cleanup SIGINT SIGTERM

echo "  - Isaac Sim 시뮬레이션 및 /clock 토픽 안정화 대기 (7초)..."
sleep 7

echo "=================================================================="
echo "🧭 [3/3] Nav2 Navigation & RViz2 실행..."
echo "=================================================================="
ros2 launch mir100_navigation mir_navigation.launch.py \
    map:="$HOME/ROKEY_P3_A1/cobot3_ws/src/smart_farm_navigation/maps/260916_AMR_test_yaml.yaml"
