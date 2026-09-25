#!/usr/bin/env bash
# WSL2 Ubuntu 24.04 한 번 설치: ROS 2 Jazzy + Nav2 + 팀 워크스페이스(smart_farm_navigation, smart_farm_manager) 빌드 + ~/nav2_env.sh
#   wsl -d Ubuntu-24.04 -- bash <저장소>/cobot3_ws/isaacpjt/smart_farm/tools/allinone_live/wsl/setup_wsl_nav2.sh
# 중간에 sudo 비밀번호를 묻는다 (그때는 wsl -d Ubuntu-24.04 로 들어가 직접 실행).
# BRANCH: WSL 에 받을 저장소 브랜치 (Isaac 은 Windows 에서 돌고, WSL 은 Nav2 패키지만 빌드한다)
set -euo pipefail
HERE=$(cd "$(dirname "$0")" && pwd)
BRANCH="${BRANCH:-feature/cabbage-place-fix}"
WS="$HOME/rokey"

sudo apt-get update
sudo apt-get install -y software-properties-common curl git python3-pip locales
sudo locale-gen en_US en_US.UTF-8 && sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
sudo add-apt-repository -y universe
ROS_APT_VER=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F tag_name | awk -F'"' '{print $4}')
curl -L -o /tmp/ros2-apt-source.deb \
  "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_VER}/ros2-apt-source_${ROS_APT_VER}.$(. /etc/os-release && echo $VERSION_CODENAME)_all.deb"
sudo dpkg -i /tmp/ros2-apt-source.deb
sudo apt-get update
sudo apt-get install -y ros-jazzy-ros-base ros-jazzy-navigation2 ros-jazzy-nav2-bringup \
  ros-jazzy-pointcloud-to-laserscan ros-jazzy-rmw-fastrtps-cpp ros-jazzy-rviz2 \
  python3-colcon-common-extensions python3-rosdep python3-yaml
# 녹화 도구 (record_drive.sh)
sudo apt-get install -y ffmpeg xdotool tmux xterm

# 팀 워크스페이스 (Nav2·매니저 패키지만. Isaac 은 Windows 에서 돈다)
mkdir -p "$WS" && cd "$WS"
if [ ! -d ROKEY_P3_A1 ]; then git clone --depth 1 -b "$BRANCH" https://github.com/shbong06-sketch/ROKEY_P3_A1.git; fi
cd ROKEY_P3_A1 && git fetch --depth 1 origin "$BRANCH" && git checkout -q FETCH_HEAD
cd cobot3_ws
set +u   # ROS setup 스크립트가 정의 안 된 변수를 읽는다
source /opt/ros/jazzy/setup.bash
sudo rosdep init 2>/dev/null || true
rosdep update && rosdep install --from-paths src --ignore-src -y -r || true
colcon build --symlink-install --packages-up-to smart_farm_navigation smart_farm_manager

cp "$HERE/nav2_env.sh" ~/nav2_env.sh
echo "설치 끝: source ~/nav2_env.sh && ros2 pkg list | grep smart_farm"
