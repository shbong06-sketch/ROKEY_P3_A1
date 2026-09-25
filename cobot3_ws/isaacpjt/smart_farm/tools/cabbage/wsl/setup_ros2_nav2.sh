#!/usr/bin/env bash
# WSL2 Ubuntu 24.04: ROS 2 Jazzy + Nav2 + the team's smart_farm_navigation, to drive the Isaac Sim (Windows) rig.
#   wsl -d Ubuntu-24.04 -- bash /mnt/d/smartfarm-sim/scripts/wsl/setup_ros2_nav2.sh
set -euo pipefail
BRANCH="${BRANCH:-feature/Inspection-Place-nav2}"
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
  python3-colcon-common-extensions python3-rosdep

# team workspace (navigation + manager packages only; Isaac runs on Windows)
mkdir -p "$WS" && cd "$WS"
if [ ! -d ROKEY_P3_A1 ]; then git clone --depth 1 -b "$BRANCH" https://github.com/shbong06-sketch/ROKEY_P3_A1.git; fi
cd ROKEY_P3_A1 && git fetch --depth 1 origin "$BRANCH" && git checkout -q FETCH_HEAD
cd cobot3_ws
set +u   # ROS setup scripts read unset variables
source /opt/ros/jazzy/setup.bash
sudo rosdep init 2>/dev/null || true
rosdep update && rosdep install --from-paths src --ignore-src -y -r || true
colcon build --symlink-install --packages-up-to smart_farm_navigation smart_farm_manager

# DDS to Windows Isaac Sim: same domain, UDP only (Fast DDS shared memory cannot cross the WSL VM boundary)
grep -q smartfarm-nav2-env ~/.bashrc || cat >> ~/.bashrc <<EOF
# smartfarm-nav2-env
source /opt/ros/jazzy/setup.bash
source $WS/ROKEY_P3_A1/cobot3_ws/install/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DOMAIN_ID=0
export FASTDDS_BUILTIN_TRANSPORTS=UDPv4
EOF
echo "setup done: open a new shell, then: ros2 launch smart_farm_navigation nav2.launch.py"
