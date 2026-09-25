# ROS 2 env for Nav2 in WSL2 (NAT networking) next to Isaac Sim on Windows.
# DDS inside WSL is plain Linux DDS; Windows <-> WSL topics go through ros_tcp_relay.py (TCP via localhost forwarding).
set +u
source /opt/ros/jazzy/setup.bash
source ~/rokey/ROKEY_P3_A1/cobot3_ws/install/setup.bash
export ROS_DOMAIN_ID=102
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
unset FASTDDS_BUILTIN_TRANSPORTS FASTRTPS_DEFAULT_PROFILES_FILE