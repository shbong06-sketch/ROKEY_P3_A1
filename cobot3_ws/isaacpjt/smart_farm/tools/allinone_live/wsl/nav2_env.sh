# WSL2 쪽 ROS 2 환경 (setup_wsl_nav2.sh 가 ~/nav2_env.sh 로 복사한다. start_nav2_stack.sh 등이 source 한다)
# WSL 안은 보통 리눅스 DDS. Windows(Isaac, 도메인 101) <-> WSL(도메인 102) 토픽은 ros_tcp_relay.py 가 옮긴다.
set +u
source /opt/ros/jazzy/setup.bash
source ~/rokey/ROKEY_P3_A1/cobot3_ws/install/setup.bash
export ROS_DOMAIN_ID=102
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
unset FASTDDS_BUILTIN_TRANSPORTS FASTRTPS_DEFAULT_PROFILES_FILE
