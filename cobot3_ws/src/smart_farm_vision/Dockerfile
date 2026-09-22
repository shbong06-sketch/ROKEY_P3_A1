FROM ros:jazzy-ros-base-noble

RUN apt-get update && apt-get install -y --no-install-recommends \
        python3-pip \
        python3-rosdep \
        python3-colcon-common-extensions \
        ros-jazzy-rclpy \
        ros-jazzy-sensor-msgs \
        ros-jazzy-cv-bridge \
    && rm -rf /var/lib/apt/lists/*

# CUDA 12.4 PyTorch wheel을 명시적으로 선택한다.
RUN pip install --no-cache-dir --break-system-packages \
        --index-url https://download.pytorch.org/whl/cu124 \
        torch==2.4.1 \
        torchvision==0.19.1

# cv_bridge와 호환되는 NumPy 1.x를 유지한다.
RUN pip install --no-cache-dir --break-system-packages \
        numpy==1.26.4 \
        opencv-python==4.10.0.84 \
        ultralytics==8.4.26

# CPU 전용 PyTorch가 잘못 설치되면 빌드를 실패시킨다.
RUN python3 -c \
    "import torch; assert torch.version.cuda == '12.4', torch.__version__"

WORKDIR /vision_ws

COPY smart_farm_interfaces /vision_ws/src/smart_farm_interfaces
COPY smart_farm_vision /vision_ws/src/smart_farm_vision

RUN rosdep update \
    && rosdep install \
        --from-paths src \
        --ignore-src \
        --rosdistro jazzy \
        -y

RUN . /opt/ros/jazzy/setup.sh \
    && colcon build \
        --packages-select \
        smart_farm_interfaces \
        smart_farm_vision

COPY smart_farm_vision/docker-entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
CMD ["sleep", "infinity"]
