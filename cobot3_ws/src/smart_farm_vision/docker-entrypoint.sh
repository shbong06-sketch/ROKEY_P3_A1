#!/bin/bash
set -e

source /opt/ros/jazzy/setup.bash

if [ -f /vision_ws/install/setup.bash ]; then
    source /vision_ws/install/setup.bash
fi

exec "$@"