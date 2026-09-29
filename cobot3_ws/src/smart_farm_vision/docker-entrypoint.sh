#!/bin/bash
set -e

source /opt/ros/jazzy/setup.bash
source /vision_ws/install/setup.bash

exec "$@"
