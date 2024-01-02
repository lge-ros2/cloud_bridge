#!/bin/bash
set -e

# setup ros2 environment
source "/usr/ros/$ROS_DISTRO/setup.sh"
source "/root/src/install/setup.sh"
exec "$@"