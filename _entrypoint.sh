#!/bin/bash
set -e

# setup ros2 environment
source "/usr/ros/$ROS_DISTRO/setup.bash"
source "/root/src/install/setup.bash"
exec "$@"