#!/bin/bash

set -e

if [ -f /usr/ros/${ROS_DISTRO}/setup.bash ]
then
  source /usr/ros/${ROS_DISTRO}/setup.bash
else
  echo "/usr/ros/${ROS_DISTRO}/setup.bash is not provided. \`apk add --no-cache ros-${ROS_DISTRO}-ros-workspace\` to setup."
fi

exec "$@"