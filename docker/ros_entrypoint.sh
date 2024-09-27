#!/bin/bash

# setup ros2 environment
fd=0
if ! [[ -t "$fd" || -p /dev/stdin ]]; then
    source ~/.bashrc
else
    source "/opt/ros/$ROS_DISTRO/setup.bash"
    source "/root/ros_ws/install/setup.bash"
fi

exec "$@"

