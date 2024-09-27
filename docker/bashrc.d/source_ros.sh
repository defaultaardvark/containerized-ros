#!/usr/bin/env bash

if [ -f ~/ros_ws/install/setup.bash ]; then
    source ~/ros_ws/install/setup.bash
elif [ -f /opt/ros/$ROS_DISTRO/setup.bash ]; then
    source /opt/ros/$ROS_DISTRO/setup.bash
fi

