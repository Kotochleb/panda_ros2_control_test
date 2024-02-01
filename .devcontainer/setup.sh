#!/bin/bash
set -e

cd /home/geppetto/ros2_ws

sudo apt update
# vcs import < src/project.repos src
rosdep update --rosdistro $ROS_DISTRO
rosdep install -y -i \
    --from-paths src \
    --rosdistro $ROS_DISTRO \
    --skip-keys libfranka 