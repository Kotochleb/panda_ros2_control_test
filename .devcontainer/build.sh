#!/bin/bash
set -e

cd /home/geppetto/ros2_ws

colcon build \
        --symlink-install \
        --packages-skip ign_ros2_control_demos ignition-gazebo6 \
        --parallel-workers $(nproc) \
        --cmake-args \
            "-DCMAKE_BUILD_TYPE=Release" \
            "-DCMAKE_EXPORT_COMPILE_COMMANDS=On" \
        -Wall -Wextra -Wpedantic