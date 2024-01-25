#!/bin/bash
set -e

# setup ros environment
source "/ros2_ws/install/setup.bash"

exec "$@"
