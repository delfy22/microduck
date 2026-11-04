#!/bin/bash
set -e

# Language servers inherit the same underlay and overlay as build commands.
source /opt/ros/lyrical/setup.bash
if [[ -f /workspace/ros2_ws/install/setup.bash ]]; then
    source /workspace/ros2_ws/install/setup.bash
fi
exec "$@"
