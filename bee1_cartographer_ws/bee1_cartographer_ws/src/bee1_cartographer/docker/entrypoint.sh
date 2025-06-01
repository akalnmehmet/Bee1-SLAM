#!/bin/bash
# Docker entrypoint script for Bee1 Cartographer

set -e

# Source ROS2 environment
source /opt/ros/humble/setup.bash

# Source workspace if built
if [ -f "/workspace/install/setup.bash" ]; then
    source /workspace/install/setup.bash
fi

# Set up display for GUI applications
export DISPLAY=${DISPLAY:-:0}

# Print system info
echo "🚗 Beemobs Bee1 Cartographer System"
echo "ROS2 Distribution: $ROS_DISTRO"
echo "RMW Implementation: $RMW_IMPLEMENTATION"
echo "ROS Domain ID: $ROS_DOMAIN_ID"
echo "Workspace: /workspace"

# Execute the command
exec "$@"