#!/bin/bash
# Launcher for Fixed Speed Baseline Mode (for comparison)
source /opt/ros/jazzy/setup.bash
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/ros2_ws/install/setup.bash"
export TURTLEBOT3_MODEL=burger

echo "Running simulation in FIXED SPEED mode (bypassing adaptive scaling)..."
ros2 launch adaptive_speed_control simulation.launch.py mode:=fixed
