#!/bin/bash
# ==============================================================================
# Adaptive Speed Control - Native Desktop Gazebo Simulation Launcher
# ==============================================================================
# Runs Gazebo Sim & RViz as native floating Windows desktop application windows
# using WSLg with LLVMpipe software OpenGL to prevent Intel Iris Xe D3D12 bugs.
# ==============================================================================

set -e

# 1. Terminate any lingering background Gazebo/ROS instances
killall -q -9 gz ruby parameter_bridge adaptive_speed_controller clutter_navigator speed_telemetry_hud rviz2 2>/dev/null || true
sleep 1

# 2. Source ROS 2 Jazzy and local overlay workspace
source /opt/ros/jazzy/setup.bash

# 3. Graphics & Display Settings for Native Windows Desktop (WSLg)
export DISPLAY=:0
export WAYLAND_DISPLAY=wayland-0
export LIBGL_ALWAYS_SOFTWARE=1
export MESA_GL_VERSION_OVERRIDE=3.3
export QT_X11_NO_MITSHM=1
export QT_QPA_PLATFORM=xcb
export GZ_IP=127.0.0.1
export GZ_PARTITION=$(hostname)

# Clean stale transport cache
rm -rf ~/.gz/sim/log/* ~/.cache/gz/* 2>/dev/null || true

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="${SCRIPT_DIR}/ros2_ws"

if [ -f "${WS_DIR}/install/setup.bash" ]; then
    source "${WS_DIR}/install/setup.bash"
else
    echo "Building workspace first..."
    cd "${WS_DIR}"
    colcon build --symlink-install
    source "${WS_DIR}/install/setup.bash"
fi

export TURTLEBOT3_MODEL=burger
export GAZEBO_MODEL_PATH="${WS_DIR}/src/adaptive_speed_control/models:${GAZEBO_MODEL_PATH}"
export GZ_SIM_RESOURCE_PATH="${WS_DIR}/src/adaptive_speed_control/models:${GZ_SIM_RESOURCE_PATH}"

echo "================================================================="
echo "   ADAPTIVE SPEED CONTROL IN CLUTTERED ENVIRONMENTS SIMULATION   "
echo "================================================================="
echo "Operating System: Ubuntu 24.04 (ROS 2 Jazzy + Gazebo Sim Harmonic)"
echo "Display Target:   Windows Native Desktop (WSLg DISPLAY=:0)"
echo "Robot Model:      TurtleBot3 Burger"
echo "World:            Cluttered Arena (Open, Slalom & Dense Maze Zones)"
echo "Mode:             Adaptive Speed Modulation + Reactive Navigation"
echo "================================================================="
echo ""
echo "Opening native Gazebo Sim and RViz windows on your Windows desktop..."
echo "To exit simulation, press Ctrl+C in this terminal."
echo ""

ros2 launch adaptive_speed_control simulation.launch.py "$@"
