#!/usr/bin/env bash
# ==============================================================================
# Native Windows Desktop Simulation Launcher (via VcXsrv / Windows X Server)
# ==============================================================================
# Directs Gazebo and RViz windows to VcXsrv running natively on Windows desktop.
# Requires VcXsrv running on Windows with "Disable access control" checked.
# ==============================================================================

echo "=================================================================="
echo "    NATIVE WINDOWS DESKTOP LAUNCHER (VCXSRV)"
echo "=================================================================="

# 1. Clean up lingering simulation processes
pkill -9 -f "gz sim" 2>/dev/null || true
pkill -9 -f "rviz2" 2>/dev/null || true
pkill -9 -f "ros2 launch adaptive_speed_control" 2>/dev/null || true
pkill -9 -f "adaptive_speed_controller" 2>/dev/null || true
pkill -9 -f "clutter_navigator" 2>/dev/null || true
sleep 1

# 2. Get Windows Host IP for WSL2
HOST_IP=$(ip route | grep default | awk '{print $3}')
if [ -z "$HOST_IP" ]; then
    HOST_IP="127.0.0.1"
fi

export DISPLAY="${HOST_IP}:0"
export LIBGL_ALWAYS_SOFTWARE=1
export MESA_GL_VERSION_OVERRIDE=3.3
export QT_X11_NO_MITSHM=1

echo "Targeting Windows X Server at: ${DISPLAY}"
echo "Checking connection to Windows X server..."

if ! timeout 2 bash -c "</dev/tcp/${HOST_IP}/6000" 2>/dev/null; then
    echo "------------------------------------------------------------------"
    echo "WARNING: Cannot connect to Windows X Server at ${HOST_IP}:6000."
    echo "Please make sure VcXsrv (XLaunch) is running on Windows with:"
    echo "  - 'Multiple windows'"
    echo "  - 'Disable access control' CHECKED"
    echo "------------------------------------------------------------------"
fi

# 3. Source ROS 2 Workspace
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="${SCRIPT_DIR}/ros2_ws"

if [ -f "/opt/ros/jazzy/setup.bash" ]; then
    source /opt/ros/jazzy/setup.bash
fi

if [ -f "${WS_DIR}/install/setup.bash" ]; then
    source "${WS_DIR}/install/setup.bash"
fi

export TURTLEBOT3_MODEL=burger
export GAZEBO_MODEL_PATH="${WS_DIR}/src/adaptive_speed_control/models:${GAZEBO_MODEL_PATH}"
export GZ_SIM_RESOURCE_PATH="${WS_DIR}/src/adaptive_speed_control/models:${GZ_SIM_RESOURCE_PATH}"

echo "Launching Gazebo Sim & RViz as native Windows desktop windows..."
ros2 launch adaptive_speed_control simulation.launch.py use_rviz:=true
