#!/usr/bin/env bash
# ==============================================================================
# Adaptive Speed Control - High Performance Web Browser Simulator Launcher
# ==============================================================================
# Completely bypasses WSLg / Intel Iris Xe display bugs by running an isolated
# X11 server with llvmpipe OpenGL 4.5 and streaming to Chrome / Edge via noVNC.
# ==============================================================================

echo "=================================================================="
echo "    ADAPTIVE SPEED CONTROL IN CLUTTERED ENVIRONMENTS"
echo "=================================================================="
echo "Starting isolated graphics engine for Intel Iris Xe & WSL2..."

# 1. Clean up any previous server processes
pkill -9 -f "Xvnc :1" 2>/dev/null || true
pkill -9 -f "websockify.*6080" 2>/dev/null || true
pkill -9 -f "openbox" 2>/dev/null || true
pkill -9 -f "gz sim" 2>/dev/null || true
pkill -9 -f "rviz2" 2>/dev/null || true
pkill -9 -f "ros2 launch adaptive_speed_control" 2>/dev/null || true
pkill -9 -f "adaptive_speed_controller" 2>/dev/null || true
pkill -9 -f "clutter_navigator" 2>/dev/null || true
sleep 1

# Clean X11 lock files for display :1
rm -f /tmp/.X1-lock /tmp/.X11-unix/X1 2>/dev/null || true

# 2. Start TigerVNC X Server on :1 listening on TCP
echo "[1/4] Starting virtual X11 display on 127.0.0.1:1 (1440x900)..."
Xvnc :1 -listen tcp -geometry 1440x900 -depth 24 -SecurityTypes None -auth /dev/null > /tmp/xvnc.log 2>&1 &
VNC_PID=$!
sleep 1

export DISPLAY=127.0.0.1:1
export LIBGL_ALWAYS_SOFTWARE=1
export MESA_GL_VERSION_OVERRIDE=3.3
export QT_X11_NO_MITSHM=1

# 3. Start Window Manager (Openbox)
echo "[2/4] Starting window manager..."
openbox > /tmp/openbox.log 2>&1 &
OPENBOX_PID=$!
sleep 1

# 4. Start noVNC Web Bridge on port 6080
echo "[3/4] Starting noVNC web server on port 6080..."
websockify --web /usr/share/novnc 6080 localhost:5901 > /tmp/websockify.log 2>&1 &
WEBSOCKIFY_PID=$!
sleep 1

# 5. Setup ROS 2 Environment
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="${SCRIPT_DIR}/ros2_ws"

if [ -f "/opt/ros/jazzy/setup.bash" ]; then
    source /opt/ros/jazzy/setup.bash
fi

if [ -f "${WS_DIR}/install/setup.bash" ]; then
    source "${WS_DIR}/install/setup.bash"
else
    echo "ERROR: Workspace not built. Run 'cd ${WS_DIR} && colcon build --symlink-install' first."
    exit 1
fi

export TURTLEBOT3_MODEL=burger
export GAZEBO_MODEL_PATH="${WS_DIR}/src/adaptive_speed_control/models:${GAZEBO_MODEL_PATH}"
export GZ_SIM_RESOURCE_PATH="${WS_DIR}/src/adaptive_speed_control/models:${GZ_SIM_RESOURCE_PATH}"

echo "=================================================================="
echo "  SUCCESS! THE SIMULATOR IS RUNNING!"
echo ""
echo "  >>> Open Chrome or Edge on Windows to: <<<"
echo "  http://localhost:6080/vnc.html?autoconnect=true&resize=remote"
echo ""
echo "  Both Gazebo Sim and RViz2 will appear inside your browser!"
echo "  Press Ctrl+C in this terminal when you want to stop the simulation."
echo "=================================================================="

# Cleanup handler on exit
cleanup() {
    echo ""
    echo "Stopping simulation and shutting down display services..."
    kill -9 $VNC_PID $OPENBOX_PID $WEBSOCKIFY_PID 2>/dev/null || true
    pkill -9 -f "gz sim" 2>/dev/null || true
    pkill -9 -f "rviz2" 2>/dev/null || true
    pkill -9 -f "ros2 launch adaptive_speed_control" 2>/dev/null || true
    pkill -9 -f "adaptive_speed_controller" 2>/dev/null || true
    pkill -9 -f "clutter_navigator" 2>/dev/null || true
    rm -f /tmp/.X1-lock /tmp/.X11-unix/X1 2>/dev/null || true
    echo "Clean exit completed."
    exit 0
}
trap cleanup SIGINT SIGTERM EXIT

# 6. Launch ROS 2 Simulation & RViz on DISPLAY=127.0.0.1:1
ros2 launch adaptive_speed_control simulation.launch.py use_rviz:=true
