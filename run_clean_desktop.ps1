# ==============================================================================
# Adaptive Speed Control - Clean Native Desktop Simulation Launcher
# ==============================================================================
# Clears all previous history/caches/logs, then launches Gazebo & RViz on desktop.
# ==============================================================================

Write-Host "==================================================================" -ForegroundColor Cyan
Write-Host "   CLEANING SIMULATION HISTORY & LAUNCHING NATIVE DESKTOP GAZEBO   " -ForegroundColor Yellow
Write-Host "==================================================================" -ForegroundColor Cyan

# 1. Clean up processes and history
Write-Host "[1/2] Purging stale ROS 2 / Gazebo logs, temporary sockets & processes..." -ForegroundColor Gray
wsl -d Ubuntu-24.04 bash -c "killall -q -9 gz ruby parameter_bridge adaptive_speed_controller clutter_navigator speed_telemetry_hud rviz2 Xvnc websockify openbox 2>/dev/null || true; rm -rf ~/.gz/sim/log/* ~/.cache/gz/* ~/.ros/log/* /tmp/launch_params* /tmp/.X1-lock 2>/dev/null || true"

Write-Host "[2/2] Launching native Gazebo Sim & RViz windows on your Windows desktop..." -ForegroundColor Green
Write-Host "Press Ctrl+C in this PowerShell window to stop the simulation.`n" -ForegroundColor Yellow

$WorkspacePath = "/mnt/d/Ayush_Files/Users/kumar/robotics2"
wsl -d Ubuntu-24.04 bash -c "cd $WorkspacePath && chmod +x run_simulation.sh && ./run_simulation.sh"
