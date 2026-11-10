# ==============================================================================
# Launch Adaptive Speed Control in Multi-Room House Simulation
# ==============================================================================
Write-Host "==================================================================" -ForegroundColor Cyan
Write-Host "     LAUNCHING ADAPTIVE SPEED CONTROL IN MULTI-ROOM HOUSE         " -ForegroundColor Yellow
Write-Host "==================================================================" -ForegroundColor Cyan
Write-Host "Opening Gazebo Sim & RViz in the 4-room apartment layout..." -ForegroundColor Green
Write-Host "Press Ctrl+C in this PowerShell window to stop the simulation.`n" -ForegroundColor Yellow

$HouseWorld = "/mnt/d/Ayush_Files/Users/kumar/robotics2/ros2_ws/install/adaptive_speed_control/share/adaptive_speed_control/worlds/multiroom_house.world"

wsl -d Ubuntu-24.04 bash -c "killall -q -9 gz ruby parameter_bridge adaptive_speed_controller clutter_navigator speed_telemetry_hud rviz2 2>/dev/null || true; source /opt/ros/jazzy/setup.bash && source /mnt/d/Ayush_Files/Users/kumar/robotics2/ros2_ws/install/setup.bash && export DISPLAY=:0 && export LIBGL_ALWAYS_SOFTWARE=1 && export QT_QPA_PLATFORM=xcb && ros2 launch adaptive_speed_control simulation.launch.py world:=$HouseWorld x_pose:=-2.5 y_pose:=-2.5"
