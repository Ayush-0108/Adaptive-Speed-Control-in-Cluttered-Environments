@echo off
setlocal
echo ==================================================================
echo   LAUNCHING ADAPTIVE SPEED CONTROL IN GAZEBO (NATIVE DESKTOP)
echo ==================================================================
echo Opening Gazebo Sim and RViz windows directly on your Windows desktop...
echo Press Ctrl+C in this window anytime to stop the simulation.
echo.

wsl -d Ubuntu-24.04 bash -c "cd /mnt/d/Ayush_Files/Users/kumar/robotics2 && chmod +x run_simulation.sh && ./run_simulation.sh"

pause
