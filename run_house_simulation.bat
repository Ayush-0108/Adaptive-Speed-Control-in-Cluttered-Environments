@echo off
setlocal
echo ==================================================================
echo      LAUNCHING ADAPTIVE SPEED CONTROL IN MULTI-ROOM HOUSE
echo ==================================================================
echo Opening Gazebo Sim & RViz in the 4-room apartment layout...
echo Press Ctrl+C in this window anytime to stop the simulation.
echo.

powershell -ExecutionPolicy Bypass -File "%~dp0run_house_simulation.ps1"
pause
