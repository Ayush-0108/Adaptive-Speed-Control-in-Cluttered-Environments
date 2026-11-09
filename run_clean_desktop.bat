@echo off
setlocal
echo ==================================================================
echo   CLEANING SIMULATION HISTORY ^& LAUNCHING NATIVE DESKTOP GAZEBO
echo ==================================================================
echo.
powershell -ExecutionPolicy Bypass -File "%~dp0run_clean_desktop.ps1"
pause
