@echo off
setlocal
echo ==================================================================
echo   LAUNCHING NATIVE DESKTOP SIMULATION (VCXSRV)
echo ==================================================================
echo.
echo Make sure VcXsrv (XLaunch) is running with "Disable access control" checked!
echo.
wsl -d Ubuntu-24.04 bash -c "cd /mnt/d/Ayush_Files/Users/kumar/robotics2 && ./run_native_desktop.sh"
pause
