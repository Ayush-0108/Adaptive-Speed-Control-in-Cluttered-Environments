@echo off
setlocal
echo ==================================================================
echo   LAUNCHING ADAPTIVE SPEED CONTROL (BROWSER 3D SIMULATION)
echo ==================================================================
echo Opening your browser to http://localhost:6080 ...
echo.

start "" "http://localhost:6080/vnc.html?autoconnect=true&resize=remote"

wsl -d Ubuntu-24.04 bash -c "cd /mnt/d/Ayush_Files/Users/kumar/robotics2 && chmod +x run_browser_simulation.sh && ./run_browser_simulation.sh"

pause
