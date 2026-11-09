# ==============================================================================
# Adaptive Speed Control - PowerShell Simulation Launcher (Native Desktop)
# ==============================================================================
# Usage in Windows PowerShell:
#   .\run_simulation.ps1
# ==============================================================================

Write-Host "==================================================================" -ForegroundColor Cyan
Write-Host "    ADAPTIVE SPEED CONTROL IN CLUTTERED ENVIRONMENTS SIMULATION    " -ForegroundColor Yellow
Write-Host "==================================================================" -ForegroundColor Cyan
Write-Host "Starting Gazebo Sim and RViz on your Windows Desktop (WSLg)..." -ForegroundColor Green
Write-Host "Press Ctrl+C in this PowerShell window to stop the simulation.`n" -ForegroundColor Yellow

$WorkspacePath = "/mnt/d/Ayush_Files/Users/kumar/robotics2"

wsl -d Ubuntu-24.04 bash -c "cd $WorkspacePath && chmod +x run_simulation.sh && ./run_simulation.sh"
