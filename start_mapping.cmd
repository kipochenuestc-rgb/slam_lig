@echo off
chcp 65001 >nul
wsl -d Ubuntu-24.04 --exec python3 /mnt/d/slam/slam_lig/scripts/start_menu.py %*
if "%~1"=="--dry-run" exit /b %errorlevel%
pause
