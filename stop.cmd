@echo off
chcp 65001 >nul
wsl -d Ubuntu-24.04 --exec python3 /mnt/d/slam/slam_lig/scripts/session_control.py stop %*
pause
