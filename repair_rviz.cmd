@echo off
chcp 65001 >nul
echo This restarts Ubuntu-24.04 graphics and closes its Linux GUI windows.
echo Mapping and terminal jobs continue. RViz will reopen on the live map.
wsl -d Ubuntu-24.04 --exec bash /mnt/d/slam/slam_lig/scripts/ros_env.sh python3 /mnt/d/slam/slam_lig/scripts/launch_rviz.py --repair
pause
