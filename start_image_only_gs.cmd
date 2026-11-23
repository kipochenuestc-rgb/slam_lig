@echo off
setlocal
set "PROJECT_WSL=/mnt/d/slam/slam_lig"
if "%~1"=="" (
  echo Usage: start_image_only_gs.cmd NEW_OUTPUT_NAME
  echo Example: start_image_only_gs.cmd image_only_run_02
  exit /b 1
)
wsl -d Ubuntu-24.04 -- bash "%PROJECT_WSL%/scripts/run_image_only_gs.sh" "%PROJECT_WSL%/runs/rgb_gs_wide" "%PROJECT_WSL%/runs/%~1" "%PROJECT_WSL%/runs/static_semantics_v2"
if errorlevel 1 exit /b 1
echo Done.
