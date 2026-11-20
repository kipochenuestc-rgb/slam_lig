#!/bin/bash
set -e
ROOT=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
bash "$ROOT/scripts/ros_env.sh" python3 "$ROOT/scripts/launch_rviz.py" --check || { echo 'WSLg COPY MODE: run repair_rviz.cmd to restore graphics first.' >&2; exit 75; }
RESULT=${1:-$(python3 -c 'import json,sys; print(json.load(open(sys.argv[1]))["saved_result"])' "$ROOT/config/dataset.json")}
export SLAM_RUN_DIR="$ROOT/runs"
bash "$ROOT/scripts/ros_env.sh" python3 "$ROOT/scripts/view_saved_map.py" "$RESULT" > "$ROOT/runs/map_viewer.log" 2>&1 &
VIEWER_PID=$!
trap 'kill "$VIEWER_PID" 2>/dev/null || true' EXIT
bash "$ROOT/scripts/ros_env.sh" python3 "$ROOT/scripts/launch_rviz.py" --config "$ROOT/config/four_lidar.rviz"
