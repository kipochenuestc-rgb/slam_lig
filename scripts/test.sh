#!/bin/bash
set -e
ROOT=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
export PYTHONPATH="$ROOT/scripts${PYTHONPATH:+:$PYTHONPATH}"
export SLAM_ROS_DOMAIN=89
exec bash "$ROOT/scripts/ros_env.sh" python3 -m unittest discover -s "$ROOT/tests" -v
