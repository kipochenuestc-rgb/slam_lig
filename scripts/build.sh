#!/bin/bash
set -eo pipefail
ROOT=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
export PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin
source /opt/ros/jazzy/setup.bash
WORKSPACE=${SLAM_WORKSPACE:-$HOME/slam_lig_ws}
mkdir -p "$WORKSPACE/src"
if [ ! -e "$WORKSPACE/src/LIO-SAM" ]; then ln -s "$ROOT/src/LIO-SAM" "$WORKSPACE/src/LIO-SAM"; fi
cd "$WORKSPACE"
export CMAKE_BUILD_PARALLEL_LEVEL=2
colcon build --packages-select lio_sam --executor sequential --cmake-args -DBUILD_TESTING=OFF -DCMAKE_BUILD_TYPE=Release -DGTSAM_DIR="$ROOT/cmake/gtsam"
