#!/bin/bash
set -e
ROOT=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
export PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin
source /opt/ros/jazzy/setup.bash
WORKSPACE=${SLAM_WORKSPACE:-$HOME/slam_lig_ws}
if [ ! -f "$WORKSPACE/install/setup.bash" ]; then
  echo "Build first: bash $ROOT/scripts/build.sh" >&2
  exit 1
fi
source "$WORKSPACE/install/setup.bash"
export ROS_DOMAIN_ID=${SLAM_ROS_DOMAIN:-50}
export ROS_AUTOMATIC_DISCOVERY_RANGE=SYSTEM_DEFAULT
export FASTRTPS_DEFAULT_PROFILES_FILE="$ROOT/config/fastdds_local.xml"
export QT_QPA_PLATFORM=xcb
export QT_X11_NO_MITSHM=1
export LIBGL_ALWAYS_SOFTWARE=1
export LP_NUM_THREADS=4
export OPENBLAS_NUM_THREADS=1
export OMP_NUM_THREADS=4
exec "$@"
