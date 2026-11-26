#!/bin/bash
set -e
export CUDA_HOME="${SLAM_GS_CUDA_HOME:-/home/kipoc/miniconda3/envs/bevseg}"
export PATH="$CUDA_HOME/bin:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin"
export CXX=/usr/bin/g++
export TORCH_CUDA_ARCH_LIST="${TORCH_CUDA_ARCH_LIST:-12.0}"
export MAX_JOBS="${MAX_JOBS:-2}"
export OPENBLAS_NUM_THREADS=1
export OMP_NUM_THREADS=4
exec "${SLAM_GS_PYTHON:-/home/kipoc/slam_gs_env/bin/python}" "$@"
