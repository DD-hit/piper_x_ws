#!/usr/bin/env bash
set -e
cd "$(dirname "$(realpath "$0")")/.."
PIPER_SKIP_OVERLAY=1 source scripts/env.sh
python3 -m colcon --log-base .colcon/log build \
  --base-paths src vendor/agx_arm_ros --build-base .colcon/build --install-base .colcon/install \
  --symlink-install --parallel-workers 4
