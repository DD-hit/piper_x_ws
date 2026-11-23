#!/usr/bin/env bash
set -e
cd "$(dirname "$0")/.."
source scripts/env.sh
python3 -m colcon build --symlink-install --parallel-workers 4
