#!/usr/bin/env bash
set -e
cd "$(dirname "$0")"
source ../piper_x_driver/scripts/env.sh
export ROS_DOMAIN_ID=74
exec python3 task.py "$@"
