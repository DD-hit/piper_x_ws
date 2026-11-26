#!/usr/bin/env bash
set -e
cd "$(dirname "$(realpath "$0")")"
source ../scripts/env.sh
export ROS_DOMAIN_ID=74
exec python3 task.py "$@"
