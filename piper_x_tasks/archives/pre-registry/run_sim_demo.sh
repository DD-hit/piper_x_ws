#!/usr/bin/env bash
set -e
cd "$(dirname "$0")"
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-74}"
export ROS_LOCALHOST_ONLY=1
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
unset CYCLONEDDS_URI FASTRTPS_DEFAULT_PROFILES_FILE FASTDDS_DEFAULT_PROFILES_FILE
mkdir -p logs
exec 9>logs/demo.lock
flock -n 9 || { echo '已有本项目任务正在执行' >&2; exit 1; }
set -o pipefail
python3 run_demo.py --ros-args -p use_sim_time:=true "$@" 2>&1 | tee "logs/$(date +%Y%m%d-%H%M%S)-$$.log"
