#!/usr/bin/env bash
# 在干净 Bash 中 source 本文件；避免混入 Conda 和其他 ROS 工作空间。
export PIPER_X_WS_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/humble/setup.bash
if [[ -f "$PIPER_X_WS_ROOT/.venv/bin/activate" ]]; then
  source "$PIPER_X_WS_ROOT/.venv/bin/activate"
fi
if [[ ${PIPER_SKIP_OVERLAY:-0} != 1 ]]; then
  if [[ -f "$PIPER_X_WS_ROOT/.colcon/install/setup.bash" ]]; then
    source "$PIPER_X_WS_ROOT/.colcon/install/setup.bash"
  elif [[ -f "$PIPER_X_WS_ROOT/install/setup.bash" ]]; then
    source "$PIPER_X_WS_ROOT/install/setup.bash"
  fi
fi
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-73}"
export ROS_LOCALHOST_ONLY=1

# 固定项目通信中间件，避免继承桌面终端的全局 Cyclone DDS 网卡配置。
unset CYCLONEDDS_URI FASTRTPS_DEFAULT_PROFILES_FILE FASTDDS_DEFAULT_PROFILES_FILE
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
