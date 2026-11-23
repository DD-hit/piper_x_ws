#!/usr/bin/env bash
set -e
cd "$(dirname "$0")/.."
source scripts/env.sh
exec ros2 launch agx_arm_moveit demo.launch.py arm_type:=piper_x effector_type:=agx_gripper follow:=false "$@"
