#!/usr/bin/env bash
set -e
cd "$(dirname "$0")/.."
mkdir -p .runtime
exec 8>.runtime/isaac-physics.lock
flock -n 8 || { echo 'ERROR: Isaac physics is already running.' >&2; exit 1; }
source scripts/env.sh
ISAAC_RUNTIME="$(ros2 pkg prefix --share piper_x_isaac)/runtime"
python3 scripts/prepare_isaac_model.py
export ROS_DOMAIN_ID=74
if [[ ${PIPER_WRIST_ATTACHMENT:-} == gemini336l || ${PIPER_GEMINI:-0} == 1 ]]; then
    export PIPER_DESCRIPTION_SHARE="$(ros2 pkg prefix --share piper_x_description)"
fi
unset PYTHONPATH VIRTUAL_ENV CONDA_PREFIX AMENT_PREFIX_PATH CMAKE_PREFIX_PATH
export ROS_DISTRO=humble
export LD_LIBRARY_PATH="/home/rl/isaacsim/exts/isaacsim.ros2.bridge/humble/lib"
exec /home/rl/isaacsim/python.sh "$ISAAC_RUNTIME/run_scene.py" "$@"
