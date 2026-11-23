#!/usr/bin/env bash
set -e
cd "$(dirname "$0")/.."
mkdir -p .runtime
exec 8>.runtime/isaac-services.lock
flock -n 8 || { echo 'ERROR: Isaac arm services are already running.' >&2; exit 1; }
source scripts/env.sh
export ROS_DOMAIN_ID=74
python3 scripts/check_isaac_feedback.py --timeout 15
echo 'INFO: Isaac feedback received. Starting arm services on ROS_DOMAIN_ID=74.'
python3 sim/isaac/joint_mapping.py 8>&- &
PIPER_MAPPING_PID=$!
setsid ros2 launch "$PWD/sim/isaac/moveit.launch.py" arm_type:=piper_x effector_type:=agx_gripper "$@" 8>&- &
PIPER_LAUNCH_PID=$!
python3 scripts/check_isaac_feedback.py --timeout 5 --watch 8>&- &
PIPER_WATCH_PID=$!
cleanup() {
    trap - EXIT
    trap '' INT TERM
    if [[ -n ${TASK_SERVICE_PID:-} ]]; then kill -TERM "$TASK_SERVICE_PID" 2>/dev/null || true; fi
    kill -TERM "$PIPER_MAPPING_PID" "$PIPER_WATCH_PID" 2>/dev/null || true
    kill -INT "$PIPER_LAUNCH_PID" 2>/dev/null || true
    for ((attempt=0; attempt<8; attempt++)); do
        kill -0 -- "-$PIPER_LAUNCH_PID" 2>/dev/null || break
        sleep 1
    done
    if kill -0 -- "-$PIPER_LAUNCH_PID" 2>/dev/null; then
        echo 'WARN: ROS launch did not exit gracefully. Terminating its process group.' >&2
        kill -TERM -- "-$PIPER_LAUNCH_PID" 2>/dev/null || true
        sleep 2
        kill -KILL -- "-$PIPER_LAUNCH_PID" 2>/dev/null || true
    fi
    for pid in "$PIPER_MAPPING_PID" "$PIPER_WATCH_PID" "${TASK_SERVICE_PID:-}"; do
        [[ -n "$pid" ]] || continue
        if kill -0 "$pid" 2>/dev/null; then
            echo "WARN: child $pid did not exit after TERM; stopping it" >&2
            kill -KILL "$pid" 2>/dev/null || true
        fi
    done
    wait || true
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM
if [[ ${PIPER_DEMO_BARRIER:-0} == 1 ]]; then
    python3 sim/isaac/load_barrier.py
fi
python3 api/ros_motion_service.py --ros-args -p use_sim_time:=true 8>&- &
TASK_SERVICE_PID=$!
if wait -n "$PIPER_MAPPING_PID" "$PIPER_LAUNCH_PID" "$PIPER_WATCH_PID" "$TASK_SERVICE_PID"; then status=0; else status=$?; fi
echo "ERROR: Arm service component exited (status $status). Check the preceding logs before restarting." >&2
exit 1
