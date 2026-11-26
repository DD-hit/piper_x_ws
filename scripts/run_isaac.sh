#!/usr/bin/env bash
# Isaac physics stays headless; optional RViz shows its measured state over RDP.
set -e
cd "$(dirname "$(realpath "$0")")/.."
mkdir -p .runtime
exec 9>.runtime/isaac.lock
if ! flock -n 9; then echo 'Isaac project session already running.' >&2; exit 1; fi
RVIZ=false
STREAM_ARGS=()
MOVEIT_ARGS=()
for arg in "$@"; do
 case "$arg" in
  --rviz) RVIZ=true ;;
  --stream) STREAM_ARGS+=(--stream) ;;
  wrist_attachment:=*) export PIPER_WRIST_ATTACHMENT="${arg#wrist_attachment:=}" ;;
  *) MOVEIT_ARGS+=("$arg") ;;
 esac
done
if [[ -n ${PIPER_WRIST_ATTACHMENT:-} && $PIPER_WRIST_ATTACHMENT != none && $PIPER_WRIST_ATTACHMENT != gemini336l ]]; then
 echo "ERROR: Unsupported wrist_attachment: $PIPER_WRIST_ATTACHMENT" >&2
 exit 2
fi
setsid bash scripts/run_isaac_scene.sh --headless "${STREAM_ARGS[@]}" > .runtime/isaac-scene.log 2>&1 &
ISAAC_PID=$!
MOVEIT_PID=
LOG_PID=
cleanup() {
 trap - EXIT INT TERM
 echo 'INFO: 正在关闭机械臂服务和物理仿真。'
 kill -TERM -- "-$ISAAC_PID" 2>/dev/null || true
 if [[ -n "$MOVEIT_PID" ]]; then
  kill -TERM "$MOVEIT_PID" 2>/dev/null || true
  wait "$MOVEIT_PID" 2>/dev/null || true
 fi
 wait "$ISAAC_PID" 2>/dev/null || true
 if [[ -n "$LOG_PID" ]]; then
  kill "$LOG_PID" 2>/dev/null || true
  wait "$LOG_PID" 2>/dev/null || true
 fi
 echo 'INFO: 本次会话已退出。'
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM
printf 'Starting Isaac physics. Log: %s/.runtime/isaac-scene.log\n' "$PWD"
READY=0
for ((i=0;i<180;i++)); do
 if grep -q PIPER_ISAAC_READY .runtime/isaac-scene.log; then READY=1; break; fi
 if ! kill -0 "$ISAAC_PID" 2>/dev/null; then tail -30 .runtime/isaac-scene.log; exit 1; fi
 sleep 1
done
if [[ "$READY" != 1 ]]; then echo 'Isaac startup timed out'; exit 1; fi
setsid bash scripts/run_isaac_moveit.sh use_rviz:="$RVIZ" "${MOVEIT_ARGS[@]}" > .runtime/isaac-services.log 2>&1 &
MOVEIT_PID=$!
: > .runtime/isaac-key-events.log
tail --pid="$MOVEIT_PID" -n +1 -F .runtime/isaac-services.log 2>/dev/null |
 stdbuf -oL grep -E 'piper_x_motion_service|INFO:|ERROR:|WARN:|piper_x_isaac_feedback_check|load_demo_barrier' |
 tee -a .runtime/isaac-key-events.log &
LOG_PID=$!
echo 'Isaac physics started. Arm services are starting. Log: .runtime/isaac-services.log'
echo 'ROS_DOMAIN_ID=74. Ctrl+C stops this session.'
if wait -n "$ISAAC_PID" "$MOVEIT_PID"; then status=0; else status=$?; fi
echo "ERROR: Isaac session component exited (status $status). Check .runtime/isaac-scene.log and .runtime/isaac-services.log." >&2
exit 1
