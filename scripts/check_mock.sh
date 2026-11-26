#!/usr/bin/env bash
set -e
cd "$(dirname "$(realpath "$0")")/.."
source scripts/env.sh
mkdir -p .runtime
setsid bash scripts/run_mock.sh use_rviz:=false > .runtime/mock.log 2>&1 &
PIPER_MOCK_PID=$!
trap 'kill -INT -- "-$PIPER_MOCK_PID" 2>/dev/null || true' EXIT
python3 scripts/verify_mock.py
