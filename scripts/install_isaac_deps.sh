#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$(realpath "$0")")/.."
isaac_python="${ISAAC_SIM_ROOT:-/home/rl/isaacsim}/python.sh"
"$isaac_python" -m pip install --upgrade --target .deps/isaac -r scripts/isaac-requirements.txt
