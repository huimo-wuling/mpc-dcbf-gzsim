#!/usr/bin/env bash
# Run one closed-loop reproduction experiment end to end:
#   launch gz-sim (headless) -> record ground truth -> stop -> analyse -> plots
#
# Usage: scripts/run_experiment.sh [duration_s] [output_json] [label] [variant]
set -o pipefail

ROOT=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
DURATION=${1:-90}
OUT=${2:-"$ROOT/results/run_ours.json"}
LABEL=${3:-"MPC-D-CBF (ours)"}
VARIANT=${4:-ours}
DISTRO=${ROS_DISTRO:-lyrical}

mkdir -p "$ROOT/results"

# shellcheck disable=SC1091
source "/opt/ros/$DISTRO/setup.bash"
# shellcheck disable=SC1091
source "$ROOT/install/setup.bash"

cleanup() {
  kill -INT "$LAUNCH_PID" 2>/dev/null
  # gz-sim and the ROS nodes are children of ros2 launch; give launch a moment
  # to tear them down, then remove leftovers by name (never the shell itself).
  for _ in 1 2 3 4 5; do
    sleep 1
    pgrep -f "mpc_dcbf_bringup simulation" >/dev/null 2>&1 || break
  done
  pkill -f "[m]pc_dcbf_bringup simulation" 2>/dev/null
  pkill -f "[m]pc-dcbf-gzsim/install" 2>/dev/null
  pkill -f "[g]z sim -s -r" 2>/dev/null
  wait "$LAUNCH_PID" 2>/dev/null
}

echo "[1/3] launching gz-sim (headless), planner variant '$VARIANT' ..."
ros2 launch mpc_dcbf_bringup simulation.launch.py headless:=true rviz:=false \
  variant:="$VARIANT" > "$ROOT/results/launch_$VARIANT.log" 2>&1 &
LAUNCH_PID=$!
trap cleanup EXIT

sleep 4

echo "[2/3] recording ${DURATION}s ..."
python3 "$ROOT/scripts/record_run.py" --duration "$DURATION" --output "$OUT" \
  || { echo "recording failed, see $ROOT/results/launch.log"; exit 1; }

cleanup
trap - EXIT

echo "[3/3] analysing ..."
python3 "$ROOT/scripts/analyze_run.py" --input "$OUT" --outdir "$ROOT/results" \
  --label "$LABEL" --tag "$VARIANT"
echo "done: $ROOT/results/metrics.md, trajectory.png, speed_clearance.png"
