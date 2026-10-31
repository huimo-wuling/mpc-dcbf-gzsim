#!/usr/bin/env bash
# Stop everything this project started.
#
# `ros2 launch` normally tears its children down on Ctrl+C, but Gazebo Sim's
# server (gz-sim-main) and its GUI client often survive -- closing the Gazebo
# window with the X button only kills the GUI client, for instance. A surviving
# server makes the next launch fail with
#   "Another world of the same name is running"
# so run this before relaunching if you are not sure everything exited.
#
# Usage: scripts/stop.sh
set -u

PATTERNS=(
  "ros2 launch mpc_dcbf_bringup"
  "gz sim -r"
  "gz-sim-gui-client"
  "gz-sim-main .*mpc_dcbf_world"
  "mpc-dcbf-gzsim/install/.*(local_planner|controller|local_map_pub|obs_kf|movetest|odom_tf|global_path_pub|parameter_bridge|robot_state_publisher|rviz2)"
)

# Send $1 to every process matching $2, skipping this script, its parent and
# anything else running stop.sh (otherwise the caller's own shell can match).
signal_matches() {
  local sig=$1 pattern=$2 pid cmd
  for pid in $(pgrep -f "$pattern" 2>/dev/null); do
    [ "$pid" = "$$" ] && continue
    [ "$pid" = "$PPID" ] && continue
    cmd=$(tr '\0' ' ' < "/proc/$pid/cmdline" 2>/dev/null) || continue
    case "$cmd" in
      *stop.sh*) continue ;;
    esac
    kill -"$sig" "$pid" 2>/dev/null
  done
}

echo "[stop] asking ros2 launch / Gazebo Sim / nodes to exit"
for p in "${PATTERNS[@]}"; do signal_matches INT "$p"; done
sleep 2
for p in "${PATTERNS[@]}"; do signal_matches TERM "$p"; done
sleep 2
for p in "${PATTERNS[@]}"; do signal_matches KILL "$p"; done
sleep 1

left=$(pgrep -fc "gz-sim-main .*mpc_dcbf_world" 2>/dev/null)
left=${left:-0}
if [ "$left" -eq 0 ]; then
  echo "[stop] clean"
else
  echo "[stop] warning: $left Gazebo server process(es) still alive:"
  ps -eo pid,cmd | grep -E "gz-sim-main .*mpc_dcbf_world" | grep -v grep
fi
