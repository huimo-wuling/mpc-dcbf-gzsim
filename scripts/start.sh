#!/usr/bin/env bash
# Start the simulation in a repeatable way.
#
# Gazebo Sim's server often outlives a Ctrl+C or a closed Gazebo window; a
# surviving server makes the next launch fail with
#   "Another world of the same name is running"
# so this wrapper cleans up first, then launches. Any extra arguments are passed
# straight to the launch file.
#
# Usage:
#   scripts/start.sh                                  # Gazebo GUI + RViz
#   scripts/start.sh headless:=true                    # gz server only + RViz
#   scripts/start.sh headless:=true rviz:=false        # fully headless
set -o pipefail

ROOT=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
DISTRO=${ROS_DISTRO:-lyrical}

bash "$ROOT/scripts/stop.sh" >/dev/null 2>&1 || true

# ROS setup scripts reference unset variables; disable -u while sourcing them
set +u
# shellcheck disable=SC1091
source "/opt/ros/$DISTRO/setup.bash"
if [ ! -f "$ROOT/install/setup.bash" ]; then
  echo "workspace is not built yet: (cd $ROOT && bash scripts/import_repos.sh && colcon build --symlink-install)"
  exit 1
fi
# shellcheck disable=SC1091
source "$ROOT/install/setup.bash"
set -u

exec ros2 launch mpc_dcbf_bringup simulation.launch.py "$@"
