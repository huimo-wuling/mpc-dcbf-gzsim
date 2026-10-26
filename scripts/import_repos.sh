#!/usr/bin/env bash
# Fetch the third-party sources that are not released for this ROS 2 distro and
# patch grid_map for the current ament_cmake (ament_target_dependencies was
# removed in recent releases). Idempotent: safe to run repeatedly.
set -euo pipefail

ROOT=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
GRID_MAP="$ROOT/src/grid_map"
FILTERS="$ROOT/src/filters"
PATCH="$ROOT/patches/grid_map_ros_ament_target_deps.patch"

clone_if_missing() {
  local url=$1 branch=$2 dest=$3
  if [ -d "$dest/.git" ]; then
    echo "[skip] $dest already present"
  else
    echo "[git]  $url ($branch) -> $dest"
    git clone --depth 1 --branch "$branch" "$url" "$dest"
  fi
}

clone_if_missing https://github.com/ANYbotics/grid_map.git ros2 "$GRID_MAP"
clone_if_missing https://github.com/ros/filters.git ros2 "$FILTERS"

if [ -f "$PATCH" ]; then
  if git -C "$GRID_MAP" apply --check "$PATCH" 2>/dev/null; then
    echo "[patch] applying $(basename "$PATCH")"
    git -C "$GRID_MAP" apply "$PATCH"
  elif git -C "$GRID_MAP" apply --reverse --check "$PATCH" 2>/dev/null; then
    echo "[skip] patch already applied"
  else
    echo "[warn] patch does not apply cleanly; trying 'patch -p1'"
    (cd "$GRID_MAP" && patch -p1 --forward < "$PATCH") || true
  fi
fi

# Unused grid_map sub-packages pull in heavy dependencies (octomap, PCL, rviz,
# OpenCV demos) that the MPC-D-CBF stack never touches.
for p in costmap_2d demos filters loader octomap pcl rviz_plugin sdf visualization; do
  touch "$GRID_MAP/grid_map_$p/COLCON_IGNORE"
done

echo "done. now run: colcon build --symlink-install"
