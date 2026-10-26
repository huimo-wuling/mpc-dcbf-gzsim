#!/usr/bin/env bash
# Reproduce Table I of Jian et al., ICRA 2023: run the four planner variants
# of the same Gazebo scenario and collect Min dist / Cons time / Reac time /
# Speed var for each.
#
#   ours     : D-CBF safety constraint + KF trajectory prediction
#   mpc      : Euclidean keep-out constraint, no prediction
#   mpc_cbf  : D-CBF constraint, no prediction (current ellipse only)
#   mpc_kf   : Euclidean keep-out constraint + KF prediction
#
# Usage: scripts/run_all_variants.sh [duration_s]
set -o pipefail

ROOT=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
DURATION=${1:-60}
DOMAIN_BASE=${ROS_DOMAIN_ID:-70}

declare -A LABELS=(
  [ours]="MPC-D-CBF (ours)"
  [mpc]="MPC (Euclid)"
  [mpc_cbf]="MPC-CBF"
  [mpc_kf]="MPC-KF"
)

i=0
for v in ours mpc mpc_cbf mpc_kf; do
  export ROS_DOMAIN_ID=$((DOMAIN_BASE + i))
  export GZ_PARTITION="mpcdcbf_${v}"
  echo "=== variant $v (domain $ROS_DOMAIN_ID) ==="
  bash "$ROOT/scripts/run_experiment.sh" "$DURATION" \
       "$ROOT/results/run_$v.json" "${LABELS[$v]}" "$v" || echo "!! variant $v failed"
  i=$((i + 1))
done

python3 "$ROOT/scripts/compare_variants.py" --results "$ROOT/results" \
        --out "$ROOT/results"
