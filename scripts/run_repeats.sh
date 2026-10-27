#!/usr/bin/env bash
# Repeat the paper's closed-loop scenario N times (identical code path, fresh
# Gazebo run each time) and summarise the Table-I style metrics, so the
# run-to-run spread of the reproduction is visible.
#
# Usage: scripts/run_repeats.sh [N] [duration_s]
set -o pipefail

ROOT=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
N=${1:-3}
DURATION=${2:-60}
DOMAIN_BASE=${ROS_DOMAIN_ID:-70}

for i in $(seq 1 "$N"); do
  export ROS_DOMAIN_ID=$((DOMAIN_BASE + i))
  export GZ_PARTITION="mpcdcbf_run${i}"
  echo "=== run $i/$N (ROS_DOMAIN_ID=$ROS_DOMAIN_ID, duration ${DURATION}s) ==="
  bash "$ROOT/scripts/run_experiment.sh" "$DURATION" \
       "$ROOT/results/run_r$i.json" "MPC-D-CBF (run $i)" || echo "!! run $i failed"
  cp "$ROOT/results/metrics.json" "$ROOT/results/metrics_r$i.json" 2>/dev/null
done

python3 - "$ROOT/results" "$N" <<'PY'
import json, os, sys
res, n = sys.argv[1], int(sys.argv[2])
rows = []
for i in range(1, n + 1):
    p = os.path.join(res, 'metrics_r%d.json' % i)
    if os.path.exists(p):
        rows.append((i, json.load(open(p))))
if not rows:
    raise SystemExit('no metrics found')
keys = ['min_dist_m', 'cons_time_s', 'react_time_s', 'speed_var', 'local_plan_hz']
lines = ['| run | ' + ' | '.join(keys) + ' |', '|' + '---|' * (len(keys) + 1)]
for i, m in rows:
    lines.append('| %d | %.3f | %.2f | %.3f | %.4f | %.1f |' % (
        i, m['min_dist_m'], m['cons_time_s'], m['react_time_s'],
        m['speed_var'], m['local_plan_hz']))
print('\n'.join(lines))
lat = [m['mpc_latency']['mean_s'] for _, m in rows]
print('\nmean MPC solve: %.1f ms (runs: %s)' % (
    1000 * sum(lat) / len(lat), ', '.join('%.1f' % (1000 * v) for v in lat)))
with open(os.path.join(res, 'repeats.md'), 'w') as f:
    f.write('\n'.join(lines) + '\n')
PY
