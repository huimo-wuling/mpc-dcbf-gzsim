#!/usr/bin/env python3
"""Build the paper-style comparison table and trajectory overlay (Fig. 8).

Reads the per-variant `run_<variant>.json` produced by
`scripts/run_all_variants.sh` plus the metrics computed by
`scripts/analyze_run.py`, and writes:

* results/comparison.md      : Table I reproduction
* results/comparison.png     : all trajectories on one axes
"""

import argparse
import json
import math
import os

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.patches as mpatches
import matplotlib.pyplot as plt

VARIANTS = [
    ('ours', 'MPC-D-CBF (ours)', 'tab:blue'),
    ('mpc', 'MPC (Euclid)', 'tab:red'),
    ('mpc_cbf', 'MPC-CBF', 'tab:green'),
    ('mpc_kf', 'MPC-KF', 'tab:purple'),
]

CYLINDER_RADIUS = 0.25


def load_metrics(results, variant):
    path = os.path.join(results, 'metrics_%s.json' % variant)
    if not os.path.exists(path):
        return None
    with open(path) as f:
        return json.load(f)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--results', default='results')
    ap.add_argument('--out', default='results')
    args = ap.parse_args()

    rows = []
    fig, ax = plt.subplots(figsize=(9, 4.6))
    have_any = False
    obs_drawn = False

    for variant, label, color in VARIANTS:
        run_path = os.path.join(args.results, 'run_%s.json' % variant)
        if not os.path.exists(run_path):
            continue
        with open(run_path) as f:
            run = json.load(f)
        m = load_metrics(args.results, variant)
        rows.append((variant, label, m))
        have_any = True

        odom = np.array(run['odom'])
        if len(odom):
            ax.plot(odom[:, 1], odom[:, 2], color=color, lw=1.8, label=label)

        if not obs_drawn:
            for name, track in sorted(run.get('gz_poses', {}).items()):
                if not track:
                    continue
                t0 = track[0]
                ax.add_patch(plt.Circle((t0[1], t0[2]), CYLINDER_RADIUS,
                                        color='0.3', alpha=0.35, zorder=1))
            obs_drawn = True

    ax.plot(-10, 0, 'ko', ms=6)
    ax.plot(9, 0, 'k*', ms=14)
    ax.annotate('start', (-10, 0), textcoords='offset points', xytext=(4, -12))
    ax.annotate('goal', (9, 0), textcoords='offset points', xytext=(6, 6))
    ax.plot([-10, 9], [0, 0], color='0.85', lw=1.0, ls='--', zorder=0)
    ax.set_xlabel('x [m]')
    ax.set_ylabel('y [m]')
    ax.set_title('MPC / MPC-CBF / MPC-KF / MPC-D-CBF in the same gz-sim scenario')
    ax.set_aspect('equal', adjustable='datalim')
    ax.grid(alpha=0.3)
    if have_any:
        ax.legend(loc='best', fontsize=8)
    fig.tight_layout()
    fig.savefig(os.path.join(args.out, 'comparison.png'), dpi=160)
    plt.close(fig)

    header = ('| planner | Min dist [m] | Cons time [s] | Reac time [s] | '
              'Speed var | local plan [Hz] | MPC solve [ms] |')
    sep = '|---|---|---|---|---|---|---|'
    lines = [header, sep]
    for variant, label, m in rows:
        if m is None:
            lines.append('| %s | n/a | n/a | n/a | n/a | n/a | n/a |' % label)
            continue
        lat = m.get('mpc_latency', {})
        lines.append('| %s | %.3f | %.2f | %.3f | %.4f | %.1f | %.1f |' % (
            label, m['min_dist_m'], m['cons_time_s'], m['react_time_s'],
            m['speed_var'], m['local_plan_hz'],
            1000.0 * lat.get('mean_s', float('nan'))))

    # reference numbers from the paper's Table I (simulation, their platform)
    lines += ['', 'Paper (Jian et al., ICRA 2023, Table I): '
              'Ours 0.828 m / 21.52 s / 0.353 s / 0.0172; '
              'MPC 0 / - / - / -; MPC-CBF 0.273 m / 22.07 s / ... / 0.1598.']

    text = '\n'.join(lines) + '\n'
    with open(os.path.join(args.out, 'comparison.md'), 'w') as f:
        f.write(text)
    print(text)


if __name__ == '__main__':
    main()
