#!/usr/bin/env python3
"""Analyse a recorded MPC-D-CBF run and produce the paper-style metrics.

Metrics mirror Table I of Jian et al., ICRA 2023 ("Min dist", "Cons time",
"Reac time", "Speed var"), computed from ground-truth poses of the robot and
of the eight cylinders of the simulation world:

* Min dist  : minimum distance between the robot centre and any cylinder
              surface over the whole run [m]
* Cons time : time from start until the robot is within 0.2 m of the goal [s]
* Reac time : time between an obstacle entering the local map (7 m) and the
              first control response (|omega| > 0.05 rad/s) [s]
* Speed var : variance of the commanded linear speed while an obstacle is
              closer than 3 m [m^2/s^2]

Usage:
    python3 analyze_run.py --input results/run_ours.json --outdir results
"""

import argparse
import json
import math
import os

import numpy as np

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

CYLINDER_RADIUS = 0.25
GOAL = (9.0, 0.0)
LOCAL_MAP_RADIUS = 7.0
REACT_RADIUS = 3.0


def load(path):
    with open(path) as f:
        return json.load(f)


def true_obstacles(run):
    """Return {name: [(t, x, y), ...]} for the cylinders, from gz ground truth."""
    out = {}
    for name, samples in run.get('gz_poses', {}).items():
        if name.startswith('cylinder_'):
            out[name] = [s for s in samples]
    return out


def nearest_dist(robot_xy, cyl_xy):
    d = math.hypot(robot_xy[0] - cyl_xy[0], robot_xy[1] - cyl_xy[1])
    return d - CYLINDER_RADIUS


def interpolate(track, t):
    """Linear interpolation of an [(t, x, y), ...] track."""
    if not track:
        return None
    if t <= track[0][0]:
        return track[0][1], track[0][2]
    if t >= track[-1][0]:
        return track[-1][1], track[-1][2]
    for i in range(len(track) - 1):
        t0, x0, y0 = track[i]
        t1, x1, y1 = track[i + 1]
        if t0 <= t <= t1:
            if t1 == t0:
                return x0, y0
            a = (t - t0) / (t1 - t0)
            return x0 + a * (x1 - x0), y0 + a * (y1 - y0)
    return track[-1][1], track[-1][2]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--input', required=True)
    ap.add_argument('--outdir', default='results')
    ap.add_argument('--label', default='MPC-D-CBF (ours)')
    ap.add_argument('--tag', default='', help='also write metrics_<tag>.json')
    args = ap.parse_args()

    run = load(args.input)
    os.makedirs(args.outdir, exist_ok=True)

    odom = np.array(run['odom'])          # t, x, y, yaw
    cmd = np.array(run['cmd'])            # t, v, omega
    cyl = true_obstacles(run)

    if len(odom) == 0:
        raise SystemExit('no odometry in %s' % args.input)

    # ---------------------------------------------------------------- metrics
    # min distance to any cylinder surface, evaluated per odometry sample
    min_dist = []
    for t, x, y, _ in odom:
        best = float('inf')
        for name, track in cyl.items():
            xy = interpolate(track, t)
            if xy is None:
                continue
            best = min(best, nearest_dist((x, y), xy))
        min_dist.append(best if math.isfinite(best) else np.nan)
    min_dist = np.array(min_dist)

    # time to goal
    d_goal = np.hypot(odom[:, 1] - GOAL[0], odom[:, 2] - GOAL[1])
    reached = np.where(d_goal < 0.2)[0]
    cons_time = float(odom[reached[0], 0]) if len(reached) else float('nan')

    # reaction time: first obstacle entering 7 m -> first control response
    react_time = float('nan')
    entry_t = None
    for i, (t, x, y, _) in enumerate(odom):
        for name, track in cyl.items():
            xy = interpolate(track, t)
            if xy is None:
                continue
            if math.hypot(x - xy[0], y - xy[1]) < LOCAL_MAP_RADIUS:
                entry_t = t
                break
        if entry_t is not None:
            break
    if entry_t is not None and len(cmd):
        after = cmd[cmd[:, 0] >= entry_t]
        reacting = after[np.abs(after[:, 2]) > 0.05]
        if len(reacting):
            react_time = float(reacting[0, 0] - entry_t)

    # speed variance while an obstacle is within 3 m
    close_speeds = []
    for t, v, _w in cmd:
        xy = (np.interp(t, odom[:, 0], odom[:, 1]), np.interp(t, odom[:, 0], odom[:, 2]))
        for name, track in cyl.items():
            o = interpolate(track, t)
            if o is None:
                continue
            if math.hypot(xy[0] - o[0], xy[1] - o[1]) < REACT_RADIUS:
                close_speeds.append(v)
                break
    speed_var = float(np.var(close_speeds)) if len(close_speeds) > 1 else float('nan')

    path_len = float(np.sum(np.hypot(np.diff(odom[:, 1]), np.diff(odom[:, 2]))))

    solve = np.array(run.get('solve_time', []), dtype=float)
    if len(solve):
        solve_stats = {
            'mean_s': float(np.mean(solve[:, 1])),
            'p95_s': float(np.percentile(solve[:, 1], 95)),
            'max_s': float(np.max(solve[:, 1])),
            'n': int(len(solve)),
        }
    else:
        solve_stats = {}
    metrics = {
        'label': args.label,
        'min_dist_m': float(np.nanmin(min_dist)),
        'cons_time_s': cons_time,
        'react_time_s': react_time,
        'speed_var': speed_var,
        'path_length_m': path_len,
        'final_xy': [float(odom[-1, 1]), float(odom[-1, 2])],
        'final_dist_to_goal_m': float(d_goal[-1]),
        'local_plan_hz': run['local_plan_msgs'] / max(run.get('duration', 1.0), 1e-9),
        'obs_predict_hz': run['obs_predict_msgs'] / max(run.get('duration', 1.0), 1e-9),
        'duration_s': run.get('duration', float(odom[-1, 0])),
        'mpc_latency': solve_stats,
    }

    with open(os.path.join(args.outdir, 'metrics.json'), 'w') as f:
        json.dump(metrics, f, indent=2)
    if args.tag:
        with open(os.path.join(args.outdir, 'metrics_%s.json' % args.tag), 'w') as f:
            json.dump(metrics, f, indent=2)

    # ------------------------------------------------------------------ plots
    fig, ax = plt.subplots(figsize=(9, 4.2))

    # obstacle positions (start and end) from ground truth
    for name, track in sorted(cyl.items()):
        if not track:
            continue
        x0, y0 = track[0][1], track[0][2]
        x1, y1 = track[-1][1], track[-1][2]
        ax.plot([x0, x1], [y0, y1], color='0.6', lw=0.8, ls=':', zorder=1)
        ax.add_patch(plt.Circle((x1, y1), CYLINDER_RADIUS, color='tab:red',
                                alpha=0.45, zorder=2))
        ax.add_patch(plt.Circle((x0, y0), CYLINDER_RADIUS, color='tab:red',
                                alpha=0.12, zorder=1))

    # predicted (CBF) ellipses at the sample with the smallest clearance
    if run.get('predicted_ellipses'):
        k = int(np.nanargmin(min_dist))
        t_star = odom[k, 0]
        best = min(run['predicted_ellipses'],
                   key=lambda s: abs(s[0] - t_star))
        for cx, cy, a, b, th in best[1]:
            e = matplotlib.patches.Ellipse((cx, cy), 2 * a, 2 * b,
                                           angle=math.degrees(th),
                                           fill=False, color='tab:green',
                                           lw=1.0, alpha=0.8, zorder=3)
            ax.add_patch(e)

    ax.plot(odom[:, 1], odom[:, 2], color='tab:blue', lw=2.0,
            label='robot trajectory (%s)' % args.label, zorder=4)
    ax.plot([-10, 9], [0, 0], color='0.8', lw=1.0, ls='--', zorder=0)
    ax.plot(-10, 0, 'ko', ms=6)
    ax.plot(*GOAL, 'k*', ms=14)
    ax.annotate('start', (-10, 0), textcoords='offset points', xytext=(4, -12))
    ax.annotate('goal', GOAL, textcoords='offset points', xytext=(6, 6))
    ax.set_xlabel('x [m]')
    ax.set_ylabel('y [m]')
    ax.set_title('MPC-D-CBF closed loop in Gazebo Sim (gz-sim 10)')
    ax.set_aspect('equal', adjustable='datalim')
    ax.grid(alpha=0.3)
    ax.legend(loc='best', fontsize=8)
    fig.tight_layout()
    fig.savefig(os.path.join(args.outdir, 'trajectory.png'), dpi=160)
    plt.close(fig)

    fig, axes = plt.subplots(2, 1, figsize=(9, 5.2), sharex=True)
    axes[0].plot(cmd[:, 0], cmd[:, 1], color='tab:blue', lw=1.2)
    axes[0].set_ylabel('linear speed [m/s]')
    axes[0].grid(alpha=0.3)
    axes[0].set_title('commands and clearance')

    axes[1].plot(odom[:, 0], min_dist, color='tab:red', lw=1.2)
    axes[1].axhline(0.0, color='k', lw=0.8, ls='--')
    axes[1].set_ylabel('min clearance [m]')
    axes[1].set_xlabel('time [s]')
    axes[1].grid(alpha=0.3)
    fig.tight_layout()
    fig.savefig(os.path.join(args.outdir, 'speed_clearance.png'), dpi=160)
    plt.close(fig)

    # ----------------------------------------------------------------- report
    def fmt(v, nd=3):
        return 'n/a' if v is None or (isinstance(v, float) and math.isnan(v)) else ('%.*f' % (nd, v))

    lines = [
        '| metric | %s |' % args.label,
        '|---|---|',
        '| Min dist [m] | %s |' % fmt(metrics['min_dist_m']),
        '| Cons time [s] | %s |' % fmt(metrics['cons_time_s'], 2),
        '| Reac time [s] | %s |' % fmt(metrics['react_time_s']),
        '| Speed var | %s |' % fmt(metrics['speed_var'], 4),
        '| path length [m] | %s |' % fmt(metrics['path_length_m'], 2),
        '| final distance to goal [m] | %s |' % fmt(metrics['final_dist_to_goal_m']),
        '| local planner rate [Hz] | %s |' % fmt(metrics['local_plan_hz'], 1),
        '| KF prediction rate [Hz] | %s |' % fmt(metrics['obs_predict_hz'], 1),
        '| MPC solve mean / p95 / max [ms] | %s / %s / %s |' % (
            fmt(1000 * metrics['mpc_latency'].get('mean_s'), 1),
            fmt(1000 * metrics['mpc_latency'].get('p95_s'), 1),
            fmt(1000 * metrics['mpc_latency'].get('max_s'), 1)),
    ]
    with open(os.path.join(args.outdir, 'metrics.md'), 'w') as f:
        f.write('\n'.join(lines) + '\n')
    print('\n'.join(lines))


if __name__ == '__main__':
    main()
