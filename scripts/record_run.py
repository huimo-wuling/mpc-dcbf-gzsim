#!/usr/bin/env python3
"""Record one closed-loop MPC-D-CBF run for the reproduction study.

Ground truth comes from the simulator itself: `/world/<world>/dynamic_pose/info`
is bridged to `tf2_msgs/TFMessage` on `/world_pose_tf` (see
`config/bridge.yaml`), so the robot and the eight cylinders are tracked exactly,
independently of the perception pipeline.

Recorded signals
----------------
* `gz_poses`            : "cylinder_i" -> [[t, x, y], ...] ground-truth poses
                          (published by the obstacle driver, see movetest.cpp)
* `odom`                : robot pose in the world frame (for convenience)
* `cmd`                 : /cmd_vel as published by the controller
* `predicted_ellipses`  : /obs_predict_pub, decimated to ~1 Hz
* `solve_time`          : /local_planner/solve_time (MPC latency)

Usage:
    python3 record_run.py --duration 90 --output results/run_ours.json
"""

import argparse
import json
import math
import time

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32, Float32MultiArray


def yaw_from_quat(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class Recorder(Node):
    def __init__(self, spawn_x, spawn_y, pose_hz=20.0):
        super().__init__('mpc_dcbf_recorder')
        self.spawn_x = spawn_x
        self.spawn_y = spawn_y
        self.t0 = time.time()
        self.pose_period = 1.0 / pose_hz
        self.next_pose_t = 0.0

        self.odom = []            # [t, x, y, yaw] world frame
        self.cmd = []             # [t, v, omega]
        self.obs = []             # [t, [(cx, cy, a, b, theta), ...]]
        self.gz_poses = {}        # name -> [[t, x, y], ...]
        self.solve_time = []      # [t, seconds]
        self.plan_msgs = 0
        self.obs_msgs = 0

        self.create_subscription(Odometry, '/base_pose_ground_truth',
                                 self.on_odom, 50)
        self.create_subscription(Twist, '/cmd_vel', self.on_cmd, 50)
        self.create_subscription(Float32MultiArray, '/local_plan',
                                 self.on_plan, 10)
        self.create_subscription(Float32MultiArray, '/obs_predict_pub',
                                 self.on_obs, 10)
        self.create_subscription(Float32MultiArray, '/obstacles/true_poses',
                                 self.on_true_poses, 20)
        self.create_subscription(Float32, '/local_planner/solve_time',
                                 self.on_solve, 50)

    # ------------------------------------------------------------------ ROS
    def on_odom(self, msg):
        p = msg.pose.pose.position
        self.odom.append([round(time.time() - self.t0, 3),
                          round(self.spawn_x + p.x, 3),
                          round(self.spawn_y + p.y, 3),
                          round(yaw_from_quat(msg.pose.pose.orientation), 4)])

    def on_cmd(self, msg):
        self.cmd.append([round(time.time() - self.t0, 3),
                         round(msg.linear.x, 4), round(msg.angular.z, 4)])

    def on_plan(self, _msg):
        self.plan_msgs += 1

    def on_obs(self, msg):
        self.obs_msgs += 1
        if self.obs_msgs % 10 != 1:
            return
        data = list(msg.data)
        ellipses = [[round(data[5 * i + k], 3) for k in range(5)]
                    for i in range(len(data) // 5)]
        self.obs.append([round(time.time() - self.t0, 3), ellipses])

    def on_solve(self, msg):
        self.solve_time.append([round(time.time() - self.t0, 3),
                                round(float(msg.data), 4)])

    def on_true_poses(self, msg):
        now = round(time.time() - self.t0, 3)
        if now < self.next_pose_t:
            return
        self.next_pose_t = now + self.pose_period
        data = list(msg.data)
        for i in range(len(data) // 2):
            self.gz_poses.setdefault('cylinder_%d' % i, []).append(
                [now, round(data[2 * i], 3), round(data[2 * i + 1], 3)])

    def dump(self, path):
        payload = {
            'duration': getattr(self, 'duration', 0.0),
            'spawn': [self.spawn_x, self.spawn_y],
            'odom': self.odom,
            'cmd': self.cmd,
            'predicted_ellipses': self.obs,
            'gz_poses': self.gz_poses,
            'solve_time': self.solve_time,
            'local_plan_msgs': self.plan_msgs,
            'obs_predict_msgs': self.obs_msgs,
        }
        with open(path, 'w') as f:
            json.dump(payload, f)
        return payload


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--duration', type=float, default=90.0)
    ap.add_argument('--output', default='results/run.json')
    ap.add_argument('--spawn-x', type=float, default=-10.0)
    ap.add_argument('--spawn-y', type=float, default=0.0)
    args = ap.parse_args()

    rclpy.init()
    node = Recorder(args.spawn_x, args.spawn_y)
    t_end = time.time() + args.duration
    while time.time() < t_end:
        rclpy.spin_once(node, timeout_sec=0.02)

    node.duration = args.duration
    payload = node.dump(args.output)
    n_solve = len(payload['solve_time'])
    mean_solve = (sum(s[1] for s in payload['solve_time']) / n_solve) if n_solve else float('nan')
    print('recorded %.1f s -> %s' % (args.duration, args.output))
    print('  odom %d, cmd %d, local_plan %d (%.1f Hz), obs_predict %d (%.1f Hz), entities %d' % (
        len(payload['odom']), len(payload['cmd']), payload['local_plan_msgs'],
        payload['local_plan_msgs'] / args.duration, payload['obs_predict_msgs'],
        payload['obs_predict_msgs'] / args.duration, len(payload['gz_poses'])))
    print('  solves %d, mean solve %.3f s' % (n_solve, mean_solve))
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
