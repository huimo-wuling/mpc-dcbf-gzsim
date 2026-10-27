#!/usr/bin/env python3
# ROS 2 port of local_planner/scripts/local_planner.py from
# jianzhuozhuTHU/MPC-D-CBF. The MPC with dynamic elliptical CBF constraints
# (CasADi/IPOPT) is unchanged; only the ROS API was ported.

import threading
import time

import numpy as np
import casadi as ca

import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node

from std_msgs.msg import Bool, Float32, Float32MultiArray, ColorRGBA
from geometry_msgs.msg import PoseStamped, Point
from nav_msgs.msg import Path
from visualization_msgs.msg import Marker


def distance_global(c1, c2):
    return np.sqrt((c1[0] - c2[0]) * (c1[0] - c2[0]) + (c1[1] - c2[1]) * (c1[1] - c2[1]))


class LocalPlanner(Node):
    def __init__(self):
        super().__init__('phri_planner')
        self.declare_parameter('replan_period', 0.05)
        self.replan_period = self.get_parameter('replan_period').value

        self.z = 0
        self.N = 25
        self.goal_state = np.zeros([self.N, 3])

        self.curr_state = None
        self.global_path = None

        # Pre-sized warm-start buffers: the original code used empty Python
        # lists and indexed them when the very first solve failed, which raised
        # IndexError while the locks were held and wedged the planner forever.
        self.last_input = np.zeros([self.N, 2])
        self.last_state = np.zeros([self.N + 1, 3])
        self.mpc_success = None

        self.curr_pose_lock = threading.Lock()
        self.global_path_lock = threading.Lock()
        self.obstacle_lock = threading.Lock()

        self.ob = []

        # Subscriptions live in a reentrant group so that /curr_state and
        # /obs_predict_pub keep being refreshed while the (blocking) MPC solve
        # runs. The replan timer must NOT be in the same group: with a
        # reentrant group rclpy happily fires overlapping timer callbacks, and
        # several concurrent IPOPT solves pile up until each takes tens of
        # seconds (the original rospy.Timer was serialised). A mutually
        # exclusive group reproduces the original behaviour.
        cb = ReentrantCallbackGroup()
        timer_cb = MutuallyExclusiveCallbackGroup()

        self.timer_replan = self.create_timer(
            self.replan_period, self.replan_cb, callback_group=timer_cb)

        self.sub_curr_state = self.create_subscription(
            Float32MultiArray, '/curr_state', self.curr_pose_cb, 10, callback_group=cb)

        self.sub_obs = self.create_subscription(
            Float32MultiArray, '/obs_predict_pub', self.obs_cb, 10, callback_group=cb)

        self.sub_goal = self.create_subscription(
            Path, '/global_path', self.global_path_cb, 25, callback_group=cb)

        self.pub_local_path_vis = self.create_publisher(Marker, '/pub_path_vis', 10)
        self.pub_local_path = self.create_publisher(Path, '/local_path', 10)
        self.pub_local_plan = self.create_publisher(Float32MultiArray, '/local_plan', 10)
        self.pub_start = self.create_publisher(Bool, '/cmd_move', 10)
        # measurement only (never fed back into the controller): IPOPT solve
        # time, recorded by scripts/record_run.py
        self.pub_solve_time = self.create_publisher(Float32, '/local_planner/solve_time', 10)

    def replan_cb(self):
        if self.choose_goal_state():
            for i in range(self.N - 1):
                y_diff = self.goal_state[i + 1, 1] - self.goal_state[i, 1]
                x_diff = self.goal_state[i + 1, 0] - self.goal_state[i, 0]
                if x_diff != 0 and y_diff != 0:
                    self.goal_state[i, 2] = np.arctan2(y_diff, x_diff)
                elif i != 0:
                    self.goal_state[i, 2] = self.goal_state[i - 1, 2]
                else:
                    self.goal_state[i, 2] = 0

            self.goal_state[-1, 2] = self.goal_state[-2, 2]

            states_sol, input_sol = self.MPC_ellip()

            cmd_move = Bool()
            cmd_move.data = bool(distance_global(self.curr_state, self.global_path[-1]) > 0.1)
            self.pub_start.publish(cmd_move)
            self.publish_local_plan(input_sol, states_sol)

    def curr_pose_cb(self, data):
        self.curr_pose_lock.acquire()
        self.curr_state = np.zeros(3)
        self.curr_state[0] = data.data[0]
        self.curr_state[1] = data.data[1]
        self.curr_state[2] = data.data[2]
        self.curr_pose_lock.release()

    def obs_cb(self, data):
        self.obstacle_lock.acquire()
        self.ob = []
        size = int(len(data.data) / 5)
        for i in range(size):
            self.ob.append(np.array(data.data[5 * i:5 * i + 5]))
        self.obstacle_lock.release()

    def global_path_cb(self, path):
        self.global_path_lock.acquire()
        size = len(path.poses)
        if size > 0:
            self.global_path = np.zeros([size, 3])
            for i in range(size):
                self.global_path[i, 0] = path.poses[i].pose.position.x
                self.global_path[i, 1] = path.poses[i].pose.position.y
        self.global_path_lock.release()

    def publish_local_plan(self, input_sol, state_sol):
        local_path = Path()
        local_plan = Float32MultiArray()
        local_path_vis = Marker()
        local_path_vis.type = Marker.LINE_LIST
        local_path_vis.scale.x = 0.05
        local_path_vis.color.g = local_path_vis.color.b = local_path_vis.color.a = 1.0

        stamp = self.get_clock().now().to_msg()
        local_path_vis.header.stamp = stamp
        local_path.header.stamp = stamp

        local_path.header.frame_id = "world"
        local_path_vis.header.frame_id = "world"

        for i in range(self.N):
            this_pose_stamped = PoseStamped()
            this_pose_stamped.pose.position.x = float(state_sol[i, 0])
            this_pose_stamped.pose.position.y = float(state_sol[i, 1])
            this_pose_stamped.pose.position.z = float(self.z)
            this_pose_stamped.pose.orientation.x = 0.0
            this_pose_stamped.pose.orientation.y = 0.0
            this_pose_stamped.pose.orientation.z = 0.0
            this_pose_stamped.pose.orientation.w = 1.0
            this_pose_stamped.header.stamp = stamp
            this_pose_stamped.header.frame_id = "world"
            local_path.poses.append(this_pose_stamped)
            for j in range(2):
                if len(input_sol[i]) != 0:
                    local_plan.data.append(float(input_sol[i][j]))

            pt = Point()
            pt.x = float(state_sol[i, 0])
            pt.y = float(state_sol[i, 1])
            pt.z = float(self.z)

            color = ColorRGBA()
            color.r = 1.0
            color.g = 0.82
            color.b = 0.1
            color.a = 1.0

            p1 = Point()
            p2 = Point()
            p3 = Point()
            p4 = Point()
            if i < self.N - 1:
                x_diff = state_sol[i + 1, 0] - state_sol[i, 0]
                y_diff = state_sol[i + 1, 1] - state_sol[i, 1]
                if x_diff != 0 and y_diff != 0:
                    theta = np.arctan2(y_diff, x_diff)
                else:
                    theta = 0
                w = 0.7
                l = 0.92
                p1.z = float(pt.z - 0.01)
                p1.x = float(0.5 * (l * np.cos(theta) - w * np.sin(theta)) + pt.x)
                p1.y = float(0.5 * (l * np.sin(theta) + w * np.cos(theta)) + pt.y)
                p2.z = float(pt.z - 0.01)
                p2.x = float(0.5 * (-l * np.cos(theta) - w * np.sin(theta)) + pt.x)
                p2.y = float(0.5 * (-l * np.sin(theta) + w * np.cos(theta)) + pt.y)
                p3.z = float(pt.z - 0.01)
                p3.x = float(0.5 * (-l * np.cos(theta) + w * np.sin(theta)) + pt.x)
                p3.y = float(0.5 * (-l * np.sin(theta) - w * np.cos(theta)) + pt.y)
                p4.z = float(pt.z - 0.01)
                p4.x = float(0.5 * (l * np.cos(theta) + w * np.sin(theta)) + pt.x)
                p4.y = float(0.5 * (l * np.sin(theta) - w * np.cos(theta)) + pt.y)

                local_path_vis.points.append(p1)
                local_path_vis.colors.append(color)
                local_path_vis.points.append(p2)
                local_path_vis.colors.append(color)
                local_path_vis.points.append(p2)
                local_path_vis.colors.append(color)
                local_path_vis.points.append(p3)
                local_path_vis.colors.append(color)
                local_path_vis.points.append(p3)
                local_path_vis.colors.append(color)
                local_path_vis.points.append(p4)
                local_path_vis.colors.append(color)
                local_path_vis.points.append(p4)
                local_path_vis.colors.append(color)
                local_path_vis.points.append(p1)
                local_path_vis.colors.append(color)
                local_path_vis.pose.orientation.x = 0.0
                local_path_vis.pose.orientation.y = 0.0
                local_path_vis.pose.orientation.z = 0.0
                local_path_vis.pose.orientation.w = 1.0

        self.pub_local_path_vis.publish(local_path_vis)
        self.pub_local_path.publish(local_path)
        self.pub_local_plan.publish(local_plan)

    def choose_goal_state(self):
        self.curr_pose_lock.acquire()
        self.global_path_lock.acquire()
        if self.global_path is None or self.curr_state is None:
            self.curr_pose_lock.release()
            self.global_path_lock.release()
            return False

        waypoint_num = self.global_path.shape[0]
        num = np.argmin(np.array([distance_global(self.curr_state, self.global_path[i]) for i in range(waypoint_num)]))

        scale = 1
        num_list = []
        for i in range(self.N):
            num_path = min(waypoint_num - 1, num + i * scale)
            num_list.append(num_path)

        for k in range(self.N):
            self.goal_state[k] = self.global_path[num_list[k]]

        self.curr_pose_lock.release()
        self.global_path_lock.release()
        return True

    def MPC_ellip(self):
        self.curr_pose_lock.acquire()
        self.global_path_lock.acquire()
        self.obstacle_lock.acquire()

        opti = ca.Opti()
        T = 0.1
        gamma_k = 0.3

        v_max = 1.2
        v_min = 0.1
        omega_max = 1.2

        opt_x0 = opti.parameter(3)

        opt_states = opti.variable(self.N + 1, 3)
        opt_controls = opti.variable(self.N, 2)
        v = opt_controls[:, 0]
        omega = opt_controls[:, 1]

        def f(x_, u_):
            return ca.vertcat(*[u_[0] * ca.cos(x_[2]), u_[0] * ca.sin(x_[2]), u_[1]])

        def exceed_ob(ob_):
            l_long_axis = ob_[2]
            l_short_axis = ob_[3]
            long_axis = np.array([np.cos(ob_[4]) * l_long_axis, np.sin(ob_[4]) * l_long_axis])

            ob_vec = np.array([ob_[0], ob_[1]])
            center_vec = self.goal_state[self.N - 1, :2] - ob_vec
            dist_center = np.linalg.norm(center_vec)
            cos_ = np.dot(center_vec, long_axis.T) / (dist_center * l_long_axis)

            if np.abs(cos_) > 0.1:
                tan_square = 1 / (cos_ ** 2) - 1
                d = np.sqrt((l_long_axis ** 2 * l_short_axis ** 2 * (1 + tan_square) /
                             (l_short_axis ** 2 + l_long_axis ** 2 * tan_square)))
            else:
                d = l_short_axis

            cross_pt = ob_vec + d * center_vec / dist_center

            vec1 = self.goal_state[self.N - 1, :2] - cross_pt
            vec2 = self.curr_state[:2] - cross_pt
            theta = np.dot(vec1.T, vec2)

            return theta > 0

        def h(curpos_, ob_):
            safe_dist = 0.5  # for jackal

            c = ca.cos(ob_[4])
            s = ca.sin(ob_[4])
            a = ca.MX([ob_[2]])
            b = ca.MX([ob_[3]])

            ob_vec = ca.MX([ob_[0], ob_[1]])
            center_vec = curpos_[:2] - ob_vec.T

            dist = b * (ca.sqrt((c ** 2 / a ** 2 + s ** 2 / b ** 2) * center_vec[0] ** 2 +
                                (s ** 2 / a ** 2 + c ** 2 / b ** 2) * center_vec[1] ** 2 +
                                2 * c * s * (1 / a ** 2 - 1 / b ** 2) * center_vec[0] * center_vec[1]) - 1) - safe_dist

            return dist

        def quadratic(x, A):
            return ca.mtimes([x, A, x.T])

        opti.subject_to(opt_states[0, :] == opt_x0.T)

        if (distance_global(self.curr_state, self.global_path[-1, :2]) > 1):
            opti.subject_to(opti.bounded(v_min, v, v_max))
        else:
            opti.subject_to(opti.bounded(-v_min, v, v_max))

        opti.subject_to(opti.bounded(-omega_max, omega, omega_max))

        for i in range(self.N):
            x_next = opt_states[i, :] + T * f(opt_states[i, :], opt_controls[i, :]).T
            opti.subject_to(opt_states[i + 1, :] == x_next)

        num_obs = int(len(self.ob) / self.N)

        for j in range(num_obs):
            if not exceed_ob(self.ob[25 * j]):
                for i in range(self.N - 1):
                    opti.subject_to(h(opt_states[i + 1, :], self.ob[j * 25 + i + 1]) >=
                                    (1 - gamma_k) * h(opt_states[i, :], self.ob[j * 25 + i]))

        obj = 0
        R = np.diag([0.1, 0.02])
        for i in range(self.N):
            Q = np.diag([1.0 + 0.05 * i, 1.0 + 0.05 * i, 0.02 + 0.005 * i])
            if i < self.N - 1:
                obj += 0.1 * quadratic(opt_states[i, :] - self.goal_state[[i]], Q) + quadratic(opt_controls[i, :], R)
            else:
                obj += 0.1 * quadratic(opt_states[i, :] - self.goal_state[[i]], Q)
        Q = np.diag([1.0, 1.0, 0.02]) * 5
        obj += quadratic(opt_states[self.N - 1, :] - self.goal_state[[self.N - 1]], Q)

        opti.minimize(obj)
        opts_setting = {'ipopt.max_iter': 2000, 'ipopt.print_level': 0, 'print_time': 0,
                        'ipopt.acceptable_tol': 1e-3, 'ipopt.acceptable_obj_change_tol': 1e-3}
        opti.solver('ipopt', opts_setting)
        opti.set_value(opt_x0, self.curr_state)

        try:
            _solve_t0 = time.perf_counter()
            sol = opti.solve()
            _solve_msg = Float32()
            _solve_msg.data = float(time.perf_counter() - _solve_t0)
            self.pub_solve_time.publish(_solve_msg)
            u_res = np.array(sol.value(opt_controls))
            state_res = np.array(sol.value(opt_states))

            self.last_input = u_res
            self.last_state = state_res
            self.mpc_success = True

        except Exception:
            self.get_logger().error("Infeasible Solution", throttle_duration_sec=2.0)

            if self.mpc_success:
                self.mpc_success = False
            else:
                # shift the previous warm start one step, zero the last input
                self.last_input = np.roll(np.asarray(self.last_input), -1, axis=0)
                self.last_input[-1] = np.zeros(2)
                self.last_state = np.roll(np.asarray(self.last_state), -1, axis=0)

            u_res = self.last_input
            state_res = self.last_state
        finally:
            self.curr_pose_lock.release()
            self.global_path_lock.release()
            self.obstacle_lock.release()
        return state_res, u_res


def main(args=None):
    rclpy.init(args=args)
    node = LocalPlanner()
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
