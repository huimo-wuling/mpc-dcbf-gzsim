#!/usr/bin/env python3
# ROS 2 port of local_planner/scripts/controller.py from
# jianzhuozhuTHU/MPC-D-CBF. Tracks the first step of /local_plan at 50 Hz and
# publishes the robot state from TF; topics and rates are unchanged.
import math

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.time import Time

from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray
from tf2_ros import (Buffer, TransformListener, LookupException,
                     ConnectivityException, ExtrapolationException)


def quart_to_rpy(x, y, z, w):
    r = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    p = math.asin(2 * (w * y - z * x))
    y = math.atan2(2 * (w * z + x * y), 1 - 2 * (z * z + y * y))
    return r, p, y


class Controller(Node):
    def __init__(self):
        super().__init__('control')
        self.N = 10

        self.local_plan_sub = self.create_subscription(
            Float32MultiArray, '/local_plan', self.local_planner_cb, 10)
        self.vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.curr_state_pub = self.create_publisher(Float32MultiArray, '/curr_state', 10)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.linear_speed = 0.0
        self.angular_speed = 0.0
        self.local_plan = np.zeros([self.N, 2])

        self.timer_localization = self.create_timer(0.01, self.get_current_state)
        self.control_timer = self.create_timer(0.02, self.control_loop_tick)  # 50 Hz

    def get_current_state(self):
        try:
            t = self.tf_buffer.lookup_transform('world', 'base_link', Time())
            q = t.transform.rotation
            _, _, yaw = quart_to_rpy(q.x, q.y, q.z, q.w)
            curr_state = Float32MultiArray()
            curr_state.data = [t.transform.translation.x,
                               t.transform.translation.y, yaw]
            self.curr_state_pub.publish(curr_state)
        except (LookupException, ConnectivityException, ExtrapolationException):
            pass

    def pub_vel(self):
        control_cmd = Twist()
        control_cmd.linear.x = self.linear_speed
        control_cmd.angular.z = self.angular_speed
        self.get_logger().info(
            "Linear Speed: %.1f, Angular Speed: %.1f" % (self.linear_speed, self.angular_speed),
            throttle_duration_sec=1.0)
        self.vel_pub.publish(control_cmd)

    def control_loop_tick(self):
        self.linear_speed = self.local_plan[0, 0]
        self.angular_speed = self.local_plan[0, 1]
        self.pub_vel()

    def local_planner_cb(self, msg):
        for i in range(self.N):
            self.local_plan[i, 0] = msg.data[0 + 2 * i]
            self.local_plan[i, 1] = msg.data[1 + 2 * i]


def main(args=None):
    rclpy.init(args=args)
    node = Controller()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
