#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped

import numpy as np
import os
from scipy.interpolate import RegularGridInterpolator

class ADPRacelineController(Node):
    def __init__(self):
        super().__init__('adp_raceline_control')

        # === Load gain table === #
        from ament_index_python.packages import get_package_share_directory
        utils_dir = os.path.join(
        	get_package_share_directory('adp_controller'),'scripts', 'utils')

        self.K_table = np.load(os.path.join(utils_dir, "adp_gain_table.npy"))  # shape: (V, K, 1, 4)
        vx_grid = np.load(os.path.join(utils_dir, "vx_grid.npy"))              # shape: (V,)
        kappa_grid = np.load(os.path.join(utils_dir, "kappa_grid.npy"))        # shape: (K,)
        self.K_interp = RegularGridInterpolator(
            (vx_grid, kappa_grid),
            self.K_table[:, :, 0, :],
            bounds_error=False,
            fill_value=0.0
        )

        # === ROS 2 Interfaces === #
        self.frenet_errors = None
        self.vx = 0.0

        self.create_subscription(Float32MultiArray, '/frenet_errors', self.frenet_callback, 10)
        self.create_subscription(Odometry, '/ego_racecar/odom', self.odom_callback, 10)
        self.drive_pub = self.create_publisher(AckermannDriveStamped, '/drive', 10)
        self.create_timer(0.02, self.control_loop)  # 50 Hz

        self.get_logger().info("ADP Raceline Controller Initialized")

    def frenet_callback(self, msg):
        self.frenet_errors = msg.data

    def odom_callback(self, msg):
        self.vx = msg.twist.twist.linear.x

    def control_loop(self):
        if self.frenet_errors is None:
            return

        e_y, dot_e_y, e_psi, dot_e_psi, kappa = self.frenet_errors
        x = np.array([e_y, dot_e_y, e_psi, dot_e_psi])
        vx = np.clip(self.vx, 1.0, 3.5)
        kappa = np.clip(kappa, -1.5, 1.5)

        try:
            K_row = self.K_interp((vx, kappa))
        except:
            self.get_logger().warn("Interpolation failed, using zero gain.")
            K_row = np.zeros(4)

        delta = float(-np.dot(K_row, x))
        delta = np.clip(delta, -0.4189, 0.4189)

        msg = AckermannDriveStamped()
        msg.drive.steering_angle = delta
        msg.drive.speed = 2.0
        self.drive_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = ADPRacelineController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

