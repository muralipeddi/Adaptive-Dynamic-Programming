#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from std_msgs.msg import Float32MultiArray
from visualization_msgs.msg import Marker

import numpy as np
import os


class FrenetErrorNode(Node):
    def __init__(self):
        super().__init__('frenet_error_node')

        # Load raceline
        raceline_path = '/home/can-02/Desktop/f1tenth_ws/src/adp_controller/waypoints/optimal_raceline.csv'
        self.raceline = np.loadtxt(raceline_path, delimiter=',',skiprows=1)
        self.xref = self.raceline[:, 0]
        self.yref = self.raceline[:, 1]
        self.psi_ref = self.raceline[:, 2]
        self.kappa_ref = self.raceline[:, 3]

        # Publishers
        self.pub_frenet = self.create_publisher(Float32MultiArray, '/frenet_errors', 10)
        self.pub_marker = self.create_publisher(Marker, '/target_point_marker', 10)

        # Subscriber
        self.create_subscription(Odometry, '/ego_racecar/odom', self.odom_callback, 10)

        self.get_logger().info("Frenet Error Node Initialized")

    def odom_callback(self, msg):
        # Get pose
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y

        # Get yaw
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        psi = np.arctan2(siny_cosp, cosy_cosp)

        # Nearest raceline point
        dists = np.hypot(self.xref - x, self.yref - y)
        idx = int(np.argmin(dists))

        target_x = self.xref[idx]
        target_y = self.yref[idx]
        psi_des = self.psi_ref[idx]
        kappa = self.kappa_ref[idx]

        # Frenet Errors
        dx = x - target_x
        dy = y - target_y
        ey = -np.sin(psi_des) * dx + np.cos(psi_des) * dy
        epsi = np.arctan2(np.sin(psi - psi_des), np.cos(psi - psi_des))
        dot_ey = vy * np.cos(psi_des) - vx * np.sin(psi_des)
        dot_epsi = 0.0  # optional

        # Publish error
        msg_out = Float32MultiArray()
        msg_out.data = [ey, dot_ey, epsi, dot_epsi, kappa]
        self.pub_frenet.publish(msg_out)

        # Publish Marker
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "target"
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = float(target_x)
        marker.pose.position.y = float(target_y)
        marker.pose.position.z = 0.1
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.3
        marker.scale.y = 0.3
        marker.scale.z = 0.3
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0

        self.pub_marker.publish(marker)
        self.get_logger().info(f"[MARKER] Published target point at ({target_x:.2f}, {target_y:.2f})")


def main(args=None):
    rclpy.init(args=args)
    node = FrenetErrorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

