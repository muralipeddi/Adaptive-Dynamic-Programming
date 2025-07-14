#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
import numpy as np

class RacelinePublisher(Node):
    def __init__(self):
        super().__init__('raceline_publisher')

        # === Absolute path to your CSV file ===
        raceline_path = '/home/can-02/Desktop/f1tenth_ws/src/adp_controller/waypoints/optimal_raceline.csv'

        try:
            self.raceline = np.loadtxt(raceline_path, delimiter=',', skiprows=1)
        except Exception as e:
            self.get_logger().error(f"Failed to load raceline CSV: {e}")
            return

        self.path_pub = self.create_publisher(Path, '/raceline_path', 10)
        self.timer = self.create_timer(1.0, self.publish_raceline)

        self.get_logger().info("Raceline loaded and publishing to /raceline_path")

    def publish_raceline(self):
        path_msg = Path()
        path_msg.header.frame_id = 'map'
        path_msg.header.stamp = self.get_clock().now().to_msg()

        for pt in self.raceline:
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.pose.position.x = float(pt[0])
            pose.pose.position.y = float(pt[1])
            pose.pose.position.z = 0.0
            pose.pose.orientation.w = 1.0
            path_msg.poses.append(pose)

        self.path_pub.publish(path_msg)

def main(args=None):
    rclpy.init(args=args)
    node = RacelinePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

