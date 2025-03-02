#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
import matplotlib.pyplot as plt
import yaml
from ament_index_python.packages import get_package_share_directory

class KinematicValidateNode(Node):
    def __init__(self):
        super().__init__('kinematic_validate_node')

        # Create subscriptions for GPS, EKF Odometry, and Wheel Odometry
        self.create_subscription(PoseStamped, '/gps', self.gps_callback, 10)
        self.create_subscription(Odometry, '/ekf_odom', self.ekf_callback, 10)
        self.create_subscription(Odometry, '/ground_truth/odom', self.odom_callback, 10)

        # Lists to store trajectory points
        self.gps_data = []
        self.ekf_data = []
        self.odom_data = []


        # Set up Matplotlib for interactive plotting
        plt.ion()
        self.fig, self.ax = plt.subplots(figsize=(7, 5))
        self.gps_line, = self.ax.plot([], [], 'ro', label='GPS', markersize=0.7)
        self.ekf_line, = self.ax.plot([], [], 'b-', label='EKF Estimate')
        self.odom_line, = self.ax.plot([], [], 'g--', label='Wheel Odometry')
        self.ax.set_title("Localization: GPS vs EKF vs Wheel Odometry")
        self.ax.set_xlabel("X Position (m)")
        self.ax.set_ylabel("Y Position (m)")
        self.ax.legend()
        
        # Use a timer to update the plot on the main thread
        self.timer = self.create_timer(0.1, self.plot_data)

    def gps_callback(self, msg):
        self.gps_data.append([msg.pose.position.x, msg.pose.position.y])

    def ekf_callback(self, msg):
        self.ekf_data.append([msg.pose.pose.position.x, msg.pose.pose.position.y])

    def odom_callback(self, msg):
        self.odom_data.append([msg.pose.pose.position.x, msg.pose.pose.position.y])

    def plot_data(self):
        if self.gps_data:
            gps_x, gps_y = zip(*self.gps_data)
            self.gps_line.set_data(gps_x, gps_y)

        if self.ekf_data:
            ekf_x, ekf_y = zip(*self.ekf_data)
            self.ekf_line.set_data(ekf_x, ekf_y)

        if self.odom_data:
            odom_x, odom_y = zip(*self.odom_data)
            self.odom_line.set_data(odom_x, odom_y)

        all_x = [pt[0] for pt in self.gps_data + self.ekf_data + self.odom_data ]
        all_y = [pt[1] for pt in self.gps_data + self.ekf_data + self.odom_data ]
        if all_x and all_y:
            self.ax.set_xlim(min(all_x) - 1, max(all_x) + 1)
            self.ax.set_ylim(min(all_y) - 1, max(all_y) + 1)
        
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()


def main(args=None):
    rclpy.init(args=args)
    node = KinematicValidateNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
