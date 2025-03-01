#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import matplotlib.pyplot as plt
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from threading import Thread

class RealTimePlot(Node):
    def __init__(self):
        super().__init__('real_time_plot')

        # Subscribe to GPS, EKF Odometry, and Wheel Odometry
        self.gps_sub = self.create_subscription(PoseStamped, '/gps', self.gps_callback, 10)
        self.ekf_sub = self.create_subscription(Odometry, '/ekf_odom', self.ekf_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, 'ground_truth/odom', self.odom_callback, 10)

        # Lists to store trajectory points
        self.gps_x, self.gps_y = [], []
        self.ekf_x, self.ekf_y = [], []
        self.odom_x, self.odom_y = [], []

        # Start the plotting thread
        self.running = True
        self.plot_thread = Thread(target=self.plot_data)
        self.plot_thread.start()

    def gps_callback(self, msg):
        """Callback for GPS data"""
        self.gps_x.append(msg.pose.position.x)
        self.gps_y.append(msg.pose.position.y)

    def ekf_callback(self, msg):
        """Callback for EKF Odometry data"""
        self.ekf_x.append(msg.pose.pose.position.x)
        self.ekf_y.append(msg.pose.pose.position.y)

    def odom_callback(self, msg):
        """Callback for Wheel Odometry data"""
        self.odom_x.append(msg.pose.pose.position.x)
        self.odom_y.append(msg.pose.pose.position.y)

    def plot_data(self):
        """Function to plot data in real-time"""
        plt.ion()  # Interactive mode on
        fig, ax = plt.subplots()
        
        while self.running:
            ax.clear()
            ax.set_title("Real-Time Localization: GPS vs EKF vs Wheel Odometry")
            ax.set_xlabel("X Position (m)")
            ax.set_ylabel("Y Position (m)")
            ax.grid(True)

            # Plot GPS data
            if self.gps_x and self.gps_y:
                ax.scatter(self.gps_x, self.gps_y, color='red', label='GPS', s=10)

            # Plot EKF estimated data
            if self.ekf_x and self.ekf_y:
                ax.plot(self.ekf_x, self.ekf_y, color='blue', label='EKF Estimate')

            # Plot Wheel Odometry data
            if self.odom_x and self.odom_y:
                ax.plot(self.odom_x, self.odom_y, color='green', linestyle='dashed', label='Wheel Odometry')

            ax.legend()
            plt.pause(0.1)  # Small delay for real-time updates
        
        plt.ioff()  # Turn off interactive mode
        plt.show()

    def destroy_node(self):
        """Stop the plotting thread when shutting down"""
        self.running = False
        self.plot_thread.join()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = RealTimePlot()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
