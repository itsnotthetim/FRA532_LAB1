#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import matplotlib.pyplot as plt

class KinematicValidateNode(Node):
    def __init__(self):
        super().__init__('kinematic_validate_node')

        # Create subscriptions
        self.create_subscription(Odometry, '/ground_truth/odom', self.ground_truth_callback, 10)
        self.create_subscription(Odometry, '/single_track/odom', self.single_track_callback, 10)
        self.create_subscription(Odometry, '/double_track/odom', self.double_track_callback, 10)
        self.create_subscription(Odometry, '/yaw_rate/odom', self.yaw_rate_callback, 10)

        # Lists to store positions ([x, y])
        self.ground_truth_odom_list = []
        self.single_track_odom_list = []
        self.double_track_odom_list = []
        self.yaw_rate_odom_list = []

        # Set up Matplotlib for interactive plotting
        plt.ion()
        self.fig, self.axs = plt.subplots(1, 3, figsize=(15, 5))  # 1 row, 3 columns

        # Subplots: each will compare ground truth with one model
        self.lines = {
            "gt_st": self.axs[0].plot([], [], 'bo-', label='Ground Truth', markersize=1)[0],  # Ground Truth vs Single Track
            "st": self.axs[0].plot([], [], 'ro-', label='Single Track', markersize=1)[0],

            "gt_dt": self.axs[1].plot([], [], 'bo-', label='Ground Truth', markersize=1)[0],  # Ground Truth vs Double Track
            "dt": self.axs[1].plot([], [], 'ro-', label='Double Track', markersize=1)[0],

            "gt_yr": self.axs[2].plot([], [], 'bo-', label='Ground Truth', markersize=1)[0],  # Ground Truth vs Yaw Rate
            "yr": self.axs[2].plot([], [], 'ro-', label='Yaw Rate', markersize=1)[0],
        }

        # Set titles for the subplots
        self.axs[0].set_title("Ground Truth vs. Single Track")
        self.axs[1].set_title("Ground Truth vs. Double Track")
        self.axs[2].set_title("Ground Truth vs. Yaw Rate")

        for ax in self.axs:
            ax.set_xlabel("X Value")
            ax.set_ylabel("Y Value")
            ax.legend()

    def ground_truth_callback(self, msg: Odometry):
        self.ground_truth_odom_list.append([msg.pose.pose.position.x, msg.pose.pose.position.y])
        self.plot_graph()

    def single_track_callback(self, msg: Odometry):
        self.single_track_odom_list.append([msg.pose.pose.position.x, msg.pose.pose.position.y])
        self.plot_graph()

    def double_track_callback(self, msg: Odometry):
        self.double_track_odom_list.append([msg.pose.pose.position.x, msg.pose.pose.position.y])
        self.plot_graph()

    def yaw_rate_callback(self, msg: Odometry):
        self.yaw_rate_odom_list.append([msg.pose.pose.position.x, msg.pose.pose.position.y])
        self.plot_graph()

    def plot_graph(self):
        # Ground Truth data
        if self.ground_truth_odom_list:
            gt_x = [p[0] for p in self.ground_truth_odom_list]
            gt_y = [p[1] for p in self.ground_truth_odom_list]
            self.lines["gt_st"].set_data(gt_x, gt_y)
            self.lines["gt_dt"].set_data(gt_x, gt_y)
            self.lines["gt_yr"].set_data(gt_x, gt_y)

        # Single Track data
        if self.single_track_odom_list:
            st_x = [p[0] for p in self.single_track_odom_list]
            st_y = [p[1] for p in self.single_track_odom_list]
            self.lines["st"].set_data(st_x, st_y)

        # Double Track data
        if self.double_track_odom_list:
            dt_x = [p[0] for p in self.double_track_odom_list]
            dt_y = [p[1] for p in self.double_track_odom_list]
            self.lines["dt"].set_data(dt_x, dt_y)

        # Yaw Rate data
        if self.yaw_rate_odom_list:
            yr_x = [p[0] for p in self.yaw_rate_odom_list]
            yr_y = [p[1] for p in self.yaw_rate_odom_list]
            self.lines["yr"].set_data(yr_x, yr_y)

        # Rescale axes and update the figure
        for ax in self.axs:
            ax.relim()
            ax.autoscale_view(True, True, True)

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
