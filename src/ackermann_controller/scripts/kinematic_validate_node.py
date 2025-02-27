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

        # Simple odom container
        class Odom:
            def __init__(self):
                self.position = [0, 0, 0]  # x, y, z
                self.orientation = [0, 0, 0, 0]  # x, y, z, w

        self.ground_truth_odom = Odom()
        self.single_track_odom = Odom()
        self.double_track_odom = Odom()
        self.yaw_rate_odom = Odom()

        # Lists to store positions ([x, y])
        self.ground_truth_odom_list = []
        self.single_track_odom_list = []
        self.double_track_odom_list = []
        self.yaw_rate_odom_list = []

        # Set up Matplotlib for interactive plotting
        plt.ion()
        self.fig, self.ax = plt.subplots()
        self.line1, = self.ax.plot([], [], 'bo-', label='Ground Truth')
        self.line2, = self.ax.plot([], [], 'ro-', label='Single Track')
        self.ax.set_xlabel("X Value")
        self.ax.set_ylabel("Y Value")
        self.ax.legend()

    def ground_truth_callback(self, msg: Odometry):
        # Update ground truth odometry data
        self.ground_truth_odom.position[0] = msg.pose.pose.position.x
        self.ground_truth_odom.position[1] = msg.pose.pose.position.y
        self.ground_truth_odom.position[2] = msg.pose.pose.position.z

        self.ground_truth_odom.orientation[0] = msg.pose.pose.orientation.x
        self.ground_truth_odom.orientation[1] = msg.pose.pose.orientation.y
        self.ground_truth_odom.orientation[2] = msg.pose.pose.orientation.z
        self.ground_truth_odom.orientation[3] = msg.pose.pose.orientation.w

        # Append [x, y] to the ground truth list
        self.ground_truth_odom_list.append([self.ground_truth_odom.position[0],
                                              self.ground_truth_odom.position[1]])
        self.plot_graph()

    def single_track_callback(self, msg: Odometry):
        # Update single track odometry data
        self.single_track_odom.position[0] = msg.pose.pose.position.x
        self.single_track_odom.position[1] = msg.pose.pose.position.y
        self.single_track_odom.position[2] = msg.pose.pose.position.z

        self.single_track_odom.orientation[0] = msg.pose.pose.orientation.x
        self.single_track_odom.orientation[1] = msg.pose.pose.orientation.y
        self.single_track_odom.orientation[2] = msg.pose.pose.orientation.z
        self.single_track_odom.orientation[3] = msg.pose.pose.orientation.w

        # Append [x, y] to the single track list
        self.single_track_odom_list.append([self.single_track_odom.position[0],
                                              self.single_track_odom.position[1]])
        self.plot_graph()

    def double_track_callback(self, msg: Odometry):
        # Update double track odometry data
        self.double_track_odom.position[0] = msg.pose.pose.position.x
        self.double_track_odom.position[1] = msg.pose.pose.position.y
        self.double_track_odom.position[2] = msg.pose.pose.position.z

        self.double_track_odom.orientation[0] = msg.pose.pose.orientation.x
        self.double_track_odom.orientation[1] = msg.pose.pose.orientation.y
        self.double_track_odom.orientation[2] = msg.pose.pose.orientation.z
        self.double_track_odom.orientation[3] = msg.pose.pose.orientation.w

        self.double_track_odom_list.append([self.double_track_odom.position[0],
                                              self.double_track_odom.position[1]])
        # Not calling plot_graph here; add if needed

    def yaw_rate_callback(self, msg: Odometry):
        # Update yaw rate odometry data
        self.yaw_rate_odom.position[0] = msg.pose.pose.position.x
        self.yaw_rate_odom.position[1] = msg.pose.pose.position.y
        self.yaw_rate_odom.position[2] = msg.pose.pose.position.z

        self.yaw_rate_odom.orientation[0] = msg.pose.pose.orientation.x
        self.yaw_rate_odom.orientation[1] = msg.pose.pose.orientation.y
        self.yaw_rate_odom.orientation[2] = msg.pose.pose.orientation.z
        self.yaw_rate_odom.orientation[3] = msg.pose.pose.orientation.w

        self.yaw_rate_odom_list.append([self.yaw_rate_odom.position[0],
                                          self.yaw_rate_odom.position[1]])
        # Not calling plot_graph here; add if needed

    def plot_graph(self):
        # Extract x and y values for ground truth
        if self.ground_truth_odom_list:
            gt_x = [p[0] for p in self.ground_truth_odom_list]
            gt_y = [p[1] for p in self.ground_truth_odom_list]
            self.line1.set_data(gt_x, gt_y)

        # Extract x and y values for single track
        if self.single_track_odom_list:
            st_x = [p[0] for p in self.single_track_odom_list]
            st_y = [p[1] for p in self.single_track_odom_list]
            self.line2.set_data(st_x, st_y)

        # Rescale axes and update the figure
        self.ax.relim()
        self.ax.autoscale_view(True, True, True)
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
