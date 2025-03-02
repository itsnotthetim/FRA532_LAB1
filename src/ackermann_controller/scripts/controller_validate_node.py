#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import matplotlib.pyplot as plt
import yaml
from ament_index_python.packages import get_package_share_directory

class KinematicValidateNode(Node):
    def __init__(self):
        super().__init__('kinematic_validate_node')

        # Create subscription for ground truth odometry
        self.create_subscription(Odometry, '/ground_truth/odom', self.ground_truth_callback, 10)

        # List to store ground truth positions ([x, y])
        self.ground_truth_odom_list = []

        # Load model path data from YAML file
        try:
            with open(get_package_share_directory("ackermann_controller")+'/yaml/path.yaml', 'r') as f:
                data = yaml.safe_load(f)
            # Assuming the YAML file is a list of dictionaries with keys: x, y, yaw.
            # Extract only the x and y values.
            self.model_path = [[entry['x'], entry['y']] for entry in data if 'x' in entry and 'y' in entry]
        except Exception as e:
            self.get_logger().error(f"Failed to load YAML file: {e}")
            self.model_path = []

        # Set up Matplotlib for interactive plotting
        plt.ion()
        self.fig, self.ax = plt.subplots(figsize=(7, 5))

        controller_name = 'Pure Pursuit'
        # controller_name = 'Stanley'
        # controller_name = 'PID'

        # Create line objects for ground truth and model path data
        # self.gt_line, = self.ax.plot([], [], 'bo-', label=controller_name, markersize=1)
        self.gt_line, = self.ax.plot([], [], 'bo-', label=controller_name, markersize=1)
        # self.gt_line, = self.ax.plot([], [], 'bo-', label=controller_name, markersize=1)
        self.model_line, = self.ax.plot([], [], 'r--', label='Path', markersize=1)

        # Configure plot appearance
        self.ax.set_title(controller_name+" vs. Path")
        self.ax.set_xlabel("X Value")
        self.ax.set_ylabel("Y Value")
        self.ax.legend()

    def ground_truth_callback(self, msg: Odometry):
        # Append new ground truth point from the odometry message
        self.ground_truth_odom_list.append([msg.pose.pose.position.x, msg.pose.pose.position.y])
        self.plot_graph()
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()

    def plot_graph(self):
        # Update ground truth data line
        if self.ground_truth_odom_list:
            gt_x = [pt[0] for pt in self.ground_truth_odom_list]
            gt_y = [pt[1] for pt in self.ground_truth_odom_list]
            self.gt_line.set_data(gt_x, gt_y)

        # Update model path data line using only x and y values
        if self.model_path:
            model_x = [pt[0] for pt in self.model_path]
            model_y = [pt[1] for pt in self.model_path]
            self.model_line.set_data(model_x, model_y)

        # Adjust plot limits to include both datasets
        all_x = []
        all_y = []
        if self.ground_truth_odom_list:
            all_x.extend([pt[0] for pt in self.ground_truth_odom_list])
            all_y.extend([pt[1] for pt in self.ground_truth_odom_list])
        if self.model_path:
            all_x.extend(model_x)
            all_y.extend(model_y)
        if all_x and all_y:
            self.ax.set_xlim(min(all_x) - 1, max(all_x) + 1)
            self.ax.set_ylim(min(all_y) - 1, max(all_y) + 1)

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
