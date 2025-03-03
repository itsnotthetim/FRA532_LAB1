#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool  # Import Bool message type
import matplotlib.pyplot as plt
import yaml
import os
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
import numpy as np

class KinematicValidateNode(Node):
    def __init__(self):
        super().__init__('kinematic_validate_node')

        # Create subscriptions for GPS, EKF Odometry, Ground Truth, Single Track Model Odometry, and is_finished
        self.create_subscription(PoseStamped, '/gps', self.gps_callback, 10)
        self.create_subscription(Odometry, '/ekf_odom', self.ekf_callback, 10)
        self.create_subscription(Odometry, '/ground_truth/odom', self.odom_callback, 10)
        self.create_subscription(Odometry, '/double_track/odom', self.single_track_callback, 10)
        self.create_subscription(Bool, '/is_finished', self.is_finished_callback, 10)

        # Load Waypoints from YAML File
        package_name = "ackermann_controller"  # Your package name
        installed_package_path = Path(get_package_share_directory(package_name))
        workspace_path = installed_package_path.parents[3]  # Move up to workspace root
        self.package_path = workspace_path / "src" / package_name  # Construct the src path
        self.path = os.path.join(self.package_path, "yaml/path.yaml")
        self.data = self.load_waypoints(self.path)

        # Lists to store trajectory and yaw data
        self.gps_data = []
        self.ekf_data = []
        self.odom_data = []
        self.single_track_data = []
        self.ekf_yaw_data = []
        self.odom_yaw_data = []
        self.single_track_yaw_data = []

        # Variables to track errors
        self.cumulative_position_error = 0.0
        self.cumulative_yaw_error = 0.0
        self.num_samples = 0

        # Variable to track the previous state of is_finished
        self.previous_is_finished = False

        # Set up Matplotlib for interactive plotting
        plt.ion()
        self.fig, (self.ax_traj, self.ax_yaw) = plt.subplots(1, 2, figsize=(14, 5))
        self.gps_line, = self.ax_traj.plot([], [], 'ro', label='GPS', markersize=0.7)
        self.ekf_line, = self.ax_traj.plot([], [], 'b-', label='EKF Estimate')
        self.odom_line, = self.ax_traj.plot([], [], 'g--', label='Wheel Odometry')
        self.single_track_line, = self.ax_traj.plot([], [], 'c-.', label='Double track Model')
        self.ax_traj.set_title("EKF: Double track Model Validation")
        self.ax_traj.set_xlabel("X Position (m)")
        self.ax_traj.set_ylabel("Y Position (m)")
        self.ax_traj.legend()

        self.ekf_yaw_line, = self.ax_yaw.plot([], [], 'b-', label='EKF Yaw')
        self.odom_yaw_line, = self.ax_yaw.plot([], [], 'g--', label='Wheel Odometry Yaw')
        self.single_track_yaw_line, = self.ax_yaw.plot([], [], 'c-.', label='Double track Yaw')
        self.ax_yaw.set_title("Yaw Data Comparison")
        self.ax_yaw.set_xlabel("Time Step")
        self.ax_yaw.set_ylabel("Yaw (rad)")
        self.ax_yaw.legend()

        # Use a timer to update the plot on the main thread
        self.timer = self.create_timer(0.1, self.plot_data)

    def load_waypoints(self, file_path):
        """Load waypoints from YAML file."""
        with open(file_path, "r") as file:
            path_data = yaml.safe_load(file)
        waypoints = [(wp["x"], wp["y"], wp["yaw"]) for wp in path_data]  # Extract (x, y, yaw) coordinates
        self.get_logger().info(f"Loaded {len(waypoints)} waypoints from {file_path}")
        return waypoints

    def extract_yaw(self, msg):
        """Extract yaw angle from quaternion."""
        q = msg.pose.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        return np.arctan2(siny_cosp, cosy_cosp)

    def gps_callback(self, msg):
        self.gps_data.append([msg.pose.position.x, msg.pose.position.y])

    def ekf_callback(self, msg):
        self.ekf_data.append([msg.pose.pose.position.x, msg.pose.pose.position.y])
        self.ekf_yaw_data.append(self.extract_yaw(msg))

    def odom_callback(self, msg):
        self.odom_data.append([msg.pose.pose.position.x, msg.pose.pose.position.y])
        self.odom_yaw_data.append(self.extract_yaw(msg))

        # Calculate position error (Euclidean distance between EKF and ground truth)
        if self.ekf_data and self.odom_data:
            ekf_pos = np.array(self.ekf_data[-1])
            odom_pos = np.array(self.odom_data[-1])
            position_error = np.linalg.norm(ekf_pos - odom_pos)
            self.cumulative_position_error += position_error

            # Calculate yaw error (absolute difference between EKF and ground truth)
            ekf_yaw = self.ekf_yaw_data[-1]
            odom_yaw = self.odom_yaw_data[-1]
            yaw_error = abs(ekf_yaw - odom_yaw)
            self.cumulative_yaw_error += yaw_error

            # Increment the number of samples
            self.num_samples += 1

            # Log mean errors
            if self.num_samples > 0:
                mean_position_error = self.cumulative_position_error / self.num_samples
                mean_yaw_error = self.cumulative_yaw_error / self.num_samples
                self.get_logger().info(f"Mean Position Error: {mean_position_error:.4f} m")
                self.get_logger().info(f"Mean Yaw Error: {mean_yaw_error:.4f} rad")

    def single_track_callback(self, msg):
        self.single_track_data.append([msg.pose.pose.position.x, msg.pose.pose.position.y])
        self.single_track_yaw_data.append(self.extract_yaw(msg))

    def is_finished_callback(self, msg):
        """Callback for the /is_finished topic."""
        if not self.previous_is_finished and msg.data:  # Check for False -> True transition
            self.get_logger().info("Received is_finished = True. Saving figure and shutting down...")
            self.save_figure()  # Save the figure before shutting down
            self.shutdown_node()
        self.previous_is_finished = msg.data  # Update the previous state

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
        if self.single_track_data:
            single_x, single_y = zip(*self.single_track_data)
            self.single_track_line.set_data(single_x, single_y)

        if self.ekf_yaw_data:
            self.ekf_yaw_line.set_data(range(len(self.ekf_yaw_data)), self.ekf_yaw_data)
        if self.odom_yaw_data:
            self.odom_yaw_line.set_data(range(len(self.odom_yaw_data)), self.odom_yaw_data)
        if self.single_track_yaw_data:
            self.single_track_yaw_line.set_data(range(len(self.single_track_yaw_data)), self.single_track_yaw_data)

        self.ax_traj.relim()
        self.ax_traj.autoscale_view()
        self.ax_yaw.relim()
        self.ax_yaw.autoscale_view()
        self.ax_traj.legend()
        self.ax_yaw.legend()
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()

    def save_figure(self):
        """Save the figure to the workspace."""
        # Construct the path to save the figure in the workspace
        figure_path = self.package_path / "figures"
        figure_path.mkdir(exist_ok=True)  # Create the directory if it doesn't exist

        # Save the figure as a PNG file
        figure_file = figure_path / "ekf_validation_figure.png"
        self.fig.savefig(figure_file, dpi=300, bbox_inches='tight')
        self.get_logger().info(f"Figure saved to {figure_file}")

    def shutdown_node(self):
        """Shutdown the node."""
        self.get_logger().info("Shutting down node...")
        self.destroy_node()
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)  # Initialize the ROS 2 Python client library
    node = KinematicValidateNode()  # Create an instance of the KinematicValidateNode

    try:
        rclpy.spin(node)  # Keep the node running and processing callbacks
    except KeyboardInterrupt:
        node.get_logger().info("Node interrupted by user.")  # Handle user interruption (e.g., Ctrl+C)
    finally:
        if rclpy.ok():  # Check if ROS 2 is still running
            node.destroy_node()  # Clean up the node
        rclpy.shutdown()  # Shut down the ROS 2 Python client library

if __name__ == '__main__':
    main()