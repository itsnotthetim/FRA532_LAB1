import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
import math
import numpy as np
import yaml
import os
from pathlib import Path
from ament_index_python.packages import get_package_share_directory

class PurePursuit(Node):
    def __init__(self):
        super().__init__('pure_pursuit')
        
        # ROS2 Publishers & Subscribers
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.subscription = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.timer = self.create_timer(0.1, self.pure_pursuit_control)

        # Load Waypoints from YAML File

        package_name = "ackermann_controller"  # Change to your package name
        # Get the installed package path
        installed_package_path = Path(get_package_share_directory(package_name))

        # Traverse up to find the workspace root (assumes standard ROS2 structure)
        workspace_path = installed_package_path.parents[3]  # Moves up to user_path/FRA532_LAB1`
        package_path = workspace_path / "src" / package_name  # Construct the src path
        self.path = os.path.join(package_path, "yaml/path.yaml")
        self.waypoints = self.load_waypoints(self.path)

        # Control Variables
        self.current_index = 0
        self.lookahead_distance = 0.3  # Adjust based on speed and turning ability
        self.position = (0.0, 0.0)

        self.yaw = 0.0

    def load_waypoints(self, file_path):
        """Load waypoints from YAML file."""
        with open(file_path, "r") as file:
            path_data = yaml.safe_load(file)

        waypoints = [(wp["x"], wp["y"]) for wp in path_data]  # Extract (x, y) coordinates
        self.get_logger().info(f"Loaded {len(waypoints)} waypoints from {file_path}")
        return waypoints


    def odom_callback(self, msg):
        """Extract position and yaw from odometry data."""
        self.position = (msg.pose.pose.position.x, msg.pose.pose.position.y)
        orientation_q = msg.pose.pose.orientation
        siny_cosp = 2 * (orientation_q.w * orientation_q.z + orientation_q.x * orientation_q.y)
        cosy_cosp = 1 - 2 * (orientation_q.y * orientation_q.y + orientation_q.z * orientation_q.z)
        self.yaw = math.atan2(siny_cosp, cosy_cosp)

    def pure_pursuit_control(self):
        """Compute control commands using the Pure Pursuit algorithm."""
        if not self.waypoints:
            self.get_logger().warn("No waypoints loaded. Cannot compute Pure Pursuit.")
            return
        
        # Find the next waypoint ahead of the robot
        goal_x, goal_y = self.waypoints[self.current_index]
        dx = goal_x - self.position[0]
        dy = goal_y - self.position[1]
        distance = math.sqrt(dx ** 2 + dy ** 2)

        # Find a waypoint that is at least lookahead_distance away
        while distance < self.lookahead_distance:
            self.current_index += 1
            if self.current_index >= len(self.waypoints):
                self.current_index = 0  # Loop back to the start of the waypoints
            goal_x, goal_y = self.waypoints[self.current_index]
            dx = goal_x - self.position[0]
            dy = goal_y - self.position[1]
            distance = math.sqrt(dx ** 2 + dy ** 2)

        # Compute control commands
        target_angle = math.atan2(dy, dx)
        alpha = target_angle - self.yaw

        # Compute steering command using bicycle model
        Ld = self.lookahead_distance
        wheelbase = 0.2  # Distance between front and rear wheels
        beta = math.atan2(2 * wheelbase * math.sin(alpha), Ld)
        
        v = 1.0  # Constant velocity (can be tuned)
        w = 2 * v * math.sin(beta) / Ld  # Angular velocity (steering control)

        # Publish velocity command
        msg = Twist()
        msg.linear.x = v
        msg.angular.z = w
        self.publisher.publish(msg)

        self.get_logger().info(f'Current Waypoint: {self.current_index}, Position: {self.position}, Goal: ({goal_x}, {goal_y})')
        self.get_logger().info(f'alpha: {alpha:.3f}, beta: {beta:.3f}, Ld: {Ld:.3f}, v: {v:.3f}, w: {w:.3f}')

def main(args=None):
    rclpy.init(args=args)
    node = PurePursuit()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
