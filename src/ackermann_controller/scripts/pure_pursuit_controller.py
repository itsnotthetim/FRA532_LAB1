#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool  # Import Bool message type
import math
import yaml
import os
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from std_msgs.msg import Bool

class PurePursuit(Node):
    def __init__(self):
        super().__init__('pure_pursuit')
        
        # ROS2 Publishers & Subscribers
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.subscription = self.create_subscription(Odometry, '/double_track/odom', self.odom_callback, 10)
        self.timer = self.create_timer(0.01, self.pure_pursuit_control)  # Run control loop at 100 Hz
        self.reached_goal_pub = self.create_publisher(Bool, "/reached_goal", 10)

        # Add a publisher for is_finished
        self.is_finished_pub = self.create_publisher(Bool, 'is_finished', 10)

        # Load Waypoints from YAML File
        package_name = "ackermann_controller"  # Your package name
        installed_package_path = Path(get_package_share_directory(package_name))
        workspace_path = installed_package_path.parents[3]  # Move up to workspace root
        package_path = workspace_path / "src" / package_name  # Construct the src path
        self.path = os.path.join(package_path, "yaml/path.yaml")
        self.waypoints = self.load_waypoints(self.path)

        # Vehicle Parameters
        self.wheelbase = 0.2  # Distance between front and rear wheels (meters)
        self.lookahead_distance = 0.3  # Adjust for smooth tracking

        # Control Variables
        self.current_index = 0
        self.position = (0.0, 0.0)
        self.yaw = 0.0
        self.reached_goal = False  # Flag to indicate completion

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

        # Convert quaternion to yaw
        q = msg.pose.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        self.yaw = math.atan2(siny_cosp, cosy_cosp)

    def find_lookahead_point(self):
        """Finds the next waypoint that is at least lookahead_distance ahead."""
        if not self.waypoints:
            self.get_logger().warn("No waypoints loaded!")
            return None

        for i in range(self.current_index, len(self.waypoints)):
            dx = self.waypoints[i][0] - self.position[0]
            dy = self.waypoints[i][1] - self.position[1]
            distance = math.sqrt(dx ** 2 + dy ** 2)

            if distance >= self.lookahead_distance:
                self.current_index = i
                return self.waypoints[i]

        return self.waypoints[-1]  # If no far enough point is found, return the last waypoint

    def pure_pursuit_control(self):
        """Computes steering control using Pure Pursuit for Ackermann model."""
        if self.reached_goal:
            return  # Stop execution once goal is reached

        lookahead = self.find_lookahead_point()
        if lookahead is None:
            return  # No valid waypoint found

        goal_x, goal_y = lookahead
        dx = goal_x - self.position[0]
        dy = goal_y - self.position[1]
        target_angle = math.atan2(dy, dx)

        # Compute heading error (alpha)
        alpha = target_angle - self.yaw

        # Compute Steering Angle (Ackermann)
        delta = math.atan2(2 * self.wheelbase * math.sin(alpha), self.lookahead_distance)

        # Convert Steering Angle to Angular Velocity (for `/cmd_vel`)
        v = 0.5  # Constant speed (can be adjusted)
        w = (2 * v * math.sin(delta)) / self.wheelbase  # Approximate angular velocity

        # Check if the robot has reached the last waypoint
        if self.current_index >= len(self.waypoints) - 1:
            final_dx = self.waypoints[-1][0] - self.position[0]
            final_dy = self.waypoints[-1][1] - self.position[1]
            final_distance = math.sqrt(final_dx ** 2 + final_dy ** 2)

            if final_distance < 0.1:  # Threshold to determine if the goal is reached
                self.reached_goal_pub.publish(Bool(data=True))
                self.get_logger().info("Final waypoint reached. Stopping the robot.")
                self.stop_robot()
                self.shutdown_node()
                return

        # Publish Velocity Command
        msg = Twist()
        msg.linear.x = v
        msg.angular.z = w
        self.publisher.publish(msg)
        self.reached_goal_pub.publish(Bool(data=False))

        # Debugging Log
        self.get_logger().info(f'Index: {self.current_index}, Position: {self.position}, Goal: ({goal_x}, {goal_y})')
        self.get_logger().info(f'alpha: {alpha:.3f}, delta: {delta:.3f}, v: {v:.3f}, w: {w:.3f}')

    def stop_robot(self):
        """Stops the robot by publishing zero velocity."""
        stop_msg = Twist()
        stop_msg.linear.x = 0.0
        stop_msg.angular.z = 0.0
        self.publisher.publish(stop_msg)

    def shutdown_node(self):    
        """Shuts down the node safely."""
        # Publish is_finished = True
        is_finished_msg = Bool()
        is_finished_msg.data = True
        self.is_finished_pub.publish(is_finished_msg)

        self.reached_goal = True  # Mark that we have reached the goal
        self.get_logger().info("Shutting down Pure Pursuit node.")
        self.destroy_node()
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = PurePursuit()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()