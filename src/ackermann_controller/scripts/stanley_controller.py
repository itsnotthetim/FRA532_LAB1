import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math
import yaml
import os
from pathlib import Path
from ament_index_python.packages import get_package_share_directory

class StanleyController(Node):
    def __init__(self):
        super().__init__('stanley_controller')
        
        # ROS2 Publishers & Subscribers
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.subscription = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.timer = self.create_timer(0.01, self.stanley_control)  # Run control loop at 100 Hz

        # Load Waypoints from YAML File
        package_name = "ackermann_controller"  # Your package name
        installed_package_path = Path(get_package_share_directory(package_name))
        workspace_path = installed_package_path.parents[3]  # Move up to workspace root
        package_path = workspace_path / "src" / package_name  # Construct the src path
        self.path = os.path.join(package_path, "yaml/path.yaml")
        self.waypoints = self.load_waypoints(self.path)

        # Vehicle Parameters
        self.wheelbase = 0.2  # Distance between front and rear wheels (meters)
        self.k = 1.0  # Gain for Stanley controller
        self.velocity = 0.5  # Constant forward velocity

        # Control Variables
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

    def find_nearest_waypoint(self):
        """Find the nearest waypoint to the front axle."""
        if not self.waypoints:
            self.get_logger().warn("No waypoints loaded!")
            return None

        min_distance = float('inf')
        nearest_point = None
        for wp in self.waypoints:
            dx = wp[0] - self.position[0]
            dy = wp[1] - self.position[1]
            distance = math.sqrt(dx ** 2 + dy ** 2)
            if distance < min_distance:
                min_distance = distance
                nearest_point = wp

        return nearest_point

    def stanley_control(self):
        """Computes steering control using the Stanley Controller."""
        if self.reached_goal:
            return  # Stop execution once goal is reached

        waypoint = self.find_nearest_waypoint()
        if waypoint is None:
            return  # No valid waypoint found

        goal_x, goal_y = waypoint
        dx = goal_x - self.position[0]
        dy = goal_y - self.position[1]
        path_heading = math.atan2(dy, dx)

        # Compute heading error
        heading_error = path_heading - self.yaw
        heading_error = math.atan2(math.sin(heading_error), math.cos(heading_error))  # Normalize

        # Compute cross-track error
        cross_track_error = math.sqrt(dx ** 2 + dy ** 2)
        sign = -1 if math.sin(self.yaw) * dx - math.cos(self.yaw) * dy > 0 else 1  # Determine side of path
        cross_track_error *= sign

        # Compute Stanley control law
        steering_angle = heading_error + math.atan2(self.k * cross_track_error, self.velocity)
        steering_angle = max(min(steering_angle, math.pi / 4), -math.pi / 4)  # Limit steering angle

        # Convert Steering Angle to Angular Velocity (for `/cmd_vel`)
        angular_velocity = (2 * self.velocity * math.sin(steering_angle)) / self.wheelbase

        # Check if the robot has reached the final waypoint
        if len(self.waypoints) > 0 and waypoint == self.waypoints[-1]:
            final_dx = self.waypoints[-1][0] - self.position[0]
            final_dy = self.waypoints[-1][1] - self.position[1]
            final_distance = math.sqrt(final_dx ** 2 + final_dy ** 2)

            if final_distance < 0.1:  # Threshold to determine if the goal is reached
                self.get_logger().info("Final waypoint reached. Stopping the robot.")
                self.stop_robot()
                self.shutdown_node()
                return

        # Publish Velocity Command
        msg = Twist()
        msg.linear.x = self.velocity
        msg.angular.z = angular_velocity
        self.publisher.publish(msg)

        # Debugging Log
        self.get_logger().info(f'Waypoint: ({goal_x}, {goal_y}), Heading Error: {heading_error:.3f}, Cross-Track Error: {cross_track_error:.3f}')
        self.get_logger().info(f'Steering Angle: {steering_angle:.3f}, Velocity: {self.velocity:.3f}, Angular Velocity: {angular_velocity:.3f}')

    def stop_robot(self):
        """Stops the robot by publishing zero velocity."""
        stop_msg = Twist()
        stop_msg.linear.x = 0.0
        stop_msg.angular.z = 0.0
        self.publisher.publish(stop_msg)

    def shutdown_node(self):
        """Shuts down the node safely."""
        self.reached_goal = True  # Mark that we have reached the goal
        self.get_logger().info("Shutting down Stanley Controller node.")
        self.destroy_node()
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = StanleyController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
