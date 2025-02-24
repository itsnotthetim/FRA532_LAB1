#!/usr/bin/python3

import rclpy
from rclpy.node import Node, SetParametersResult
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from tf_transformations import euler_from_quaternion
import yaml
import numpy as np
from ament_index_python.packages import get_package_share_directory

class PID_controller:
    """
    PID Controller implementation for control systems.
    Implements a discrete-time PID controller with anti-windup protection.
    """
    def __init__(self, kp, ki, kd):
        # Controller gains
        self.kp = kp  # Proportional gain
        self.ki = ki  # Integral gain
        self.kd = kd  # Derivative gain
        
        # State variables
        self.y_n = 0.0    # Current output
        self.y_n_1 = 0.0  # Previous output
        self.e_n = 0.0    # Current error
        self.e_n_1 = 0.0  # Previous error
        self.e_n_2 = 0.0  # Error from two steps ago

    def update_controller(self, error, sat):
        """
        Updates PID controller output based on error input.
        
        Args:
            error: Current error (setpoint - measured_value)
            at: Output saturation limits (±sat)
        
        Returns:
            float: Controller output
        """
        e_n = error  # Current error sample

        # Anti-windup logic:
        # Only update controller if:
        # 1. Output is not saturated, or
        # 2. Error is trying to reduce the output
        if not ((self.y_n >= sat and e_n > 0) or (self.y_n <= -sat and e_n < 0)):
            # PID equation in discrete form:
            self.y_n += ((self.kp + self.ki + self.kd) * e_n) - ((self.kp + (2 * self.kd)) * self.e_n_1) + (self.kd * self.e_n_2)

        # Update error history for next iteration
        self.e_n_2 = self.e_n_1
        self.e_n_1 = e_n
        self.e_n = e_n
        
        # Saturate output to prevent excessive control signals
        if self.y_n > sat:
            self.y_n = sat
        elif self.y_n < -sat:
            self.y_n = -sat

        return self.y_n

class PIDControllerNode(Node):
    def __init__(self):
        super().__init__('pid_controller_node')
        
        # Declare parameters
        self.declare_parameter('rate', 100)
        self.declare_parameter('long_Kp', 0.05)
        self.declare_parameter('long_Ki', 0.0)
        self.declare_parameter('long_Kd', 0.0)
        self.declare_parameter('lat_Kp', 0.1)
        self.declare_parameter('lat_Ki', 0.0)
        self.declare_parameter('lat_Kd', 0.0)
        self.declare_parameter('ang_Kp', 0.5)
        self.declare_parameter('ang_Ki', 0.1)
        self.declare_parameter('ang_Kd', 0.0)

        # Rate
        self.rate = self.get_parameter('rate').value

        # PID parameters
        self.long_Kp = self.get_parameter('long_Kp').value
        self.long_Ki = self.get_parameter('long_Ki').value
        self.long_Kd = self.get_parameter('long_Kd').value
        self.lat_Kp = self.get_parameter('lat_Kp').value
        self.lat_Ki = self.get_parameter('lat_Ki').value
        self.lat_Kd = self.get_parameter('lat_Kd').value
        self.ang_Kp = self.get_parameter('ang_Kp').value
        self.ang_Ki = self.get_parameter('ang_Ki').value
        self.ang_Kd = self.get_parameter('ang_Kd').value

        # Parameters subscribers
        self.add_on_set_parameters_callback(self.parameter_callback)

        # Set points
        with open(get_package_share_directory("ackermann_controller")+'/yaml/path.yaml', 'r') as f:
            self.setpoints = yaml.safe_load(f)
        self.count_path = 0

        # Get odom
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        class odom:
            position = [0, 0, 0] # x, y, z
            orientation = [0, 0, 0, 0] # x, y, z, w
        self.odom = odom()

        # Create PID controllers
        self.long_pid = PID_controller(self.long_Kp, self.long_Ki, self.long_Kd)
        self.lat_pid = PID_controller(self.lat_Kp, self.lat_Ki, self.lat_Kd)
        self.ang_pid = PID_controller(self.ang_Kp, self.ang_Ki, self.ang_Kd)
        
        # Publish cmd_vel
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        # Timer
        self.create_timer(1/self.rate, self.timer_callback)

    def update_parameters(self):
        self.rate = self.get_parameter('rate').value
        self.long_Kp = self.get_parameter('long_Kp').value
        self.long_Ki = self.get_parameter('long_Ki').value
        self.long_Kd = self.get_parameter('long_Kd').value
        self.lat_Kp = self.get_parameter('lat_Kp').value
        self.lat_Ki = self.get_parameter('lat_Ki').value
        self.lat_Kd = self.get_parameter('lat_Kd').value
        self.ang_Kp = self.get_parameter('ang_Kp').value
        self.ang_Ki = self.get_parameter('ang_Ki').value
        self.ang_Kd = self.get_parameter('ang_Kd').value

    def parameter_callback(self, params):
        for param in params:
            if param.name in ['rate', 'long_Kp', 'long_Ki', 'long_Kd', 
                            'lat_Kp', 'lat_Ki', 'lat_Kd', 
                            'ang_Kp', 'ang_Ki', 'ang_Kd']:
                self.get_logger().info(f"Parameter {param.name} updated to {param.value}")
        self.update_parameters()
        return SetParametersResult(successful=True)

    def get_distance(self, x, y, _x, _y):
        dx = _x - x
        dy = _y - y
        return np.sqrt(dx**2 + dy**2)

    def get_cte(self, robot_x, robot_y, robot_yaw, path_x, path_y, path_yaw):
        # Convert path points to numpy arrays
        path_points = np.column_stack((path_x, path_y))
        robot_pos = np.array([robot_x, robot_y])

        # Compute Euclidean distances from robot to all path points
        distances = np.linalg.norm(path_points - robot_pos, axis=1)

        # Find the closest path point
        min_index = np.argmin(distances)
        closest_point = path_points[min_index]
        
        # Compute CTE as Euclidean distance
        cte = float(distances[min_index])

        return cte

    def odom_callback(self, msg: Odometry):
        self.odom.position = [msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z]
        self.odom.orientation = [msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]

    def timer_callback(self):
        # Get current position and orientation
        x = self.odom.position[0]
        y = self.odom.position[1]
        _, _, yaw = euler_from_quaternion(self.odom.orientation)

        # Get set points
        all_x = [point['x'] for point in self.setpoints]
        all_y = [point['y'] for point in self.setpoints]
        all_yaw = [point['yaw'] for point in self.setpoints]

        # Calculate error
        # error_long = np.cos(yaw) * (_x - x) + np.sin(yaw) * (_y - y)
        # error_long = self.get_distance(x, y, _x, _y) # Using distance error
        error_lat =  self.get_cte(x, y, yaw,all_x, all_y, all_yaw) # Using cross track error
        # error_ang = _yaw - yaw
        # error_ang = np.arctan2(np.sin(error_ang), np.cos(error_ang))

        # Calculate output
        # linear_x = self.long_pid.update_controller(error_long, 1)
        # linear_x = float(self.long_pid.update_controller(error_long, 1))
        angular_z = float(self.lat_pid.update_controller(error_lat, 1))
        # angular_z = float(self.ang_pid.update_controller(error_ang, 1))

        # Publish cmd_vel
        msg = Twist()
        msg.linear.x = 1.0
        msg.angular.z = angular_z
        self.cmd_vel_publisher.publish(msg)
        
        self.get_logger().info(f"Error: {1.0}, {error_lat}")
        self.get_logger().info(f"Output: {msg.linear.x}, {msg.angular.z}")

def main(args=None):
    rclpy.init(args=args)
    node = PIDControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()  
