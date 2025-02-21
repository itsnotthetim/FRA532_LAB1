#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from tf_transformations import euler_from_quaternion
import yaml
import numpy as np

class PIDController:
    def __init__(self, Kp, Ki, Kd, dt, windupMax):

        # PID parameters
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd

        # PID terms
        self.pTerm = 0
        self.iTerm = 0
        self.dTerm = 0

        # Sampling time
        self.dt = dt

        # PID last output value returned     
        self.outValue = 0
        
        # Last error
        self.error = 0
        self.last_error = 0
        
        # Last y measured
        self.last_y = 0
    
        # Anti windup        
        self.windupMax = windupMax

    def output(self, y_measured):
        
        # Calculate output
        # P term
        self.pTerm = self.Kp * self.error
        # I term            
        self.iTerm += self.Ki * self.error * self.dt
        # D term
        self.dTerm = self.Kd * (self.last_y - y_measured)/self.dt
        
        # Check for windup problems if anti-windup is enabled
        self.antiWindUp()
        
        # Update variables
        self.last_error = self.error
        self.last_y = y_measured
        
        # Output value to be returned
        self.outValue = self.pTerm + self.iTerm + self.dTerm
        
        return self.outValue
    
    def antiWindUp(self):
        if self.windupMax != 0:
            if self.iTerm > self.windupMax:
                self.iTerm = self.windupMax
            elif self.iTerm < -self.windupMax:
                self.iTerm = -self.windupMax

class PIDControllerNode(Node):
    def __init__(self):
        super().__init__('pid_controller_node')
        
        # Declare parameters
        self.declare_parameter('rate', 100)
        self.declare_parameter('x_Kp', 1.0)
        self.declare_parameter('x_Ki', 0.0)
        self.declare_parameter('x_Kd', 0.0)
        self.declare_parameter('y_Kp', 1.0)
        self.declare_parameter('y_Ki', 0.0)
        self.declare_parameter('y_Kd', 0.0)
        self.declare_parameter('yaw_Kp', 1.0)
        self.declare_parameter('yaw_Ki', 0.0)
        self.declare_parameter('yaw_Kd', 0.0)

        # Rate
        self.rate = self.get_parameter('rate').value

        # PID parameters
        self.x_Kp = self.get_parameter('x_Kp').value
        self.x_Ki = self.get_parameter('x_Ki').value
        self.x_Kd = self.get_parameter('x_Kd').value
        self.y_Kp = self.get_parameter('y_Kp').value
        self.y_Ki = self.get_parameter('y_Ki').value
        self.y_Kd = self.get_parameter('y_Kd').value
        self.yaw_Kp = self.get_parameter('yaw_Kp').value
        self.yaw_Ki = self.get_parameter('yaw_Ki').value
        self.yaw_Kd = self.get_parameter('yaw_Kd').value

        # Set points
        with open('//yaml/path.yaml', 'r') as f:
            self.setpoints = yaml.safe_load(f)
        self.count_path = 0


        # Get odom
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        class odom:
            position = [0, 0, 0] # x, y, z
            orientation = [0, 0, 0, 0] # x, y, z, w
        self.odom = odom()

        # Create PID controllers
        self.x_pid = PIDController(self.x_Kp, self.x_Ki, self.x_Kd, 0, 1/self.rate, 0)
        self.y_pid = PIDController(self.y_Kp, self.y_Ki, self.y_Kd, 0, 1/self.rate, 0)
        self.yaw_pid = PIDController(self.yaw_Kp, self.yaw_Ki, self.yaw_Kd, 0, 1/self.rate, 0)

        # Publish cmd_vel
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        # Timer
        self.create_timer(1/self.rate, self.timer_callback)

    def get_distance(self, x, y, _x, _y):
        dx = _x - x
        dy = _y - y
        return np.sqrt(dx**2 + dy**2)

    def odom_callback(self, msg: Odometry):
        self.odom.position = [msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z]
        self.odom.orientation = [msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]

    def timer_callback(self):
        # Get current position and orientation
        x = self.odom.position[0]
        y = self.odom.position[1]
        _, _, yaw = euler_from_quaternion(self.odom.orientation)

        # Get set points
        self.count_path += 1
        if self.count_path >= len(self.setpoints):
            self.count_path = 0
        setpoint = self.setpoints[self.count_path]
        self.position_pid.error = self.get_distance(x, y, setpoint['position'][0], setpoint['position'][1])
        self.orientation_pid.error =  setpoint['orientation'] - yaw

        # Calculate output
        linear_x = self.position_pid.output(x)
        angular_z = self.orientation_pid.output(yaw)

        # Publish cmd_vel
        msg = Twist()
        msg.linear.x = linear_x
        msg.angular.z = angular_z
        self.cmd_vel_publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = PIDControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()
