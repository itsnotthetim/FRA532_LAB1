#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from ackermann_msgs.msg import AckermannDriveStamped
import math

class AckermannIKNode(Node):
    def __init__(self):
        super().__init__('ackermann_ik_node')
        
        # Declare parameters
        self.declare_parameter('wheelbase', 0.2)  # Distance between front and rear wheels
        self.declare_parameter('max_steering_angle', math.radians(30))  # Steering limits
        
        # Retrieve parameter values
        self.wheelbase = self.get_parameter('wheelbase').value
        self.max_steering_angle = self.get_parameter('max_steering_angle').value
        
        # Subscribe to cmd_vel topic
        self.cmd_vel_subscriber = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        
        # Publisher for Ackermann drive
        self.ackermann_publisher = self.create_publisher(
            AckermannDriveStamped, '/ackermann_cmd', 10)
        
        self.get_logger().info("Ackermann IK Node Initialized")
    
    def cmd_vel_callback(self, msg: Twist):
        linear_velocity = msg.linear.x  # Forward velocity (m/s)
        angular_velocity = msg.angular.z  # Rotation velocity (rad/s)

        # Compute steering angle using bicycle model formula with No-Slip Condition
        if abs(linear_velocity) > 0.001:
            turning_radius = linear_velocity / angular_velocity if abs(angular_velocity) > 0.001 else float('inf')
            steering_angle = math.atan2(self.wheelbase, turning_radius)
        else:
            steering_angle = 0.0
        
        # Clamp the steering angle within limits
        steering_angle = max(-self.max_steering_angle, min(self.max_steering_angle, steering_angle))
        
        # Create Ackermann message
        ackermann_msg = AckermannDriveStamped()
        ackermann_msg.drive.speed = linear_velocity
        ackermann_msg.drive.steering_angle = steering_angle
        
        # Publish Ackermann command
        self.ackermann_publisher.publish(ackermann_msg)
        self.get_logger().info(f"Published Ackermann: Speed={linear_velocity:.2f} m/s, Steering={math.degrees(steering_angle):.2f}°")


def main(args=None):
    rclpy.init(args=args)
    node = AckermannIKNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
