#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray

import math

class AckermannIKNode(Node):
    def __init__(self):
        super().__init__('ackermann_ik_node')
        
        # Declare parameters
        self.declare_parameter('wheel_base', 0.2)  # WB: Distance between front and rear axles
        self.declare_parameter('wheel_radius', 0.045)  # WB: Distance between front and rear axles
        self.declare_parameter('track_width', 0.14)  # TW: Distance between left and right wheels
        self.declare_parameter('steering_ratio', 1.0)  # γ: Steering ratio
        self.declare_parameter('model', 'ackermann') # Model selection: bicycle, ackermann

        # Get parameter values
        self.WB = self.get_parameter('wheel_base').value
        self.wheel_radius = self.get_parameter('wheel_radius').value
        self.TW = self.get_parameter('track_width').value
        self.gamma = self.get_parameter('steering_ratio').value
        self.model = self.get_parameter('model').value
        
        self.linear_velocity = 0.0  # Linear velocity v
        self.angular_velocity = 0.0  # Angular velocity Ωz


        # Publish Velocity to Wheels
        self.rear_wheel_publisher = self.create_publisher(Float64MultiArray, '/forward_velocity_controllers/commands', 10)
        self.front_wheel_publisher = self.create_publisher(Float64MultiArray, '/forward_position_controllers/commands', 10)

        # Subscribe to cmd_vel topic
        self.cmd_vel_subscriber = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        
       
    
    def cmd_vel_callback(self, msg: Twist):
        self.linear_velocity = msg.linear.x  # Forward velocity (m/s)
        self.angular_velocity = msg.angular.z  # Rotation velocity (rad/s)



        if self.linear_velocity != 0.0:
            delta = math.atan((self.WB * self.angular_velocity) / self.linear_velocity)

            delta_Ack = delta / self.gamma
            delta_L = math.atan((self.WB * math.tan(delta_Ack)) / (self.WB - 0.5 * self.TW * math.tan(delta_Ack)))
            delta_R = math.atan((self.WB * math.tan(delta_Ack)) / (self.WB + 0.5 * self.TW * math.tan(delta_Ack)))
        
        else:
            delta = 0.0
            delta_Ack = 0.0
            delta_L = 0.0
            delta_R = 0.0

        
        if self.model == 'bicycle':
            steering_angle_left_wheel = delta
            steering_angle_right_wheel = delta
        elif self.model == 'ackermann':
            steering_angle_left_wheel = delta_L
            steering_angle_right_wheel = delta_R

        speed_rear_wheel = self.linear_velocity / self.wheel_radius

        front_wheel_msg = Float64MultiArray()
        front_wheel_msg.data = [steering_angle_left_wheel, steering_angle_right_wheel]
        self.front_wheel_publisher.publish(front_wheel_msg)

        rear_wheel_msg = Float64MultiArray()
        rear_wheel_msg.data = [speed_rear_wheel, speed_rear_wheel]
        self.rear_wheel_publisher.publish(rear_wheel_msg)

    

def main(args=None):
    rclpy.init(args=args)
    node = AckermannIKNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
