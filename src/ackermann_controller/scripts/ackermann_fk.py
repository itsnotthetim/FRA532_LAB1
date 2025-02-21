#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Quaternion, Twist
import math
import tf_transformations

class AckermannFKWheelOdometry(Node):
    def __init__(self):
        super().__init__('ackermann_fk_wheel_odometry')
        
      
        # Declare parameters
        self.dt = 0.01
        self.declare_parameter('wheelbase', 0.2)  # Distance between front and rear axles
        self.declare_parameter('track_width', 0.14)  # Distance between left and right wheels
        self.declare_parameter('wheel_radius', 0.045)  # Radius of the wheels
        self.declare_parameter('kinematic_model', 'single_track')  # Model selection: yaw_rate, single_track, double_track
        
        # Retrieve parameter values
        self.wheelbase = self.get_parameter('wheelbase').value
        self.track_width = self.get_parameter('track_width').value
        self.wheel_radius = self.get_parameter('wheel_radius').value
        self.kinematic_model = self.get_parameter('kinematic_model').value
        
        # Subscribe to wheel velocities (JointState message)
        self.wheel_subscriber = self.create_subscription(
            JointState, '/joint_states', self.wheel_callback, 10)
        
        # Publisher for odometry
        self.odom_publisher = self.create_publisher(Odometry, '/odom', 10)
        
        self.create_timer(self.dt, self.timer_callback)
        
        self.odom = [0, 0, 0, 0, 0, 0]
        self.rear_vel = [0.0, 0.0]
        self.get_logger().info("Ackermann FK Wheel Odometry Node Initialized")

    def timer_callback(self):
        self.state_space()
        self.odom = self.update_state_space
        self.odom_publisher()

    def wheel_callback(self, msg: JointState):

        # Initialize indices
        index_l, index_r, index_fl, index_fr = None, None, None, None

        # Create a mapping of joint names to their respective indices
        indices = {name: i for i, name in enumerate(msg.name)}

        # Assign values if they exist in the message
        index_fl = indices.get("front_left_wheel")
        index_fr = indices.get("front_right_wheel")
        index_l = indices.get("rear_left_wheel")
        index_r = indices.get("rear_right_wheel")

        # Compute velocities and steering angles only if indices are found
        if index_l is not None and index_r is not None:
            self.vel_rear = [msg.velocity[index_r] * self.wheel_radius, msg.velocity[index_l] * self.wheel_radius]

        if index_fl is not None and index_fr is not None:
            self.delta = (msg.position[index_fr] + msg.position[index_fl]) / 2

    def odom_publisher(self):
        odom_msg = Odometry()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = "odom"
        odom_msg.child_frame_id = "base_link"
        odom_msg.pose.pose.position.x = self.odom[0]
        odom_msg.pose.pose.position.y = self.odom[1]
        odom_msg.pose.pose.position.z = 0.0
        odom_msg.pose.pose.orientation = Quaternion(*self.quaternion)
        odom_msg.twist.twist.linear.x = self.odom[4]
        odom_msg.twist.twist.angular.z = self.odom[5]
        self.odom_publisher.publish
        
    def state_space(self):
        self.update_state_space = [0, 0, 0, 0, 0, 0]
        self.update_state_space[0] = self.odom[0] + self.odom[1] * self.dt * math.cos(self.odom[3] + self.odom[2] + (self.odom[5]* self.dt)/2)
        self.update_state_space[1] = self.odom[1] + self.odom[4] * self.dt * math.sin(self.odom[3] + self.odom[2] + (self.odom[5]* self.dt)/2)
        self.update_state_space[2] = self.odom[2] + self.odom[5] * self.dt
        self.quaternion = tf_transformations.quaternion_from_euler(0, 0, self.odom[2])
        self.update_state_space[4] = (self.rear_vel[0] + self.rear_vel[1]) / 2

        if(self.kinematic_model == 'single_track'):
            self.update_state_space[5] = self.odom[4]/self.wheelbase * math.tan(self.delta)
        elif(self.kinematic_model == 'double_track'):
            self.update_state_space[5] = (self.rear_vel[1] - self.rear_vel[0]) / self.track_width
        elif(self.kinematic_model == 'yaw_rate'):
            self.update_state_space[5] = self.odom[5]
        
def main(args=None):
    rclpy.init(args=args)
    node = AckermannFKWheelOdometry()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
