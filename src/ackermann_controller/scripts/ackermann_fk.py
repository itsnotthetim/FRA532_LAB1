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
        self.declare_parameter('wheelbase', 1.0)  # Distance between front and rear axles
        self.declare_parameter('track_width', 0.5)  # Distance between left and right wheels
        self.declare_parameter('wheel_radius', 0.15)  # Radius of the wheels
        self.declare_parameter('kinematic_model', 'yaw_rate')  # Model selection: yaw_rate, single_track, double_track
        
        # Retrieve parameter values
        self.wheelbase = self.get_parameter('wheelbase').value
        self.track_width = self.get_parameter('track_width').value
        self.wheel_radius = self.get_parameter('wheel_radius').value
        self.kinematic_model = self.get_parameter('kinematic_model').value
        
        # Subscribe to wheel velocities (JointState message)
        self.wheel_subscriber = self.create_subscription(
            JointState, '/wheel_states', self.wheel_callback, 10)
        
        # Publisher for odometry
        self.odom_publisher = self.create_publisher(Odometry, '/odom', 10)
        
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.last_time = self.get_clock().now()
        
        self.get_logger().info("Ackermann FK Wheel Odometry Node Initialized")

    def wheel_callback(self, msg: JointState):
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        self.last_time = current_time
        
        if len(msg.velocity) < 2:
            return
        
        v_left = msg.velocity[0] * self.wheel_radius  # Convert to linear velocity
        v_right = msg.velocity[1] * self.wheel_radius  # Convert to linear velocity
        
        # Compute velocity and yaw rate
        v = (v_left + v_right) / 2.0
        omega = (v_right - v_left) / self.track_width
        
        if self.kinematic_model == 'yaw_rate':
            pass  # Keep omega as is
        elif self.kinematic_model == 'single_track':
            steering_angle = math.atan2(self.wheelbase * omega, v) if abs(v) > 0.001 else 0.0
            omega = v * math.tan(steering_angle) / self.wheelbase
        elif self.kinematic_model == 'double_track':
            beta = math.atan2((self.track_width / 2.0) * omega, v) if abs(v) > 0.001 else 0.0
            omega = omega / math.cos(beta) if abs(beta) > 0.001 else omega
        
        # Update pose
        self.yaw += omega * dt
        self.x += v * math.cos(self.yaw) * dt
        self.y += v * math.sin(self.yaw) * dt
        
        # Create odometry message
        odom_msg = Odometry()
        odom_msg.header.stamp = current_time.to_msg()
        odom_msg.header.frame_id = "odom"
        odom_msg.child_frame_id = "base_link"
        
        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        odom_msg.pose.pose.position.z = 0.0
        
        quaternion = tf_transformations.quaternion_from_euler(0, 0, self.yaw)
        odom_msg.pose.pose.orientation = Quaternion(*quaternion)
        
        odom_msg.twist.twist.linear.x = v
        odom_msg.twist.twist.angular.z = omega
        
        # Publish odometry
        self.odom_publisher.publish(odom_msg)
        
        self.get_logger().info(f"[{self.kinematic_model.upper()} MODEL] Odometry Updated: X={self.x:.2f}, Y={self.y:.2f}, Yaw={math.degrees(self.yaw):.2f}°")


def main(args=None):
    rclpy.init(args=args)
    node = AckermannFKWheelOdometry()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
