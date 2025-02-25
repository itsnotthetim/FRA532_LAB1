#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState, Imu
from geometry_msgs.msg import Quaternion, Twist, TransformStamped
from gazebo_msgs.msg import ModelStates
import math
import tf_transformations 
from tf2_ros import TransformBroadcaster
from nav2_msgs.srv import SetInitialPose

class AckermannFKWheelOdometry(Node):
    def __init__(self):
        super().__init__('ackermann_fk_wheel_odometry', namespace='')
      
        # Declare parameters
        self.dt = 0.01
        self.declare_parameter('wheelbase', 0.2)  # Distance between front and rear axles
        self.declare_parameter('track_width', 0.14)  # Distance between left and right wheels
        self.declare_parameter('wheel_radius', 0.045)  # Radius of the wheels
        self.declare_parameter('kinematic_model', 'ground_truth')  # Model selection: yaw_rate, single_track, double_track
        self.declare_parameter('pub_tf', True) # Publish tf or not

        # Retrieve parameter values
        self.wheelbase = self.get_parameter('wheelbase').value
        self.track_width = self.get_parameter('track_width').value
        self.wheel_radius = self.get_parameter('wheel_radius').value
        self.kinematic_model = self.get_parameter('kinematic_model').value
        
        # Subscribe to wheel velocities (JointState message)
        self.wheel_subscriber = self.create_subscription(
            JointState, '/joint_states', self.wheel_callback, 10)
        
        self.imu_subscriber = self.create_subscription(Imu, '/imu_plugin/out', self.imu_callback, 10)
        self.imu_subscriber = self.create_subscription(ModelStates, '/gazebo/model_states', self.ground_truth_callback, 10)

        # Publisher for odometry
        self.odom_publisher = self.create_publisher(Odometry, 'odom', 10)
        
        self.create_timer(self.dt, self.timer_callback)
        
        self.tf_broadcaster = TransformBroadcaster(self)
        self.odom = [0, 0, 0, 0, 0, 0]
        self.odom_ground_truth = [0, 0, 0, 0, 0, 0]

        self.quaternion = [0, 0, 0, 0]
        self.rear_vel = [0.0, 0.0]
        self.delta = 0.0
        self.yaw = 0.0
        self.get_logger().info("Ackermann FK Wheel Odometry Node Initialized")

        # Set initial position and orientation
        self.is_set_initial_pose = False

    def timer_callback(self):
        self.state_space()
        # print(self.odom)  

    def odom_pub(self, odom):
        odom_msg = Odometry()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = "odom"
        odom_msg.child_frame_id = "base_footprint"
        odom_msg.pose.pose.position.x = float(odom[0])
        odom_msg.pose.pose.position.y = float(odom[1])
        odom_msg.pose.pose.position.z = 0.0

        quaternion = tf_transformations.quaternion_from_euler(0, 0, float(odom[2]))
        odom_msg.pose.pose.orientation.x = quaternion[0]
        odom_msg.pose.pose.orientation.y = quaternion[1]
        odom_msg.pose.pose.orientation.z = quaternion[2]
        odom_msg.pose.pose.orientation.w = quaternion[3]

        odom_msg.twist.twist.linear.x = float(odom[4])
        odom_msg.twist.twist.angular.z = float(odom[5])

        self.odom_publisher.publish(odom_msg)

        if self.get_parameter('pub_tf').value:
            tfs = TransformStamped()
            tfs.header.stamp = self.get_clock().now().to_msg()
            tfs.header.frame_id = "odom"
            tfs.child_frame_id = "base_footprint"

            tfs.transform.translation.x = float(odom[0])
            tfs.transform.translation.y = float(odom[1])
            tfs.transform.translation.z = 0.0
            tfs.transform.rotation.x = quaternion[0]
            tfs.transform.rotation.y = quaternion[1]
            tfs.transform.rotation.z = quaternion[2]
            tfs.transform.rotation.w = quaternion[3]
        
            self.tf_broadcaster.sendTransform(tfs)
        
    def state_space(self):
        self.update_state_space = [0, 0, 0, 0, 0, 0]
        self.update_state_space[0] = self.odom[0] + self.odom[4] * self.dt * math.cos(self.odom[3] + self.odom[2] + (self.odom[5]* self.dt)/2)
        self.update_state_space[1] = self.odom[1] + self.odom[4] * self.dt * math.sin(self.odom[3] + self.odom[2] + (self.odom[5]* self.dt)/2)
        self.update_state_space[2] = self.odom[2] + self.odom[5] * self.dt
        self.update_state_space[4] = (self.rear_vel[0] + self.rear_vel[1]) / 2

        if(self.kinematic_model == 'single_track'):
            self.update_state_space[5] = (self.odom[4]/self.wheelbase) * math.tan(self.delta)
        elif(self.kinematic_model == 'double_track'):
            self.update_state_space[5] = (self.rear_vel[0] - self.rear_vel[1]) / self.track_width
        elif(self.kinematic_model == 'yaw_rate'):
            self.update_state_space[5] = self.yaw

        if self.kinematic_model != 'ground_truth':
            self.odom = self.update_state_space
            self.odom_pub(self.odom)
        else:
            self.odom = self.odom_ground_truth
            self.odom_pub(self.odom_ground_truth)

    def wheel_callback(self, msg: JointState):

        # Initialize indices
        index_l, index_r, index_fl, index_fr = None, None, None, None

        # Create a mapping of joint names to their respective indices
        indices = {name: i for i, name in enumerate(msg.name)}

        # Assign values if they exist in the message
        index_fl = indices.get("left_steering_hinge_wheel")
        index_fr = indices.get("right_steering_hinge_wheel")
        index_l = indices.get("rear_left_wheel")
        index_r = indices.get("rear_right_wheel")

        # Compute velocities and steering angles only if indices are found
        if index_l is not None and index_r is not None:
            self.rear_vel = [msg.velocity[index_r] * self.wheel_radius, msg.velocity[index_l] * self.wheel_radius]

        if index_fl is not None and index_fr is not None:
            self.delta = (msg.position[index_fr] + msg.position[index_fl]) / 2
    
    def imu_callback(self, msg: Imu):
        self.yaw = msg.angular_velocity.z
    
    def ground_truth_callback(self, msg: ModelStates):
        index = msg.name.index("ackbot")
        self.posX = msg.pose[index].position.x
        self.posY = msg.pose[index].position.y
        self.posZ = msg.pose[index].position.z

        self.q1 = msg.pose[index].orientation.x
        self.q2 = msg.pose[index].orientation.y
        self.q3 = msg.pose[index].orientation.z
        self.q4 = msg.pose[index].orientation.w
        self.euler = tf_transformations.euler_from_quaternion([self.q1, self.q2, self.q3, self.q4])
        self.yaw_ground_truth = self.euler[2]

        self.linX = msg.twist[index].linear.x
        self.angZ = msg.twist[index].angular.z
        self.odom_ground_truth = [self.posX, self.posY, self.yaw_ground_truth, 0 ,self.linX, self.angZ]

        if self.is_set_initial_pose == False:
            self.odom = self.odom_ground_truth
            self.is_set_initial_pose = True

        
def main(args=None):
    rclpy.init(args=args)
    node = AckermannFKWheelOdometry()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
