#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
import yaml
from ament_index_python.packages import get_package_share_directory

class DataSaverNode(Node):
    def __init__(self):
        super().__init__('data_saver_node')

        # Subscriptions for odometry topics
        self.create_subscription(Odometry, '/ground_truth/odom', self.ground_truth_callback, 10)
        # self.create_subscription(Odometry, '/single_track/odom', self.single_track_callback, 10)
        # self.create_subscription(Odometry, '/double_track/odom', self.double_track_callback, 10)
        # self.create_subscription(Odometry, '/yaw_rate/odom', self.yaw_rate_callback, 10)
        
        # Subscription for reached_goal flag
        self.create_subscription(Bool, '/reached_goal', self.reached_goal_callback, 10)

        # Data storage dictionaries (only time, position, and yaw)
        self.gt_time = []  # Ground truth timestamps (seconds)
        self.gt_pos = []   # List of [x, y]
        self.gt_yaw = []   # Yaw in radians

        self.st_time = []  # Single track
        self.st_pos = []
        self.st_yaw = []

        self.dt_time = []  # Double track
        self.dt_pos = []
        self.dt_yaw = []

        self.yr_time = []  # Yaw rate model
        self.yr_pos = []
        self.yr_yaw = []

    def quaternion_to_yaw(self, q):
        # Convert quaternion to yaw (radians)
        return math.atan2(2 * (q.w * q.z + q.x * q.y),
                          1 - 2 * (q.y * q.y + q.z * q.z))

    def extract_time(self, stamp):
        # Convert ROS2 stamp to seconds (float)
        return stamp.sec + stamp.nanosec * 1e-9

    def ground_truth_callback(self, msg: Odometry):
        t = self.extract_time(msg.header.stamp)
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        yaw = self.quaternion_to_yaw(msg.pose.pose.orientation)
        self.gt_time.append(t)
        self.gt_pos.append([x, y])
        self.gt_yaw.append(yaw)

    def single_track_callback(self, msg: Odometry):
        t = self.extract_time(msg.header.stamp)
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        yaw = self.quaternion_to_yaw(msg.pose.pose.orientation)
        self.st_time.append(t)
        self.st_pos.append([x, y])
        self.st_yaw.append(yaw)

    def double_track_callback(self, msg: Odometry):
        t = self.extract_time(msg.header.stamp)
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        yaw = self.quaternion_to_yaw(msg.pose.pose.orientation)
        self.dt_time.append(t)
        self.dt_pos.append([x, y])
        self.dt_yaw.append(yaw)

    def yaw_rate_callback(self, msg: Odometry):
        t = self.extract_time(msg.header.stamp)
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        yaw = self.quaternion_to_yaw(msg.pose.pose.orientation)
        self.yr_time.append(t)
        self.yr_pos.append([x, y])
        self.yr_yaw.append(yaw)

    def reached_goal_callback(self, msg: Bool):
        if msg.data:
            self.get_logger().info("Reached goal. Saving data to YAML file.")
            self.save_data_to_yaml()
        else:
            self.get_logger().info("Goal not reached yet.")

    def save_data_to_yaml(self):
        # Organize the data into a dictionary
        data = {
            "ground_truth": {
                "time": self.gt_time,
                "pos": self.gt_pos,
                "yaw": self.gt_yaw
            }
        }
        # Save the dictionary to a YAML file
        with open("/home/sunny/FRA532_LAB1/src/ackermann_controller/yaml/lab1.2/stan-0.25.yaml", "w") as f:
            yaml.dump(data, f)
        self.get_logger().info("Data saved to pid-0.5.yaml")

def main(args=None):
    rclpy.init(args=args)
    node = DataSaverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()