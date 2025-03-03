#!/usr/bin/env python3

import math
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
import matplotlib.pyplot as plt

class KinematicValidateNode(Node):
    def __init__(self):
        super().__init__('kinematic_validate_node')

        # Subscriptions for odometry topics
        self.create_subscription(Odometry, '/ground_truth/odom', self.ground_truth_callback, 10)
        self.create_subscription(Odometry, '/single_track/odom', self.single_track_callback, 10)
        self.create_subscription(Odometry, '/double_track/odom', self.double_track_callback, 10)
        self.create_subscription(Odometry, '/yaw_rate/odom', self.yaw_rate_callback, 10)
        
        # Subscription for reached_goal flag
        self.create_subscription(Bool, '/reached_goal', self.reached_goal_callback, 10)

        # Data storage (only time, position, and yaw)
        self.gt_time = []  # seconds
        self.gt_pos = []   # list of [x, y]
        self.gt_yaw = []   # yaw in radians

        self.st_time = []
        self.st_pos = []
        self.st_yaw = []

        self.dt_time = []
        self.dt_pos = []
        self.dt_yaw = []

        self.yr_time = []
        self.yr_pos = []
        self.yr_yaw = []

    def quaternion_to_yaw(self, q):
        # Convert quaternion to yaw (radians)
        return math.atan2(2 * (q.w * q.z + q.x * q.y),
                          1 - 2 * (q.y * q.y + q.z * q.z))

    def extract_time(self, stamp):
        # Convert ROS2 timestamp to float seconds
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
            self.get_logger().info("Reached goal! Plotting collected data and error distributions.")
            self.plot_graphs()
        else:
            self.get_logger().info("Goal not reached yet.")

    def compute_position_errors(self, gt_pos, pred_pos):
        # Compute Euclidean distance errors between ground truth and predicted positions.
        n = min(len(gt_pos), len(pred_pos))
        errors = []
        for i in range(n):
            dx = gt_pos[i][0] - pred_pos[i][0]
            dy = gt_pos[i][1] - pred_pos[i][1]
            errors.append(math.sqrt(dx*dx + dy*dy))
        return errors

    def compute_yaw_errors(self, gt_yaw, pred_yaw):
        # Compute absolute yaw errors, wrapped properly to [-pi, pi].
        n = min(len(gt_yaw), len(pred_yaw))
        errors = []
        for i in range(n):
            diff = gt_yaw[i] - pred_yaw[i]
            # Wrap error to [-pi, pi]
            diff = math.atan2(math.sin(diff), math.cos(diff))
            errors.append(abs(diff))
        return errors

    def plot_graphs(self):
        # ---------------- Figure 1: Position Trajectories ----------------
        fig_pos, axs_pos = plt.subplots(1, 3, figsize=(15, 5))
        # Ground Truth vs Single Track
        gt_x = [p[0] for p in self.gt_pos]
        gt_y = [p[1] for p in self.gt_pos]
        st_x = [p[0] for p in self.st_pos]
        st_y = [p[1] for p in self.st_pos]
        axs_pos[0].plot(gt_x, gt_y, 'bo-', label='Ground Truth', markersize=1)
        axs_pos[0].plot(st_x, st_y, 'ro-', label='Single Track', markersize=1)
        axs_pos[0].set_title("Position: GT vs Single Track")
        axs_pos[0].set_xlabel("X")
        axs_pos[0].set_ylabel("Y")
        axs_pos[0].legend()

        # Ground Truth vs Double Track
        dt_x = [p[0] for p in self.dt_pos]
        dt_y = [p[1] for p in self.dt_pos]
        axs_pos[1].plot(gt_x, gt_y, 'bo-', label='Ground Truth', markersize=1)
        axs_pos[1].plot(dt_x, dt_y, 'ro-', label='Double Track', markersize=1)
        axs_pos[1].set_title("Position: GT vs Double Track")
        axs_pos[1].set_xlabel("X")
        axs_pos[1].set_ylabel("Y")
        axs_pos[1].legend()

        # Ground Truth vs Yaw Rate
        yr_x = [p[0] for p in self.yr_pos]
        yr_y = [p[1] for p in self.yr_pos]
        axs_pos[2].plot(gt_x, gt_y, 'bo-', label='Ground Truth', markersize=1)
        axs_pos[2].plot(yr_x, yr_y, 'ro-', label='Yaw Rate', markersize=1)
        axs_pos[2].set_title("Position: GT vs Yaw Rate")
        axs_pos[2].set_xlabel("X")
        axs_pos[2].set_ylabel("Y")
        axs_pos[2].legend()

        # ---------------- Figure 2: Yaw vs Time ----------------
        fig_yaw, axs_yaw = plt.subplots(1, 3, figsize=(15, 5))
        axs_yaw[0].plot(self.gt_time, self.gt_yaw, 'bo-', label='Ground Truth', markersize=1)
        axs_yaw[0].plot(self.st_time, self.st_yaw, 'ro-', label='Single Track', markersize=1)
        axs_yaw[0].set_title("Yaw: GT vs Single Track")
        axs_yaw[0].set_xlabel("Time (s)")
        axs_yaw[0].set_ylabel("Yaw (rad)")
        axs_yaw[0].legend()

        axs_yaw[1].plot(self.gt_time, self.gt_yaw, 'bo-', label='Ground Truth', markersize=1)
        axs_yaw[1].plot(self.dt_time, self.dt_yaw, 'ro-', label='Double Track', markersize=1)
        axs_yaw[1].set_title("Yaw: GT vs Double Track")
        axs_yaw[1].set_xlabel("Time (s)")
        axs_yaw[1].set_ylabel("Yaw (rad)")
        axs_yaw[1].legend()

        axs_yaw[2].plot(self.gt_time, self.gt_yaw, 'bo-', label='Ground Truth', markersize=1)
        axs_yaw[2].plot(self.yr_time, self.yr_yaw, 'ro-', label='Yaw Rate', markersize=1)
        axs_yaw[2].set_title("Yaw: GT vs Yaw Rate")
        axs_yaw[2].set_xlabel("Time (s)")
        axs_yaw[2].set_ylabel("Yaw (rad)")
        axs_yaw[2].legend()

        # ---------------- Figure 3: Position Error Distribution ----------------
        fig_pos_err, axs_pos_err = plt.subplots(1, 3, figsize=(15, 5))
        st_pos_err = self.compute_position_errors(self.gt_pos, self.st_pos)
        dt_pos_err = self.compute_position_errors(self.gt_pos, self.dt_pos)
        yr_pos_err = self.compute_position_errors(self.gt_pos, self.yr_pos)

        axs_pos_err[0].hist(st_pos_err, bins=50, color='r', alpha=0.7, label='Single Track')
        axs_pos_err[0].set_title("Position Error: GT vs Single Track")
        axs_pos_err[0].set_xlabel("Error (m)")
        axs_pos_err[0].set_ylabel("Frequency")
        axs_pos_err[0].legend()

        axs_pos_err[1].hist(dt_pos_err, bins=50, color='r', alpha=0.7, label='Double Track')
        axs_pos_err[1].set_title("Position Error: GT vs Double Track")
        axs_pos_err[1].set_xlabel("Error (m)")
        axs_pos_err[1].set_ylabel("Frequency")
        axs_pos_err[1].legend()

        axs_pos_err[2].hist(yr_pos_err, bins=50, color='r', alpha=0.7, label='Yaw Rate')
        axs_pos_err[2].set_title("Position Error: GT vs Yaw Rate")
        axs_pos_err[2].set_xlabel("Error (m)")
        axs_pos_err[2].set_ylabel("Frequency")
        axs_pos_err[2].legend()

        # ---------------- Figure 4: Yaw Error Distribution ----------------
        fig_yaw_err, axs_yaw_err = plt.subplots(1, 3, figsize=(15, 5))
        st_yaw_err = self.compute_yaw_errors(self.gt_yaw, self.st_yaw)
        dt_yaw_err = self.compute_yaw_errors(self.gt_yaw, self.dt_yaw)
        yr_yaw_err = self.compute_yaw_errors(self.gt_yaw, self.yr_yaw)

        axs_yaw_err[0].hist(st_yaw_err, bins=50, color='r', alpha=0.7, label='Single Track')
        axs_yaw_err[0].set_title("Yaw Error: GT vs Single Track")
        axs_yaw_err[0].set_xlabel("Error (rad)")
        axs_yaw_err[0].set_ylabel("Frequency")
        axs_yaw_err[0].legend()

        axs_yaw_err[1].hist(dt_yaw_err, bins=50, color='r', alpha=0.7, label='Double Track')
        axs_yaw_err[1].set_title("Yaw Error: GT vs Double Track")
        axs_yaw_err[1].set_xlabel("Error (rad)")
        axs_yaw_err[1].set_ylabel("Frequency")
        axs_yaw_err[1].legend()

        axs_yaw_err[2].hist(yr_yaw_err, bins=50, color='r', alpha=0.7, label='Yaw Rate')
        axs_yaw_err[2].set_title("Yaw Error: GT vs Yaw Rate")
        axs_yaw_err[2].set_xlabel("Error (rad)")
        axs_yaw_err[2].set_ylabel("Frequency")
        axs_yaw_err[2].legend()

        plt.show()

def main(args=None):
    rclpy.init(args=args)
    node = KinematicValidateNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
