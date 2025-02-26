import rclpy
from rclpy.node import Node
import numpy as np
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, Quaternion
import tf_transformations


class EKFLocalization(Node):
    def __init__(self):
        super().__init__('ekf_localization')

        # Subscribe to odometry and GPS
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.gps_sub = self.create_subscription(PoseStamped, '/gps_pose', self.gps_callback, 10)

        # Publisher for fused odometry
        self.ekf_pub = self.create_publisher(Odometry, '/ekf_odom', 10)

        # State vector: [x, y, theta]
        self.x_est = np.array([0.0, 0.0, 0.0])  # Initial estimate
        self.P_est = np.eye(3) * 1.0  # Initial covariance matrix

        # Process noise covariance (Q) - adjustable
        self.process_noise = 0.1
        self.Q = np.eye(3) * self.process_noise

        # Measurement noise covariance (R) - adjustable
        self.measurement_noise = 0.5
        self.R = np.eye(2) * self.measurement_noise

        # Time step
        self.dt = 0.1

    def odom_callback(self, msg):
        """ EKF Prediction Step using Odometry """
        v = msg.twist.twist.linear.x  # Forward velocity
        omega = msg.twist.twist.angular.z  # Angular velocity
        theta = self.x_est[2]

        # State transition function (Prediction)
        x_pred = np.array([
            self.x_est[0] + v * np.cos(theta) * self.dt,
            self.x_est[1] + v * np.sin(theta) * self.dt,
            self.x_est[2] + omega * self.dt
        ])

        # Jacobian of motion model (F)
        F = np.array([
            [1, 0, -v * np.sin(theta) * self.dt],
            [0, 1,  v * np.cos(theta) * self.dt],
            [0, 0,  1]
        ])

        # Predicted covariance
        P_pred = F @ self.P_est @ F.T + self.Q

        # Update estimated values
        self.x_est = x_pred
        self.P_est = P_pred

        # Publish the updated estimate
        self.publish_estimate()

    def gps_callback(self, msg):
        """ EKF Update Step using GPS """
        z = np.array([msg.pose.position.x, msg.pose.position.y])  # Measurement

        # Measurement matrix (H)
        H = np.array([[1, 0, 0], [0, 1, 0]])

        # Innovation (residual)
        y = z - H @ self.x_est

        # Innovation covariance
        S = H @ self.P_est @ H.T + self.R

        # Kalman Gain
        K = self.P_est @ H.T @ np.linalg.inv(S)

        # Updated state estimate
        self.x_est = self.x_est + K @ y

        # Updated covariance
        self.P_est = (np.eye(3) - K @ H) @ self.P_est

        # Publish the updated estimate
        self.publish_estimate()

    def publish_estimate(self):
        """ Publish the EKF estimated pose as Odometry message """
        odom_msg = Odometry()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = "odom"

        # Assign estimated state
        odom_msg.pose.pose.position.x = self.x_est[0]
        odom_msg.pose.pose.position.y = self.x_est[1]

        # Convert theta (yaw) to quaternion
        q = tf_transformations.quaternion_from_euler(0, 0, self.x_est[2])
        odom_msg.pose.pose.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])

        self.ekf_pub.publish(odom_msg)


def main(args=None):
    rclpy.init(args=args)
    node = EKFLocalization()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
