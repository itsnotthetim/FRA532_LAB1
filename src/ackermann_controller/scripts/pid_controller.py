#!/usr/bin/python3

import rclpy
from rclpy.node import Node


class PIDControllerNode(Node):
    def __init__(self):
        super().__init__('pid_controller_node')

def main(args=None):
    rclpy.init(args=args)
    node = PIDControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()
