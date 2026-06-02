#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

class MoveJoints(Node):
    def __init__(self):
        super().__init__('move_joints')
        logger = self.get_logger()
        logger.info("Starting MoveJoints node")

def main(args=None):
    rclpy.init(args=args)
    move_joints_node = MoveJoints()
    rclpy.spin(move_joints_node)
    move_joints_node.destroy_node()
    rclpy.shutdown()