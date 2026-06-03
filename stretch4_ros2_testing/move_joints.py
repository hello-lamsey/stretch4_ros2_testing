#!/usr/bin/env python3

import sys
import termios
import tty
import threading
import time

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from control_msgs.msg import JointJog

class MoveJoints(Node):
    def __init__(self):
        super().__init__('move_joints')
        self.logger = self.get_logger()

        # Publisher
        self.pub_joint_jog = self.create_publisher(
            JointJog,
            topic="joint_vel",
            qos_profile=10
        )

        # Save the current terminal settings so we can restore them later
        self.settings = termios.tcgetattr(sys.stdin)

        # Run the CLI loop in a separate daemon thread
        self.cli_thread = threading.Thread(target=self.cli_loop, daemon=True)
        self.cli_thread.start()

        self.logger.info("MoveJoints node initialized. CLI thread active.")

    def get_key(self):
        """Puts terminal into raw mode to capture a single character instantly."""
        tty.setraw(sys.stdin.fileno())
        try:
            key = sys.stdin.read(1)
        finally:
            # Always restore terminal settings even if reading fails
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

    def _test_movement(self):
        msg_joint_jog = JointJog()
        msg_joint_jog.joint_names = ["lift_joint", "arm_joint"]
        msg_joint_jog.velocities = [0.04, 0.04]
        msg_joint_jog.duration = 1.0  # seconds
        
        self.logger.info("Publishing movement command...")
        self.pub_joint_jog.publish(msg_joint_jog)

        # Using time.sleep here is safe because this runs inside our background thread
        time.sleep(1)
        
        # Stop command
        msg_joint_jog.velocities = [0.0, 0.0]
        self.pub_joint_jog.publish(msg_joint_jog)
        self.logger.info("Movement stopped.")

    def cli_loop(self):
        """Background loop handling user keyboard inputs."""
        print("\n" + "="*30)
        print("  ROS 2 JOINT CONTROLLER CLI  ")
        print("="*30)
        print(" [M] - Move Joints (1 sec)")
        print(" [Q] - Quit Application")
        print("="*30 + "\n")

        while rclpy.ok():
            key = self.get_key()
            
            if key.lower() == 'm':
                self._test_movement()
            elif key.lower() == 'q' or key == '\x03':  # 'q' or Ctrl+C
                self.logger.info("Shutdown requested via CLI.")
                rclpy.shutdown()
                break

def main(args=None):
    rclpy.init(args=args)
    move_joints_node = MoveJoints()
    
    try:
        rclpy.spin(move_joints_node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        # Final safety check to make sure terminal isn't left corrupted
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, move_joints_node.settings)
        move_joints_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()