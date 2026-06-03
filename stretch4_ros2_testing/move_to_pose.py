import rclpy

from hello_helpers.hello_misc import HelloNode

class MoveToPose(HelloNode):
    def __init__(self):
        super().__init__()
        self.main("move_to_pose_node")
        self.logger = self.get_logger()
        self.logger.info("MoveToPose node initialized.")
        self.demo()

    def demo(self):
        self.logger.info("Executing main function of MoveToPose node.")
        self.switch_mode("position")
        self.move_to_pose({
            "lift_joint": 0.5,
            "arm_joint": 0.1,
            "wrist_yaw_joint": 0.0,
            "wrist_roll_joint": 0.0,
            "wrist_pitch_joint": 0.0,
        })
        self.logger.info("Move to pose command executed.")

def main(args=None):
    move_to_pose_node = MoveToPose()
    
    try:
        rclpy.spin(move_to_pose_node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        move_to_pose_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()