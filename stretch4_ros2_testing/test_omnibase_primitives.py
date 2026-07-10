import time
import math
import rclpy
from hello_helpers.hello_misc import HelloNode

class TestOmnibasePrimitives(HelloNode):
    def __init__(self):
        super().__init__()
        # HelloNode's main() method initializes the node, spins it in a background thread,
        # and connects to the joint trajectory action server.
        self.main("test_omnibase_primitives_node")
        self.logger = self.get_logger()
        self.logger.info("TestOmnibasePrimitives node initialized.")

    def run_sequential_test(self):
        self.logger.info("Starting sequential base motion primitive test...")
        self.switch_mode("position")
        
        self.logger.info("Step 1: Moving forward 0.1m")
        self.move_to_pose({"translate_mobile_base": 0.1})
        time.sleep(2.0)
        
        self.logger.info("Step 2: Moving left 0.1m")
        self.move_to_pose({"translate_mobile_base_y": 0.1})
        time.sleep(2.0)
        
        self.logger.info("Step 3: Rotating 30 degrees CCW (0.524 rad)")
        self.move_to_pose({"rotate_mobile_base": math.radians(30)})
        time.sleep(2.0)
        
        self.logger.info("Sequential test completed successfully.")

    def run_xy_test(self):
        self.logger.info("Starting XY translation test...")
        self.switch_mode("position")
        
        self.logger.info("Moving to (0.1, 0.1) in the XY plane")
        self.move_to_pose({
            "translate_mobile_base": 0.1,
            "translate_mobile_base_y": 0.1
        })
        time.sleep(2.0)
        
        self.logger.info("XY translation test completed successfully.")

    def run_all_dof_at_once_test(self):
        self.logger.info("Starting all three DOF at once test (should trigger validation error)...")
        self.switch_mode("position")
        
        try:
            self.move_to_pose({
                "translate_mobile_base": 0.1,
                "translate_mobile_base_y": 0.1,
                "rotate_mobile_base": math.radians(30)
            })
            self.logger.info("Successfully commanded all three DOF at once (unexpected).")
        except Exception as e:
            self.logger.error(f"Commanding all three DOF at once failed as expected: {e}")

def main(args=None):
    test_node = TestOmnibasePrimitives()
    
    try:
        # Call the requested sequential test sequence
        test_node.run_sequential_test()
        test_node.run_xy_test()
        test_node.run_all_dof_at_once_test()
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        test_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
