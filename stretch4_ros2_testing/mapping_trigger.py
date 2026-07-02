#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
import threading

# Nav2 and TF imports
from nav2_msgs.action import NavigateToPose
from tf2_ros import Buffer, TransformListener
from tf2_ros import TransformException

class WaypointDemoNode(Node):
    def __init__(self):
        super().__init__('waypoint_demo_node')
        
        # 1. Setup TF2 for getting current location
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # 2. Setup Nav2 Action Client
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
        # Store our waypoints in memory for the demo
        self.saved_waypoints = {}

    def save_current_location(self, name):
        """Looks up the latest transform to save as a waypoint."""
        try:
            # Get the transform from 'map' to 'base_link'
            t = self.tf_buffer.lookup_transform(
                'map',
                'base_link',
                rclpy.time.Time()
            )
            
            # Extract x, y, and rotation (quaternion)
            self.saved_waypoints[name] = t.transform
            self.get_logger().info(f"Waypoint '{name}' saved successfully!")
            
        except TransformException as ex:
            self.get_logger().error(f"Could not get current location: {ex}")

    def send_goal(self, name):
        """Sends a saved waypoint to Nav2."""
        if name not in self.saved_waypoints:
            self.get_logger().error(f"Waypoint '{name}' not found!")
            return
            
        if not self.nav_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error("Nav2 action server not available!")
            return

        goal_msg = NavigateToPose.Goal()
        waypoint_transform = self.saved_waypoints[name]
        
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        
        goal_msg.pose.pose.position.x = waypoint_transform.translation.x
        goal_msg.pose.pose.position.y = waypoint_transform.translation.y
        goal_msg.pose.pose.position.z = waypoint_transform.translation.z
        goal_msg.pose.pose.orientation = waypoint_transform.rotation
        
        self.get_logger().info(f"Sending robot to '{name}'...")
        self.nav_client.send_goal_async(goal_msg)


def user_input_loop(node):
    """Runs in the main thread and blocks waiting for user input."""
    help_text = "Commands: 'save <name>', 'go <name>', 'list', 'quit'"
    print(help_text)
    
    while rclpy.ok():
        try:
            # This blocks! But that's OK, ROS is spinning in the background.
            user_input = input("\n> ").strip().split()
            if not user_input:
                continue
                
            cmd = user_input[0].lower()
            
            if cmd == 'quit':
                break
            elif cmd == 'save' and len(user_input) > 1:
                node.save_current_location(user_input[1])
            elif cmd == 'go' and len(user_input) > 1:
                node.send_goal(user_input[1])
            elif cmd == 'list':
                print(f"Saved waypoints: {list(node.saved_waypoints.keys())}")
            else:
                print(help_text)
                
        except KeyboardInterrupt:
            # Catch Ctrl+C gracefully
            break

def main(args=None):
    rclpy.init(args=args)
    node = WaypointDemoNode()

    # --- THE MAGIC HAPPENS HERE ---
    # Spin ROS in a background daemon thread. 
    # daemon=True ensures the thread dies immediately when the main thread exits.
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    # Run the blocking UI in the main thread
    user_input_loop(node)

    # Cleanup once the user types 'quit' or hits Ctrl+C
    print("\nShutting down demo...")
    node.destroy_node()
    rclpy.shutdown()
    spin_thread.join()

if __name__ == '__main__':
    main()