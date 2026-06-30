#!/usr/bin/env python3
import time
import threading
import numpy as np

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType
from rcl_interfaces.srv import SetParameters

from control_msgs.msg import JointJog
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry

from stretch4_kinematics.state.joint_positions import StretchJointPositions
from stretch4_kinematics.state.joint_velocities import StretchJointVelocities
from stretch4_kinematics.kinematic_models.tool_frame_kinematics import ToolFrameKinematics

class FlyingGripper(Node):
    """
    ROS 2 node that controls the Stretch 4 robot using differential inverse kinematics.

    This node subscribes to joint states and odometry to maintain the current state of
    the robot, computes joint velocities to achieve target task-space velocities,
    and publishes the commands to the ROS 2 driver.
    """

    def __init__(self) -> None:
        """
        Initialize the FlyingGripper node.

        Sets up publishers, subscribers, service clients, and internal state
        objects for kinematics and joint positions.
        """
        super().__init__('flying_gripper')

        self.kinematic_model = ToolFrameKinematics()
        self.stretch_joint_position = StretchJointPositions()

        # pub
        self.pub_joint_vel = self.create_publisher(JointJog, 'joint_vel', 10)
        self.pub_base_twist = self.create_publisher(Twist, 'cmd_vel', 10)

        # sub
        self.sub_joint_state = self.create_subscription(
            JointState,
            'joint_states',
            self.joint_states_callback,
            10
        )

        self.sub_odom = self.create_subscription(
            Odometry,
            'odom',
            self.odom_callback,
            10
        )

        self.sub_wheel_odom = self.create_subscription(
            Odometry,
            'wheel_odom',
            self.odom_callback,
            10
        )

        # srv
        reentrant_cb = ReentrantCallbackGroup()
        self.stretch_driver_set_parameters_client = self.create_client(
            SetParameters,
            'stretch_driver/set_parameters',
            callback_group=reentrant_cb
        )

    def _switch_mode(self, mode_name: str) -> SetParameters.Response:
        """
        Switch the Stretch driver mode by sending a parameter update request.

        Args:
            mode_name: The target mode name to switch to.

        Returns:
            The response from the parameter setting service.
        """
        request = SetParameters.Request()
        request.parameters = [Parameter(name='mode', value=ParameterValue(type=ParameterType.PARAMETER_STRING, string_value=mode_name))]
        result_future = self.stretch_driver_set_parameters_client.call_async(request)
        while not result_future.done():
            time.sleep(0.2)
        return result_future.result()

    def odom_callback(self, msg: Odometry) -> None:
        """
        Callback to update the robot base's estimated position and orientation.

        Extracts the position and yaw angle from the odometry message and
        updates the stored stretch joint position.

        Args:
            msg: The odometry message containing the current base pose.
        """
        # Quaternion to Euler yaw (base_theta)
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        base_theta = np.arctan2(siny_cosp, cosy_cosp)

        # Update the base position values in the stored joint position
        self.stretch_joint_position.base_x = msg.pose.pose.position.x
        self.stretch_joint_position.base_y = msg.pose.pose.position.y
        self.stretch_joint_position.base_theta = base_theta

    def joint_states_callback(self, joint_state: JointState) -> None:
        """
        Callback to update the robot's arm, lift, and wrist joint positions.

        Extracts the joint positions from the joint state message and updates the
        stored joint positions.

        Args:
            joint_state: The joint state message containing current joint positions.
        """
        new_joint_position = self._joint_state_to_position(joint_state)
        self.stretch_joint_position.lift = new_joint_position.lift
        self.stretch_joint_position.arm = new_joint_position.arm
        self.stretch_joint_position.wrist_yaw = new_joint_position.wrist_yaw
        self.stretch_joint_position.wrist_pitch = new_joint_position.wrist_pitch
        self.stretch_joint_position.wrist_roll = new_joint_position.wrist_roll

    def _joint_state_to_position(self, joint_state: JointState) -> StretchJointPositions:
        """
        Convert a JointState message to a StretchJointPositions object.

        Args:
            joint_state: The joint state message to convert.

        Returns:
            The converted joint positions.
        """
        joint_dict = dict(zip(joint_state.name, joint_state.position))
        
        # Extract lift
        lift = joint_dict.get('lift_joint', 0.5)
        
        # Extract arm (total arm extension is 5 times arm_l4_joint in the ROS 2 driver)
        if 'arm_l4_joint' in joint_dict:
            arm = joint_dict['arm_l4_joint'] * 5.0
        elif 'arm_joint' in joint_dict:
            arm = joint_dict['arm_joint']
        else:
            arm = 0.0
            
        # Extract wrists
        wrist_yaw = joint_dict.get('wrist_yaw_joint', 0.0)
        wrist_pitch = joint_dict.get('wrist_pitch_joint', 0.0)
        wrist_roll = joint_dict.get('wrist_roll_joint', 0.0)
        
        return StretchJointPositions(
            lift=lift,
            arm=arm,
            wrist_yaw=wrist_yaw,
            wrist_pitch=wrist_pitch,
            wrist_roll=wrist_roll
        )

    def _velocity_command_to_ros2(
        self,
        stretch_joint_velocities: StretchJointVelocities,
        duration: float
    ) -> tuple[JointJog, Twist]:
        """
        Convert a joint velocities object to ROS 2 JointJog and Twist messages.

        Applies the time-step duration to the wrist joint velocities to convert
        them to relative position steps, accommodating the driver's relative position
        control mapping for those joints.

        Args:
            stretch_joint_velocities: The commanded joint velocities.
            duration: The time duration of the command step in seconds.

        Returns:
            A tuple containing the JointJog message for the arm and wrist joints,
            and the Twist message for the mobile base.
        """
        joint_jog = JointJog()
        joint_velocities = [
            stretch_joint_velocities.lift,
            stretch_joint_velocities.arm,
            stretch_joint_velocities.wrist_yaw * duration,
            stretch_joint_velocities.wrist_pitch * duration,
            stretch_joint_velocities.wrist_roll * duration,
        ]
        joint_names = [
            'lift_joint',
            'arm_joint',
            'wrist_yaw_joint',
            'wrist_pitch_joint',
            'wrist_roll_joint',
        ]
        joint_jog.joint_names = joint_names
        joint_jog.velocities = joint_velocities
        joint_jog.duration = duration
        
        base_twist = Twist()
        base_twist.linear.x = stretch_joint_velocities.base_x
        base_twist.linear.y = stretch_joint_velocities.base_y
        base_twist.angular.z = stretch_joint_velocities.base_theta

        return joint_jog, base_twist

    def test_primitive_motions(
        self,
        v: float=0.1,
        move_time: float=3.0,
        hz: float=10.
    ):
        """
        Test primitive motions of the robot.
        
        Args:
            v: The velocity to use for the primitive motions.
            move_time: The time to move for each primitive motion.
            hz: The frequency of the control loop.
        """
        task_space_velocities = np.array(
            [
                [v, 0., 0., 0., 0., 0.],
                [0., v, 0., 0., 0., 0.],
                [0., 0., v, 0., 0., 0.],
            ]
        )
        dt_step = 1.0 / hz
        
        for v_task in task_space_velocities:
            print(f"Target Velocity: {v_task}")
            start_time = time.time()
            while rclpy.ok() and time.time() - start_time < move_time:
                top = time.time()
                q_dot = self.kinematic_model.differential_ik(
                    q=self.stretch_joint_position,
                    target_frame="tool_attachment_site_link",
                    v_desired=v_task
                )
                # print(f"Joint Velocities: {q_dot.to_numpy()}")
                joint_jog, base_twist = self._velocity_command_to_ros2(q_dot, dt_step)
                self.pub_joint_vel.publish(joint_jog)
                self.pub_base_twist.publish(base_twist)
                time.sleep(max(dt_step - (time.time() - top), 0))

        zero_vel = StretchJointVelocities()
        stop_joint_jog, stop_base_twist = self._velocity_command_to_ros2(zero_vel, 0.1)
        self.pub_joint_vel.publish(stop_joint_jog)
        self.pub_base_twist.publish(stop_base_twist)
        time.sleep(0.1)

    def _compute_circle_basis(self, normal: np.ndarray) -> tuple[np.ndarray, np.ndarray] | None:
        """
        Compute orthonormal vectors u (up) and v (tangent) spanning the circle plane.
        """
        normal = np.array(normal, dtype=float)
        normal_norm = np.linalg.norm(normal)
        if normal_norm < 1e-6:
            self.get_logger().error("Normal vector cannot be a zero vector.")
            return None
        n_hat = normal / normal_norm

        # Project global Z onto the plane to find the local "up" direction
        global_z = np.array([0.0, 0.0, 1.0])
        dot_z = np.dot(n_hat, global_z)

        # If the plane is horizontal (normal is vertical), default the "up" direction to global X
        if np.abs(dot_z) > 0.99:
            global_x = np.array([1.0, 0.0, 0.0])
            u = global_x - np.dot(n_hat, global_x) * n_hat
        else:
            u = global_z - dot_z * n_hat
        
        u = u / np.linalg.norm(u)
        v = np.cross(n_hat, u)
        v = v / np.linalg.norm(v)

        return u, v

    def _compute_circular_velocity(
        self,
        radius: float,
        omega: float,
        elapsed: float,
        u: np.ndarray,
        v: np.ndarray
    ) -> np.ndarray:
        """
        Compute target circular velocity in the world/base frame.
        """
        return -radius * omega * np.sin(omega * elapsed) * u + radius * omega * np.cos(omega * elapsed) * v

    def test_circle_motion(
        self,
        radius: float,
        time_to_complete: float,
        normal: np.ndarray,
        hz: float = 10.0
    ) -> None:
        """
        Execute feedforward circular motion of the end-effector.
        Args:
            radius: Radius of the circle in meters.
            time_to_complete: Duration in seconds to complete one full circle.
            normal: A 3-element numpy array representing the normal vector to the circle's plane.
            hz: The frequency of the control loop.
        """
        basis = self._compute_circle_basis(normal)
        if basis is None:
            return
        u, v = basis

        omega = 2.0 * np.pi / time_to_complete
        dt_step = 1.0 / hz

        self.get_logger().info(
            f"Starting circular motion. Radius: {radius}m, Time: {time_to_complete}s, "
            f"Up Vector: {u}, Tangent Vector: {v}"
        )

        start_time = time.time()
        while rclpy.ok():
            top = time.time()
            elapsed = top - start_time
            if elapsed >= time_to_complete:
                break

            # Compute target velocity in the world/base frame
            v_world = self._compute_circular_velocity(radius, omega, elapsed, u, v)

            # Solve joint velocities via differential IK
            q_dot = self.kinematic_model.differential_ik(
                q=self.stretch_joint_position,
                target_frame="tool_attachment_site_link",
                v_desired=v_world
            )

            # Command the robot
            joint_jog, base_twist = self._velocity_command_to_ros2(q_dot, dt_step)
            self.pub_joint_vel.publish(joint_jog)
            self.pub_base_twist.publish(base_twist)

            # Precise rate sleep
            elapsed_loop = time.time() - top
            time.sleep(max(dt_step - elapsed_loop, 0.0))

        # Stop the robot when the circle is completed
        zero_vel = StretchJointVelocities()
        stop_joint_jog, stop_base_twist = self._velocity_command_to_ros2(zero_vel, 0.1)
        self.pub_joint_vel.publish(stop_joint_jog)
        self.pub_base_twist.publish(stop_base_twist)
        self.get_logger().info("Circular motion test completed.")

    def main(self) -> None:
        """
        Execute the main control loop to command the robot through a sequence of task-space velocities.

        Iterates through the target task-space velocities, computing the joint
        velocities using differential IK and publishing them at the target control rate.
        """
        self._switch_mode("velocity")
        # v = 0.1
        # dt = 3.0
        # hz = 10.
        # self.test_primitive_motions(v, dt, hz)

        radius = 0.2
        time_to_complete = 10.0
        normal = np.array([0.0, 0.0, 1.0])
        hz = 10.
        self.test_circle_motion(radius, time_to_complete, normal, hz)
        

def main(args: list[str] | None = None) -> None:
    """
    Main entry point for running the flying gripper node.

    Args:
        args: Optional command-line arguments passed to rclpy initialization.
    """
    rclpy.init(args=args)
    node = FlyingGripper()
    
    # Spin node in a background thread to receive joint states and odometry updates
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()
    
    # main
    node.main()
    
    # Clean up
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()