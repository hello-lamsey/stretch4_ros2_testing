#!/usr/bin/env python3
"""
Nullspace base obstacle avoidance demo for Stretch 4.

Controls the Stretch 4 robot to avoid obstacles detected by the local costmap
using an artificial potential field linear repulsion while projecting secondary
base velocities into the nullspace of a primary end-effector position-holding task.
"""

import argparse
import sys
import threading
import time

from control_msgs.msg import JointJog
from geometry_msgs.msg import Point, Twist
from nav_msgs.msg import OccupancyGrid, Odometry
import numpy as np
from rcl_interfaces.msg import ParameterDescriptor, SetParametersResult
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from visualization_msgs.msg import Marker
from stretch4_kinematics.kinematic_models.tool_frame_kinematics import ToolFrameKinematics
from stretch4_kinematics.state.joint_positions import StretchJointPositions
from stretch4_kinematics.state.joint_velocities import StretchJointVelocities
import tf2_ros


class NullspaceBaseObstacleAvoidanceNode(Node):
    """
    ROS 2 node for nullspace base velocity control repelling from costmap obstacles.

    Subscribes to:
    - /local_costmap/costmap (nav_msgs/OccupancyGrid): Obstacle local costmap.
    - /joint_states (sensor_msgs/JointState): Joint positions for kinematics.
    - /wheel_odom (nav_msgs/Odometry): Base position and yaw.

    Publishes:
    - /joint_vel (control_msgs/JointJog): Arm/lift/wrist velocity commands.
    - /cmd_vel (geometry_msgs/Twist): Mobile base linear velocity commands.
    """

    def __init__(self, numerical: bool = True) -> None:
        """
        Initialize the obstacle avoidance node, subscribers, publishers, and timer.

        Args:
            numerical: When True, runs in dry-run mode printing computed velocities
                without publishing motion commands to the physical robot.
        """
        super().__init__('nullspace_base_obstacle_avoidance')

        self.numerical = numerical
        self.lock = threading.Lock()

        # Kinematics solver
        self.kinematics = ToolFrameKinematics()
        self.current_joint_pos = StretchJointPositions()

        # Flag indicating whether we have received initial joint states and odometry
        self.has_joint_states = False
        self.has_odom = False
        self.has_costmap = False

        # Parameters
        self.declare_parameter('repulsion_margin_in', 12.0)   # Repulsion band outward from base (12 in)
        self.declare_parameter('robot_radius_m', 0.22)       # Physical robot base footprint radius (0.22 m)
        self.declare_parameter('max_repulsion_speed', 0.15)  # m/s base speed limit
        self.declare_parameter('control_rate_hz', 20.0)
        self.declare_parameter('occupied_threshold', 50)     # Costmap threshold [0-100]

        self.robot_radius = float(self.get_parameter('robot_radius_m').value)
        margin_in = float(self.get_parameter('repulsion_margin_in').value)
        # Margin converted from inches to meters (6 in = 0.1524 m)
        self.repulsion_margin = margin_in * 0.0254
        # Outer detection radius = base radius + repulsion margin (0.22 + 0.1524 = 0.3724 m)
        self.repulsion_radius = self.robot_radius + self.repulsion_margin

        self.max_repulsion_speed = float(self.get_parameter('max_repulsion_speed').value)
        self.control_rate = float(self.get_parameter('control_rate_hz').value)
        self.occupied_threshold = int(self.get_parameter('occupied_threshold').value)

        # Costmap cache
        self.latest_costmap: OccupancyGrid | None = None

        # TF2 buffer and listener for transform lookups
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Publishers
        self.pub_joint_vel = self.create_publisher(JointJog, 'joint_vel', 10)
        self.pub_base_twist = self.create_publisher(Twist, 'cmd_vel', 10)
        self.pub_desired_vel_marker = self.create_publisher(
            Marker, '/nullspace_avoidance/desired_velocity_marker', 10
        )
        self.pub_cmd_vel_marker = self.create_publisher(
            Marker, '/nullspace_avoidance/cmd_vel_marker', 10
        )
        self.pub_boundary_marker = self.create_publisher(
            Marker, '/nullspace_avoidance/repulsion_boundary_marker', 10
        )

        # Subscribers
        self.sub_costmap = self.create_subscription(
            OccupancyGrid,
            '/local_costmap/costmap',
            self.costmap_callback,
            10,
        )

        self.sub_joint_state = self.create_subscription(
            JointState,
            'joint_states',
            self.joint_states_callback,
            10,
        )

        self.sub_wheel_odom = self.create_subscription(
            Odometry,
            'wheel_odom',
            self.odom_callback,
            10,
        )

        # Control loop timer
        timer_period = 1.0 / self.control_rate
        self.timer = self.create_timer(timer_period, self.control_loop_callback)

        # Dynamic parameter update callback
        self.add_on_set_parameters_callback(self.parameters_callback)

        mode_str = "NUMERICAL DRY-RUN (no robot movement)" if self.numerical else "ACTIVE MOTION"
        self.get_logger().info(
            f"NullspaceBaseObstacleAvoidanceNode initialized in {mode_str}.\n"
            f"  Robot Radius: {self.robot_radius:.3f} m\n"
            f"  Repulsion Margin: {margin_in:.1f} in ({self.repulsion_margin:.3f} m)\n"
            f"  Total Outer Radius: {self.repulsion_radius:.3f} m\n"
            f"  Max Repulsion Speed: {self.max_repulsion_speed:.2f} m/s\n"
            f"  Control Rate: {self.control_rate:.1f} Hz\n"
            f"  Costmap Occupancy Threshold: {self.occupied_threshold}"
        )

    def parameters_callback(self, params: list[rclpy.parameter.Parameter]) -> SetParametersResult:
        """
        Handle dynamic ROS 2 parameter updates.

        Args:
            params: List of updated parameters.

        Returns:
            SetParametersResult indicating success or failure.
        """
        with self.lock:
            for param in params:
                if param.name == 'repulsion_margin_in':
                    margin_in = float(param.value)
                    self.repulsion_margin = margin_in * 0.0254
                    self.repulsion_radius = self.robot_radius + self.repulsion_margin
                    self.get_logger().info(
                        f"Updated repulsion_margin_in: {margin_in:.1f} in. "
                        f"New outer radius: {self.repulsion_radius:.3f} m"
                    )
                elif param.name == 'robot_radius_m':
                    self.robot_radius = float(param.value)
                    self.repulsion_radius = self.robot_radius + self.repulsion_margin
                    self.get_logger().info(
                        f"Updated robot_radius_m: {self.robot_radius:.3f} m. "
                        f"New outer radius: {self.repulsion_radius:.3f} m"
                    )
                elif param.name == 'max_repulsion_speed':
                    self.max_repulsion_speed = float(param.value)
                    self.get_logger().info(f"Updated max_repulsion_speed: {self.max_repulsion_speed:.2f} m/s")
                elif param.name == 'occupied_threshold':
                    self.occupied_threshold = int(param.value)
                    self.get_logger().info(f"Updated occupied_threshold: {self.occupied_threshold}")

        return SetParametersResult(successful=True)

    def costmap_callback(self, msg: OccupancyGrid) -> None:
        """
        Cache incoming OccupancyGrid local costmap message.

        Args:
            msg: The OccupancyGrid message containing costmap data.
        """
        with self.lock:
            self.latest_costmap = msg
            self.has_costmap = True

    def odom_callback(self, msg: Odometry) -> None:
        """
        Update base position and orientation from wheel odometry.

        Args:
            msg: The odometry message containing base pose.
        """
        # Quaternion to Euler yaw (base_theta)
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        base_theta = float(np.arctan2(siny_cosp, cosy_cosp))

        with self.lock:
            self.current_joint_pos.base_x = msg.pose.pose.position.x
            self.current_joint_pos.base_y = msg.pose.pose.position.y
            self.current_joint_pos.base_theta = base_theta
            self.has_odom = True

    def joint_states_callback(self, joint_state: JointState) -> None:
        """
        Extract lift, arm, and wrist joint positions from JointState message.

        Args:
            joint_state: JointState message containing current robot joint positions.
        """
        joint_dict = dict(zip(joint_state.name, joint_state.position))

        lift = joint_dict.get('lift_joint', 0.5)
        if 'arm_l4_joint' in joint_dict:
            arm = joint_dict['arm_l4_joint'] * 5.0
        elif 'arm_joint' in joint_dict:
            arm = joint_dict['arm_joint']
        else:
            arm = 0.0

        wrist_yaw = joint_dict.get('wrist_yaw_joint', 0.0)
        wrist_pitch = joint_dict.get('wrist_pitch_joint', 0.0)
        wrist_roll = joint_dict.get('wrist_roll_joint', 0.0)

        with self.lock:
            self.current_joint_pos.lift = lift
            self.current_joint_pos.arm = arm
            self.current_joint_pos.wrist_yaw = wrist_yaw
            self.current_joint_pos.wrist_pitch = wrist_pitch
            self.current_joint_pos.wrist_roll = wrist_roll
            self.has_joint_states = True

    def compute_artificial_potential_field_repulsion(
        self,
        costmap: OccupancyGrid,
    ) -> tuple[np.ndarray, int]:
        """
        Compute the net repulsive velocity vector using a 16-bin sector histogram.

        To eliminate self-detection of the robot's arm/wrist and avoid cell-summing bias:
        1. Evaluates obstacles within the 1-foot repulsion radius in the robot's base frame.
        2. Filters out candidate obstacles in a 45-degree wedge centered along the robot's
           arm direction (+X axis in robot frame, from -22.5 deg to +22.5 deg) where the
           telescoping arm and wrist are located.
        3. Discretizes the remaining surroundings into 16 radial angular bins (22.5 deg each).
        4. Identifies the single closest obstacle point in each bin (closest-obstacle heuristic),
           preventing walls with dozens of cells from overpowering isolated nearby hazards.
        5. Computes a linear repulsion gain [0, 1] on that nearest point for each active bin:
             gain_k = (repulsion_radius - d_min_k) / repulsion_radius
             f_k = gain_k * u_repulsive_k
        6. Combines the sector forces and clamps the resultant speed to max_repulsion_speed,
           transforming the net vector into the world/costmap frame for nullspace control.

        Args:
            costmap: OccupancyGrid message from the local costmap.

        Returns:
            A tuple of:
            - np.ndarray: [v_base_world_x, v_base_world_y] in m/s.
            - int: Count of active obstacle cells within the repulsion radius.
        """
        costmap_frame = costmap.header.frame_id
        robot_frame = 'base_footprint'

        # Query TF to find position and orientation of base_footprint in costmap frame
        try:
            tf_msg = self.tf_buffer.lookup_transform(
                costmap_frame,
                robot_frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.05),
            )
            rx = tf_msg.transform.translation.x
            ry = tf_msg.transform.translation.y
            q = tf_msg.transform.rotation
            siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
            cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
            base_yaw = float(np.arctan2(siny_cosp, cosy_cosp))
        except Exception:
            # Fallback to wheel odom cached coordinates if TF lookup fails
            rx = self.current_joint_pos.base_x
            ry = self.current_joint_pos.base_y
            base_yaw = self.current_joint_pos.base_theta

        info = costmap.info
        res = info.resolution
        origin_x = info.origin.position.x
        origin_y = info.origin.position.y
        width = info.width
        height = info.height

        robot_col = int((rx - origin_x) / res)
        robot_row = int((ry - origin_y) / res)

        cell_radius = int(np.ceil(self.repulsion_radius / res))
        col_min = max(0, robot_col - cell_radius)
        col_max = min(width - 1, robot_col + cell_radius)
        row_min = max(0, robot_row - cell_radius)
        row_max = min(height - 1, robot_row + cell_radius)

        data = np.array(costmap.data, dtype=np.int8).reshape((height, width))

        # Setup 16 radial bins covering [-pi, pi] (each bin is 22.5 degrees = pi/8 rad)
        num_bins = 16
        bin_closest_dist = np.full(num_bins, np.inf, dtype=float)
        # Store unit repulsive vector (pointing away from obstacle in robot frame) for each bin
        bin_repulse_dir = np.zeros((num_bins, 2), dtype=float)

        # 45-degree self-exclusion slice centered on robot's +X axis (where arm/wrist telescope)
        # Angle relative to robot frame in [-pi, pi]. Arm lies along +X axis (theta ~ 0).
        # Exclusion half-angle: 22.5 degrees (pi / 8 radians), covering [-pi/8, +pi/8].
        exclusion_half_angle = np.pi / 8.0

        obstacle_count = 0
        cos_yaw = np.cos(base_yaw)
        sin_yaw = np.sin(base_yaw)

        for r in range(row_min, row_max + 1):
            for c in range(col_min, col_max + 1):
                cost = data[r, c]
                if cost >= self.occupied_threshold:
                    ox = origin_x + (c + 0.5) * res
                    oy = origin_y + (r + 0.5) * res

                    # Vector in world/costmap frame from robot to obstacle
                    dx_world = ox - rx
                    dy_world = oy - ry
                    dist = float(np.hypot(dx_world, dy_world))

                    # Consider obstacles within the outer repulsion radius.
                    # Ignore hits strictly inside the chassis boundary (dist < robot_radius - 0.05)
                    # to prevent internal noise or sensor self-reflections from triggering.
                    if (self.robot_radius - 0.05) <= dist <= self.repulsion_radius:
                        # Transform obstacle offset vector into robot base frame
                        # x_robot = cos(yaw)*dx + sin(yaw)*dy
                        # y_robot = -sin(yaw)*dx + cos(yaw)*dy
                        dx_robot = cos_yaw * dx_world + sin_yaw * dy_world
                        dy_robot = -sin_yaw * dx_world + cos_yaw * dy_world

                        # Obstacle angle in robot base frame: theta in [-pi, pi]
                        angle_robot = float(np.arctan2(dy_robot, dx_robot))

                        # Check if obstacle falls inside the 45-degree arm self-exclusion slice
                        if abs(angle_robot) < exclusion_half_angle:
                            # Skip self-occluded hits on robot's arm/wrist
                            continue

                        obstacle_count += 1

                        # Map angle_robot in [-pi, pi] to bin index [0, num_bins - 1]
                        # Bin width is (2*pi / num_bins)
                        bin_idx = int(np.floor((angle_robot + np.pi) / (2.0 * np.pi / num_bins)))
                        bin_idx = min(num_bins - 1, max(0, bin_idx))

                        # Closest-obstacle heuristic: update minimum distance for this sector
                        if dist < bin_closest_dist[bin_idx]:
                            bin_closest_dist[bin_idx] = dist
                            # Repulsion direction points FROM obstacle TO robot in robot frame
                            bin_repulse_dir[bin_idx, 0] = -dx_robot / dist
                            bin_repulse_dir[bin_idx, 1] = -dy_robot / dist

        # Accumulate linear repulsion forces across all active bins
        f_repulsive_robot = np.zeros(2, dtype=float)
        active_bins = 0

        for b in range(num_bins):
            d_min = bin_closest_dist[b]
            if d_min <= self.repulsion_radius:
                # Linear gain in [0, 1] across the repulsion margin:
                # 0.0 at d_min == repulsion_radius (outer boundary)
                # 1.0 at d_min <= robot_radius (perimeter boundary)
                effective_margin = max(0.0, min(self.repulsion_margin, self.repulsion_radius - d_min))
                gain = effective_margin / self.repulsion_margin
                f_repulsive_robot[0] += gain * bin_repulse_dir[b, 0]
                f_repulsive_robot[1] += gain * bin_repulse_dir[b, 1]
                active_bins += 1

        # Transform repulsive vector from robot base frame back to world/costmap frame
        force_norm = float(np.linalg.norm(f_repulsive_robot))
        if force_norm > 1e-4:
            # Desired base speed bounded by max_repulsion_speed
            # Clamped based on the strongest sector gain
            max_gain = float(np.max([
                max(0.0, min(self.repulsion_margin, self.repulsion_radius - d)) / self.repulsion_margin
                for d in bin_closest_dist if d <= self.repulsion_radius
            ]))
            v_speed = min(self.max_repulsion_speed, self.max_repulsion_speed * max_gain)

            unit_dir_robot = f_repulsive_robot / force_norm
            # Rotate robot frame vector to world/costmap frame
            v_world_x = cos_yaw * unit_dir_robot[0] - sin_yaw * unit_dir_robot[1]
            v_world_y = sin_yaw * unit_dir_robot[0] + cos_yaw * unit_dir_robot[1]
            v_base_world_des = np.array([v_world_x, v_world_y], dtype=float) * v_speed
        else:
            v_base_world_des = np.zeros(2, dtype=float)

        return v_base_world_des, obstacle_count

    def control_loop_callback(self) -> None:
        """
        Execute one control cycle: compute APF repulsion and nullspace joint velocities.
        """
        with self.lock:
            ready = self.has_joint_states and self.has_odom and self.has_costmap
            if not ready:
                return
            q_curr = StretchJointPositions(
                base_x=self.current_joint_pos.base_x,
                base_y=self.current_joint_pos.base_y,
                base_theta=self.current_joint_pos.base_theta,
                lift=self.current_joint_pos.lift,
                arm=self.current_joint_pos.arm,
                wrist_yaw=self.current_joint_pos.wrist_yaw,
                wrist_pitch=self.current_joint_pos.wrist_pitch,
                wrist_roll=self.current_joint_pos.wrist_roll,
            )
            costmap = self.latest_costmap

        if costmap is None:
            return

        # 1. Compute artificial potential field repulsion velocity in world frame
        v_base_world_des, obstacle_count = self.compute_artificial_potential_field_repulsion(costmap)

        # 2. Nullspace base velocity control:
        # If no obstacle is within the repulsion radius, output exact zero velocities
        if obstacle_count == 0 or np.linalg.norm(v_base_world_des) < 1e-4:
            dq_cmd = StretchJointVelocities()
            v_base_world_des = np.zeros(2, dtype=float)
        else:
            # Primary task: Keep end-effector stationary in local tool frame (v_ee_local_des = zeros(6))
            # Secondary task: Drive mobile base in world XY plane via nullspace projector P1
            v_ee_local_des = np.zeros(6, dtype=float)
            dq_cmd = self.kinematics.nullspace_base_control(
                q=q_curr,
                v_ee_local_des=v_ee_local_des,
                v_base_world_des=v_base_world_des,
            )

        dt_step = 1.0 / self.control_rate

        # 3. Output handling (numerical probing vs live hardware publishing)
        if self.numerical:
            # Print probe telemetry on a single line with carriage return
            status = (
                f"[NUMERICAL] Obs: {obstacle_count:3d} | "
                f"V_world: [{v_base_world_des[0]:+.2f}, {v_base_world_des[1]:+.2f}] m/s | "
                f"Base(x,y,th): [{dq_cmd.base_x:+.2f}, {dq_cmd.base_y:+.2f}, {dq_cmd.base_theta:+.2f}] | "
                f"Arm/Lift: [{dq_cmd.arm:+.3f}, {dq_cmd.lift:+.3f}] | "
                f"Wrists: [{dq_cmd.wrist_yaw:+.2f}, {dq_cmd.wrist_pitch:+.2f}, {dq_cmd.wrist_roll:+.2f}]"
            )
            sys.stdout.write(f"\r{status}")
            sys.stdout.flush()
        else:
            # Active robot motion: publish JointJog and Twist
            joint_jog = JointJog()
            joint_jog.joint_names = [
                'lift_joint',
                'arm_joint',
                'wrist_yaw_joint',
                'wrist_pitch_joint',
                'wrist_roll_joint',
            ]
            joint_jog.velocities = [
                dq_cmd.lift,
                dq_cmd.arm,
                dq_cmd.wrist_yaw,
                dq_cmd.wrist_pitch,
                dq_cmd.wrist_roll,
            ]
            joint_jog.duration = dt_step

            base_twist = Twist()
            base_twist.linear.x = dq_cmd.base_x
            base_twist.linear.y = dq_cmd.base_y
            base_twist.angular.z = dq_cmd.base_theta

            self.pub_joint_vel.publish(joint_jog)
            self.pub_base_twist.publish(base_twist)

        # 4. Always publish RViz visual markers (in both numerical and active modes)
        self.publish_velocity_markers(
            v_base_world_des=v_base_world_des,
            dq_cmd=dq_cmd,
            base_pos=q_curr,
        )

    def publish_velocity_markers(
        self,
        v_base_world_des: np.ndarray,
        dq_cmd: StretchJointVelocities,
        base_pos: StretchJointPositions,
    ) -> None:
        """
        Publish RViz visualization markers for desired world velocity and commanded velocity.

        Args:
            v_base_world_des: Desired 2D base velocity in world frame [vx, vy].
            dq_cmd: Computed 8-DOF velocities containing base_x, base_y, base_theta.
            base_pos: Current robot base position and orientation.
        """
        now = self.get_clock().now().to_msg()

        # Marker 1: Desired world-frame repulsion velocity vector (cyan arrow anchored at base)
        m_world = Marker()
        m_world.header.frame_id = 'odom'
        m_world.header.stamp = now
        m_world.ns = 'desired_world_vel'
        m_world.id = 0
        m_world.type = Marker.ARROW
        m_world.action = Marker.ADD

        # Start point at current base footprint position
        p_start = Point(x=base_pos.base_x, y=base_pos.base_y, z=0.15)
        # Scale arrow visually: 1.0 m length per 0.1 m/s (scale factor = 3.0)
        scale_fac = 3.0
        p_end = Point(
            x=base_pos.base_x + float(v_base_world_des[0]) * scale_fac,
            y=base_pos.base_y + float(v_base_world_des[1]) * scale_fac,
            z=0.15,
        )
        m_world.points = [p_start, p_end]
        m_world.scale.x = 0.04  # Shaft diameter
        m_world.scale.y = 0.08  # Head diameter
        m_world.scale.z = 0.10  # Head length
        # Cyan color for desired APF vector
        m_world.color.r = 0.0
        m_world.color.g = 1.0
        m_world.color.b = 1.0
        m_world.color.a = 0.9 if np.linalg.norm(v_base_world_des) > 1e-4 else 0.0

        self.pub_desired_vel_marker.publish(m_world)

        # Marker 2: Commanded base velocity vector in robot frame (yellow arrow)
        m_cmd = Marker()
        m_cmd.header.frame_id = 'base_footprint'
        m_cmd.header.stamp = now
        m_cmd.ns = 'cmd_base_vel'
        m_cmd.id = 1
        m_cmd.type = Marker.ARROW
        m_cmd.action = Marker.ADD

        p_cmd_start = Point(x=0.0, y=0.0, z=0.20)
        p_cmd_end = Point(
            x=float(dq_cmd.base_x) * scale_fac,
            y=float(dq_cmd.base_y) * scale_fac,
            z=0.20,
        )
        m_cmd.points = [p_cmd_start, p_cmd_end]
        m_cmd.scale.x = 0.04
        m_cmd.scale.y = 0.08
        m_cmd.scale.z = 0.10
        # Yellow color for commanded base velocity
        m_cmd.color.r = 1.0
        m_cmd.color.g = 0.9
        m_cmd.color.b = 0.0
        m_cmd.color.a = 0.9 if (abs(dq_cmd.base_x) > 1e-4 or abs(dq_cmd.base_y) > 1e-4) else 0.0

        self.pub_cmd_vel_marker.publish(m_cmd)

        # Marker 3: Repulsion boundary visual ring (cylinder) centered on base_footprint
        m_bound = Marker()
        m_bound.header.frame_id = 'base_footprint'
        m_bound.header.stamp = now
        m_bound.ns = 'repulsion_boundary'
        m_bound.id = 2
        m_bound.type = Marker.CYLINDER
        m_bound.action = Marker.ADD
        m_bound.pose.position.x = 0.0
        m_bound.pose.position.y = 0.0
        m_bound.pose.position.z = 0.02
        m_bound.pose.orientation.w = 1.0
        # Diameter = 2 * repulsion_radius
        m_bound.scale.x = 2.0 * self.repulsion_radius
        m_bound.scale.y = 2.0 * self.repulsion_radius
        m_bound.scale.z = 0.02
        # Transparent green/cyan ring indicating the 6-inch repulsion envelope
        m_bound.color.r = 0.2
        m_bound.color.g = 0.8
        m_bound.color.b = 0.5
        m_bound.color.a = 0.25

        self.pub_boundary_marker.publish(m_bound)


def main(args: list[str] | None = None) -> None:
    """
    Entry point for the nullspace base obstacle avoidance node.

    Parses CLI flags (--numerical / --active) and executes the ROS 2 node.

    Args:
        args: Command-line arguments passed into the script.
    """
    # Filter ROS arguments from sys.argv
    parser = argparse.ArgumentParser(description="Stretch 4 Nullspace Base Obstacle Avoidance Demo")
    parser.add_argument(
        '--numerical',
        action='store_true',
        default=True,
        help="Run in dry-run numerical mode without sending motion commands to the physical robot (default: True).",
    )
    parser.add_argument(
        '--active',
        action='store_true',
        help="Enable active robot motion commands. WARNING: Causes physical robot movement.",
    )

    parsed_args, remaining_ros_args = parser.parse_known_args(args=args if args is not None else sys.argv[1:])

    # If --active is set, disable numerical dry-run
    run_numerical = not parsed_args.active if parsed_args.active else parsed_args.numerical

    # Reconstruct sys.argv for ROS 2 init
    ros_argv = [sys.argv[0]] + remaining_ros_args
    rclpy.init(args=ros_argv)

    node = NullspaceBaseObstacleAvoidanceNode(numerical=run_numerical)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Publish zero velocities to halt robot when stopped
        if not run_numerical and rclpy.ok():
            stop_jog = JointJog()
            stop_jog.joint_names = [
                'lift_joint',
                'arm_joint',
                'wrist_yaw_joint',
                'wrist_pitch_joint',
                'wrist_roll_joint',
            ]
            stop_jog.velocities = [0.0, 0.0, 0.0, 0.0, 0.0]
            stop_twist = Twist()
            node.pub_joint_vel.publish(stop_jog)
            node.pub_base_twist.publish(stop_twist)

        sys.stdout.write("\nShutting down NullspaceBaseObstacleAvoidanceNode.\n")
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
