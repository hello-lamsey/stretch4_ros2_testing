import os

from ament_index_python.packages import get_package_share_path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
import launch.conditions
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node

def generate_launch_description() -> LaunchDescription:
    """
    Generate launch description for Stretch 4 driver and standalone local costmap.

    Launches:
    - stretch_driver with velocity mode and wheel_odom tf broadcast.
    - nav2_costmap_2d configured for local costmap only.
    - nav2_lifecycle_manager to configure and activate the local costmap node.

    Returns:
        LaunchDescription: The launch description object.
    """
    pkg_testing = str(get_package_share_path('stretch4_ros2_testing'))
    pkg_stretch_core = str(get_package_share_path('stretch_core'))

    # Launch configuration variables
    params_file = LaunchConfiguration('params_file')
    autostart = LaunchConfiguration('autostart')
    log_level = LaunchConfiguration('log_level')

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(pkg_testing, 'config', 'local_costmap_params.yaml'),
        description='Full path to local costmap ROS 2 parameters file',
    )

    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart',
        default_value='true',
        description='Automatically configure and activate the costmap lifecycle node',
    )

    declare_log_level_cmd = DeclareLaunchArgument(
        'log_level',
        default_value='info',
        description='Log level for ROS 2 nodes',
    )

    # 1. Stretch Driver Launch
    stretch_driver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_stretch_core, 'launch', 'stretch_driver.launch.py'])
        ),
        launch_arguments={
            'mode': 'velocity',
            'broadcast_odom_tf': 'True',
            'fail_out_of_range_goal': 'False',
            'log_level': log_level,
        }.items(),
    )

    # 2. Dual Hesai LiDAR Launch (publishes /scan_filtered)
    dual_hesai_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_stretch_core, 'launch', 'dual_hesai.launch.py'])
        ),
        launch_arguments={
            'filter_type': 'sor_ransac',
            'use_rviz': 'true',
        }.items(),
    )

    # 3. Static transform: odom -> wheel_odom
    # Maps the global odom frame to Stretch's wheel_odom TF tree
    odom_to_wheel_odom_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='odom_to_wheel_odom_broadcaster',
        arguments=['--x', '0', '--y', '0', '--z', '0',
                   '--yaw', '0', '--pitch', '0', '--roll', '0',
                   '--frame-id', 'odom', '--child-frame-id', 'wheel_odom'],
    )

    # 4. Standalone Local Costmap Node (Lifecycle Node)
    local_costmap_node = Node(
        package='nav2_costmap_2d',
        executable='nav2_costmap_2d',
        name='local_costmap',
        namespace='local_costmap',
        output='screen',
        parameters=[params_file],
        arguments=['--ros-args', '--log-level', log_level],
    )

    # 5. Nav2 Lifecycle Manager for the local costmap
    lifecycle_manager_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_costmap',
        namespace='local_costmap',
        output='screen',
        arguments=['--ros-args', '--log-level', log_level],
        parameters=[
            {'autostart': autostart},
            {'node_names': ['local_costmap']},
            {'bond_timeout': 30.0},
        ],
    )

    use_rviz = LaunchConfiguration('use_rviz')

    declare_use_rviz_cmd = DeclareLaunchArgument(
        'use_rviz',
        default_value='false',
        description='Whether to launch RViz2 for costmap and velocity visualization',
    )

    # 6. RViz2 node with custom costmap and velocity marker configuration
    rviz_config_file = os.path.join(pkg_testing, 'config', 'nullspace_avoidance.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        condition=launch.conditions.IfCondition(use_rviz),
    )

    ld = LaunchDescription()
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_log_level_cmd)
    ld.add_action(declare_use_rviz_cmd)
    ld.add_action(stretch_driver_launch)
    ld.add_action(dual_hesai_launch)
    ld.add_action(odom_to_wheel_odom_tf)
    ld.add_action(local_costmap_node)
    ld.add_action(lifecycle_manager_node)
    ld.add_action(rviz_node)

    return ld
