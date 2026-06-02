from ament_index_python.packages import get_package_share_path

from launch_ros.actions import Node

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
)
from launch.launch_description_sources import (
    FrontendLaunchDescriptionSource,
    PythonLaunchDescriptionSource,
)
from launch.substitutions import (
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)

core_package = str(get_package_share_path("stretch_core"))

def generate_launch_description() -> LaunchDescription:
    ld = LaunchDescription()

    # Stretch Driver
    stretch_driver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([core_package, "launch", "stretch_driver.launch.py"])
        ),
        # TODO: The tablet_placement code should change the mode, not the launch file
        launch_arguments={
            "mode": "velocity",
            "broadcast_odom_tf": "True",
            "fail_out_of_range_goal": "False",
            "log_level": "info",
        }.items(),
    )
    ld.add_action(stretch_driver_launch)

    # testing node
    move_joints_node = Node(
        package="stretch4_ros2_testing",
        executable="move_joints",
        name="move_joints",
    )
    ld.add_action(move_joints_node)

    return ld
