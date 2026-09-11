from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    max_ee_speed_arg = DeclareLaunchArgument(
        'max_ee_speed',
        default_value='0.1',
        description='Maximum allowed end-effector linear speed in m/s'
    )
    angular_speed_arg = DeclareLaunchArgument(
        'angular_speed',
        default_value='0.25',
        description='Oscillating base rotation angular speed in rad/s'
    )
    switch_period_arg = DeclareLaunchArgument(
        'switch_period',
        default_value='3.0',
        description='Period in seconds between base rotation direction flips'
    )
    target_frame_arg = DeclareLaunchArgument(
        'target_frame',
        default_value='tool_attachment_site_link',
        description='End-effector target frame for velocity calculation'
    )

    safety_filter_node = Node(
        package='stretch4_ros2_testing',
        executable='ee_velocity_safety_filter',
        name='ee_velocity_safety_filter',
        output='screen',
        parameters=[{
            'max_ee_speed': LaunchConfiguration('max_ee_speed'),
            'target_frame': LaunchConfiguration('target_frame'),
            'input_cmd_vel_topic': '/teleop/cmd_vel',
            'output_cmd_vel_topic': '/cmd_vel',
            'input_joint_vel_topic': '/teleop/joint_vel',
            'output_joint_vel_topic': '/joint_vel',
            'joint_states_topic': '/joint_states',
        }]
    )

    base_oscillator_node = Node(
        package='stretch4_ros2_testing',
        executable='test_base_oscillator',
        name='test_base_oscillator',
        output='screen',
        parameters=[{
            'angular_speed': LaunchConfiguration('angular_speed'),
            'switch_period': LaunchConfiguration('switch_period'),
            'publish_rate': 20.0,
            'cmd_vel_topic': '/teleop/cmd_vel',
        }]
    )

    return LaunchDescription([
        max_ee_speed_arg,
        angular_speed_arg,
        switch_period_arg,
        target_frame_arg,
        safety_filter_node,
        base_oscillator_node,
    ])
