from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    forgetting_factor_arg = DeclareLaunchArgument(
        'lambda',
        default_value='0.98',
        description='Forgetting Factor for Self Tuning Regulator'
    )
    desired_angle_arg = DeclareLaunchArgument(
        'desired_angle',
        default_value='0.0',
        description='Desired angle for the pendulum to stabilize on'
    )
    u1_bound_arg = DeclareLaunchArgument(
        'u1_bound',
        default_value='6.0',
        description='Control Bound for Joint 1'
    )
    u2_bound_arg = DeclareLaunchArgument(
        'u2_bound',
        default_value='3.0',
        description='Control Bound For Joint 2'
    )

    str_node = Node(
        package='pendulum_control',
        executable='two_link_control_node',
        output='screen',
        parameters=[{
            'lambda': LaunchConfiguration('lambda'),
            'desired_angle': LaunchConfiguration('desired_angle'),
            'u1_bound': LaunchConfiguration('u1_bound'),
            'u2_bound': LaunchConfiguration('u2_bound'),
            'use_sim_time': True
        }]
    )
    delay_str = TimerAction(
        period=1.0,
        actions=[forgetting_factor_arg, desired_angle_arg,
                 u1_bound_arg, u2_bound_arg, str_node]
    )
    ld = LaunchDescription()
    ld.add_action(delay_str)
    return ld
