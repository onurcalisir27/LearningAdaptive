from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():

    forgetting_factor_arg = DeclareLaunchArgument(
        'lambda',
        default_value='0.999',
        description='Forgetting Factor for Self Tuning Regulator'
    )

    desired_angle1_arg = DeclareLaunchArgument(
        'desired1',
        default_value='0.0',
        description='Desired angle for the first pendulum link'
    )

    desired_angle2_arg = DeclareLaunchArgument(
        'desired2',
        default_value='0.0',
        description='Desired angle for the second pendulum link'
    )

    u1_bound_arg = DeclareLaunchArgument(
        'u1_bound',
        default_value='10.0',
        description='Control Bound for Joint 1'
    )

    u2_bound_arg = DeclareLaunchArgument(
        'u2_bound',
        default_value='5.0',
        description='Control Bound For Joint 2'
    )

    str_node = Node(
        package='pendulum_ctrl',
        executable='two_link_control_node',
        output='screen',
        parameters=[{
            'lambda': LaunchConfiguration('lambda'),
            'desired1': LaunchConfiguration('desired1'),
            'desired2': LaunchConfiguration('desired2'),
            'u1_bound': LaunchConfiguration('u1_bound'),
            'u2_bound': LaunchConfiguration('u2_bound'),
            'use_sim_time': True
        }]
    )

    ld = LaunchDescription()
    ld.add_action(forgetting_factor_arg)
    ld.add_action(desired_angle1_arg)
    ld.add_action(desired_angle2_arg)
    ld.add_action(u1_bound_arg)
    ld.add_action(u2_bound_arg)
    ld.add_action(str_node)
    return ld
