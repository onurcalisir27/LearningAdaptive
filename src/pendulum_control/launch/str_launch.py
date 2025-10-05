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
    u_bound_arg = DeclareLaunchArgument(
        'u_bound',
        default_value='3.0',
        description='Control Input bound for the controller'
    )

    lambda_ = LaunchConfiguration('lambda')
    desired_angle = LaunchConfiguration('desired_angle')
    u_bound = LaunchConfiguration('u_bound')

    str_node = Node(
        package='pendulum_control',
        executable='pendulum_control_node',
        output='screen',
        parameters=[{
            'lambda': lambda_,
            'desired_angle': desired_angle,
            'u_bound': u_bound,
            'use_sim_time': True
        }]
    )

    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        # arguments=['-d', rviz_config],
    )

    delay_str = TimerAction(
        period=3.0,
        actions=[forgetting_factor_arg, desired_angle_arg,
                 u_bound_arg, str_node]
    )

    ld = LaunchDescription()
    ld.add_action(delay_str)
    ld.add_action(rviz2)
    return ld
