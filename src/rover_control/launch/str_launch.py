from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
def generate_launch_description():
    p_controller_arg = DeclareLaunchArgument(
        'kp',
        default_value='0.0',
        description='Proportional Gain'
    )
    i_controller_arg = DeclareLaunchArgument(
        'ki',
        default_value='0.0',
        description='Integral Gain'
    )
    d_controller_arg = DeclareLaunchArgument(
        'kd',
        default_value='0.0',
        description='Derivative Gain'
    )
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
        default_value='30.0',
        description='Control Input bound for the controller'
    )
    update_arg = DeclareLaunchArgument(
        'update_freq',
        default_value='1',
        description='System ID Parameter Update Frequency'
    )

    sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Using Simulation Time (Gazebo Time) for computations'
    )

    lambda_ = LaunchConfiguration('lambda')
    desired_angle = LaunchConfiguration('desired_angle')
    u_bound = LaunchConfiguration('u_bound')
    update_freq = LaunchConfiguration('update_freq')
    kp = LaunchConfiguration('kp')
    ki = LaunchConfiguration('ki')
    kd = LaunchConfiguration('kd')

    str_node = Node(
        package='rover_control',
        executable='pendulum_control_node',
        output='screen',
        parameters=[{
            'lambda': lambda_,
            'desired_angle': desired_angle,
            'u_bound': u_bound,
            'update_freq': update_freq,
            'kp': kp,
            'ki': ki,
            'kd': kd,
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
                 u_bound_arg, update_arg, p_controller_arg,
                 i_controller_arg, d_controller_arg, str_node]
    )
    ld = LaunchDescription()
    ld.add_action(delay_str)
    ld.add_action(rviz2)
    ld.add_action(sim_time_arg)
    return ld
