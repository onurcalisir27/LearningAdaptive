import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, Command
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():

    pkg_share = get_package_share_directory('rover_control')
    sim_dir = get_package_share_directory('rover_sim')
    urdf_file = os.path.join(pkg_share, 'description', 'pendulum_free.urdf.xacro')
    world_file = os.path.join(sim_dir, 'worlds', 'test.sdf')
    robot_description = Command(['xacro ', urdf_file])
    params = {
        'robot_description': ParameterValue(
            robot_description,
            value_type=str
        ),
        'use_sim_time': True
    }
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

    lambda_ = LaunchConfiguration('lambda')
    desired_angle = LaunchConfiguration('desired_angle')
    u_bound = LaunchConfiguration('u_bound')
    update_freq = LaunchConfiguration('update_freq')
    kp = LaunchConfiguration('kp')
    ki = LaunchConfiguration('ki')
    kd = LaunchConfiguration('kd')

    urdf_pub = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[params])

    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('ros_gz_sim'),
                'launch',
                'gz_sim.launch.py'
            ])
        ]),
        launch_arguments={
            'gz_args': f'-r -v1 {world_file}',
            'on_exit_shutdown': 'true',
        }.items())

    robot_spawner = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic', '/robot_description',
            '-name', 'rover_sim',
            '-x', '1.5',
            '-y', '1.5',
            '-z', '0.2'
        ],
        output='screen')

    controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['pendulum_controller', '--controller-manager', '/controller_manager'],
        output='screen')

    delay_controller = TimerAction(
        period=3.0,
        actions=[controller]
    )

    joints = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        output='screen')

    delay_joints = TimerAction(
        period=7.0,
        actions=[joints]
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            # Clock sync
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
        ],
        parameters=[{
            'use_sim_time': True
        }],
        output='screen'
    )

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

    delay_str = TimerAction(
        period=15.0,
        actions=[forgetting_factor_arg, desired_angle_arg,
                 u_bound_arg, update_arg, p_controller_arg,
                 i_controller_arg, d_controller_arg, str_node]
    )

    ld = LaunchDescription()
    ld.add_action(urdf_pub)
    ld.add_action(simulation)
    ld.add_action(robot_spawner)
    ld.add_action(delay_controller)
    ld.add_action(delay_joints)
    ld.add_action(bridge)
    ld.add_action(delay_str)
    return ld
