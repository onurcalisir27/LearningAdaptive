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
from launch.substitutions import PythonExpression
def generate_launch_description():

    pkg_share = get_package_share_directory('pendulum_ctrl')
    world_file = os.path.join(pkg_share, 'worlds', 'pendulum_test_world.sdf')

    two_link_arg = DeclareLaunchArgument(
        'two_link',
        default_value='false',
        description='Whether the simulation is of the two linked pendulum or \
        simple one link'
    )

    robot_description = Command([
        'xacro ',
        PythonExpression([
            "'", os.path.join(pkg_share, 'description', 'two_link.urdf.xacro'),
            "' if '", LaunchConfiguration('two_link'), "' == 'true' else '",
            os.path.join(pkg_share, 'description', 'pendulum.urdf.xacro'), "'"
        ])
    ])

    params = {
        'robot_description': ParameterValue(
            robot_description,
            value_type=str
        ),
        'use_sim_time': True
    }

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
            '-name', 'pendulum',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.2'
        ],
        output='screen')

    controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['pendulum', '--controller-manager', '/controller_manager'],
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
        period=6.0,
        actions=[joints]
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            # Clock sync
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
            "/world/controller_test/set_pose@ros_gz_interfaces/srv/SetEntityPose",
        ],
        parameters=[{
            'use_sim_time': True
        }],
        output='screen'
    )

    service_handler = Node(
        package='pendulum_ctrl',
        executable='pendulum_service_handle',
        output='screen',
        parameters=[{
            'is_two_link': LaunchConfiguration('two_link')
            }
        ]
    )

    ld = LaunchDescription()
    ld.add_action(two_link_arg)
    ld.add_action(urdf_pub)
    ld.add_action(simulation)
    ld.add_action(robot_spawner)
    ld.add_action(delay_controller)
    ld.add_action(delay_joints)
    ld.add_action(bridge)
    ld.add_action(service_handler)
    return ld

