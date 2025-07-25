import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import TimerAction
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    
    ld = LaunchDescription()
    pkg_dir = get_package_share_directory('rover_hardware')
    control_params = os.path.join(pkg_dir,'config', 'rover_control.yaml')
    urdf_file = os.path.join(pkg_dir, 'description', 'robot.urdf.xacro')

    robot_desc = Command(['xacro ', urdf_file])
    robot_params = {
            'robot_description': ParameterValue(
            robot_desc,
            value_type=str
        )
    }

    robot_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_params]
    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[control_params],
        output="both",
        remappings=[
            ("~/robot_description", "/robot_description")
        ]
    )

    joint_state_node = Node(
        package="controller_manager",
        executable="spawner",
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
    )

    controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['rover_control', '--controller-manager', '/controller_manager'],
    )

    twist_stamper = Node(
        package='twist_stamper',
        executable='twist_stamper',
        name='twist_stamper',
        output='screen',
        remappings=[
            ('cmd_vel_in', '/cmd_vel'),
            ('cmd_vel_out', 'rover_control/cmd_vel')
        ],
        parameters=[{
            'frame_id': 'base_footprint',
            'use_sim_time': False
        }]
    )

    delay_joint_state_broadcaster = TimerAction(
        period=3.0,
        actions=[joint_state_node]
    )

    delay_controller_spawner = TimerAction(
        period=7.0,
        actions=[controller_spawner]
    )
    
    delay_twist_stamper = TimerAction(
        period=10.0,
        actions=[twist_stamper]
    )

    ld.add_action(robot_node)
    ld.add_action(control_node)
    ld.add_action(delay_joint_state_broadcaster)
    ld.add_action(delay_controller_spawner)
    ld.add_action(delay_twist_stamper)
    return ld

