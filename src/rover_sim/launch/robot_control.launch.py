import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import TimerAction
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    
    ld = LaunchDescription()

    pkg_dir = get_package_share_directory('rover_sim')
    control_params = os.path.join(pkg_dir, 'config', 'rover_control.yaml')
    urdf_file = os.path.join(pkg_dir, 'description', 'robot.urdf.xacro')
    sensor_fusion_launch_path = os.path.join(pkg_dir, 'launch', 'sensor_fusion.launch.py')

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
        arguments=['DiffDrivePiController', '--controller-manager', '/controller_manager'],
    )

    sensor_fusion_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(sensor_fusion_launch_path),
        launch_arguments={}.items()
    )

    delay_joint_state_broadcaster = TimerAction(
        period=3.0,
        actions=[joint_state_node]
    )

    delay_controller_spawner = TimerAction(
        period=8.0,
        actions=[controller_spawner]
    )

    delay_sensor_fusion = TimerAction(
        period=12.0,
        actions=[sensor_fusion_launch]
    )

    delay_feedback = TimerAction(
        period=15.0,
        actions=[feedback_node]
    )

    ld.add_action(robot_node)
    ld.add_action(control_node)
    ld.add_action(delay_joint_state_broadcaster)
    ld.add_action(delay_controller_spawner)

    return ld

