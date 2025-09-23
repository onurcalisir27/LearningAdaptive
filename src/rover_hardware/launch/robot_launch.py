import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import TimerAction

def generate_launch_description():
    
    ld = LaunchDescription()
    pkg_dir = get_package_share_directory('rover_hardware')
    robot_control_launch_path = os.path.join(pkg_dir, 'launch', 'robot_control.launch.py')
    sensors_launch_path = os.path.join(pkg_dir, 'launch', 'sensors.launch.py')
    camera_launch_path = os.path.join(pkg_dir, 'launch', 'camera.launch.py')

    robot_control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(robot_control_launch_path),
        launch_arguments={}.items()
    )
    sensors_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(sensors_launch_path),
        launch_arguments={}.items()
    )
    camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(camera_launch_path),
        launch_arguments={}.items()
    )

    feedback_node = Node(
        package='rover_hardware',
        executable='feedback_node',
        output='screen',
    )

    delay_sensors = TimerAction(
        period=12.0,
        actions=[sensors_launch]
    )
    
    delay_camera = TimerAction(
        period=15.0,
        actions=[camera_launch]
    )

    delay_feedback = TimerAction(
        period=20.0,
        actions=[feedback_node]
    )


    ld.add_action(robot_control_launch)
    ld.add_action(delay_sensors)
    ld.add_action(delay_camera)
    ld.add_action(delay_feedback)
    return ld

