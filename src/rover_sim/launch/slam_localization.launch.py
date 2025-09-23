import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Launch Object
    ld = LaunchDescription()

    #  Generate path to config files
    robot_dir = get_package_share_directory('rover_sim')
    slam_dir = get_package_share_directory('slam_toolbox')

    localization_config = os.path.join(robot_dir, 'config', 'slam_localization.yaml')
    rviz_config = os.path.join(robot_dir, 'rviz', 'slam_mapping.rviz')

    # Declare launch parameters
    sim_time = DeclareLaunchArgument('use_sim_time', default_value='True', description='Enable sim time use')

    # Generate launch
    slam_localization = os.path.join(slam_dir, 'launch', 'localization_launch.py')
    slam_localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(slam_localization),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'slam_params_file': localization_config,
        }.items()
    )

    rviz_launch = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config],
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ]
    )

    ld.add_action(sim_time)
    ld.add_action(slam_localization_launch)
    ld.add_action(rviz_launch)

    return ld
