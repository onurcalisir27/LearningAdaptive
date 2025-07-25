import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Launch Description
    ld = LaunchDescription()

    # Path to config files
    robot_dir = get_package_share_directory('rover_sim')
    nav2_dir = get_package_share_directory('nav2_bringup')

    amcl_config = os.path.join(robot_dir, 'config', 'amcl.yaml')
    nav_config = os.path.join(robot_dir, 'config', 'navigation.yaml')
    map_config = os.path.join(robot_dir, 'maps', 'better_map.yaml')
    rviz_config = os.path.join(robot_dir, 'rviz', 'navigation.rviz')
    
    # Launch parameters
    sim_time = DeclareLaunchArgument('use_sim_time', default_value='True', description='Enable sim time use')

    # Launch actions
    amcl_localization = os.path.join(nav2_dir, 'launch', 'localization_launch.py')
    amcl_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(amcl_localization),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'params_file': amcl_config,
            'map': map_config,
        }.items()
    )

    navigation = os.path.join(nav2_dir, 'launch', 'navigation_launch.py')
    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(navigation),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'params_file': nav_config,
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
    ld.add_action(amcl_launch)
    ld.add_action(navigation_launch)
    ld.add_action(rviz_launch)

    return ld
