from launch import LaunchDescription
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    
    # Get the path to your URDF file
    pkg_share = get_package_share_directory('rover_sim')
    xacro_file = os.path.join(pkg_share, 'description', 'robot.urdf.xacro')
    
    # Process the URDF file using Command substitution
    robot_description_config = Command(['xacro ', xacro_file])
    
    # Create robot_state_publisher node parameters
    params = {
        'robot_description': ParameterValue(
            robot_description_config,
            value_type=str
        ),
    }
    return LaunchDescription([

        # Node to publish robot description
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[params]
        ),
        
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            output='screen'
        ),
        
        # Launch RViz
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen'
        )
    ])