import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():

    ld = LaunchDescription()

    pkg_dir = get_package_share_directory('rover_hardware')
    imu_dir = get_package_share_directory('ros2_mpu6050')
    ekf_params = os.path.join(pkg_dir, 'config', 'ekf.yaml')
    imu_launch_path = os.path.join(imu_dir, 'launch', 'ros2_mpu6050.launch.py')
    
    imu_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(imu_launch_path),
        launch_arguments={}.items()
    )

    imu_filter = Node(
        package='rover_hardware',
        executable='imu_filter_node',
        name='imu_filter_node',
        output='screen'
    )

    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_params]
    )

    ld.add_action(imu_launch)
    ld.add_action(imu_filter)
    ld.add_action(ekf_node)

    return ld


