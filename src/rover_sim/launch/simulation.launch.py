import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, Command
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_share = get_package_share_directory('rover_sim')
    urdf_file = os.path.join(pkg_share, 'description', 'robot.urdf.xacro')
    world_file = os.path.join(pkg_share, 'worlds', 'lab2office.sdf')
    ekf_file = os.path.join(pkg_share, 'config', 'ekf.yaml')
                                   
    robot_description = Command(['xacro ', urdf_file])
    params = {
        'robot_description': ParameterValue(
            robot_description,
            value_type=str
        ),
        'use_sim_time': True
    }

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[params]
        ),

        # Gazebo World Spawner
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('ros_gz_sim'),
                    'launch',
                    'gz_sim.launch.py'
                ])
            ]),
            launch_arguments={
                'gz_args': f'-r -s -v1 {world_file}',
                'on_exit_shutdown': 'true',
                'gui': 'false'
            }.items()
        ),

        Node(
            package='ros_gz_sim',
            executable='create',
            arguments=[
                '-topic', '/robot_description',
                '-name', 'rover_sim',
                '-x', '0.0',
                '-y', '0.0',
                '-z', '0.1'
            ],
            output='screen'
        ),

        TimerAction(
            period=3.0,
            actions=[
                Node(
                    package='controller_manager',
                    executable='spawner',
                    arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
                    output='screen'
                )
            ]
        ),

        TimerAction(
            period=5.0,
            actions=[
                Node(
                    package='controller_manager',
                    executable='spawner',
                    arguments=['rover_control', '--controller-manager', '/controller_manager'],
                    output='screen',
                    remappings=[
                            ('/rover_control/cmd_vel_out', '/cmd_vel'),  
                    ]
                )
            ]
        ),

        Node(
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
                'use_sim_time': True
            }]
        ),
        
        # ROS-Gazebo Topic Sharing
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=[
                # Clock sync
                "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
                # RGB image
                "/camera/image@sensor_msgs/msg/Image@gz.msgs.Image",
                # Depth image  
                "/camera/depth_image@sensor_msgs/msg/Image@gz.msgs.Image",
                # Camera info
                "/camera/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo",
                # Imu topics
                "imu@sensor_msgs/msg/Imu@gz.msgs.IMU",
                # LiDAR topics
                "/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan",
            ],
            parameters=[{
                'use_sim_time': True
            }],
            output='screen'
        ),

        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[ekf_file,
                       {'use_sim_time': True}],
        )
    ])
