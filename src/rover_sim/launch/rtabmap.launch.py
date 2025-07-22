
from launch import LaunchDescription, launch_description_sources
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node
from launch_ros.actions import SetParameter
import os
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.actions import TimerAction

def generate_launch_description():

    localization = LaunchConfiguration('localization')

    parameters={
          'frame_id':'base_footprint',
          'odom_frame_id':'odom',
          'odom_tf_linear_variance':0.001,
          'odom_tf_angular_variance':0.001,

          'subscribe_rgbd': True,
          'subscribe_scan': False,
          'approx_sync': True,
          'sync_queue_size': 100,
          'topic_queue_size': 30,
          'approx_sync_max_interval': 0.15,
          # RTAB-Map's internal parameters should be strings
          'RGBD/NeighborLinkRefining': 'true',    # Do odometry correction with consecutive laser scans
          'RGBD/ProximityBySpace':     'true',    # Local loop closure detection (using estimated position) with locations in WM
          'RGBD/ProximityByTime':      'false',   # Local loop closure detection with locations in STM
          'RGBD/ProximityPathMaxNeighbors': '10', # Do also proximity detection by space by merging close scans together.
          
          'Reg/Strategy':              '0',       # 0=Visual, 1=ICP, 2=Visual+ICP
          
          'Vis/MinInliers':            '12',      # 3D visual words minimum inliers to accept loop closure
          'RGBD/OptimizeFromGraphEnd': 'false',   # Optimize graph from initial node so /map -> /odom transform will be generated
          'RGBD/OptimizeMaxError':     '4',       # Reject any loop closure causing large errors (>3x link's covariance) in the map
          'Reg/Force3DoF':             'true',    # 2D SLAM
          
          'RGBD/CreateOccupancyGrid': 'true',
          'Grid/FromDepth':            'true',    # Create 2D occupancy grid from laser scan
          'Grid/Sensor':                '1',
          'Grid/3D':                  'false',    # But create 2D occupancy grid

          'Mem/DepthCompressionFormat':'.png',    # Fix the depth compression warning
          'Mem/STMSize':               '30',      # increased to 30 to avoid adding too many loop closures on just seen locations
          
          'RGBD/LocalRadius':          '5',       # limit length of proximity detections
          'Kp/MaxFeatures':           '500',      # Increase features detected
          'Vis/MaxDepth':             '4.0',      # Limit depth for indoor use
          #'Vis/FeatureType':          '8',

          'Icp/CorrespondenceRatio':   '0.2',     # minimum scan overlap to accept loop closure
          'Icp/PM':                    'false',
          'Icp/PointToPlane':          'true',    # Default false
          'Icp/MaxCorrespondenceDistance': '0.1', # Default 0.15
          'Icp/VoxelSize':             '0.05'
    }
    
    remappings=[
         ('rgb/image',       '/color/image'),
         ('depth/image',     '/stereo/depth'),
         ('rgb/camera_info', '/color/camera_info'),
        ]
    
    config_rviz = os.path.join(
        get_package_share_directory('rover_sim'), 'rviz', 'rtabmap.rviz'
    )

    ld = LaunchDescription()

    declare_rtab_map_viz = DeclareLaunchArgument('rtabmap_viz',  default_value='false',  description='Launch RTAB-Map UI (optional).')
    declare_rviz = DeclareLaunchArgument('rviz', default_value='false', description='Launch RVIZ (optional).')
    declare_localization = DeclareLaunchArgument('localization', default_value='false', description='Launch in localization mode.')
    declare_rviz_config = DeclareLaunchArgument('rviz_cfg', default_value=config_rviz,  description='Configuration path of rviz2.')
    
    rtab_node = Node(
        package='rtabmap_sync', executable='rgbd_sync', output='screen',
        parameters=[parameters,
            {'rgb_image_transport':'raw',
            'depth_image_transport':'raw',
            'approx_sync_max_interval': 0.15}],
        remappings=remappings)
    
    slam_node = Node(
        condition=UnlessCondition(localization),
        package='rtabmap_slam', executable='rtabmap', output='screen',
        parameters=[parameters],
        remappings=remappings,
        arguments=['-d'])# This will delete the previous database (~/.ros/rtabmap.db)
        
    localization_node = Node(
        condition=IfCondition(localization),
        package='rtabmap_slam', executable='rtabmap', output='screen',
        parameters=[parameters,
            {'Mem/IncrementalMemory':'False',
            'Mem/InitWMWithAllNodes':'True'}],
        remappings=remappings)

    # Visualization:
    rtabmap_viz_node = Node(
        package='rtabmap_viz', executable='rtabmap_viz', output='screen',
        condition=IfCondition(LaunchConfiguration("rtabmap_viz")),
        parameters=[parameters],
        remappings=remappings)
    
    rviz_node = Node(
        package='rviz2', executable='rviz2', name="rviz2", output='screen',
        condition=IfCondition(LaunchConfiguration("rviz")),
        arguments=[["-d"], [LaunchConfiguration("rviz_cfg")]])

    delay_rtab = TimerAction(
        period=10.0,
        actions=[rtab_node, slam_node, localization_node, rtabmap_viz_node, rviz_node]
    )
    ld.add_action(declare_rtab_map_viz)
    ld.add_action(declare_rviz)
    ld.add_action(declare_rviz_config)
    ld.add_action(declare_localization)

    ld.add_action(delay_rtab)
    
    # ld.add_action(rtab_node)
    # ld.add_action(slam_node)
    # ld.add_action(localization_node)
    # ld.add_action(rtabmap_viz_node)
    # ld.add_action(rviz_node)

    return ld
