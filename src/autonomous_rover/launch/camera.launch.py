from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import launch_ros.actions
from launch.actions import TimerAction

def generate_launch_description():
    lrcheck = LaunchConfiguration('lrcheck', default = True)
    rectify = LaunchConfiguration('rectify', default = True)
    depth_aligned = LaunchConfiguration('depth_aligned', default = True)

    stereo_fps = LaunchConfiguration('stereo_fps', default = 10)
    confidence = LaunchConfiguration('confidence', default = 200)
    LRchecktresh = LaunchConfiguration('LRchecktresh', default = 5)

    rgbScaleNumerator = LaunchConfiguration('rgbScaleNumerator',  default = 1)
    rgbScaleDinominator = LaunchConfiguration('rgbScaleDinominator',    default = 4)
    
    angularVelCovariance = LaunchConfiguration('angularVelCovariance', default = 0.00002)
    linearAccelCovariance = LaunchConfiguration('linearAccelCovariance', default = 0.00002)

    enableDotProjector = LaunchConfiguration('enableDotProjector', default = False)
    enableFloodLight = LaunchConfiguration('enableFloodLight', default = False)

    dotProjectorIntensity = LaunchConfiguration('dotProjectorIntensity', default = 0.5)
    floodLightIntensity = LaunchConfiguration('floodLightIntensity', default = 0.5)
    enableRosBaseTimeUpdate   = LaunchConfiguration('enableRosBaseTimeUpdate', default = False)

    declare_lrcheck_cmd = DeclareLaunchArgument(
        'lrcheck',
        default_value=lrcheck,
        description='LR-Check is used to remove incorrectly calculated disparity pixels due to occlusions at object borders. Set to true to enable it')

    declare_rectify_cmd = DeclareLaunchArgument(
        'rectify',
        default_value=rectify,
        description='enable this to publish rectified images used for depth estimation')

    declare_depth_aligned_cmd = DeclareLaunchArgument(
        'depth_aligned',
        default_value=depth_aligned,
        description='When depth_aligned is enabled depth map from stereo will be aligned to the RGB camera in the center.')

    declare_stereo_fps_cmd = DeclareLaunchArgument(
        'stereo_fps',
        default_value=stereo_fps,
        description='Sets the FPS of the cameras used in the stereo setup.')

    declare_confidence_cmd = DeclareLaunchArgument(
        'confidence',
        default_value=confidence,
        description='Set the confidence of the depth from 0-255. Max value means allow depth of all confidence. Default is set to 200')

    declare_LRchecktresh_cmd = DeclareLaunchArgument(
        'LRchecktresh',
        default_value=LRchecktresh,
        description='Set the LR threshold from 1-10 to get more accurate depth. Default value is 5.')

    declare_rgbScaleNumerator_cmd = DeclareLaunchArgument(
        'rgbScaleNumerator',
        default_value=rgbScaleNumerator,
        description='Number of the scale Factor Numberator on top of RGB resolution selection.')

    declare_rgbScaleDinominator_cmd = DeclareLaunchArgument(
        'rgbScaleDinominator',
        default_value=rgbScaleDinominator,
        description='Number of the scale Factor Dinominator on top of RGB resolution selection.')

    declare_angularVelCovariance_cmd = DeclareLaunchArgument(
        'angularVelCovariance',
        default_value=angularVelCovariance,
        description='Set the angular velocity covariance of the IMU.')

    declare_linearAccelCovariance_cmd = DeclareLaunchArgument(
        'linearAccelCovariance',
        default_value=linearAccelCovariance,
        description='Set the Linear acceleration covariance of the IMU.')

    declare_enableDotProjector_cmd = DeclareLaunchArgument(
        'enableDotProjector',
        default_value=enableDotProjector,
        description='set this to true to enable the dot projector structure light (Available only on Pro models).')
    
    declare_enableFloodLight_cmd = DeclareLaunchArgument(
        'enableFloodLight',
        default_value=enableFloodLight,
        description='Set this to true to enable the flood light for night vision (Available only on Pro models).')
   
    declare_dotProjectorIntensity_cmd = DeclareLaunchArgument(
        'dotProjectorIntensity',
        default_value=dotProjectorIntensity,
        description='Set the mA at which you intend to drive the dotProjector. Default is set to 0.5.')

    declare_floodLightIntensity_cmd = DeclareLaunchArgument(
        'floodLightIntensity',
        default_value=floodLightIntensity,
        description='Set the mA at which you intend to drive the FloodLight. Default is set to 0.5.')
   
    declare_enableRosBaseTimeUpdate_cmd = DeclareLaunchArgument(
        'enableRosBaseTimeUpdate',
        default_value=enableRosBaseTimeUpdate,
        description='Whether to update ROS time on each message.')

    stereo_node = launch_ros.actions.Node(
            package='autonomous_rover', executable='camera_driver_node',
            output='screen',
            parameters=[{'lrcheck':                 lrcheck},
                        {'rectify':                 rectify},
                        {'depth_aligned':           depth_aligned},
                        {'stereo_fps':              stereo_fps},
                        {'confidence':              confidence},
                        {'LRchecktresh':            LRchecktresh},
                        {'rgbScaleNumerator':       rgbScaleNumerator},
                        {'rgbScaleDinominator':     rgbScaleDinominator},
                        {'angularVelCovariance':    angularVelCovariance},
                        {'linearAccelCovariance':   linearAccelCovariance},
                        {'enableDotProjector':      enableDotProjector},
                        {'enableFloodLight':        enableFloodLight},
                        {'dotProjectorIntensity':   dotProjectorIntensity},
                        {'floodLightIntensity':     floodLightIntensity},
                        {'enableRosBaseTimeUpdate': enableRosBaseTimeUpdate}
                        ])
    
    color_compressed_republisher = launch_ros.actions.Node(
        package='image_transport',
        executable='republish',
        name='color_compressor',
        arguments=['raw', 'compressed'],
        remappings=[
            ('in', '/color/image'),
            ('out/compressed', '/color/image/compressed')
        ],
        parameters=[{
            'compressed.quality': 30
        }]
    )

    depth_compressed_republisher = launch_ros.actions.Node(
        package='image_transport', 
        executable='republish',
        name='depth_compressor',
        arguments=['raw', 'compressedDepth'],
        remappings=[
            ('in', '/stereo/depth'),
            ('out/compressedDepth', '/stereo/depth/compressedDepth')
        ]
    )
    
    camera_info_relay_color = launch_ros.actions.Node(
        package='topic_tools',
        executable='relay',
        name='camera_info_relay_color',
        arguments=['/color/camera_info', '/color/image/camera_info']
    )
    
    ld = LaunchDescription()
    ld.add_action(declare_lrcheck_cmd)
    ld.add_action(declare_rectify_cmd)
    ld.add_action(declare_depth_aligned_cmd)

    ld.add_action(declare_stereo_fps_cmd)
    ld.add_action(declare_confidence_cmd)
    ld.add_action(declare_LRchecktresh_cmd)

    ld.add_action(declare_rgbScaleNumerator_cmd)
    ld.add_action(declare_rgbScaleDinominator_cmd)

    ld.add_action(declare_angularVelCovariance_cmd)
    ld.add_action(declare_linearAccelCovariance_cmd)

    ld.add_action(declare_enableDotProjector_cmd)
    ld.add_action(declare_enableFloodLight_cmd)
    ld.add_action(declare_dotProjectorIntensity_cmd)
    ld.add_action(declare_floodLightIntensity_cmd)

    delay_camera = TimerAction(
        period=1.0,
        actions=[stereo_node]
    )   
    delay_compression = TimerAction(
        period=4.0,
        actions=[color_compressed_republisher, depth_compressed_republisher]
    )
    delay_relay = TimerAction(
        period=7.0,
        actions=[camera_info_relay_color]
    )
    ld.add_action(delay_camera)
    ld.add_action(delay_compression)
    ld.add_action(delay_relay)
    return ld
