import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, Command
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():

    control_dir = get_package_share_directory('rover_control')
    sim_dir = get_package_share_directory('rover_sim')

    sim_path = os.path.join(sim_dir, 'launch', 'simulation.launch.py')
    localization_path = os.path.join(sim_dir, 'launch', 'slam_localization.launch.py')
    map_path = os.path.join(control_dir, 'maps', 'control_map.yaml')

    # Declare Launch Arguments
    forgetting_factor_arg = DeclareLaunchArgument(
        'lambda',
        default_value='0.98',
        description='Forgetting Factor for Self Tuning Regulator'
    )
    # desired_angle_arg = DeclareLaunchArgument(
    #     'desired_angle',
    #     default_value='0.0',
    #     description='Desired angle for the pendulum to stabilize on'
    # )

    # Add Launch Configurations
    forgetting_factor = LaunchConfiguration('forgetting_factor')
    # desired_angle = LaunchConfiguration('desired_angle')


    # Add the Gazebo Simulation of Autonomous Rover
    simulation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(sim_path),
    )

    # Add the Map Processing
    localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(localization_path)
    )

    # Add the A* Planner Node
    planner_node = Node(
        package='rover_control',
        executable='planner_node',
        output='screen',
    )

    # Add the Controller Node
    controller_node = Node(
        package='rover_control',
        executable='control_node',
        output='screen',
        # parameters=[{
        #     'forgetting_factor': forgetting_factor,
        #     'desired_angle': desired_angle,
        # }]
    )

    # Add Timer Delay for resolving conflicts
    localization = TimerAction(
        actions=[localization_launch],
        period=12.0
    )

    planner = TimerAction(
        actions=[planner_node],
        period=20.0
    )

    controller = TimerAction(
        actions=[controller_node],
        period=22.0
    )

    ld = LaunchDescription()
    ld.add_action(simulation_launch)
    ld.add_action(localization)
    ld.add_action(planner)
    ld.add_action(controller)

    return ld


