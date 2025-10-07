from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    ld = LaunchDescription()
    forgetting_factor_arg = DeclareLaunchArgument(
        'lambda',
        default_value='0.98',
        description='Forgetting Factor for Self Tuning Regulator'
    )
    u_bound_arg = DeclareLaunchArgument(
        'u_bound',
        default_value='3.0',
        description='Control Input bound for the controller'
    )
    pendulum_action = ComposableNodeContainer(
        name='pendulum_trajectory_tracking',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package='pendulum_ctrl',
                plugin='pendulum_action::PendulumActionServer',
                name='pendulum_server',
                extra_arguments=[{'use_intra_process_comms': True}]
            ),
            ComposableNode(
                package='pendulum_ctrl',
                plugin='pendulum_action::PendulumActionClient',
                name='pendulum_client',
                extra_arguments=[{'use_intra_process_comms': True}]
            ),
            ComposableNode(
                package='pendulum_ctrl',
                plugin='pendulum_action::PendulumControlNode',
                name='pendulum_controller',
                parameters=[{
                    'lambda': LaunchConfiguration('lambda'),
                    'u_bound': LaunchConfiguration('u_bound')
                }],
                extra_arguments=[{'use_intra_process_comms': True}]
            )
        ]
    )
    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
    )

    ld.add_action(forgetting_factor_arg)
    ld.add_action(u_bound_arg)
    ld.add_action(rviz2)
    ld.add_action(pendulum_action)
    return ld


