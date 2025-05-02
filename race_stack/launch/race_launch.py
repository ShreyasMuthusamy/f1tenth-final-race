from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    opponent_estimator_config = os.path.join(
        get_package_share_directory('race_stack'),
        'config',
        'opponent_estimator.yaml'
    )
    ego_controller_config = os.path.join(
        get_package_share_directory('race_stack'),
        'config',
        'ego_controller.yaml'
    )

    opp_hat_la = DeclareLaunchArgument(
        'opponent_estimator_config',
        default_value=opponent_estimator_config,
        description='Parameters for the opponent estimator node')
    ego_u_la = DeclareLaunchArgument(
        'ego_controller_config',
        default_value=ego_controller_config,
        description='Parameters for the ego controller node')

    ld = LaunchDescription([opp_hat_la, ego_u_la])

    opp_hat_node = Node(
        package='opponent_estimator',
        executable='opponent_estimator_node',
        name='opponent_estimator_node',
        parameters=[LaunchConfiguration('opponent_estimator_config')]
    )
    ego_u_node = Node(
        package='ego_controller',
        executable='controller_node',
        name='controller_node',
        parameters=[LaunchConfiguration('ego_controller_config')]
    )

    # finalize
    ld.add_action(opp_hat_node)
    ld.add_action(ego_u_node)

    return ld
