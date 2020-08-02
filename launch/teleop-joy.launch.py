import os
from ament_index_python.packages import get_package_share_directory
import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.substitutions import TextSubstitution


def generate_launch_description():
    joy_config = LaunchConfiguration('joy_config')
    joy_dev = LaunchConfiguration('joy_dev')
    config_filepath = LaunchConfiguration('config_filepath')

    ## arguments
    argument_joy_config = DeclareLaunchArgument(
        'joy_config', default_value='ds4'
        )
    argument_joy_dev = DeclareLaunchArgument(
        'joy_dev', default_value='/dev/input/js0'
        )
    argument_config_path = DeclareLaunchArgument(
        'config_filepath',
        default_value=[
            launch.substitutions.TextSubstitution(text=os.path.join(
                get_package_share_directory('turtlesim_teleop'), 'config', '')),
            joy_config, launch.substitutions.TextSubstitution(text='.config.yaml')]
        )

    ## nodes
    node_joy = Node(
        package='joy', node_executable='joy_node', name='joy_node',
        parameters=[{
            'dev': joy_dev,
            'deadzone': 0.3,
            'autorepeat_rate': 20.0,
        }]
    )
    node_teleop_joy = Node(
        package='teleop_twist_joy', node_executable='teleop_node',
        name='teleop_twist_joy_node',
        parameters=[config_filepath],
        arguments=[
            'cmd_vel:=/turtle1/cmd_vel',
        ]
    )
    
    ld = LaunchDescription()
    ld.add_action(argument_joy_config)
    ld.add_action(argument_joy_dev)
    ld.add_action(argument_config_path)
    ld.add_action(node_joy)
    ld.add_action(node_teleop_joy)
    return ld
