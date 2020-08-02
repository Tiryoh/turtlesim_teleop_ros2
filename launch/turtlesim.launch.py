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


def generate_launch_description():
    ## arguments
    argument_teleopmode = DeclareLaunchArgument(
        'teleop_mode', default_value='joy', description='joy or keyboard',
        )

    ## nodes
    node_turtlesim = Node(
        package='turtlesim', node_executable='turtlesim_node', name='turtlesim_node1',
        )

    ## including launch files
    def func_launch_teleop_node(context):
        if context.launch_configurations['teleop_mode'] == 'joy':
            return [IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('turtlesim_teleop'), 'launch'),
                    '/teleop-joy.launch.py'
                    ]),)]
    launch_teleop_node = OpaqueFunction(function=func_launch_teleop_node)
    
    ld = LaunchDescription()
    ld.add_action(argument_teleopmode)
    ld.add_action(node_turtlesim)
    ld.add_action(launch_teleop_node)
    return ld