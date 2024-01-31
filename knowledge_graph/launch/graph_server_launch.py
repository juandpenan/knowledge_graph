import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
    EmitEvent
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.launch_context import LaunchContext
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterFile
import yaml
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.events.lifecycle import ChangeState
from launch.events.matchers import matches_action
import lifecycle_msgs


def generate_launch_description():
    # Get the launch directory
    bringup_dir = get_package_share_directory('knowledge_graph')
    params_dir = os.path.join(bringup_dir, 'config')

    # file_name = LaunchConfiguration('file_name')

    # declare_file_name_cmd = DeclareLaunchArgument(
    #     'file_name', default_value='', description='Full path to graph file to load'
    # )

    with open(os.path.join(params_dir, 'params.yaml'), "r") as stream:
        try:
            configured_params = yaml.safe_load(stream)['knowledge_graph_server']['ros__parameters']
            configured_params["file_name"] = os.path.join(bringup_dir, "3dsg", configured_params["file_name"])

        except yaml.YAMLError as exc:
            print(exc)

    graph_server_cmd = Node(
                name='graph_server',
                package='knowledge_graph',
                executable='graph_server',
                parameters=[configured_params],
                output='screen',
            )

    container = ComposableNodeContainer(
            name='graph_server_container',
            namespace='',
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
                ComposableNode(
                    package='knowledge_graph',
                    plugin='knowledge_graph::KnowledgeGraphServer',
                    name='knowledge_graph_server',                
                    parameters=[configured_params]
                   )                
            ],
            output='both',
    )
    configure_node = EmitEvent(event=ChangeState(
            lifecycle_node_matcher=matches_action(container),
            transition_id=lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE,
            ))

    
    ld = LaunchDescription()    
    # ld.add_action(container)
    # ld.add_action(configure_node)
    # ld.add_action(declare_file_name_cmd)
    ld.add_action(graph_server_cmd)
    return ld