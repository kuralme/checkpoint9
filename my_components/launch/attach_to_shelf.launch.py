import os
from ament_index_python.packages import get_package_share_directory

import launch
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.actions import Node
from launch.event_handlers import OnProcessExit
from launch.actions import Shutdown, RegisterEventHandler

def generate_launch_description():
    rosject_dir = get_package_share_directory('my_components')
    rviz_config_dir = os.path.join(rosject_dir, 'config', 'Pre_approach.rviz')

    # Composable nodes
    preapproach_composed_node = ComposableNode(
                    package='my_components',
                    plugin='my_components::PreApproach',
                    name='pre_approach'
    )
    approach_server_composed_node = ComposableNode(
                    package='my_components',
                    plugin='my_components::AttachServer',
                    name='attach_server'
    )

    # """Generate launch description with multiple components."""
    composition_container = ComposableNodeContainer(
            name='my_container',
            namespace='',
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
                preapproach_composed_node,
                approach_server_composed_node
            ],
            output='screen',
    )

    # RViz node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_dir]
    )

    # Shutdown handler after composition container exits
    shutdown_after_composition = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=composition_container,
            on_exit=[Shutdown()]
        )
    )
    return launch.LaunchDescription([
        composition_container,
        rviz_node,
        shutdown_after_composition
    ])