import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, Shutdown, RegisterEventHandler
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.event_handlers import OnProcessExit

def generate_launch_description():
    proj_dir = get_package_share_directory('attach_shelf')
    rviz_config_dir = os.path.join(proj_dir, 'config', 'Pre_approach.rviz')

    # Declare arguments
    obstacle_arg = DeclareLaunchArgument("obstacle", default_value="0.3")
    degrees_arg = DeclareLaunchArgument("degrees", default_value="-90")
    final_approach_arg = DeclareLaunchArgument("final_approach", default_value="false")

    # Server node
    server_node = Node(
        package='attach_shelf',
        executable='approach_shelf_server_node',
        name="approach_shelf_server",
        output='screen'
    )

    # Client node
    client_node = Node(
        package='attach_shelf',
        executable='approach_client_node',
        name="pre_approach_client",
        output='screen',
        parameters=[
            {"obstacle": LaunchConfiguration("obstacle")},
            {"degrees": LaunchConfiguration("degrees")},
            {"final_approach": LaunchConfiguration("final_approach")}
        ]
    )

    # RViz node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_dir]
    )

    # Shutdown handler after client exits
    shutdown_after_client = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=client_node,
            on_exit=[Shutdown()]
        )
    )

    return LaunchDescription([
        obstacle_arg,
        degrees_arg,
        final_approach_arg,
        server_node,
        client_node,
        rviz_node,
        shutdown_after_client,
    ])
