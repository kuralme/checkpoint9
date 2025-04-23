import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    proj_dir = get_package_share_directory('attach_shelf')
    rviz_config_dir = os.path.join(proj_dir, 'config', 'Pre_approach.rviz')

    return LaunchDescription([
        DeclareLaunchArgument("obstacle", default_value="0.3"),
        DeclareLaunchArgument("degrees", default_value="-90"),
        DeclareLaunchArgument("final_approach", default_value="false"),

        Node(
            package='attach_shelf',
            executable='approach_shelf_server_node',
            name="approach_shelf_server",
            output='screen'
        ),
        Node(
            package='attach_shelf',
            executable='approach_client_node',
            name="pre_approach_client",
            output='screen',
            parameters=[
                {"obstacle": LaunchConfiguration("obstacle")},
                {"degrees": LaunchConfiguration("degrees")},
                {"final_approach": LaunchConfiguration("final_approach")}
            ]
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            output='screen',
            arguments=['-d', rviz_config_dir]
        ),
    ])