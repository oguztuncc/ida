from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "initial_zoom",
                default_value="8.0",
                description="Initial canvas zoom factor",
            ),
            Node(
                package="ida_otonom",
                executable="parkur_editor_node",
                name="parkur_editor_node",
                output="screen",
                parameters=[
                    {
                        "initial_zoom": LaunchConfiguration("initial_zoom"),
                    }
                ],
            ),
        ]
    )
