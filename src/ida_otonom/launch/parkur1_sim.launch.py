from launch import LaunchDescription
from launch_ros.actions import Node


MISSION_FILE = "/home/talay/Documents/ida/src/ida_otonom/ida_otonom/missions/mission.json"


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="ida_otonom",
            executable="sim_gps_node",
            name="sim_gps_node",
            output="screen",
        ),
        Node(
            package="ida_otonom",
            executable="mission_manager_node",
            name="mission_manager_node",
            output="screen",
            parameters=[{"mission_file": MISSION_FILE}],
        ),
        Node(
            package="ida_otonom",
            executable="gps_guidance_node",
            name="gps_guidance_node",
            output="screen",
            parameters=[{"mission_file": MISSION_FILE}],
        ),
        Node(
            package="ida_otonom",
            executable="controller_node",
            name="controller_node",
            output="screen",
        ),
        Node(
            package="ida_otonom",
            executable="logger_node",
            name="logger_node",
            output="screen",
        ),
        Node(
            package="ida_otonom",
            executable="yki_bridge_node",
            name="yki_bridge_node",
            output="screen",
        ),
    ])
