from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    mission_file = LaunchConfiguration("mission_file")
    arrival_radius_m = LaunchConfiguration("arrival_radius_m")
    log_dir = LaunchConfiguration("log_dir")
    enable_logger = LaunchConfiguration("enable_logger")
    enable_costmap_logger = LaunchConfiguration("enable_costmap_logger")
    enable_yki_bridge = LaunchConfiguration("enable_yki_bridge")
    yki_udp_ip = LaunchConfiguration("yki_udp_ip")
    yki_udp_port = LaunchConfiguration("yki_udp_port")

    default_mission = PathJoinSubstitution(
        [FindPackageShare("ida_otonom"), "missions", "mission.json"]
    )

    common_mission_params = {
        "mission_file": ParameterValue(mission_file, value_type=str)
    }

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "mission_file",
                default_value=default_mission,
                description="Parkur-1 waypoint JSON path.",
            ),
            DeclareLaunchArgument(
                "arrival_radius_m",
                default_value="3.0",
                description="Waypoint reached radius in meters.",
            ),
            DeclareLaunchArgument(
                "log_dir",
                default_value="/tmp/ida_otonom_logs",
                description="Telemetry output directory for simulation.",
            ),
            DeclareLaunchArgument(
                "enable_logger",
                default_value="true",
                description="Start CSV telemetry logger.",
            ),
            DeclareLaunchArgument(
                "enable_costmap_logger",
                default_value="false",
                description="Start local costmap logger.",
            ),
            DeclareLaunchArgument(
                "enable_yki_bridge",
                default_value="false",
                description="Start UDP telemetry bridge to YKI.",
            ),
            DeclareLaunchArgument(
                "yki_udp_ip",
                default_value="127.0.0.1",
                description="YKI UDP target IP.",
            ),
            DeclareLaunchArgument(
                "yki_udp_port",
                default_value="5005",
                description="YKI UDP target port.",
            ),
            Node(
                package="ida_otonom",
                executable="sim_gps_node",
                name="sim_gps_node",
                output="screen",
                parameters=[{"cmd_vel_topic": "/control/cmd_vel_safe"}],
            ),
            Node(
                package="ida_otonom",
                executable="mission_manager_node",
                name="mission_manager_node",
                output="screen",
                parameters=[common_mission_params, {"auto_start": True}],
            ),
            Node(
                package="ida_otonom",
                executable="gps_guidance_node",
                name="gps_guidance_node",
                output="screen",
                parameters=[
                    common_mission_params,
                    {
                        "arrival_radius_m": ParameterValue(
                            arrival_radius_m,
                            value_type=float,
                        )
                    },
                ],
            ),
            Node(
                package="ida_otonom",
                executable="controller_node",
                name="controller_node",
                output="screen",
            ),
            Node(
                package="ida_otonom",
                executable="safety_node",
                name="safety_node",
                output="screen",
            ),
            Node(
                package="ida_otonom",
                executable="logger_node",
                name="logger_node",
                output="screen",
                condition=IfCondition(enable_logger),
                parameters=[
                    {"log_dir": ParameterValue(log_dir, value_type=str)}
                ],
            ),
            Node(
                package="ida_otonom",
                executable="local_costmap_node",
                name="local_costmap_node",
                output="screen",
                condition=IfCondition(enable_costmap_logger),
                parameters=[
                    {"log_dir": ParameterValue(log_dir, value_type=str)}
                ],
            ),
            Node(
                package="ida_otonom",
                executable="yki_bridge_node",
                name="yki_bridge_node",
                output="screen",
                condition=IfCondition(enable_yki_bridge),
                parameters=[
                    {
                        "udp_ip": ParameterValue(
                            yki_udp_ip,
                            value_type=str,
                        )
                    },
                    {
                        "udp_port": ParameterValue(
                            yki_udp_port,
                            value_type=int,
                        )
                    },
                ],
            ),
        ]
    )
