from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config_file = LaunchConfiguration("config_file")
    mission_file = LaunchConfiguration("mission_file")
    log_dir = LaunchConfiguration("log_dir")
    enable_yki_bridge = LaunchConfiguration("enable_yki_bridge")
    enable_perception = LaunchConfiguration("enable_perception")
    enable_costmap_logger = LaunchConfiguration("enable_costmap_logger")
    mavros_bridge_enabled = LaunchConfiguration("mavros_bridge_enabled")
    mavros_output_mode = LaunchConfiguration("mavros_output_mode")

    default_config = PathJoinSubstitution(
        [FindPackageShare("ida_otonom"), "config", "ida_real.yaml"]
    )
    default_mission = PathJoinSubstitution(
        [FindPackageShare("ida_otonom"), "missions", "mission.json"]
    )

    mission_params = {
        "mission_file": ParameterValue(mission_file, value_type=str)
    }
    log_params = {
        "log_dir": ParameterValue(log_dir, value_type=str)
    }

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "config_file",
                default_value=default_config,
                description="Real IDA hardware configuration YAML.",
            ),
            DeclareLaunchArgument(
                "mission_file",
                default_value=default_mission,
                description="Mission waypoint JSON loaded before start.",
            ),
            DeclareLaunchArgument(
                "log_dir",
                default_value="/tmp/ida_otonom_logs",
                description="Local recording directory.",
            ),
            DeclareLaunchArgument(
                "enable_yki_bridge",
                default_value="true",
                description="Start telemetry/command bridge.",
            ),
            DeclareLaunchArgument(
                "enable_perception",
                default_value="false",
                description="Start camera processing node.",
            ),
            DeclareLaunchArgument(
                "enable_costmap_logger",
                default_value="true",
                description="Start lidar costmap recorder.",
            ),
            DeclareLaunchArgument(
                "mavros_bridge_enabled",
                default_value="false",
                description="Allow bridge to publish MAVROS commands.",
            ),
            DeclareLaunchArgument(
                "mavros_output_mode",
                default_value="disabled",
                description="disabled, cmd_vel, or manual_control.",
            ),
            Node(
                package="ida_otonom",
                executable="mission_manager_node",
                name="mission_manager_node",
                output="screen",
                parameters=[config_file, mission_params],
            ),
            Node(
                package="ida_otonom",
                executable="gps_guidance_node",
                name="gps_guidance_node",
                output="screen",
                parameters=[config_file, mission_params],
            ),
            Node(
                package="ida_otonom",
                executable="controller_node",
                name="controller_node",
                output="screen",
                parameters=[config_file],
            ),
            Node(
                package="ida_otonom",
                executable="safety_node",
                name="safety_node",
                output="screen",
                parameters=[config_file],
            ),
            Node(
                package="ida_otonom",
                executable="rc_kill_node",
                name="rc_kill_node",
                output="screen",
                parameters=[config_file],
            ),
            Node(
                package="ida_otonom",
                executable="power_relay_node",
                name="power_relay_node",
                output="screen",
                parameters=[config_file],
            ),
            Node(
                package="ida_otonom",
                executable="mavros_bridge_node",
                name="mavros_bridge_node",
                output="screen",
                parameters=[
                    config_file,
                    {
                        "enabled": ParameterValue(
                            mavros_bridge_enabled,
                            value_type=bool,
                        ),
                        "output_mode": ParameterValue(
                            mavros_output_mode,
                            value_type=str,
                        ),
                    },
                ],
            ),
            Node(
                package="ida_otonom",
                executable="logger_node",
                name="logger_node",
                output="screen",
                parameters=[config_file, log_params],
            ),
            Node(
                package="ida_otonom",
                executable="local_costmap_node",
                name="local_costmap_node",
                output="screen",
                condition=IfCondition(enable_costmap_logger),
                parameters=[config_file, log_params],
            ),
            Node(
                package="ida_otonom",
                executable="perception_node",
                name="perception_node",
                output="screen",
                condition=IfCondition(enable_perception),
                parameters=[
                    config_file,
                    {
                        "save_dir": ParameterValue(
                            log_dir,
                            value_type=str,
                        )
                    },
                ],
            ),
            Node(
                package="ida_otonom",
                executable="yki_bridge_node",
                name="yki_bridge_node",
                output="screen",
                condition=IfCondition(enable_yki_bridge),
                parameters=[config_file],
            ),
        ]
    )
