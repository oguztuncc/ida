from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config_file = LaunchConfiguration("config_file")
    mission_file = LaunchConfiguration("mission_file")
    mission_files = LaunchConfiguration("mission_files")
    enable_multi_mission = LaunchConfiguration("enable_multi_mission")
    transition_delay_s = LaunchConfiguration("transition_delay_s")
    arrival_radius_m = LaunchConfiguration("arrival_radius_m")
    log_dir = LaunchConfiguration("log_dir")
    enable_logger = LaunchConfiguration("enable_logger")
    enable_costmap_logger = LaunchConfiguration("enable_costmap_logger")
    enable_visualizer = LaunchConfiguration("enable_visualizer")
    enable_corridor_planning = LaunchConfiguration("enable_corridor_planning")
    enable_geofence = LaunchConfiguration("enable_geofence")
    enable_yki_bridge = LaunchConfiguration("enable_yki_bridge")
    yki_mavlink_url = LaunchConfiguration("yki_mavlink_url")

    default_config = PathJoinSubstitution(
        [FindPackageShare("ida_otonom"), "config", "parkur1_sim.yaml"]
    )
    default_mission = PathJoinSubstitution(
        [FindPackageShare("ida_otonom"), "missions", "parkur1U2.json"]
    )
    default_parkur2_mission = PathJoinSubstitution(
        [FindPackageShare("ida_otonom"), "missions", "parkur2_sim.json"]
    )
    common_mission_params = {
        "mission_file": ParameterValue(mission_file, value_type=str)
    }

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "config_file",
                default_value=default_config,
                description="Parkur-1 duba corridor simulation YAML.",
            ),
            DeclareLaunchArgument(
                "mission_file",
                default_value=default_mission,
                description="Parkur-1 waypoint JSON path.",
            ),
            DeclareLaunchArgument(
                "mission_files",
                default_value=[default_mission, ",", default_parkur2_mission],
                description="Comma-separated list of mission files for multi-mission mode.",
            ),
            DeclareLaunchArgument(
                "enable_multi_mission",
                default_value="false",
                description="Enable multi-mission mode (Parkur 1 -> Parkur 2 auto transition).",
            ),
            DeclareLaunchArgument(
                "transition_delay_s",
                default_value="2.0",
                description="Delay in seconds between missions.",
            ),
            DeclareLaunchArgument(
                "arrival_radius_m",
                default_value="1.0",
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
                "enable_visualizer",
                default_value="true",
                description="Start turtle-style simulation visualizer.",
            ),
            DeclareLaunchArgument(
                "enable_corridor_planning",
                default_value="true",
                description=(
                    "Use side buoy corridor tracking instead of only "
                    "waypoint tracking."
                ),
            ),
            DeclareLaunchArgument(
                "enable_geofence",
                default_value="false",
                description="Monitor GPS course boundary and recover inward.",
            ),
            DeclareLaunchArgument(
                "enable_yki_bridge",
                default_value="false",
                description="Start MAVLink telemetry bridge to YKI.",
            ),
            DeclareLaunchArgument(
                "yki_mavlink_url",
                default_value="udpout:127.0.0.1:14550",
                description="MAVLink connection URL for YKI telemetry/commands.",
            ),
            Node(
                package="ida_otonom",
                executable="sim_gps_node",
                name="sim_gps_node",
                output="screen",
                condition=UnlessCondition(enable_corridor_planning),
                parameters=[{"cmd_vel_topic": "/control/cmd_vel_safe"}],
            ),
            Node(
                package="ida_otonom",
                executable="parkur2_sim_node",
                name="parkur2_sim_node",
                output="screen",
                condition=IfCondition(enable_corridor_planning),
                parameters=[
                    config_file,
                    {
                        "world_variant": "custom",
                        "custom_world_path": ParameterValue(
                            mission_file,
                            value_type=str,
                        ),
                        "detection_topic": "/perception/buoy_detections_raw",
                    },
                ],
            ),
            Node(
                package="ida_otonom",
                executable="mission_manager_node",
                name="mission_manager_node",
                output="screen",
                parameters=[
                    config_file,
                    common_mission_params,
                    {"auto_start": True},
                    {
                        "enable_multi_mission": ParameterValue(
                            enable_multi_mission,
                            value_type=bool,
                        )
                    },
                    {
                        "transition_delay_s": ParameterValue(
                            transition_delay_s,
                            value_type=float,
                        )
                    },
                    {
                        "mission_files": ParameterValue(
                            mission_files,
                            value_type=str,
                        )
                    },
                ],
            ),
            Node(
                package="ida_otonom",
                executable="gps_guidance_node",
                name="gps_guidance_node",
                output="screen",
                parameters=[
                    config_file,
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
                executable="geofence_monitor_node",
                name="geofence_monitor_node",
                output="screen",
                condition=IfCondition(enable_geofence),
                parameters=[config_file, common_mission_params],
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
                executable="lidar_processor_node",
                name="lidar_processor_node",
                output="screen",
                condition=IfCondition(enable_corridor_planning),
                parameters=[config_file],
            ),
            Node(
                package="ida_otonom",
                executable="sensor_cross_validator_node",
                name="sensor_cross_validator_node",
                output="screen",
                condition=IfCondition(enable_corridor_planning),
                parameters=[config_file],
            ),
            Node(
                package="ida_otonom",
                executable="course_memory_node",
                name="course_memory_node",
                output="screen",
                condition=IfCondition(enable_corridor_planning),
                parameters=[config_file],
            ),
            Node(
                package="ida_otonom",
                executable="semantic_buoy_classifier_node",
                name="semantic_buoy_classifier_node",
                output="screen",
                condition=IfCondition(enable_corridor_planning),
                parameters=[config_file],
            ),
            Node(
                package="ida_otonom",
                executable="corridor_tracker_node",
                name="corridor_tracker_node",
                output="screen",
                condition=IfCondition(enable_corridor_planning),
                parameters=[config_file],
            ),
            Node(
                package="ida_otonom",
                executable="parkur2_planner_node",
                name="parkur2_planner_node",
                output="screen",
                condition=IfCondition(enable_corridor_planning),
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
                executable="logger_node",
                name="logger_node",
                output="screen",
                condition=IfCondition(enable_logger),
                parameters=[
                    config_file,
                    {"log_dir": ParameterValue(log_dir, value_type=str)},
                ],
            ),
            Node(
                package="ida_otonom",
                executable="sim_visualizer_node",
                name="sim_visualizer_node",
                output="screen",
                condition=IfCondition(enable_visualizer),
                parameters=[config_file],
            ),
            Node(
                package="ida_otonom",
                executable="local_costmap_node",
                name="local_costmap_node",
                output="screen",
                condition=IfCondition(enable_costmap_logger),
                parameters=[
                    config_file,
                    {"log_dir": ParameterValue(log_dir, value_type=str)},
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
                        "mavlink_connection_url": ParameterValue(
                            yki_mavlink_url,
                            value_type=str,
                        )
                    },
                ],
            ),
        ]
    )
