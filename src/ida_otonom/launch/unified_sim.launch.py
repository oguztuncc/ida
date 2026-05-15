from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Unified simulation launch for Parkur 1 -> Parkur 2."""
    config_file = LaunchConfiguration("config_file")
    mission_files = LaunchConfiguration("mission_files")
    transition_delay_s = LaunchConfiguration("transition_delay_s")
    arrival_radius_m = LaunchConfiguration("arrival_radius_m")
    log_dir = LaunchConfiguration("log_dir")
    enable_logger = LaunchConfiguration("enable_logger")
    enable_costmap_logger = LaunchConfiguration("enable_costmap_logger")
    enable_visualizer = LaunchConfiguration("enable_visualizer")

    default_config = PathJoinSubstitution(
        [FindPackageShare("ida_otonom"), "config", "unified_sim.yaml"]
    )
    default_parkur1_mission = PathJoinSubstitution(
        [FindPackageShare("ida_otonom"), "missions", "unified_parkur1.json"]
    )
    default_parkur2_mission = PathJoinSubstitution(
        [FindPackageShare("ida_otonom"), "missions", "unified_parkur2.json"]
    )

    return LaunchDescription(
        [
            LogInfo(msg="Starting Unified Simulasyon: Parkur 1 -> Parkur 2"),

            DeclareLaunchArgument(
                "config_file",
                default_value=default_config,
                description="Unified simulation configuration YAML.",
            ),
            DeclareLaunchArgument(
                "mission_files",
                default_value=[default_parkur1_mission, ",", default_parkur2_mission],
                description="Comma-separated list of mission files.",
            ),
            DeclareLaunchArgument(
                "transition_delay_s",
                default_value="3.0",
                description="Delay in seconds between missions.",
            ),
            DeclareLaunchArgument(
                "arrival_radius_m",
                default_value="3.0",
                description="Waypoint reached radius in meters.",
            ),
            DeclareLaunchArgument(
                "log_dir",
                default_value="/tmp/ida_otonom_logs",
                description="Telemetry output directory.",
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
                description="Start unified simulation visualizer.",
            ),

            # Unified Simülasyon Node (Tek dünya, her iki parkur)
            Node(
                package="ida_otonom",
                executable="unified_sim_node",
                name="unified_sim_node",
                output="screen",
                parameters=[
                    config_file,
                    {"detection_topic": "/perception/buoy_detections_raw"},
                ],
            ),

            # Mission Manager - Multi-Mission Mode
            Node(
                package="ida_otonom",
                executable="mission_manager_node",
                name="mission_manager_node",
                output="screen",
                parameters=[
                    {"auto_start": True},
                    {"enable_multi_mission": True},
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

            # GPS Guidance
            Node(
                package="ida_otonom",
                executable="gps_guidance_node",
                name="gps_guidance_node",
                output="screen",
                parameters=[
                    config_file,
                    {
                        "arrival_radius_m": ParameterValue(
                            arrival_radius_m,
                            value_type=float,
                        )
                    },
                ],
            ),

            # Controller
            Node(
                package="ida_otonom",
                executable="controller_node",
                name="controller_node",
                output="screen",
                parameters=[config_file],
            ),

            # LiDAR Processor
            Node(
                package="ida_otonom",
                executable="lidar_processor_node",
                name="lidar_processor_node",
                output="screen",
                parameters=[config_file],
            ),
            Node(
                package="ida_otonom",
                executable="sensor_cross_validator_node",
                name="sensor_cross_validator_node",
                output="screen",
                parameters=[config_file],
            ),

            # Course Memory
            Node(
                package="ida_otonom",
                executable="course_memory_node",
                name="course_memory_node",
                output="screen",
                parameters=[config_file],
            ),

            # Semantic Buoy Classifier
            Node(
                package="ida_otonom",
                executable="semantic_buoy_classifier_node",
                name="semantic_buoy_classifier_node",
                output="screen",
                parameters=[config_file],
            ),

            # Corridor Tracker
            Node(
                package="ida_otonom",
                executable="corridor_tracker_node",
                name="corridor_tracker_node",
                output="screen",
                parameters=[config_file],
            ),

            # Parkur 2 Planner
            Node(
                package="ida_otonom",
                executable="parkur2_planner_node",
                name="parkur2_planner_node",
                output="screen",
                parameters=[config_file],
            ),

            # Safety Node
            Node(
                package="ida_otonom",
                executable="safety_node",
                name="safety_node",
                output="screen",
                parameters=[config_file],
            ),

            # Logger
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

            # Visualizer
            Node(
                package="ida_otonom",
                executable="sim_visualizer_node",
                name="sim_visualizer_node",
                output="screen",
                condition=IfCondition(enable_visualizer),
                parameters=[config_file],
            ),

            # Local Costmap
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
        ]
    )
