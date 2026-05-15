from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Multi-mission launch for Parkur 1 -> Parkur 2 transition."""
    # Parkur 1 parametreleri
    parkur1_config = LaunchConfiguration("parkur1_config")
    parkur1_mission = LaunchConfiguration("parkur1_mission")
    # Parkur 2 parametreleri
    parkur2_config = LaunchConfiguration("parkur2_config")
    parkur2_mission = LaunchConfiguration("parkur2_mission")

    # Multi-mission parametreleri
    mission_files = LaunchConfiguration("mission_files")
    transition_delay_s = LaunchConfiguration("transition_delay_s")
    arrival_radius_m = LaunchConfiguration("arrival_radius_m")
    log_dir = LaunchConfiguration("log_dir")
    enable_logger = LaunchConfiguration("enable_logger")
    enable_costmap_logger = LaunchConfiguration("enable_costmap_logger")
    enable_visualizer = LaunchConfiguration("enable_visualizer")
    # Default paths
    default_parkur1_config = PathJoinSubstitution(
        [FindPackageShare("ida_otonom"), "config", "parkur1_sim.yaml"]
    )
    default_parkur1_mission = PathJoinSubstitution(
        [FindPackageShare("ida_otonom"), "missions", "mission.json"]
    )
    default_parkur2_config = PathJoinSubstitution(
        [FindPackageShare("ida_otonom"), "config", "parkur2_sim.yaml"]
    )
    default_parkur2_mission = PathJoinSubstitution(
        [FindPackageShare("ida_otonom"), "missions", "parkur2_sim.json"]
    )

    return LaunchDescription(
        [
            # Log bilgisi
            LogInfo(msg="Starting Multi-Mission Mode: Parkur 1 -> Parkur 2"),

            # Parkur 1 parametreleri
            DeclareLaunchArgument(
                "parkur1_config",
                default_value=default_parkur1_config,
                description="Parkur-1 simulation configuration YAML.",
            ),
            DeclareLaunchArgument(
                "parkur1_mission",
                default_value=default_parkur1_mission,
                description="Parkur-1 mission JSON file.",
            ),

            # Parkur 2 parametreleri
            DeclareLaunchArgument(
                "parkur2_config",
                default_value=default_parkur2_config,
                description="Parkur-2 simulation configuration YAML.",
            ),
            DeclareLaunchArgument(
                "parkur2_mission",
                default_value=default_parkur2_mission,
                description="Parkur-2 mission JSON file.",
            ),
            DeclareLaunchArgument(
                "mission_files",
                default_value=[parkur1_mission, ",", parkur2_mission],
                description="Comma-separated Parkur-1 and Parkur-2 mission files.",
            ),

            # Multi-mission parametreleri
            DeclareLaunchArgument(
                "transition_delay_s",
                default_value="5.0",
                description="Delay in seconds between Parkur 1 and Parkur 2.",
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
                description="Start simulation visualizer.",
            ),
            DeclareLaunchArgument(
                "enable_corridor_planning",
                default_value="true",
                description="Use corridor tracking for Parkur 1.",
            ),

            # ==================== PARKUR 1 NODES ====================

            # Parkur 1 Sim Node (GPS veya Corridor)
            Node(
                package="ida_otonom",
                executable="parkur2_sim_node",
                name="parkur1_sim_node",
                output="screen",
                parameters=[
                    parkur1_config,
                    {"world_variant": "parkur1"},
                    {"detection_topic": "/perception/buoy_detections_raw"},
                ],
            ),

            # ==================== PARKUR 2 NODES ====================

            # Parkur 2 Sim Node
            Node(
                package="ida_otonom",
                executable="parkur2_sim_node",
                name="parkur2_sim_node",
                output="screen",
                parameters=[
                    parkur2_config,
                    {"detection_topic": "/perception/buoy_detections_raw"},
                ],
            ),

            # ==================== SHARED NODES ====================

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
                    parkur1_config,
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
                parameters=[parkur1_config],
            ),

            # LiDAR Processor
            Node(
                package="ida_otonom",
                executable="lidar_processor_node",
                name="lidar_processor_node",
                output="screen",
                parameters=[parkur1_config],
            ),

            # LiDAR + RealSense depth çapraz doğrulama
            Node(
                package="ida_otonom",
                executable="sensor_cross_validator_node",
                name="sensor_cross_validator_node",
                output="screen",
                parameters=[parkur1_config],
            ),

            # Course Memory
            Node(
                package="ida_otonom",
                executable="course_memory_node",
                name="course_memory_node",
                output="screen",
                parameters=[parkur1_config],
            ),

            # Semantic Buoy Classifier
            Node(
                package="ida_otonom",
                executable="semantic_buoy_classifier_node",
                name="semantic_buoy_classifier_node",
                output="screen",
                parameters=[parkur1_config],
            ),

            # Corridor Tracker
            Node(
                package="ida_otonom",
                executable="corridor_tracker_node",
                name="corridor_tracker_node",
                output="screen",
                parameters=[parkur1_config],
            ),

            # Parkur 2 Planner
            Node(
                package="ida_otonom",
                executable="parkur2_planner_node",
                name="parkur2_planner_node",
                output="screen",
                parameters=[parkur1_config],
            ),

            # Safety Node
            Node(
                package="ida_otonom",
                executable="safety_node",
                name="safety_node",
                output="screen",
                parameters=[parkur1_config],
            ),

            # Logger
            Node(
                package="ida_otonom",
                executable="logger_node",
                name="logger_node",
                output="screen",
                condition=IfCondition(enable_logger),
                parameters=[
                    parkur1_config,
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
                parameters=[parkur1_config],
            ),

            # Local Costmap
            Node(
                package="ida_otonom",
                executable="local_costmap_node",
                name="local_costmap_node",
                output="screen",
                condition=IfCondition(enable_costmap_logger),
                parameters=[
                    parkur1_config,
                    {"log_dir": ParameterValue(log_dir, value_type=str)},
                ],
            ),
        ]
    )
