from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Parkur 3 simulation launch file."""
    config_file = LaunchConfiguration("config_file")
    mission_file = LaunchConfiguration("mission_file")
    log_dir = LaunchConfiguration("log_dir")
    enable_visualizer = LaunchConfiguration("enable_visualizer")
    enable_logger = LaunchConfiguration("enable_logger")
    target_color = LaunchConfiguration("target_color")

    default_config = PathJoinSubstitution(
        [FindPackageShare("ida_otonom"), "config", "parkur3_sim.yaml"]
    )
    default_mission = PathJoinSubstitution(
        [FindPackageShare("ida_otonom"), "missions", "parkur3_sim.json"]
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
                description="Parkur-3 simulation configuration YAML.",
            ),
            DeclareLaunchArgument(
                "mission_file",
                default_value=default_mission,
                description="Parkur-3 simulation mission JSON.",
            ),
            DeclareLaunchArgument(
                "log_dir",
                default_value="/tmp/ida_otonom_logs",
                description="Telemetry output directory.",
            ),
            DeclareLaunchArgument(
                "enable_visualizer",
                default_value="true",
                description="Start simulation visualizer.",
            ),
            DeclareLaunchArgument(
                "enable_logger",
                default_value="true",
                description="Start CSV telemetry logger.",
            ),
            DeclareLaunchArgument(
                "target_color",
                default_value="",
                description="Initial Parkur-3 target color: red, blue, or yellow.",
            ),

            # ==================== SIMULATION ====================

            # Simülasyon Node (Parkur 2 sim node'u kullanılabilir veya yeni yazılır)
            Node(
                package="ida_otonom",
                executable="parkur2_sim_node",
                name="parkur2_sim_node",
                output="screen",
                parameters=[
                    config_file,
                    {"world_variant": "parkur3"},
                ],
            ),

            # ==================== PARKUR 3 NODES ====================

            # Renk Alıcı
            Node(
                package="ida_otonom",
                executable="color_receiver_node",
                name="color_receiver_node",
                output="screen",
                parameters=[
                    config_file,
                    {
                        "default_color": ParameterValue(
                            target_color,
                            value_type=str,
                        )
                    },
                ],
            ),

            # Renkli Duba Bulucu
            Node(
                package="ida_otonom",
                executable="color_buoy_finder_node",
                name="color_buoy_finder_node",
                output="screen",
                parameters=[config_file],
            ),

            # Parkur 3 Planner
            Node(
                package="ida_otonom",
                executable="parkur3_planner_node",
                name="parkur3_planner_node",
                output="screen",
                parameters=[config_file],
            ),

            # ==================== SHARED NODES ====================

            # Mission Manager
            Node(
                package="ida_otonom",
                executable="mission_manager_node",
                name="mission_manager_node",
                output="screen",
                parameters=[
                    config_file,
                    mission_params,
                    {"auto_start": True},
                ],
            ),

            # GPS Guidance
            Node(
                package="ida_otonom",
                executable="gps_guidance_node",
                name="gps_guidance_node",
                output="screen",
                parameters=[config_file, mission_params],
            ),

            # Controller
            Node(
                package="ida_otonom",
                executable="controller_node",
                name="controller_node",
                output="screen",
                parameters=[config_file, {"use_planner_bearing": True}],
            ),

            # LiDAR Processor
            Node(
                package="ida_otonom",
                executable="lidar_processor_node",
                name="lidar_processor_node",
                output="screen",
                parameters=[config_file],
            ),

            # Safety
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
                parameters=[config_file, log_params],
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
        ]
    )
