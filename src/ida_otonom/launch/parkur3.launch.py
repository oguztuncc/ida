from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Parkur 3 real hardware launch file."""
    config_file = LaunchConfiguration("config_file")
    mission_file = LaunchConfiguration("mission_file")
    log_dir = LaunchConfiguration("log_dir")
    enable_yki_bridge = LaunchConfiguration("enable_yki_bridge")
    enable_costmap_logger = LaunchConfiguration("enable_costmap_logger")
    mavros_bridge_enabled = LaunchConfiguration("mavros_bridge_enabled")
    mavros_output_mode = LaunchConfiguration("mavros_output_mode")
    model_path = LaunchConfiguration("model_path")
    enable_yolo = LaunchConfiguration("enable_yolo")
    color_image_topic = LaunchConfiguration("color_image_topic")
    depth_image_topic = LaunchConfiguration("depth_image_topic")
    camera_info_topic = LaunchConfiguration("camera_info_topic")

    default_config = PathJoinSubstitution(
        [FindPackageShare("ida_otonom"), "config", "parkur3.yaml"]
    )
    default_mission = PathJoinSubstitution(
        [FindPackageShare("ida_otonom"), "missions", "parkur3.json"]
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
                description="Parkur-3 configuration YAML.",
            ),
            DeclareLaunchArgument(
                "mission_file",
                default_value=default_mission,
                description="Parkur-3 mission JSON.",
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
            DeclareLaunchArgument(
                "model_path",
                default_value="models/buoy_yolo.pt",
                description="YOLO colored buoy model path on Jetson.",
            ),
            DeclareLaunchArgument(
                "enable_yolo",
                default_value="true",
                description="Load YOLO model if available.",
            ),
            DeclareLaunchArgument(
                "color_image_topic",
                default_value="/camera/camera/color/image_raw",
                description="RealSense RGB image topic.",
            ),
            DeclareLaunchArgument(
                "depth_image_topic",
                default_value="/camera/camera/aligned_depth_to_color/image_raw",
                description="RealSense aligned depth image topic.",
            ),
            DeclareLaunchArgument(
                "camera_info_topic",
                default_value="/camera/camera/color/camera_info",
                description="RealSense color camera info topic.",
            ),

            # ==================== PARKUR 3 NODES ====================

            # Renk Alıcı (IHA'dan)
            Node(
                package="ida_otonom",
                executable="color_receiver_node",
                name="color_receiver_node",
                output="screen",
                parameters=[config_file],
            ),

            # Görüntü İşleme ile Renkli Duba Tespiti
            Node(
                package="ida_otonom",
                executable="buoy_detector_node",
                name="buoy_detector_node",
                output="screen",
                parameters=[
                    config_file,
                    {
                        "model_path": ParameterValue(
                            model_path,
                            value_type=str,
                        ),
                        "enable_yolo": ParameterValue(
                            enable_yolo,
                            value_type=bool,
                        ),
                        "color_image_topic": ParameterValue(
                            color_image_topic,
                            value_type=str,
                        ),
                        "depth_image_topic": ParameterValue(
                            depth_image_topic,
                            value_type=str,
                        ),
                        "camera_info_topic": ParameterValue(
                            camera_info_topic,
                            value_type=str,
                        ),
                    },
                ],
            ),

            # Renkli Duba Bulucu (LiDAR)
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
                parameters=[config_file, mission_params],
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

            # Local Costmap
            Node(
                package="ida_otonom",
                executable="local_costmap_node",
                name="local_costmap_node",
                output="screen",
                condition=IfCondition(enable_costmap_logger),
                parameters=[config_file, log_params],
            ),

            # Safety
            Node(
                package="ida_otonom",
                executable="safety_node",
                name="safety_node",
                output="screen",
                parameters=[config_file],
            ),

            # RC Kill
            Node(
                package="ida_otonom",
                executable="rc_kill_node",
                name="rc_kill_node",
                output="screen",
                parameters=[config_file],
            ),

            # Power Relay
            Node(
                package="ida_otonom",
                executable="power_relay_node",
                name="power_relay_node",
                output="screen",
                parameters=[config_file],
            ),

            # MAVROS Bridge
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

            # Logger
            Node(
                package="ida_otonom",
                executable="logger_node",
                name="logger_node",
                output="screen",
                parameters=[config_file, log_params],
            ),

            # YKI Bridge
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
