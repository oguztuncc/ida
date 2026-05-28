from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config_file = LaunchConfiguration("config_file")
    mission_file = LaunchConfiguration("mission_file")
    log_dir = LaunchConfiguration("log_dir")
    enable_yki_bridge = LaunchConfiguration("enable_yki_bridge")
    enable_perception = LaunchConfiguration("enable_perception")
    enable_buoy_detector = LaunchConfiguration("enable_buoy_detector")
    enable_parkur2_stack = LaunchConfiguration("enable_parkur2_stack")
    enable_parkur3_stack = LaunchConfiguration("enable_parkur3_stack")
    enable_geofence = LaunchConfiguration("enable_geofence")
    enable_costmap_logger = LaunchConfiguration("enable_costmap_logger")
    enable_rc_kill = LaunchConfiguration("enable_rc_kill")
    enable_power_relay = LaunchConfiguration("enable_power_relay")
    enable_remote_kill = LaunchConfiguration("enable_remote_kill")
    enable_power_monitor = LaunchConfiguration("enable_power_monitor")
    remote_kill_port = LaunchConfiguration("remote_kill_port")
    power_monitor_port = LaunchConfiguration("power_monitor_port")
    mavros_bridge_enabled = LaunchConfiguration("mavros_bridge_enabled")
    mavros_output_mode = LaunchConfiguration("mavros_output_mode")
    yki_mavlink_url = LaunchConfiguration("yki_mavlink_url")
    model_path = LaunchConfiguration("model_path")
    enable_yolo = LaunchConfiguration("enable_yolo")
    color_image_topic = LaunchConfiguration("color_image_topic")
    depth_image_topic = LaunchConfiguration("depth_image_topic")
    camera_info_topic = LaunchConfiguration("camera_info_topic")
    vehicle_profile = LaunchConfiguration("vehicle_profile")
    enable_parkur2_only = PythonExpression(
        [
            "'",
            enable_parkur2_stack,
            "'.lower() == 'true' and '",
            enable_parkur3_stack,
            "'.lower() != 'true'",
        ]
    )

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
                "enable_buoy_detector",
                default_value="true",
                description="Start YOLO buoy detector and annotated recorder.",
            ),
            DeclareLaunchArgument(
                "enable_parkur2_stack",
                default_value="true",
                description="Start Parkur-2 LiDAR/camera corridor and obstacle stack.",
            ),
            DeclareLaunchArgument(
                "enable_parkur3_stack",
                default_value="false",
                description="Start Parkur-3 color handoff and target planner.",
            ),
            DeclareLaunchArgument(
                "enable_geofence",
                default_value="true",
                description="Monitor GPS course boundary and recover inward.",
            ),
            DeclareLaunchArgument(
                "enable_costmap_logger",
                default_value="true",
                description="Start lidar costmap recorder.",
            ),
            DeclareLaunchArgument(
                "enable_rc_kill",
                default_value="false",
                description="Read Pixhawk RC channel kill as backup.",
            ),
            DeclareLaunchArgument(
                "enable_power_relay",
                default_value="false",
                description="Drive a relay directly from Jetson GPIO.",
            ),
            DeclareLaunchArgument(
                "enable_remote_kill",
                default_value="false",
                description="Read LoRa/Arduino kill over serial.",
            ),
            DeclareLaunchArgument(
                "enable_power_monitor",
                default_value="false",
                description="Read Arduino battery/contactor status over serial.",
            ),
            DeclareLaunchArgument(
                "remote_kill_port",
                default_value="/dev/ttyUSB0",
                description="Serial port for LoRa/Arduino kill bridge.",
            ),
            DeclareLaunchArgument(
                "power_monitor_port",
                default_value="/dev/ttyUSB1",
                description="Serial port for Arduino power monitor.",
            ),
            DeclareLaunchArgument(
                "mavros_bridge_enabled",
                default_value="true",
                description="Allow bridge to publish MAVROS commands.",
            ),
            DeclareLaunchArgument(
                "mavros_output_mode",
                default_value="manual_control",
                description="disabled, cmd_vel, or manual_control.",
            ),
            DeclareLaunchArgument(
                "yki_mavlink_url",
                default_value="udpout:127.0.0.1:14550",
                description="MAVLink connection URL for YKI telemetry/commands.",
            ),
            DeclareLaunchArgument(
                "model_path",
                default_value="models/buoy_yolo.pt",
                description="YOLO buoy model path on Jetson.",
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
            DeclareLaunchArgument(
                "vehicle_profile",
                default_value="ida_katamaran",
                description="Arac profili YAML adi (config/vehicle_profiles/ altinda).",
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
                executable="geofence_monitor_node",
                name="geofence_monitor_node",
                output="screen",
                condition=IfCondition(enable_geofence),
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
                condition=IfCondition(enable_rc_kill),
                parameters=[config_file],
            ),
            Node(
                package="ida_otonom",
                executable="power_relay_node",
                name="power_relay_node",
                output="screen",
                condition=IfCondition(enable_power_relay),
                parameters=[
                    config_file,
                    {
                        "enabled": ParameterValue(
                            enable_power_relay,
                            value_type=bool,
                        )
                    },
                ],
            ),
            Node(
                package="ida_otonom",
                executable="remote_kill_node",
                name="remote_kill_node",
                output="screen",
                condition=IfCondition(enable_remote_kill),
                parameters=[
                    config_file,
                    {
                        "enabled": ParameterValue(
                            enable_remote_kill,
                            value_type=bool,
                        ),
                        "port": ParameterValue(remote_kill_port, value_type=str),
                    },
                ],
            ),
            Node(
                package="ida_otonom",
                executable="power_monitor_node",
                name="power_monitor_node",
                output="screen",
                condition=IfCondition(enable_power_monitor),
                parameters=[
                    config_file,
                    {
                        "enabled": ParameterValue(
                            enable_power_monitor,
                            value_type=bool,
                        ),
                        "port": ParameterValue(
                            power_monitor_port,
                            value_type=str,
                        ),
                    },
                ],
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
                executable="buoy_detector_node",
                name="buoy_detector_node",
                output="screen",
                condition=IfCondition(enable_buoy_detector),
                parameters=[
                    config_file,
                    log_params,
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
                        "record_dir": ParameterValue(log_dir, value_type=str),
                        "detection_topic": "/perception/buoy_detections_raw",
                    },
                ],
            ),
            Node(
                package="ida_otonom",
                executable="lidar_processor_node",
                name="lidar_processor_node",
                output="screen",
                condition=IfCondition(enable_parkur2_only),
                parameters=[
                    config_file,
                    {
                        "vehicle_profile": ParameterValue(
                            vehicle_profile,
                            value_type=str,
                        )
                    },
                ],
            ),
            Node(
                package="ida_otonom",
                executable="sensor_cross_validator_node",
                name="sensor_cross_validator_node",
                output="screen",
                condition=IfCondition(enable_parkur2_only),
                parameters=[config_file],
            ),
            Node(
                package="ida_otonom",
                executable="course_memory_node",
                name="course_memory_node",
                output="screen",
                condition=IfCondition(enable_parkur2_only),
                parameters=[config_file],
            ),
            Node(
                package="ida_otonom",
                executable="semantic_buoy_classifier_node",
                name="semantic_buoy_classifier_node",
                output="screen",
                condition=IfCondition(enable_parkur2_only),
                parameters=[config_file],
            ),
            Node(
                package="ida_otonom",
                executable="corridor_tracker_node",
                name="corridor_tracker_node",
                output="screen",
                condition=IfCondition(enable_parkur2_only),
                parameters=[
                    config_file,
                    {
                        "vehicle_profile": ParameterValue(
                            vehicle_profile,
                            value_type=str,
                        )
                    },
                ],
            ),
            Node(
                package="ida_otonom",
                executable="parkur2_planner_node",
                name="parkur2_planner_node",
                output="screen",
                condition=IfCondition(enable_parkur2_only),
                parameters=[
                    config_file,
                    {
                        "vehicle_profile": ParameterValue(
                            vehicle_profile,
                            value_type=str,
                        )
                    },
                ],
            ),
            Node(
                package="ida_otonom",
                executable="color_receiver_node",
                name="color_receiver_node",
                output="screen",
                condition=IfCondition(enable_parkur3_stack),
                parameters=[config_file],
            ),
            Node(
                package="ida_otonom",
                executable="color_buoy_finder_node",
                name="color_buoy_finder_node",
                output="screen",
                condition=IfCondition(enable_parkur3_stack),
                parameters=[config_file],
            ),
            Node(
                package="ida_otonom",
                executable="parkur3_planner_node",
                name="parkur3_planner_node",
                output="screen",
                condition=IfCondition(enable_parkur3_stack),
                parameters=[config_file],
            ),
            Node(
                package="ida_otonom",
                executable="yki_bridge_node",
                name="yki_bridge_node",
                output="screen",
                condition=IfCondition(enable_yki_bridge),
                parameters=[
                    config_file,
                    {
                        "mavlink_connection_url": ParameterValue(
                            yki_mavlink_url,
                            value_type=str,
                        ),
                    },
                ],
            ),
        ]
    )
