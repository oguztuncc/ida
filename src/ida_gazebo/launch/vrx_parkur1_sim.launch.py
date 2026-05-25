from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, ExecuteProcess, RegisterEventHandler, Shutdown,
    TimerAction,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    ida_gazebo_dir = get_package_share_directory('ida_gazebo')
    ida_otonom_dir = get_package_share_directory('ida_otonom')
    vrx_gz_dir = get_package_share_directory('vrx_gz')

    model_file = os.path.join(
        ida_gazebo_dir, 'models', 'ida_katamaran', 'model.sdf'
    )
    world_file = os.path.join(vrx_gz_dir, 'worlds', 'sydney_regatta.sdf')
    default_mission = os.path.join(ida_otonom_dir, 'missions', 'parkur1U2.json')
    default_config = os.path.join(ida_otonom_dir, 'config', 'parkur1_sim.yaml')
    world_name = 'sydney_regatta'
    gui = LaunchConfiguration('gui')
    spawn_x = LaunchConfiguration('spawn_x')
    spawn_y = LaunchConfiguration('spawn_y')
    spawn_z = LaunchConfiguration('spawn_z')
    spawn_yaw = LaunchConfiguration('spawn_yaw')
    auto_start = LaunchConfiguration('auto_start')
    enable_waypoint_markers = LaunchConfiguration('enable_waypoint_markers')
    enable_course_objects = LaunchConfiguration('enable_course_objects')
    enable_sim_buoy_detector = LaunchConfiguration('enable_sim_buoy_detector')
    enable_corridor_planner = LaunchConfiguration('enable_corridor_planner')
    include_obstacles = LaunchConfiguration('include_obstacles')
    enable_evaluator = LaunchConfiguration('enable_evaluator')
    eval_timeout_s = LaunchConfiguration('eval_timeout_s')
    eval_arrival_radius_m = LaunchConfiguration('eval_arrival_radius_m')
    eval_max_cross_track_m = LaunchConfiguration('eval_max_cross_track_m')

    navsat_gz = (
        f'/world/{world_name}/model/ida_katamaran/link/base_link/'
        'sensor/gps_sensor/navsat'
    )
    imu_gz = (
        f'/world/{world_name}/model/ida_katamaran/link/base_link/'
        'sensor/imu_sensor/imu'
    )
    scan_gz = (
        f'/world/{world_name}/model/ida_katamaran/link/base_link/'
        'sensor/lidar_sensor/scan'
    )

    gz_gui = ExecuteProcess(
        cmd=['gz', 'sim', '-r', '-v4', LaunchConfiguration('world')],
        output='screen',
        condition=IfCondition(gui)
    )
    gz_headless = ExecuteProcess(
        cmd=['gz', 'sim', '-r', '-s', '-v4', LaunchConfiguration('world')],
        output='screen',
        condition=UnlessCondition(gui)
    )

    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-file', model_file,
            '-name', 'ida_katamaran',
            '-x', spawn_x, '-y', spawn_y, '-z', spawn_z, '-Y', spawn_yaw
        ],
        output='screen'
    )
    spawn_timer = TimerAction(period=10.0, actions=[spawn_entity])
    evaluator_node = Node(
        package='ida_gazebo',
        executable='mission_eval_node.py',
        name='mission_eval_node',
        output='screen',
        condition=IfCondition(enable_evaluator),
        parameters=[{
            'mission_file': LaunchConfiguration('mission_file'),
            'timeout_s': ParameterValue(eval_timeout_s, value_type=float),
            'arrival_radius_m': ParameterValue(
                eval_arrival_radius_m,
                value_type=float,
            ),
            'max_cross_track_m': ParameterValue(
                eval_max_cross_track_m,
                value_type=float,
            ),
            'max_stall_s': 35.0,
            'shutdown_on_finish': True,
        }]
    )

    return LaunchDescription([
        DeclareLaunchArgument('mission_file', default_value=default_mission),
        DeclareLaunchArgument('config_file', default_value=default_config),
        DeclareLaunchArgument('world', default_value=world_file),
        DeclareLaunchArgument('gui', default_value='true',
                              description='Gazebo GUI penceresini ac'),
        DeclareLaunchArgument('spawn_x', default_value='-528.0',
                              description='IDA spawn X/east konumu'),
        DeclareLaunchArgument('spawn_y', default_value='193.0',
                              description='IDA spawn Y/north konumu'),
        DeclareLaunchArgument('spawn_z', default_value='0.0',
                              description='IDA spawn yuksekligi'),
        DeclareLaunchArgument('spawn_yaw', default_value='0.646',
                              description='IDA spawn yaw radyan'),
        DeclareLaunchArgument('auto_start', default_value='false',
                              description='Mission otomatik baslasin'),
        DeclareLaunchArgument('enable_waypoint_markers', default_value='true',
                              description='Waypoint markerlarini Gazeboya ekle'),
        DeclareLaunchArgument('enable_course_objects', default_value='true',
                              description='Parkur duba ve engellerini ekle'),
        DeclareLaunchArgument('enable_sim_buoy_detector', default_value='true',
                              description='JSON tabanli sim duba algisi ac'),
        DeclareLaunchArgument('enable_corridor_planner', default_value='true',
                              description='Koridor/planner stackini ac'),
        DeclareLaunchArgument('include_obstacles', default_value='true',
                              description='Obstacle dubalarini simde kullan'),
        DeclareLaunchArgument('enable_evaluator', default_value='false',
                              description='Headless gorev evaluatorunu ac'),
        DeclareLaunchArgument('eval_timeout_s', default_value='180.0',
                              description='Evaluator timeout saniye'),
        DeclareLaunchArgument('eval_arrival_radius_m', default_value='1.1',
                              description='Evaluator waypoint kabul yaricapi'),
        DeclareLaunchArgument('eval_max_cross_track_m', default_value='5.5',
                              description='Evaluator maksimum hattan sapma'),

        gz_gui,
        gz_headless,
        spawn_timer,

        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=[
                f'{scan_gz}@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
                (
                    '/model/ida_katamaran/front_camera/image@'
                    'sensor_msgs/msg/Image[gz.msgs.Image'
                ),
                (
                    '/model/ida_katamaran/front_camera/camera_info@'
                    'sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo'
                ),
                f'{imu_gz}@sensor_msgs/msg/Imu[gz.msgs.IMU',
                f'{navsat_gz}@sensor_msgs/msg/NavSatFix[gz.msgs.NavSat',
                '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            ],
            remappings=[
                (scan_gz, '/scan'),
                (
                    '/model/ida_katamaran/front_camera/image',
                    '/camera/color/image_raw',
                ),
                (
                    '/model/ida_katamaran/front_camera/camera_info',
                    '/camera/color/camera_info',
                ),
                (imu_gz, '/model/ida_katamaran/imu'),
                (navsat_gz, '/model/ida_katamaran/navsat'),
            ],
            output='screen'
        ),

        Node(
            package='ida_gazebo',
            executable='sim_nav_converter.py',
            name='sim_nav_converter',
            output='screen',
            parameters=[{
                'sim_origin_lat': -33.724223,
                'sim_origin_lon': 150.679736,
                'spawn_east_m': ParameterValue(spawn_x, value_type=float),
                'spawn_north_m': ParameterValue(spawn_y, value_type=float),
                'target_origin_lat': 40.1181,
                'target_origin_lon': 26.4081,
            }]
        ),

        Node(
            package='ida_gazebo',
            executable='cmd_vel_to_thrust.py',
            name='cmd_vel_to_thrust',
            output='screen',
            parameters=[{
                'cmd_vel_topic': '/control/cmd_vel_safe',
                'wheelbase_m': 0.60,
                'max_thrust': 58.9,
                'yaw_sign': -1.0,
            }]
        ),

        Node(
            package='ida_gazebo',
            executable='spawn_waypoint_markers.py',
            name='waypoint_marker_spawner',
            output='screen',
            condition=IfCondition(enable_waypoint_markers),
            parameters=[{
                'mission_file': LaunchConfiguration('mission_file'),
                'spawn_x': ParameterValue(spawn_x, value_type=float),
                'spawn_y': ParameterValue(spawn_y, value_type=float),
                'marker_z': 0.65,
                'world_name': world_name,
                'spawn_delay_s': 14.0,
            }]
        ),
        Node(
            package='ida_gazebo',
            executable='spawn_course_objects.py',
            name='course_object_spawner',
            output='screen',
            condition=IfCondition(enable_course_objects),
            parameters=[{
                'mission_file': LaunchConfiguration('mission_file'),
                'spawn_x': ParameterValue(spawn_x, value_type=float),
                'spawn_y': ParameterValue(spawn_y, value_type=float),
                'world_name': world_name,
                'spawn_delay_s': 14.0,
                'buoy_radius_m': 0.15,
                'buoy_height_m': 0.90,
                'include_boundaries': True,
                'include_obstacles': ParameterValue(
                    include_obstacles,
                    value_type=bool,
                ),
            }]
        ),
        Node(
            package='ida_gazebo',
            executable='sim_buoy_detector.py',
            name='sim_buoy_detector',
            output='screen',
            condition=IfCondition(enable_sim_buoy_detector),
            parameters=[{
                'mission_file': LaunchConfiguration('mission_file'),
                'detection_range_max_m': 18.0,
                'detection_fov_deg': 180.0,
                'publish_rate_hz': 8.0,
                'include_boundaries': True,
                'include_obstacles': ParameterValue(
                    include_obstacles,
                    value_type=bool,
                ),
            }]
        ),
        evaluator_node,
        RegisterEventHandler(
            OnProcessExit(
                target_action=evaluator_node,
                on_exit=[Shutdown(reason='mission evaluator finished')],
            )
        ),

        Node(
            package='ida_otonom',
            executable='mission_manager_node',
            name='mission_manager_node',
            output='screen',
            parameters=[
                LaunchConfiguration('config_file'),
                {'mission_file': LaunchConfiguration('mission_file')},
                {'auto_start': ParameterValue(auto_start, value_type=bool)},
            ]
        ),
        Node(
            package='ida_otonom',
            executable='gps_guidance_node',
            name='gps_guidance_node',
            output='screen',
            parameters=[LaunchConfiguration('config_file')]
        ),
        Node(
            package='ida_otonom',
            executable='controller_node',
            name='controller_node',
            output='screen',
            parameters=[LaunchConfiguration('config_file')]
        ),
        Node(
            package='ida_otonom',
            executable='lidar_processor_node',
            name='lidar_processor_node',
            output='screen',
            parameters=[LaunchConfiguration('config_file')]
        ),
        Node(
            package='ida_otonom',
            executable='sensor_cross_validator_node',
            name='sensor_cross_validator_node',
            output='screen',
            condition=IfCondition(enable_corridor_planner),
            parameters=[LaunchConfiguration('config_file')]
        ),
        Node(
            package='ida_otonom',
            executable='course_memory_node',
            name='course_memory_node',
            output='screen',
            condition=IfCondition(enable_corridor_planner),
            parameters=[LaunchConfiguration('config_file')]
        ),
        Node(
            package='ida_otonom',
            executable='semantic_buoy_classifier_node',
            name='semantic_buoy_classifier_node',
            output='screen',
            condition=IfCondition(enable_corridor_planner),
            parameters=[LaunchConfiguration('config_file')]
        ),
        Node(
            package='ida_otonom',
            executable='corridor_tracker_node',
            name='corridor_tracker_node',
            output='screen',
            condition=IfCondition(enable_corridor_planner),
            parameters=[LaunchConfiguration('config_file')]
        ),
        Node(
            package='ida_otonom',
            executable='parkur2_planner_node',
            name='parkur2_planner_node',
            output='screen',
            condition=IfCondition(enable_corridor_planner),
            parameters=[LaunchConfiguration('config_file')]
        ),
        Node(
            package='ida_otonom',
            executable='safety_node',
            name='safety_node',
            output='screen',
            parameters=[LaunchConfiguration('config_file')]
        ),
    ])
