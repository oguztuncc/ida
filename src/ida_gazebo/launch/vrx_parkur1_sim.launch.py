from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, ExecuteProcess, TimerAction
)
from launch.conditions import IfCondition, UnlessCondition
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
    spawn_z = LaunchConfiguration('spawn_z')
    auto_start = LaunchConfiguration('auto_start')

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
            '-x', '-528.0', '-y', '193.0', '-z', spawn_z, '-Y', '0'
        ],
        output='screen'
    )
    spawn_timer = TimerAction(period=10.0, actions=[spawn_entity])

    return LaunchDescription([
        DeclareLaunchArgument('mission_file', default_value=default_mission),
        DeclareLaunchArgument('config_file', default_value=default_config),
        DeclareLaunchArgument('world', default_value=world_file),
        DeclareLaunchArgument('gui', default_value='true',
                              description='Gazebo GUI penceresini ac'),
        DeclareLaunchArgument('spawn_z', default_value='-0.05',
                              description='IDA spawn yuksekligi'),
        DeclareLaunchArgument('auto_start', default_value='false',
                              description='Mission otomatik baslasin'),

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
                'max_thrust': 20.0,
                'yaw_sign': -1.0,
            }]
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
            executable='safety_node',
            name='safety_node',
            output='screen',
            parameters=[LaunchConfiguration('config_file')]
        ),
    ])
