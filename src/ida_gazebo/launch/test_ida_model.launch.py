from launch import LaunchDescription
from launch.actions import ExecuteProcess, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Paket dizinlerini bul
    ida_gazebo_dir = get_package_share_directory('ida_gazebo')
    vrx_gz_dir = get_package_share_directory('vrx_gz')

    model_file = os.path.join(ida_gazebo_dir, 'models', 'ida_katamaran', 'model.sdf')
    world_file = os.path.join(vrx_gz_dir, 'worlds', 'sydney_regatta.sdf')

    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='false',
                              description='GUI ile çalıştır'),

        # Gazebo Sim server (GUI'siz test için -s flag)
        ExecuteProcess(
            cmd=['gz', 'sim', '-r', '-s', '-v4', world_file],
            output='screen',
            shell=False
        ),

        # IDA Katamaran modelini spawn et
        Node(
            package='ros_gz_sim',
            executable='create',
            arguments=[
                '-file', model_file,
                '-name', 'ida_katamaran',
                '-x', '0', '-y', '0', '-z', '0.5',
                '-Y', '0'
            ],
            output='screen'
        ),

        # Temel topic bridge (clock + lidar + camera)
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=[
                '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
                '/model/ida_katamaran/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
                '/model/ida_katamaran/front_camera/image@sensor_msgs/msg/Image[gz.msgs.Image',
                '/model/ida_katamaran/front_camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
                '/model/ida_katamaran/navsat@sensor_msgs/msg/NavSatFix[gz.msgs.NavSat',
                '/model/ida_katamaran/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
            ],
            output='screen'
        ),
    ])
