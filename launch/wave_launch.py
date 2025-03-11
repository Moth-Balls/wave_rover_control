import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    sllidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('sllidar_ros2'), 'launch', 'sllidar_a1_launch.py')
        )
    )

    imu_filter_config = os.path.join(
        get_package_share_directory('wave_rover_control'),
        'config',
        'imu_filter.yaml'
    )

    return LaunchDescription([
        Node(
            package='wave_rover_control',
            executable='controller',
            name='controller',
            output='screen',
            parameters=[{'use_sim_time': False}]
        ),
        Node(
            package='wave_rover_control',
            executable='platform_feedback',
            name='platform_feedback',
            output='screen',
            parameters=[{'use_sim_time': False, 'imu_filter_config': imu_filter_config}]
        ),
        Node(
            package='wave_rover_control',
            executable='serial_gateway',
            name='serial_gateway',
            output='screen',
            parameters=[{'use_sim_time': False}]
        ),
        sllidar_launch
    ])