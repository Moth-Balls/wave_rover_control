import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
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
            parameters=[{'use_sim_time': False}]
        ),
        Node(
            package='wave_rover_control',
            executable='serial_gateway',
            name='serial_gateway',
            output='screen',
            parameters=[{'use_sim_time': False}]
        ),
    ])