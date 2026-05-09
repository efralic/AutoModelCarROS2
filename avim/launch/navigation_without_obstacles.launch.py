"""
Prueba 1: Reconocimiento de señales de tránsito  (ROS 2)
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([

        Node(
            package='lane_detection',
            executable='lane_detection',
            name='lane_detection',
            output='screen',
            parameters=[{'camera_topic': '/arducam/image_raw'}],
        ),

        Node(
            package='lane_detection',
            executable='stop_sign_detector.py',
            name='stop_sign_detector',
            output='screen',
            parameters=[{'camera_topic': '/usb_cam/image_raw'}],
        ),

        Node(
            package='control',
            executable='Master',
            name='Master',
            output='screen',
        ),
    ])
