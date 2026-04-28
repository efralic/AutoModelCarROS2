"""
Prueba 2: Navegación con obstáculos estáticos  (ROS 2)
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
            parameters=[{'camera_topic': '/usb_cam/image_raw'}],
        ),

        Node(
            package='object_detection',
            executable='object_detection',
            name='object_detection',
            output='screen',
        ),

        Node(
            package='control',
            executable='Master_static',
            name='Master_static',
            output='screen',
        ),
    ])
