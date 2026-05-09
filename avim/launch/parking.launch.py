"""
Prueba 4: Estacionamiento autónomo  (ROS 2)
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
            package='object_detection_parking',
            executable='object_detection_parking',
            name='object_detection_parking',
            output='screen',
        ),

        Node(
            package='control',
            executable='Master_parking',
            name='Master_parking',
            output='screen',
        ),
    ])
