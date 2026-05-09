"""
Prueba 3: Navegación con obstáculos en movimiento  (ROS 2)
FIX original: usa Master_static (tiene lógica de rebase)
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
            package='object_detection',
            executable='object_detection',
            name='object_detection',
            output='screen',
        ),

        # Igual que en el original: usa Master_static para la lógica de rebase
        Node(
            package='control',
            executable='Master_static',
            name='Master',
            output='screen',
        ),
    ])
