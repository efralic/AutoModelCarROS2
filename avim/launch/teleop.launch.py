"""
Teleoperación con joystick PS4
ROS 2
══════════════════════════════════════════════════════════════
Lanza:
  • joy_node           → lee el joystick PS4 desde /dev/input/js0
  • teleop_controller  → traduce a /AutoModelMini/manual_control/*
  • motor_driver.py    → recibe comandos y mueve motores y servo

No lanza cámaras ni Masters: solo control manual del carrito.
══════════════════════════════════════════════════════════════
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        output='screen',
        parameters=[{
            'dev':             '/dev/input/js0',
            'deadzone':        0.1,
            'autorepeat_rate': 20.0,
        }],
    )

    teleop_node = Node(
        package='hardware_interface',
        executable='teleop_controller.py',
        name='teleop_controller',
        output='screen',
    )

    motor_driver_node = Node(
        package='hardware_interface',
        executable='motor_driver.py',
        name='motor_driver',
        output='screen',
        respawn=True,
    )

    return LaunchDescription([
        joy_node,
        teleop_node,
        motor_driver_node,
    ])
