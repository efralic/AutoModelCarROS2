"""
TMR 2026 - Teleoperación independiente  (ROS 2)
Lanza solo los nodos necesarios para control manual con PS4.
NO lanza ningún Master autónomo.

Uso:
  ros2 launch AVIM teleop.launch.py
"""

from launch import LaunchDescription
from launch.actions import GroupAction
from launch_ros.actions import Node, PushRosNamespace


def generate_launch_description():
    return LaunchDescription([

        # ── Cámara USB ───────────────────────────────────────────────────────
        # NOTA: En ROS 2 el ejecutable de usb_cam es 'usb_cam_node_exe'
        Node(
            package='usb_cam',
            executable='usb_cam_node_exe',
            name='usb_cam',
            output='screen',
            respawn=True,
            parameters=[{
                'video_device':    '/dev/video0',
                'image_width':     320,
                'image_height':    240,
                'pixel_format':    'yuyv',
                'camera_frame_id': 'camera',
                'io_method':       'mmap',
                'framerate':       30.0,
            }],
        ),

        # ── Hardware ─────────────────────────────────────────────────────────
        GroupAction([
            PushRosNamespace('hardware'),
            Node(
                package='hardware_interface',
                executable='motor_driver.py',
                name='motor_driver',
                output='screen',
                respawn=True,
            ),
            Node(
                package='hardware_interface',
                executable='lights_controller.py',
                name='lights_controller',
                output='screen',
                respawn=True,
            ),
        ]),

        # ── Joystick PS4 ─────────────────────────────────────────────────────
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            output='screen',
            parameters=[{
                'dev':             '/dev/input/js0',
                'deadzone':        0.1,
                'autorepeat_rate': 20.0,
            }],
        ),

        # ── Teleop controller ────────────────────────────────────────────────
        Node(
            package='hardware_interface',
            executable='teleop_controller.py',
            name='teleop_controller',
            output='screen',
        ),
    ])
