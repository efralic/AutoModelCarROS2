"""
navigation_with_static_obstacles.launch.py  –  Prueba 2 del TMR 2026
Lanza:
  1. Cámara Nuwa-HP60C (driver ascamera) → publica depth + RGB
  2. Arducam IMX219 (camera_ros) → /camera/image_raw
  3. lane_detection                  → /distance_center_line
  4. object_detection                → /objects_points (lee Nuwa-HP60C)
  5. Master_static                   → toma decisiones de evasión
  6. hardware nodes (motor, ultrasonidos)

Cambios respecto a la versión Orbbec:
  • Se ELIMINÓ orbbec_camera_node
  • Se AGREGÓ IncludeLaunchDescription(hp60c.launch.py) del paquete ascamera
  • El tópico de profundidad que ahora consume object_detection es:
        /ascamera_hp60c/camera_publisher/depth0/image_raw
"""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os


def generate_launch_description():

    # ── 1. Cámara Nuwa-HP60C  (driver ascamera) ──────────────────────────────
    # Incluye hp60c.launch.py del paquete ascamera (workspace ascam_ros2_ws).
    # Esto lanza el nodo ascamera_node que publica:
    #   /ascamera_hp60c/camera_publisher/rgb0/image
    #   /ascamera_hp60c/camera_publisher/depth0/image_raw  ← lo que usamos
    #   /ascamera_hp60c/camera_publisher/depth0/points
    nuwa_camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare('ascamera'),
            '/launch/hp60c.launch.py'
        ])
    )

    # ── 2. Arducam IMX219 (cámara de carril) ─────────────────────────────────
    arducam_node = Node(
        package='camera_ros',
        executable='camera_node',
        name='arducam_csi',
        parameters=[{
            'width':  640,
            'height': 480,
            'format': 'YUYV',
            'camera': '/base/soc/i2c0mux/i2c@1/imx219@10',
        }],
        output='screen',
    )

    # ── 3. Detección de carril ──────────────────────────────────────────────
    lane_detection_node = Node(
        package='lane_detection',
        executable='lane_detection',
        name='lane_detection',
        parameters=[{
            'camera_topic':  '/camera/image_raw',
            'debug_output':  False,
        }],
        output='screen',
    )

    # ── 4. Detección de objetos con Nuwa-HP60C ──────────────────────────────
    object_detection_node = Node(
        package='object_detection',
        executable='object_detection_node',
        name='object_detection',
        parameters=[{
            # Tópico de profundidad de la Nuwa-HP60C
            'depth_topic':
                '/ascamera_hp60c/camera_publisher/depth0/image_raw',
            'minimum_points': 1,
            'epsilon':        95,
            'depth_step':     8,
        }],
        output='screen',
    )

    # ── 5. Master_static (prueba 2 – obstáculos estáticos) ──────────────────
    master_static_node = Node(
        package='control',
        executable='Master_static',
        name='master_static',
        output='screen',
    )

    # ── 6. Hardware en la Pi ────────────────────────────────────────────────
    motor_driver_node = Node(
        package='hardware_interface',
        executable='motor_driver.py',
        name='motor_driver',
        output='screen',
        respawn=True,
    )

    ultrasonic_node = Node(
        package='hardware_interface',
        executable='ultrasonic_sensors.py',
        name='ultrasonic_sensors',
        output='screen',
        respawn=True,
    )

    return LaunchDescription([
        nuwa_camera_launch,
        arducam_node,
        lane_detection_node,
        object_detection_node,
        master_static_node,
        motor_driver_node,
        ultrasonic_node,
    ])
