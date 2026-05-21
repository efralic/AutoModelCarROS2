"""
Prueba 1 – Navegación SIN obstáculos (señales de tránsito)
ROS 2
══════════════════════════════════════════════════════════════
Lanza:
  • Arducam IMX219    → /camera/image_raw       (CSI, para lane_detection)
  • Nuwa-HP60C        → /ascamera_hp60c/...     (USB, para stop_sign + objects)
  • lane_detection    → /distance_center_line
  • stop_sign_detector → /stop_sign_detected
  • object_detection  → /objects_points
  • Master            → publica /AutoModelMini/manual_control/*
  • motor_driver.py + ultrasonic_sensors.py     (en la Pi)
══════════════════════════════════════════════════════════════
"""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    # ── Cámaras ──────────────────────────────────────────────────────────
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

    nuwa_camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare('ascamera'),
            '/launch/hp60c.launch.py'
        ])
    )

    # ── Visión ───────────────────────────────────────────────────────────
    lane_detection_node = Node(
        package='lane_detection',
        executable='lane_detection',
        name='lane_detection',
        parameters=[{'camera_topic': '/camera/image_raw'}],
        output='screen',
    )

    stop_sign_node = Node(
        package='lane_detection',
        executable='stop_sign_detector.py',
        name='stop_sign_detector',
        parameters=[{
            'camera_topic': '/ascamera_hp60c/camera_publisher/rgb0/image',
        }],
        output='screen',
    )

    object_detection_node = Node(
        package='object_detection',
        executable='object_detection_node',
        name='object_detection',
        parameters=[{
            'depth_topic':
                '/ascamera_hp60c/camera_publisher/depth0/image_raw',
        }],
        output='screen',
    )

    # ── Control ──────────────────────────────────────────────────────────
    master_node = Node(
        package='control',
        executable='Master',
        name='Master',
        output='screen',
    )

    # ── Hardware en la Pi ────────────────────────────────────────────────
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
        arducam_node,
        nuwa_camera_launch,
        lane_detection_node,
        stop_sign_node,
        object_detection_node,
        master_node,
        motor_driver_node,
        ultrasonic_node,
    ])
