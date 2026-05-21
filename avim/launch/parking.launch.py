"""
Prueba 4 – Estacionamiento autónomo
ROS 2
══════════════════════════════════════════════════════════════
Lanza:
  • Arducam IMX219          → /camera/image_raw (CSI)
  • Nuwa-HP60C              → /ascamera_hp60c/camera_publisher/depth0/image_raw
  • lane_detection
  • object_detection_parking (especializado para parking)
  • Master_parking
  • motor_driver.py + ultrasonic_sensors.py
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

    object_detection_parking_node = Node(
        package='object_detection_parking',
        executable='object_detection_parking',
        name='object_detection_parking',
        parameters=[{
            'depth_topic':
                '/ascamera_hp60c/camera_publisher/depth0/image_raw',
        }],
        output='screen',
    )

    # ── Control ──────────────────────────────────────────────────────────
    master_parking_node = Node(
        package='control',
        executable='Master_parking',
        name='Master_parking',
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
        object_detection_parking_node,
        master_parking_node,
        motor_driver_node,
        ultrasonic_node,
    ])
