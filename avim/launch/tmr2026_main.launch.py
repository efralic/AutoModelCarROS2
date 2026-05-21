"""
TMR 2026 – Main Launch File  (ROS 2)
════════════════════════════════════════════════════════════════
Cámaras:
  • Arducam IMX219 (CSI)  → camera_ros → /camera/image_raw
                            solo para lane_detection
  • Nuwa-HP60C (USB)      → ascamera   → /ascamera_hp60c/camera_publisher/...
                            para stop_sign + object_detection

Hardware en la Raspberry Pi:
  • motor_driver.py        (I2C 0x34 + servo pigpio)
  • ultrasonic_sensors.py  (HC-SR04 via pigpio)
  • teleop_controller.py   (joystick PS4)

Hardware ESP32 via micro-ROS:
  • SOLO LEDs (blinkers y señales)
  • Requiere micro-ROS Agent corriendo (systemd: microros-agent.service)

Uso:
  Prueba 1 – Señales de tránsito  : ros2 launch avim tmr2026_main.launch.py prueba:=1
  Prueba 2 – Obstáculos estáticos : ros2 launch avim tmr2026_main.launch.py prueba:=2
  Prueba 3 – Obstáculos dinámicos : ros2 launch avim tmr2026_main.launch.py prueba:=3
  Prueba 4 – Estacionamiento      : ros2 launch avim tmr2026_main.launch.py prueba:=4
════════════════════════════════════════════════════════════════
"""

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, GroupAction,
                            IncludeLaunchDescription)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    # ══════════════════════════════════════════════════════════════════════
    #  ARGUMENTOS
    # ══════════════════════════════════════════════════════════════════════
    args = [
        DeclareLaunchArgument('prueba',        default_value='1',
            description='Prueba TMR: 1=señales, 2=estáticos, 3=dinámicos, 4=parking'),
        DeclareLaunchArgument('enable_teleop', default_value='true',
            description='Habilitar teleoperación PS4'),
        DeclareLaunchArgument('debug_vision',  default_value='false',
            description='Publicar imágenes de debug de visión'),
    ]

    prueba        = LaunchConfiguration('prueba')
    enable_teleop = LaunchConfiguration('enable_teleop')
    debug_vision  = LaunchConfiguration('debug_vision')

    # ══════════════════════════════════════════════════════════════════════
    #  CÁMARA 1: Arducam IMX219  (CSI) → solo para lane_detection
    #  Publica: /camera/image_raw  (NV21, ~15 FPS)
    # ══════════════════════════════════════════════════════════════════════
    arducam_node = Node(
        package='camera_ros',
        executable='camera_node',
        name='arducam_csi',
        output='screen',
        respawn=True,
        parameters=[{
            'width':  640,
            'height': 480,
            'format': 'YUYV',
            'camera': '/base/soc/i2c0mux/i2c@1/imx219@10',
        }],
    )

    # ══════════════════════════════════════════════════════════════════════
    #  CÁMARA 2: Nuwa-HP60C (USB) → profundidad + RGB
    #  Publica:
    #    /ascamera_hp60c/camera_publisher/rgb0/image         (color)
    #    /ascamera_hp60c/camera_publisher/depth0/image_raw   (16UC1 mm)
    #    /ascamera_hp60c/camera_publisher/depth0/points      (point cloud)
    # ══════════════════════════════════════════════════════════════════════
    nuwa_camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare('ascamera'),
            '/launch/hp60c.launch.py'
        ])
    )

    # ══════════════════════════════════════════════════════════════════════
    #  VISIÓN
    # ══════════════════════════════════════════════════════════════════════
    vision_group = GroupAction([
        PushRosNamespace('vision'),

        # ── Lane detection (Arducam IMX219 CSI) ───────────────────────────
        Node(
            package='lane_detection',
            executable='lane_detection',
            name='lane_detection',
            output='screen',
            respawn=True,
            parameters=[{
                'camera_topic': '/camera/image_raw',
                'debug_output': debug_vision,
            }],
        ),

        # ── STOP sign detector (Nuwa-HP60C RGB) – solo prueba 1 ───────────
        Node(
            package='lane_detection',
            executable='stop_sign_detector.py',
            name='stop_sign_detector',
            output='screen',
            respawn=True,
            parameters=[{
                'camera_topic': '/ascamera_hp60c/camera_publisher/rgb0/image',
                'use_yolo':              False,
                'detection_threshold':   3,
            }],
            condition=IfCondition(PythonExpression(["'", prueba, "' == '1'"])),
        ),

        # ── Object detection (Nuwa-HP60C depth) – pruebas 1, 2, 3 ─────────
        Node(
            package='object_detection',
            executable='object_detection_node',
            name='object_detection',
            output='screen',
            respawn=True,
            parameters=[{
                'depth_topic':
                    '/ascamera_hp60c/camera_publisher/depth0/image_raw',
                'minimum_points': 1,
                'epsilon':        95,
                'depth_step':     8,
            }],
            condition=UnlessCondition(PythonExpression(["'", prueba, "' == '4'"])),
        ),

        # ── Object detection parking (Nuwa-HP60C depth) – prueba 4 ────────
        Node(
            package='object_detection_parking',
            executable='object_detection_parking_node',
            name='object_detection_parking',
            output='screen',
            respawn=True,
            parameters=[{
                'depth_topic':
                    '/ascamera_hp60c/camera_publisher/depth0/image_raw',
            }],
            condition=IfCondition(PythonExpression(["'", prueba, "' == '4'"])),
        ),
    ])

    # ══════════════════════════════════════════════════════════════════════
    #  CONTROL (Master según la prueba)
    # ══════════════════════════════════════════════════════════════════════
    masters = [
        Node(package='control', executable='Master',         name='Master',
             output='screen', respawn=True,
             condition=IfCondition(PythonExpression(["'", prueba, "' == '1'"]))),
        Node(package='control', executable='Master_static',  name='Master',
             output='screen', respawn=True,
             condition=IfCondition(PythonExpression(["'", prueba, "' == '2'"]))),
        Node(package='control', executable='Master_static',  name='Master',
             output='screen', respawn=True,
             condition=IfCondition(PythonExpression(["'", prueba, "' == '3'"]))),
        Node(package='control', executable='Master_parking', name='Master',
             output='screen', respawn=True,
             condition=IfCondition(PythonExpression(["'", prueba, "' == '4'"]))),
    ]

    # ══════════════════════════════════════════════════════════════════════
    #  HARDWARE EN LA PI (motor, ultrasonidos)
    # ══════════════════════════════════════════════════════════════════════
    hardware_pi = GroupAction([
        Node(
            package='hardware_interface',
            executable='motor_driver.py',
            name='motor_driver',
            output='screen',
            respawn=True,
        ),
        Node(
            package='hardware_interface',
            executable='ultrasonic_sensors.py',
            name='ultrasonic_sensors',
            output='screen',
            respawn=True,
        ),
    ])

    # ══════════════════════════════════════════════════════════════════════
    #  TELEOPERACIÓN PS4 (opcional)
    # ══════════════════════════════════════════════════════════════════════
    teleop_group = GroupAction(
        condition=IfCondition(enable_teleop),
        actions=[
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
            Node(
                package='hardware_interface',
                executable='teleop_controller.py',
                name='teleop_controller',
                output='screen',
            ),
        ],
    )

    return LaunchDescription(
        args + [
            arducam_node,
            nuwa_camera_launch,
            vision_group,
            *masters,
            hardware_pi,
            teleop_group,
        ]
    )
