"""
TMR 2026 – Main Launch File  (ROS 2)
════════════════════════════════════════════════════════════════
Cámaras:
  • Arducam IMX219  (CSI)  →  v4l2_camera  →  /arducam/image_raw
                                                solo para lane_detection
  • Orbbec Astra Pro (USB) →  orbbec_camera →  /camera/color/image_raw
                                                /camera/depth/image_raw
                                                para stop_sign + object_detection

Hardware ESP32 via micro-ROS:
  • motor_driver, lights_controller, ultrasonic_sensors
  • Requiere: ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0 -b 115200

Uso:
  Prueba 1 – Señales de tránsito     : ros2 launch AVIM tmr2026_main.launch.py prueba:=1
  Prueba 2 – Obstáculos estáticos    : ros2 launch AVIM tmr2026_main.launch.py prueba:=2
  Prueba 3 – Obstáculos dinámicos    : ros2 launch AVIM tmr2026_main.launch.py prueba:=3
  Prueba 4 – Estacionamiento         : ros2 launch AVIM tmr2026_main.launch.py prueba:=4
════════════════════════════════════════════════════════════════
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node, PushRosNamespace


def generate_launch_description():

    # ══════════════════════════════════════════════════════════════════════
    #  ARGUMENTOS
    # ══════════════════════════════════════════════════════════════════════
    args = [
        DeclareLaunchArgument('prueba',         default_value='1',
            description='Prueba TMR: 1=señales, 2=estáticos, 3=dinámicos, 4=parking'),
        DeclareLaunchArgument('enable_teleop',  default_value='true',
            description='Habilitar teleoperación PS4'),
        DeclareLaunchArgument('debug_vision',   default_value='false',
            description='Publicar imágenes de debug de visión'),
        DeclareLaunchArgument('arducam_device', default_value='/dev/video0',
            description='Dispositivo v4l2 de la Arducam IMX219 (CSI)'),
    ]

    prueba        = LaunchConfiguration('prueba')
    enable_teleop = LaunchConfiguration('enable_teleop')
    debug_vision  = LaunchConfiguration('debug_vision')
    arducam_dev   = LaunchConfiguration('arducam_device')

    # ══════════════════════════════════════════════════════════════════════
    #  CÁMARA 1: Arducam IMX219  (CSI → v4l2)
    #  Solo se usa para lane_detection.cpp
    #  Tópico publicado: /arducam/image_raw
    # ══════════════════════════════════════════════════════════════════════
    # NOTA: En Ubuntu 22.04 + ROS 2 Humble la cámara CSI de la Pi
    # aparece como /dev/video0 a través del stack libcamera/v4l2.
    # Si tienes 2 cámaras USB además, puede ser /dev/video2 o /dev/video4.
    # Verificar con: ls /dev/video*  y  v4l2-ctl --list-devices
    arducam_node = Node(
        package='v4l2_camera',
        executable='v4l2_camera_node',
        name='arducam',
        namespace='arducam',        # → tópicos bajo /arducam/
        output='screen',
        respawn=True,
        parameters=[{
            'video_device':  arducam_dev,
            'image_size':    [320, 240],
            'camera_frame_id': 'arducam_frame',
            'pixel_format':  'YUYV',
        }],
        # Renombrar el tópico estándar de v4l2_camera al nombre que
        # espera lane_detection
        remappings=[
            ('/arducam/image_raw', '/arducam/image_raw'),
        ],
    )

    # ══════════════════════════════════════════════════════════════════════
    #  CÁMARA 2: Orbbec Astra Pro  (USB)
    #  Publica:
    #    /camera/color/image_raw   → stop_sign_detector + object_detection
    #    /camera/depth/image_raw   → object_detection (profundidad)
    # ══════════════════════════════════════════════════════════════════════
    # NOTA: instalar driver con:
    #   sudo apt install ros-humble-orbbec-camera
    # Luego: ros2 launch orbbec_camera astra_pro.launch.py
    # (aquí lo lanzamos inline para tener un solo launch point)
    astra_node = Node(
        package='orbbec_camera',
        executable='orbbec_camera_node',
        name='orbbec_camera',
        namespace='camera',         # → tópicos bajo /camera/
        output='screen',
        respawn=True,
        parameters=[{
            'camera_name':        'camera',
            'depth_registration': True,     # alinea depth con color
            'color_width':        640,
            'color_height':       480,
            'color_fps':          30,
            'depth_width':        640,
            'depth_height':       480,
            'depth_fps':          30,
            'enable_point_cloud': False,    # no necesario para tu caso
        }],
    )

    # ══════════════════════════════════════════════════════════════════════
    #  VISIÓN
    # ══════════════════════════════════════════════════════════════════════
    vision_group = GroupAction([
        PushRosNamespace('vision'),

        # ── Lane detection (usa Arducam IMX219 CSI) ───────────────────────
        Node(
            package='lane_detection',
            executable='lane_detection',
            name='lane_detection',
            output='screen',
            respawn=True,
            parameters=[{
                # Suscrito a la Arducam, NO a la Astra Pro
                'camera_topic': '/arducam/image_raw',
                'debug_output':  debug_vision,
            }],
        ),

        # ── STOP sign detector (usa Orbbec Astra Pro RGB) ─────────────────
        # Solo en prueba 1 (señales de tránsito)
        Node(
            package='lane_detection',
            executable='stop_sign_detector.py',
            name='stop_sign_detector',
            output='screen',
            respawn=True,
            parameters=[{
                # Suscrito a color de la Astra Pro
                'camera_topic':         '/camera/color/image_raw',
                'use_yolo':             False,
                'detection_threshold':  3,
            }],
            condition=IfCondition(PythonExpression(["'", prueba, "' == '1'"])),
        ),

        # ── Object detection normal (pruebas 1, 2, 3) ─────────────────────
        # Usa Astra Pro (color + depth para detección 3D de obstáculos)
        Node(
            package='object_detection',
            executable='object_detection',
            name='object_detection',
            output='screen',
            respawn=True,
            parameters=[{
                'color_topic': '/camera/color/image_raw',
                'depth_topic': '/camera/depth/image_raw',
            }],
            condition=UnlessCondition(PythonExpression(["'", prueba, "' == '4'"])),
        ),

        # ── Object detection parking (prueba 4) ───────────────────────────
        Node(
            package='object_detection_parking',
            executable='object_detection_parking',
            name='object_detection_parking',
            output='screen',
            respawn=True,
            parameters=[{
                'color_topic': '/camera/color/image_raw',
                'depth_topic': '/camera/depth/image_raw',
            }],
            condition=IfCondition(PythonExpression(["'", prueba, "' == '4'"])),
        ),
    ])

    # ══════════════════════════════════════════════════════════════════════
    #  CONTROL (Master según prueba)
    # ══════════════════════════════════════════════════════════════════════
    masters = [
        Node(package='control', executable='Master',          name='Master',
             output='screen', respawn=True,
             condition=IfCondition(PythonExpression(["'", prueba, "' == '1'"]))),
        Node(package='control', executable='Master_static',   name='Master',
             output='screen', respawn=True,
             condition=IfCondition(PythonExpression(["'", prueba, "' == '2'"]))),
        Node(package='control', executable='Master_static',   name='Master',
             output='screen', respawn=True,
             condition=IfCondition(PythonExpression(["'", prueba, "' == '3'"]))),
        Node(package='control', executable='Master_parking',  name='Master',
             output='screen', respawn=True,
             condition=IfCondition(PythonExpression(["'", prueba, "' == '4'"]))),
    ]

    # ══════════════════════════════════════════════════════════════════════
    #  TELEOPERACIÓN PS4 (corre en la Pi, hardware en ESP32)
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

    # ══════════════════════════════════════════════════════════════════════
    #  LAUNCH DESCRIPTION
    # ══════════════════════════════════════════════════════════════════════
    return LaunchDescription(
        args + [
            arducam_node,
            astra_node,
            vision_group,
            *masters,
            teleop_group,
        ]
    )
