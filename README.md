# AutoModelCarROS2

# AVIM – Autonomous Vehicle Interface Manager

> Sistema de conducción autónoma para el **TMR 2026** desarrollado sobre ROS 2 Humble.  
> Raspberry Pi 4B + ESP32 (micro-ROS) + Orbbec Astra Pro + Arducam IMX219.

---

## Tabla de contenido

- [Arquitectura del sistema](#arquitectura-del-sistema)
- [Hardware requerido](#hardware-requerido)
- [Estructura del repositorio](#estructura-del-repositorio)
- [Instalación rápida](#instalación-rápida)
- [Uso](#uso)
- [Pruebas individuales](#pruebas-individuales)
- [Tópicos ROS 2](#tópicos-ros-2)
- [Dependencias](#dependencias)

---

## Arquitectura del sistema

```
┌──────────────────────────────────────────────────────────────────┐
│                     RASPBERRY PI 4B  (ROS 2 Humble)              │
│                                                                  │
│  ┌─────────────┐  ┌──────────────┐  ┌───────────────────────┐   │
│  │lane_detection│  │object_detect.│  │  control (Masters)    │   │
│  │(Arducam CSI) │  │(Astra Pro)   │  │  Master.cpp           │   │
│  └─────────────┘  └──────────────┘  │  Master_static.cpp    │   │
│  ┌─────────────┐  ┌──────────────┐  │  Master_parking_*.cpp │   │
│  │stop_sign    │  │teleop        │  └───────────────────────┘   │
│  │detector     │  │controller    │                               │
│  └─────────────┘  └──────────────┘                               │
│                          │ USB Serial (/dev/ttyUSB0)             │
│              micro-ROS Agent ←──────────────────────────────────┐│
└─────────────────────────────────────────────────────────────────┘│
                                                                    │
┌───────────────────────────────────────────────────────────────── ┘
│                        ESP32  (micro-ROS)                        │
│                                                                  │
│  ┌──────────────┐  ┌─────────────────┐  ┌───────────────────┐   │
│  │ motor_driver │  │lights_controller│  │ultrasonic_sensors │   │
│  │ I2C + Servo  │  │   GPIO LEDs     │  │  HC-SR04 × 4      │   │
│  └──────────────┘  └─────────────────┘  └───────────────────┘   │
└──────────────────────────────────────────────────────────────────┘

Cámaras:
  Arducam IMX219 (CSI) ──→ /arducam/image_raw        → lane_detection
  Orbbec Astra Pro (USB) → /camera/color/image_raw   → stop_sign + object_detection
                           /camera/depth/image_raw   → object_detection (3D)
```

---

## Hardware requerido

| Componente | Modelo | Conexión |
|---|---|---|
| Computadora principal | Raspberry Pi 4B (4 GB RAM) | — |
| Microcontrolador | ESP32 DevKitC | USB Serial a Pi |
| Cámara de carril | Arducam IMX219 | CSI (ribbon cable) |
| Cámara de profundidad | Orbbec Astra Pro | USB-A |
| Módulo de motores | Encoder Motor Module (I2C 0x34) | I2C (GPIO 21/22 del ESP32) |
| Servo de dirección | Servo estándar | GPIO 13 del ESP32 |
| Sensores ultrasonido | HC-SR04 × 4 | GPIO del ESP32 |
| Control remoto | PS4 DualShock | USB/Bluetooth a Pi |
| Almacenamiento | microSD 32 GB (Clase 10) | — |

---

## Estructura del repositorio

```
AutoModelCarROS2/
├── README.md
├── .gitignore
│
├── AVIM/                        # Launch files principales
│   ├── launch/
│   │   ├── tmr2026_main.launch.py
│   │   ├── navigation_without_obstacles.launch.py
│   │   ├── navigation_with_static_obstacles.launch.py
│   │   ├── navigation_with_dynamic_obstacles.launch.py
│   │   ├── parking.launch.py
│   │   └── teleop.launch.py
│   ├── CMakeLists.txt
│   └── package.xml
│
├── control/                     # Nodos maestros de control (C++)
│   ├── src/
│   │   ├── Master.cpp                    # Prueba 1 y 3
│   │   ├── Master_static.cpp             # Prueba 2
│   │   ├── Master_parking_bateria.cpp    # Prueba 4 (batería)
│   │   └── Master_parking_paralelo.cpp   # Prueba 4 (paralelo)
│   ├── CMakeLists.txt
│   └── package.xml
│
├── hardware_interface/          # Teleop en la Pi
│   ├── src/
│   │   └── teleop_controller.py
│   ├── CMakeLists.txt
│   └── package.xml
│
├── esp32_firmware/              # Firmware del ESP32 (NO es paquete ROS 2)
│   └── avim_esp32_hardware.ino  # motor + lights + ultrasonic
│
├── lane_detection/              # Detección de carril y señal STOP
│   ├── src/
│   │   ├── lane_detection.cpp
│   │   └── stop_sign_detector.py
│   ├── CMakeLists.txt
│   └── package.xml
│
├── object_detection/            # Detección de obstáculos + msg custom
│   ├── msg/
│   │   └── PointsObjects.msg
│   ├── src/
│   │   └── object_detection.cpp
│   ├── CMakeLists.txt
│   └── package.xml
│
└── object_detection_parking/    # Detección para estacionamiento
    ├── src/
    │   └── object_detection_parking.cpp
    ├── CMakeLists.txt
    └── package.xml
```

---

## Instalación rápida

> Para la guía completa paso a paso (instalación de OS, drivers, configuración del ESP32, etc.)  
> consulta el archivo **`SETUP_GUIDE.docx`** incluido en este repositorio.

```bash
# 1. Crear workspace
mkdir -p ~/ros2_ws/src && cd ~/ros2_ws/src

# 2. Clonar repositorio
git clone https://github.com/tu-equipo/avim_ros2.git .

# 3. Instalar dependencias de ROS 2
sudo apt install -y \
  ros-humble-v4l2-camera \
  ros-humble-orbbec-camera \
  ros-humble-joy \
  ros-humble-cv-bridge \
  ros-humble-image-transport \
  ros-humble-micro-ros-agent

# 4. Compilar (orden importante: object_detection primero)
cd ~/ros2_ws
colcon build --packages-up-to object_detection_parking
source install/setup.bash

# 5. Iniciar micro-ROS Agent (en terminal separada)
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0 -b 115200
```

---

## Uso

### Lanzar una prueba completa

```bash
source ~/ros2_ws/install/setup.bash

# Prueba 1 – Reconocimiento de señales STOP
ros2 launch AVIM tmr2026_main.launch.py prueba:=1

# Prueba 2 – Obstáculos estáticos
ros2 launch AVIM tmr2026_main.launch.py prueba:=2

# Prueba 3 – Obstáculos dinámicos
ros2 launch AVIM tmr2026_main.launch.py prueba:=3

# Prueba 4 – Estacionamiento
ros2 launch AVIM tmr2026_main.launch.py prueba:=4

# Solo teleoperación
ros2 launch AVIM teleop.launch.py
```

### Argumentos opcionales

| Argumento | Default | Descripción |
|---|---|---|
| `prueba` | `1` | Número de prueba TMR (1-4) |
| `enable_teleop` | `true` | Habilitar joystick PS4 |
| `debug_vision` | `false` | Publicar imágenes de debug |
| `arducam_device` | `/dev/video0` | Dispositivo CSI de la Arducam |

```bash
# Ejemplo con opciones
ros2 launch AVIM tmr2026_main.launch.py prueba:=2 debug_vision:=true
```

---

## Pruebas individuales

```bash
# Ver todos los nodos activos
ros2 node list

# Ver todos los tópicos
ros2 topic list

# Verificar ESP32 conectado
ros2 node list | grep esp32

# Monitorear obstáculos detectados
ros2 topic echo /objects_points

# Monitorear distancia al centro de carril
ros2 topic echo /distance_center_line

# Monitorear ultrasonidos
ros2 topic echo /sensors/ultrasonic/front

# Ver imagen de debug del detector STOP
ros2 run rqt_image_view rqt_image_view /stop_sign_detector/debug_image

# Mover motores manualmente
ros2 topic pub /AutoModelMini/manual_control/speed std_msgs/msg/Int16 "data: -300"

# Centrar servo
ros2 topic pub /AutoModelMini/manual_control/steering std_msgs/msg/Int16 "data: 90"
```

---

## Tópicos ROS 2

### Publicados por los nodos

| Tópico | Tipo | Publicado por |
|---|---|---|
| `/distance_center_line` | `std_msgs/Int16` | `lane_detection` |
| `/stop_sign_detected` | `std_msgs/Bool` | `stop_sign_detector` |
| `/objects_points` | `object_detection/PointsObjects` | `object_detection` / `object_detection_parking` |
| `/sensors/ultrasonic/front` | `sensor_msgs/Range` | ESP32 |
| `/sensors/ultrasonic/back` | `sensor_msgs/Range` | ESP32 |
| `/sensors/ultrasonic/left` | `sensor_msgs/Range` | ESP32 |
| `/sensors/ultrasonic/right` | `sensor_msgs/Range` | ESP32 |
| `/arducam/image_raw` | `sensor_msgs/Image` | `v4l2_camera` |
| `/camera/color/image_raw` | `sensor_msgs/Image` | `orbbec_camera` |
| `/camera/depth/image_raw` | `sensor_msgs/Image` | `orbbec_camera` |

### Suscritos por los Masters

| Tópico | Tipo | Suscrito por |
|---|---|---|
| `/AutoModelMini/manual_control/speed` | `std_msgs/Int16` | ESP32 → motor |
| `/AutoModelMini/manual_control/steering` | `std_msgs/Int16` | ESP32 → servo |
| `/lights/blinkers` | `std_msgs/Bool` | ESP32 → LEDs |
| `/lights/left_signal` | `std_msgs/Bool` | ESP32 → LEDs |
| `/lights/right_signal` | `std_msgs/Bool` | ESP32 → LEDs |
| `/vehicle_mode` | `std_msgs/String` | ESP32 |
| `/emergency_stop` | `std_msgs/Bool` | ESP32 |

---

## Dependencias

### ROS 2 (apt)
```
ros-humble-rclcpp         ros-humble-rclpy
ros-humble-std-msgs       ros-humble-sensor-msgs
ros-humble-geometry-msgs  ros-humble-cv-bridge
ros-humble-image-transport ros-humble-v4l2-camera
ros-humble-orbbec-camera  ros-humble-joy
ros-humble-micro-ros-agent
```

### Sistema
```
libopencv-dev   python3-opencv   python3-colcon-common-extensions
```

### ESP32 (Arduino IDE)
```
micro_ros_arduino    ESP32Servo
```

---

## Controles PS4

| Botón/Eje | Acción |
|---|---|
| R2 | Acelerar hacia adelante |
| L2 | Reversa |
| Stick izquierdo (horizontal) | Dirección |
| Cruz ✕ | Cambiar modo Autónomo ↔ Teleop |
| Triángulo △ | Paro de emergencia |

---

## Equipo

> TMR 2026 — [Woosanos]  
> [Universidad Veracruzana]

---
