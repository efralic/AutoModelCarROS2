#!/usr/bin/env python3
"""
Ultrasonic Sensors Reader  –  ROS 2 (Humble / Iron)
Raspberry Pi 4B  –  4 sensores HC-SR04 vía pigpio

Pines GPIO (BCM):
  front : TRIG=5,  ECHO=6
  back  : TRIG=16, ECHO=20
  left  : TRIG=21, ECHO=26
  right : TRIG=19, ECHO=12

Cambios ROS 1 → ROS 2:
  • rospy → rclpy
  • rospy.Publisher → create_publisher
  • rospy.Time.now() → self.get_clock().now().to_msg()
  • rospy.Rate → create_timer
  • Range.ULTRASOUND → sensor_msgs.msg.Range.ULTRASOUND (igual)
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range
import pigpio
import time


class UltrasonicSensors(Node):

    def __init__(self):
        super().__init__('ultrasonic_sensors')

        # ── Pines GPIO (trigger, echo) ────────────────────────────────────
        self.sensors = {
            'front': {'trigger': 5,  'echo': 6 },
            'back':  {'trigger': 16, 'echo': 20},
            'left':  {'trigger': 21, 'echo': 26},
            'right': {'trigger': 19, 'echo': 12},
        }

        # ── pigpio ────────────────────────────────────────────────────────
        self.pi = pigpio.pi()
        if not self.pi.connected:
            self.get_logger().error(
                "pigpio no conectado. Ejecuta: sudo systemctl start pigpiod")
            raise SystemExit(1)

        # Configurar pines
        for name, pins in self.sensors.items():
            self.pi.set_mode(pins['trigger'], pigpio.OUTPUT)
            self.pi.set_mode(pins['echo'],    pigpio.INPUT)
            self.pi.write(pins['trigger'], 0)

        # ── Publishers ────────────────────────────────────────────────────
        # ROS 1: rospy.Publisher(f'/sensors/ultrasonic/{name}', Range, queue_size=1)
        # ROS 2: create_publisher(Range, topic, 1)
        self.pubs = {}
        for name in self.sensors:
            self.pubs[name] = self.create_publisher(
                Range, f'/sensors/ultrasonic/{name}', 1)

        # ── Timer a 10 Hz (reemplaza rospy.Rate(10) en run()) ────────────
        # ROS 1: while not rospy.is_shutdown(): rate.sleep()
        # ROS 2: create_timer(periodo_segundos, callback)
        self.create_timer(0.1, self._publish_measurements)

        self.get_logger().info(
            "Ultrasonidos ROS 2 listos | front back left right | 10 Hz")

    # ── Medición HC-SR04 ──────────────────────────────────────────────────────
    def _measure_distance(self, trigger: int, echo: int) -> float:
        """
        Mide distancia con HC-SR04.
        Retorna distancia en metros (0.02 – 4.0 m) o -1 si hay timeout.
        """
        # Pulso trigger de 10 µs
        self.pi.gpio_trigger(trigger, 10, 1)

        timeout = time.time() + 0.1  # 100 ms timeout

        # Esperar inicio del echo
        pulse_start = time.time()
        while self.pi.read(echo) == 0:
            pulse_start = time.time()
            if pulse_start > timeout:
                return -1.0

        # Esperar fin del echo
        pulse_end = time.time()
        while self.pi.read(echo) == 1:
            pulse_end = time.time()
            if pulse_end > timeout:
                return -1.0

        # Calcular distancia
        pulse_duration = pulse_end - pulse_start
        distance_m     = pulse_duration * 17150 / 100   # cm → m

        return distance_m if 0.02 <= distance_m <= 4.0 else -1.0

    # ── Publicar mediciones (callback del timer) ──────────────────────────────
    def _publish_measurements(self):
        for name, pins in self.sensors.items():
            distance = self._measure_distance(
                pins['trigger'], pins['echo'])

            msg = Range()
            # ROS 1: msg.header.stamp = rospy.Time.now()
            # ROS 2: msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.stamp    = self.get_clock().now().to_msg()
            msg.header.frame_id = f"ultrasonic_{name}"
            msg.radiation_type  = Range.ULTRASOUND
            msg.field_of_view   = 0.26   # ~15°
            msg.min_range       = 0.02
            msg.max_range       = 4.0
            msg.range           = distance if distance > 0 else msg.max_range

            self.pubs[name].publish(msg)

    def cleanup(self):
        self.get_logger().info("Limpiando ultrasonidos...")
        self.pi.stop()


def main(args=None):
    rclpy.init(args=args)
    node = UltrasonicSensors()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Cerrando ultrasonic_sensors...")
    finally:
        node.cleanup()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
