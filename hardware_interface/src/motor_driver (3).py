#!/usr/bin/env python3
"""
Motor Driver  –  ROS 2 (Humble / Iron)
Raspberry Pi 4B
Controla motores vía Encoder Motor Module (I2C 0x34)
Controla servo de dirección vía pigpio hardware PWM

Cambios ROS 1 → ROS 2:
  • rospy → rclpy
  • rospy.init_node → Node.__init__
  • rospy.Subscriber → create_subscription
  • rospy.loginfo/warn → get_logger().info/warn
  • rospy.Rate / is_shutdown → create_timer
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int16, Bool, String
import smbus2
import pigpio
import time


# ── Registros del módulo motor I2C ───────────────────────────────────────────
I2C_ADDR                    = 0x34
MOTOR_TYPE_ADDR             = 0x14
MOTOR_ENCODER_POLARITY_ADDR = 0x15
MOTOR_FIXED_SPEED_ADDR      = 0x33
MOTOR_TYPE_JGB37_520_12V_110RPM = 3


class MotorDriver(Node):

    def __init__(self):
        super().__init__('motor_driver')

        # ── I2C bus (GPIO 2=SDA, GPIO 3=SCL → bus 1 en la Pi) ────────────
        try:
            self.bus = smbus2.SMBus(1)
            self.get_logger().info("I2C bus abierto en /dev/i2c-1")
        except Exception as e:
            self.get_logger().error(f"Error abriendo I2C bus: {e}")
            raise SystemExit(1)

        # ── pigpio para servo PWM ─────────────────────────────────────────
        self.pi = pigpio.pi()
        if not self.pi.connected:
            self.get_logger().error(
                "pigpio no conectado. Ejecuta: sudo systemctl start pigpiod")
            raise SystemExit(1)

        # ── Pines y límites del servo ─────────────────────────────────────
        self.SERVO_PIN       = 13
        self.SERVO_MIN_PULSE = 1222   # 65°
        self.SERVO_MAX_PULSE = 1778   # 115°
        self.SERVO_CENTER    = 1500   # 90°

        # ── Estado del vehículo ───────────────────────────────────────────
        self.autonomous_mode = True
        self.emergency_stop  = False

        # ── Inicializar hardware ──────────────────────────────────────────
        self._setup_motor_module()
        self.pi.set_servo_pulsewidth(self.SERVO_PIN, self.SERVO_CENTER)
        self.get_logger().info("Servo centrado en GPIO 13")

        # ── Subscriptores ─────────────────────────────────────────────────
        self.create_subscription(
            Int16, '/AutoModelMini/manual_control/speed',
            self.speed_callback, 1)
        self.create_subscription(
            Int16, '/AutoModelMini/manual_control/steering',
            self.steering_callback, 1)
        self.create_subscription(
            String, '/vehicle_mode',
            self.mode_callback, 1)
        self.create_subscription(
            Bool, '/emergency_stop',
            self.emergency_callback, 1)

        self.get_logger().info(
            "Motor Driver ROS 2 listo (Encoder Motor Module I2C 0x34)")

    # ── Setup del módulo motor ────────────────────────────────────────────────
    def _setup_motor_module(self):
        try:
            self.bus.write_i2c_block_data(
                I2C_ADDR, MOTOR_TYPE_ADDR,
                [MOTOR_TYPE_JGB37_520_12V_110RPM])
            time.sleep(0.05)
            self.bus.write_i2c_block_data(
                I2C_ADDR, MOTOR_ENCODER_POLARITY_ADDR, [0])
            time.sleep(0.05)
            self.get_logger().info(
                "Encoder Motor Module configurado (JGB37-520 12V 110RPM)")
        except Exception as e:
            self.get_logger().error(f"Error configurando módulo motor: {e}")

    def _write_motor_speeds(self, m1: int, m2: int, m3: int = 0, m4: int = 0):
        """
        Escribe velocidades a los 4 canales del módulo motor.
        Rango: -100 a 100. Convierte a complemento a 2 (byte sin signo).
        """
        def to_u8(v):
            v = max(-100, min(100, int(v)))
            return v & 0xFF

        data = [to_u8(m1), to_u8(m2), to_u8(m3), to_u8(m4)]
        try:
            self.bus.write_i2c_block_data(
                I2C_ADDR, MOTOR_FIXED_SPEED_ADDR, data)
        except Exception as e:
            self.get_logger().error(f"Error I2C write: {e}")

    # ── Callbacks ─────────────────────────────────────────────────────────────
    def speed_callback(self, msg: Int16):
        """
        Velocidad del Master (-500..500)
        Negativo = adelante, Positivo = reversa, 0 = stop
        """
        if not self.autonomous_mode or self.emergency_stop:
            return
        self._set_motor_speed(msg.data)

    def steering_callback(self, msg: Int16):
        """Ángulo de dirección 65-115° (90 = centro)"""
        if not self.autonomous_mode or self.emergency_stop:
            return
        angle = max(65, min(115, int(msg.data)))
        self._set_servo_angle(angle)

    def mode_callback(self, msg: String):
        if msg.data == "AUTONOMOUS":
            self.autonomous_mode = True
            self.get_logger().info("Modo: AUTÓNOMO")
        elif msg.data == "TELEOP":
            self.autonomous_mode = False
            self.get_logger().warn("Modo: TELEOP - Control manual activo")

    def emergency_callback(self, msg: Bool):
        self.emergency_stop = msg.data
        if self.emergency_stop:
            self.get_logger().warn("PARO DE EMERGENCIA ACTIVADO!")
            self._write_motor_speeds(0, 0)

    # ── Control de actuadores ─────────────────────────────────────────────────
    def _set_motor_speed(self, speed: int):
        """
        Mapea velocidad del Master (-500..500) → módulo motor (-100..100)
        Negativo = adelante = valor positivo al motor
        """
        if speed == 0:
            self._write_motor_speeds(0, 0)
            return

        motor_val = int((speed / 500.0) * 100)
        motor_val = max(-100, min(100, motor_val))
        m_speed   = -motor_val  # negativo master = adelante = positivo motor

        self._write_motor_speeds(m_speed, m_speed)
        direction = "ADELANTE" if m_speed > 0 else "REVERSA"
        self.get_logger().debug(f"Motores: {direction}, val={m_speed}")

    def _set_servo_angle(self, angle: int):
        """Mapea ángulo (65-115°) → pulso servo (1222-1778 µs)"""
        pulse = int(
            (angle - 65) * (self.SERVO_MAX_PULSE - self.SERVO_MIN_PULSE)
            / (115 - 65) + self.SERVO_MIN_PULSE
        )
        self.pi.set_servo_pulsewidth(self.SERVO_PIN, pulse)
        self.get_logger().debug(f"Servo: {angle}°, pulso={pulse}µs")

    def cleanup(self):
        self.get_logger().info("Limpiando motor driver...")
        self._write_motor_speeds(0, 0)
        self.pi.set_servo_pulsewidth(self.SERVO_PIN, 0)
        time.sleep(0.3)
        self.pi.stop()
        self.bus.close()


def main(args=None):
    rclpy.init(args=args)
    node = MotorDriver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Cerrando motor driver...")
    finally:
        node.cleanup()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
