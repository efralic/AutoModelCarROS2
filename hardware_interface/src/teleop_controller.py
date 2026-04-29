#!/usr/bin/env python3
"""
Teleop Node para TMR 2026  –  ROS 2 (Humble / Iron)
Control del vehículo con PS4 DualShock.
Se ejecuta en la Raspberry Pi 4B.

Cambios ROS 1 → ROS 2:
  • rospy  → rclpy
  • rospy.init_node(...)  → Node.__init__ / rclpy.init()
  • rospy.Subscriber(...)  → node.create_subscription(...)
  • rospy.Publisher(...)   → node.create_publisher(...)
  • rospy.Rate / rospy.is_shutdown()  → node.create_timer / rclpy.ok()
  • rospy.Timer (oneshot)  → node.create_timer con bandera
  • rospy.loginfo/warn  → node.get_logger().info/warn
  • msg = Int16(); msg.data = x  → mismo patrón (sin cambios en mensajes std)
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Int16, String, Bool


class TeleopController(Node):

    def __init__(self):
        # ROS 1: rospy.init_node('teleop_controller', anonymous=False)
        # ROS 2: super().__init__('teleop_controller')
        super().__init__('teleop_controller')

        # ── Estado de modo ─────────────────────────────────────────────────
        self.autonomous_mode   = True
        self.last_cross_state  = 0
        self._release_timer    = None   # reemplaza rospy.Timer oneshot

        # ── Límites de control ─────────────────────────────────────────────
        self.max_speed  = 435
        self.max_angle  = 45
        self.deadzone   = 0.1

        # ── Índices PS4 ────────────────────────────────────────────────────
        self.AXIS_STEER   = 0
        self.AXIS_R2      = 5
        self.AXIS_L2      = 2
        self.BTN_CROSS    = 0
        self.BTN_TRIANGLE = 2

        # ── Publishers ─────────────────────────────────────────────────────
        # ROS 1: rospy.Publisher(topic, Type, queue_size=N)
        # ROS 2: self.create_publisher(Type, topic, N)
        self.speed_pub     = self.create_publisher(Int16,  '/AutoModelMini/manual_control/speed',    1)
        self.steering_pub  = self.create_publisher(Int16,  '/AutoModelMini/manual_control/steering', 1)
        self.mode_pub      = self.create_publisher(String, '/vehicle_mode',   1)
        self.emergency_pub = self.create_publisher(Bool,   '/emergency_stop', 1)

        # ── Subscriber ─────────────────────────────────────────────────────
        # ROS 1: rospy.Subscriber(topic, Type, callback, queue_size=N)
        # ROS 2: self.create_subscription(Type, topic, callback, N)
        self.create_subscription(Joy, '/joy', self.joy_callback, 1)

        # ── Timer de publicación de modo (20 Hz) ───────────────────────────
        # ROS 1: rate = rospy.Rate(20); while not rospy.is_shutdown(): rate.sleep()
        # ROS 2: create_timer(periodo_en_segundos, callback)
        self.create_timer(1.0 / 20.0, self.timer_callback)

        self.publish_mode()

        self.get_logger().info("Teleop Controller inicializado (ROS 2)")
        self.get_logger().info("Controles PS4:")
        self.get_logger().info("  R2              : Acelerar (adelante)")
        self.get_logger().info("  L2              : Reversa / frenar")
        self.get_logger().info("  Stick izq. horiz: Dirección izq/der")
        self.get_logger().info("  Cruz (X)        : Cambiar modo Autónomo/Teleop")
        self.get_logger().info("  Triángulo       : Paro de emergencia")

    # ── Timer periódico (reemplaza el loop while en run()) ─────────────────
    def timer_callback(self):
        self.publish_mode()

    # ── Callback del joystick ──────────────────────────────────────────────
    def joy_callback(self, joy_msg: Joy):

        # Cambio de modo — Cruz (flanco de subida)
        cross_now = joy_msg.buttons[self.BTN_CROSS]
        if cross_now == 1 and self.last_cross_state == 0:
            self.toggle_mode()
        self.last_cross_state = cross_now

        # Paro de emergencia — Triángulo
        if joy_msg.buttons[self.BTN_TRIANGLE] == 1:
            self.emergency_stop()
            return

        # Comandos solo en modo TELEOP
        if not self.autonomous_mode:

            # Dirección
            steer_raw = joy_msg.axes[self.AXIS_STEER]
            if abs(steer_raw) < self.deadzone:
                steer_raw = 0.0
            angle = int(90 - steer_raw * self.max_angle)
            angle = max(45, min(135, angle))

            # Velocidad
            r2 = -joy_msg.axes[self.AXIS_R2]   # 0=suelto, 1=fondo
            l2 = -joy_msg.axes[self.AXIS_L2]

            if r2 > 0.05:
                speed = int(-r2 * self.max_speed)   # adelante (negativo)
            elif l2 > 0.05:
                speed = int(l2 * self.max_speed)    # reversa (positivo)
            else:
                speed = 0

            self.publish_speed(speed)
            self.publish_steering(angle)

    # ── Helpers ────────────────────────────────────────────────────────────
    def toggle_mode(self):
        self.autonomous_mode = not self.autonomous_mode
        self.publish_mode()
        if self.autonomous_mode:
            self.get_logger().info("Modo: AUTÓNOMO")
            self.publish_speed(0)
            self.publish_steering(90)
        else:
            self.get_logger().warn("Modo: TELEOP - Control manual activo")

    def emergency_stop(self):
        self.get_logger().warn("PARO DE EMERGENCIA!")
        self.publish_speed(0)
        self.publish_steering(90)
        msg = Bool(); msg.data = True
        self.emergency_pub.publish(msg)

        # ROS 1: rospy.Timer(rospy.Duration(1.0), self.release_emergency, oneshot=True)
        # ROS 2: crear un timer de 1 s y cancelarlo en el callback
        if self._release_timer is not None:
            self._release_timer.cancel()
        self._release_timer = self.create_timer(1.0, self._release_emergency_cb)

    def _release_emergency_cb(self):
        """Oneshot: se cancela a sí mismo tras dispararse."""
        msg = Bool(); msg.data = False
        self.emergency_pub.publish(msg)
        self.get_logger().info("Paro de emergencia liberado")
        # Cancelar para que no vuelva a dispararse
        self._release_timer.cancel()
        self._release_timer = None

    def publish_speed(self, speed: int):
        msg = Int16(); msg.data = speed
        self.speed_pub.publish(msg)

    def publish_steering(self, angle: int):
        msg = Int16(); msg.data = angle
        self.steering_pub.publish(msg)

    def publish_mode(self):
        msg = String()
        msg.data = "AUTONOMOUS" if self.autonomous_mode else "TELEOP"
        self.mode_pub.publish(msg)


def main(args=None):
    # ROS 1: if __name__ == '__main__': controller = TeleopController(); controller.run()
    # ROS 2: rclpy.init() → spin(node) → shutdown()
    rclpy.init(args=args)
    node = TeleopController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Cerrando teleop controller...")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
