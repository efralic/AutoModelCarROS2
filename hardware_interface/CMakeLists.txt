cmake_minimum_required(VERSION 3.8)
project(hardware_interface)

find_package(ament_cmake        REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy              REQUIRED)

# ── Instalar los 3 scripts Python ────────────────────────────────────────────
# motor_driver.py      → corre en la Pi (I2C + pigpio servo)
# ultrasonic_sensors.py → corre en la Pi (pigpio GPIO)
# teleop_controller.py  → corre en la Pi (joystick PS4)
install(PROGRAMS
  src/motor_driver.py
  src/ultrasonic_sensors.py
  src/teleop_controller.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
