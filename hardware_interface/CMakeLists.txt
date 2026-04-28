cmake_minimum_required(VERSION 3.8)
project(hardware_interface)

find_package(ament_cmake       REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy             REQUIRED)

# ── Instalar scripts Python ───────────────────────────────────────────────
# ROS 1: catkin_install_python(PROGRAMS src/teleop_controller.py ...)
# ROS 2: ament_python_install_package  +  install(PROGRAMS ...)
install(PROGRAMS
  src/teleop_controller.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
