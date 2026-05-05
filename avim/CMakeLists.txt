cmake_minimum_required(VERSION 3.8)
project(AVIM)

# Buscar ament_cmake (reemplaza catkin)
find_package(ament_cmake REQUIRED)

# ── Instalar launch files ────────────────────────────────────────────────────
# En ROS 2 los launch files se instalan explícitamente con install()
install(
  DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# ── Cerrar el paquete (reemplaza catkin_package) ─────────────────────────────
ament_package()
