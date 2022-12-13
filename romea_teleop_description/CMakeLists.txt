cmake_minimum_required(VERSION 3.5)
project(romea_teleop_description)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY config 
  DESTINATION share/${PROJECT_NAME}
)

ament_python_install_module(python/romea_teleop_description.py)

ament_package()
