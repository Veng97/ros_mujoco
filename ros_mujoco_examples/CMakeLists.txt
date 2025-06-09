cmake_minimum_required(VERSION 3.22)
project(ros_mujoco_examples)

find_package(ament_cmake REQUIRED)

option(CMAKE_EXPORT_COMPILE_COMMANDS "Export compile commands" OFF)

# Install folders
install(
  DIRECTORY config launch models
  DESTINATION share/${PROJECT_NAME}
)

ament_package()