cmake_minimum_required(VERSION 3.22)

project(ros_mujoco)

# Get installation directory of MuJoCo from the MUJOCO_DIR environment variable
if(NOT DEFINED ENV{MUJOCO_DIR})
    message(FATAL_ERROR "MUJOCO_DIR environment variable is not set. Please set it before configuring CMake.")
endif()
set(MUJOCO_DIR $ENV{MUJOCO_DIR})
set(MUJOCO_INCLUDE_DIRS "${MUJOCO_DIR}/include")
set(MUJOCO_BIN_DIR "${MUJOCO_DIR}/bin")

find_package(glfw3 3.3 REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(
  ${PROJECT_NAME}
  "msg/ExternalForce.msg"
  "msg/ScalarStamped.msg"
  DEPENDENCIES
  std_msgs
  geometry_msgs
  builtin_interfaces
)

install(
  DIRECTORY launch models
  DESTINATION share/${PROJECT_NAME}
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()

add_subdirectory(plugin)
