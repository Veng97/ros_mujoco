cmake_minimum_required(VERSION 3.22)

project(ros_mujoco)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -O3)
endif()

include(ExternalProject)

# Set MuJoCo version and installation path
set(MUJOCO_VERSION "3.2.0")
set(MUJOCO_URL "https://github.com/google-deepmind/mujoco/releases/download/${MUJOCO_VERSION}/mujoco-${MUJOCO_VERSION}-linux-x86_64.tar.gz")
set(MUJOCO_INSTALL_DIR "${CMAKE_CURRENT_BINARY_DIR}/mujoco")

# Set the MuJoCo variables
set(MUJOCO_INCLUDE_DIR "${MUJOCO_INSTALL_DIR}/include")
set(MUJOCO_BIN_DIR "${MUJOCO_INSTALL_DIR}/bin")
set(MUJOCO_LIB_DIR "${MUJOCO_INSTALL_DIR}/lib")

# Add MuJoCo as an external project
ExternalProject_Add(
  mujoco
  URL ${MUJOCO_URL}
  DOWNLOAD_EXTRACT_TIMESTAMP TRUE
  CONFIGURE_COMMAND ""
  BUILD_COMMAND ""
  INSTALL_COMMAND ${CMAKE_COMMAND} -E copy_directory <SOURCE_DIR> ${MUJOCO_INSTALL_DIR}
)

find_package(glfw3 3.3 REQUIRED)
find_package(ament_cmake REQUIRED)

# Find ROS packages
set(THIS_PACKAGE_INCLUDE_DEPENDS
  rclcpp
  std_msgs
  std_srvs
  geometry_msgs
  visualization_msgs
  sensor_msgs
  tf2_ros
  ros_mujoco_interfaces
)

foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

# Create ROS2 plugin
add_library(
  RosMujocoPlugin SHARED
  src/plugin/Plugin.cpp
  src/plugin/RosContext.cpp
  src/plugin/ActuatorCommand.cpp
  src/plugin/ClockPublisher.cpp
  src/plugin/ExternalForce.cpp
  src/plugin/ImagePublisher.cpp 
  src/plugin/PosePublisher.cpp
  src/plugin/SensorPublisher.cpp
)

add_dependencies(RosMujocoPlugin mujoco)
target_include_directories(RosMujocoPlugin PUBLIC ${PROJECT_SOURCE_DIR}/include PRIVATE ${MUJOCO_INCLUDE_DIR})
target_link_libraries(RosMujocoPlugin glfw ${MUJOCO_LIB_DIR}/libmujoco.so)

ament_target_dependencies(RosMujocoPlugin ${THIS_PACKAGE_INCLUDE_DEPENDS})

set(MUJOCO_SHARE_DIR "share/${PROJECT_NAME}/mujoco")

# Install MuJoCo (without creating a symlink so that the 'simulate' executable is able to find local libraries/plugins)
install(
  DIRECTORY ${MUJOCO_BIN_DIR} ${MUJOCO_LIB_DIR}
  DESTINATION ${MUJOCO_SHARE_DIR}
  USE_SOURCE_PERMISSIONS
  FILES_MATCHING PATTERN "*"
)

# Install library within the MuJoCo plugin directory (then the default MuJoCo executables can find it)
install(
  TARGETS RosMujocoPlugin
  EXPORT "${TARGETS_EXPORT_NAME}"
  LIBRARY DESTINATION "${MUJOCO_SHARE_DIR}/bin/mujoco_plugin"
  ARCHIVE DESTINATION "${MUJOCO_SHARE_DIR}/bin/mujoco_plugin"
  RUNTIME DESTINATION "${CMAKE_INSTALL_BINDIR}"
)

# Install files
install(
  DIRECTORY launch models
  DESTINATION share/${PROJECT_NAME}
)

ament_package()