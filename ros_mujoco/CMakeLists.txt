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
set(MUJOCO_LIB "${MUJOCO_INSTALL_DIR}/lib/libmujoco.so")

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
  src/Plugin.cpp
  src/RosContext.cpp
  src/ActuatorCommand.cpp
  src/ClockPublisher.cpp
  src/ExternalForce.cpp
  src/ImagePublisher.cpp 
  src/PosePublisher.cpp
  src/SensorPublisher.cpp
)

add_dependencies(RosMujocoPlugin mujoco)
target_include_directories(RosMujocoPlugin PUBLIC ${PROJECT_SOURCE_DIR}/include PRIVATE ${MUJOCO_INCLUDE_DIR})
target_link_libraries(RosMujocoPlugin ${MUJOCO_LIB} glfw)

ament_target_dependencies(RosMujocoPlugin ${THIS_PACKAGE_INCLUDE_DEPENDS})

# Install MuJoCo binaries (without creating a symlink so that the 'simulate' executable is able to find local libraries/plugins)
install(
  DIRECTORY ${MUJOCO_BIN_DIR}/
  DESTINATION share/${PROJECT_NAME}/mujoco
  USE_SOURCE_PERMISSIONS
  FILES_MATCHING PATTERN "*"
)

install(
  TARGETS RosMujocoPlugin
  EXPORT "${TARGETS_EXPORT_NAME}"
  LIBRARY DESTINATION "share/${PROJECT_NAME}/mujoco/mujoco_plugin"
  ARCHIVE DESTINATION "share/${PROJECT_NAME}/mujoco/mujoco_plugin"
  RUNTIME DESTINATION "${CMAKE_INSTALL_BINDIR}"
)

# install(
#   CODE "execute_process(COMMAND ${CMAKE_COMMAND} -E copy \$<TARGET_FILE:RosMujocoPlugin> \$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}/bin/mujoco_plugin/libRosMujocoPlugin.so)"
# )

# Install files
install(
  DIRECTORY launch models
  DESTINATION share/${PROJECT_NAME}
)

ament_package()