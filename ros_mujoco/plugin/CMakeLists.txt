
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -O3)
endif()

add_library(
  RosMujoco SHARED
  RosContext.cpp
  ActuatorCommand.cpp
  ClockPublisher.cpp
  ExternalForce.cpp
  ImagePublisher.cpp 
  PosePublisher.cpp
  SensorPublisher.cpp
)

# Include directories
target_include_directories(RosMujoco 
  PUBLIC 
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include> 
    $<INSTALL_INTERFACE:include>
    ${ament_INCLUDE_DIRS}
  PRIVATE 
    ${MUJOCO_INCLUDE_DIRS}
)

# Link dependencies
ament_target_dependencies(RosMujoco
  PUBLIC
    rclcpp
    std_msgs
    geometry_msgs
    visualization_msgs
    tf2_ros
    sensor_msgs
    builtin_interfaces
)

rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(RosMujoco 
  PUBLIC 
    ${cpp_typesupport_target} 
    ${ament_LIBRARIES} 
    ${LIB_MUJOCO} 
    glfw
)

# Define the plugin library
add_library(RosMujocoPlugin SHARED Plugin.cpp)
target_include_directories(RosMujocoPlugin PRIVATE ${MUJOCO_INCLUDE_DIRS})
target_link_libraries(RosMujocoPlugin PUBLIC RosMujoco)

install(
  TARGETS RosMujoco RosMujocoPlugin
  EXPORT "${TARGETS_EXPORT_NAME}"
  LIBRARY DESTINATION "${CMAKE_INSTALL_LIBDIR}"
  ARCHIVE DESTINATION "${CMAKE_INSTALL_LIBDIR}"
  RUNTIME DESTINATION "${CMAKE_INSTALL_BINDIR}"
)

add_custom_command(
  TARGET RosMujoco
  POST_BUILD
  COMMAND ${CMAKE_COMMAND} -E copy $<TARGET_FILE:RosMujoco>
          ${MUJOCO_BIN_DIR}/mujoco_plugin
)

add_custom_command(
  TARGET RosMujocoPlugin
  POST_BUILD
  COMMAND ${CMAKE_COMMAND} -E copy $<TARGET_FILE:RosMujocoPlugin>
          ${MUJOCO_BIN_DIR}/mujoco_plugin
)