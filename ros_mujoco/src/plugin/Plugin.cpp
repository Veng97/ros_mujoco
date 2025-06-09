#include "ros_mujoco/plugin/ActuatorCommand.hpp"
#include "ros_mujoco/plugin/ClockPublisher.hpp"
#include "ros_mujoco/plugin/ExternalForce.hpp"
#include "ros_mujoco/plugin/ImagePublisher.hpp"
#include "ros_mujoco/plugin/PosePublisher.hpp"
#include "ros_mujoco/plugin/SensorPublisher.hpp"

#include <mujoco/mjplugin.h>

namespace RosMujoco {

mjPLUGIN_LIB_INIT
{
  ClockPublisher::registerPlugin();
  ActuatorCommand::registerPlugin();
  ExternalForce::registerPlugin();
  ImagePublisher::registerPlugin();
  PosePublisher::registerPlugin();
  SensorPublisher::registerPlugin();
}

}  // namespace RosMujoco
