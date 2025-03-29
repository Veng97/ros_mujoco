#include <mujoco/mjplugin.h>

#include "ActuatorCommand.h"
#include "ClockPublisher.h"
#include "ExternalForce.h"
#include "ImagePublisher.h"
#include "PosePublisher.h"
#include "SensorPublisher.h"

namespace RosMujoco {

mjPLUGIN_LIB_INIT {
  ActuatorCommand::RegisterPlugin();
  ClockPublisher::RegisterPlugin();
  ExternalForce::RegisterPlugin();
  ImagePublisher::RegisterPlugin();
  PosePublisher::RegisterPlugin();
  SensorPublisher::RegisterPlugin();
}

} // namespace RosMujoco
