#include <mujoco/mjplugin.h>

#include "ActuatorCommand.h"
#include "ClockPublisher.h"
#include "ExternalForce.h"
#include "ImagePublisher.h"
#include "PosePublisher.h"
#include "SensorPublisher.h"

namespace RosMujoco
{

mjPLUGIN_LIB_INIT
{
    ActuatorCommand::registerPlugin();
    ClockPublisher::registerPlugin();
    ExternalForce::registerPlugin();
    ImagePublisher::registerPlugin();
    PosePublisher::registerPlugin();
    SensorPublisher::registerPlugin();
}

} // namespace RosMujoco
