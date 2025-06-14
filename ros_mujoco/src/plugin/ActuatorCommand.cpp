
#include "ros_mujoco/plugin/ActuatorCommand.hpp"

#include <mujoco/mujoco.h>

#include <iostream>

namespace RosMujoco {

void ActuatorCommand::registerPlugin()
{
  mjpPlugin plugin;
  mjp_defaultPlugin(&plugin);

  plugin.name = "RosMujoco::ActuatorCommand";

  // Allow plugins to be placed on either the body element or the actuator element
  plugin.capabilityflags |= mjPLUGIN_ACTUATOR;
  plugin.capabilityflags |= mjPLUGIN_PASSIVE;

  const char *attributes[] = {"actuator_name", "topic_name"};

  plugin.nattribute = sizeof(attributes) / sizeof(attributes[0]);
  plugin.attributes = attributes;

  plugin.nstate = +[](const mjModel *, int) { return 0; };

  plugin.nsensordata = +[](const mjModel *, int, int) { return 0; };

  plugin.needstage = mjSTAGE_VEL;

  plugin.init = +[](const mjModel *m, mjData *d, int plugin_id) {
    auto *plugin_instance = ActuatorCommand::create(m, d, plugin_id);
    if (plugin_instance == nullptr)
    {
      return -1;
    }
    d->plugin_data[plugin_id] = reinterpret_cast<uintptr_t>(plugin_instance);
    return 0;
  };

  plugin.destroy = +[](mjData *d, int plugin_id) {
    delete reinterpret_cast<ActuatorCommand *>(d->plugin_data[plugin_id]);
    d->plugin_data[plugin_id] = 0;
  };

  plugin.reset = +[](const mjModel *m, double *, void *plugin_data, int plugin_id) {
    auto *plugin_instance = reinterpret_cast<class ActuatorCommand *>(plugin_data);
    plugin_instance->reset(m, plugin_id);
  };

  plugin.compute = +[](const mjModel *m, mjData *d, int plugin_id, int) {
    auto *plugin_instance = reinterpret_cast<class ActuatorCommand *>(d->plugin_data[plugin_id]);
    plugin_instance->compute(m, d, plugin_id);
  };

  mjp_registerPlugin(&plugin);
}

ActuatorCommand *ActuatorCommand::create(const mjModel *m, mjData *d, int plugin_id)
{
  // Option: actuator_name
  const char *actuator_name_char = mj_getPluginConfig(m, plugin_id, "actuator_name");
  if (strlen(actuator_name_char) == 0)
  {
    mju_error("[ActuatorCommand] `actuator_name` is missing.");
    return nullptr;
  }
  int actuator_id = 0;
  for (; actuator_id < m->nu; actuator_id++)
  {
    if (strcmp(actuator_name_char, mj_id2name(m, mjOBJ_ACTUATOR, actuator_id)) == 0)
    {
      break;
    }
  }
  if (actuator_id == m->nu)
  {
    mju_error(
        "[ActuatorCommand] The actuator with the specified name was not "
        "found.");
    return nullptr;
  }

  // Option: topic_name
  std::string topic_name{mj_getPluginConfig(m, plugin_id, "topic_name")};

  return new ActuatorCommand(m, d, actuator_id, topic_name);
}

ActuatorCommand::ActuatorCommand(const mjModel *m, mjData *, int actuator_id, std::string topic_name)
    : ros_context_(RosContext::getInstance())
    , actuator_id_(actuator_id)
{
  const std::string actuator_name{mj_id2name(m, mjOBJ_ACTUATOR, actuator_id)};

  if (topic_name.empty())
  {
    topic_name = "mujoco/" + actuator_name;
  }

  sub_ = ros_context_->getNode()->create_subscription<std_msgs::msg::Float64>(topic_name, 1, std::bind(&ActuatorCommand::callback, this, std::placeholders::_1));

  std::cout << "[ActuatorCommand] Configured (" << actuator_name << "):\n";
  std::cout << "  - topic_name: " << topic_name << "\n";
  std::cout << std::flush;
}

void ActuatorCommand::reset(const mjModel *, int) {}

void ActuatorCommand::compute(const mjModel *, mjData *d, int)
{
  ros_context_->spinUntilComplete(d);

  // Set actuator command
  if (!std::isnan(ctrl_))
  {
    d->ctrl[actuator_id_] = ctrl_;
    ctrl_ = std::numeric_limits<mjtNum>::quiet_NaN();
  }
}

void ActuatorCommand::callback(const std_msgs::msg::Float64 &msg) { ctrl_ = msg.data; }

}  // namespace RosMujoco
