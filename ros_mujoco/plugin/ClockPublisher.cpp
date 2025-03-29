#include "ClockPublisher.h"

#include <mujoco/mujoco.h>

#include <builtin_interfaces/msg/time.hpp>
#include <iostream>

namespace RosMujoco {

void ClockPublisher::RegisterPlugin() {
  mjpPlugin plugin;
  mjp_defaultPlugin(&plugin);

  plugin.name = "RosMujoco::ClockPublisher";
  plugin.capabilityflags |= mjPLUGIN_PASSIVE;

  const char *attributes[] = {"topic_name", "publish_rate", "use_sim_time"};

  plugin.nattribute = sizeof(attributes) / sizeof(attributes[0]);
  plugin.attributes = attributes;

  plugin.nstate = +[](const mjModel *, // m
                      int              // plugin_id
                   ) { return 0; };

  plugin.nsensordata = +[](const mjModel *, // m
                           int,             // plugin_id
                           int              // sensor_id
                        ) { return 0; };

  plugin.needstage = mjSTAGE_POS;

  plugin.init = +[](const mjModel *m, mjData *d, int plugin_id) {
    auto *plugin_instance = ClockPublisher::Create(m, d, plugin_id);
    if (!plugin_instance) {
      return -1;
    }
    d->plugin_data[plugin_id] = reinterpret_cast<uintptr_t>(plugin_instance);
    return 0;
  };

  plugin.destroy = +[](mjData *d, int plugin_id) {
    delete reinterpret_cast<ClockPublisher *>(d->plugin_data[plugin_id]);
    d->plugin_data[plugin_id] = 0;
  };

  plugin.reset = +[](const mjModel *m, double *, // plugin_state
                     void *plugin_data, int plugin_id) {
    auto *plugin_instance =
        reinterpret_cast<class ClockPublisher *>(plugin_data);
    plugin_instance->reset(m, plugin_id);
  };

  plugin.compute =
      +[](const mjModel *m, mjData *d, int plugin_id, int // capability_bit
       ) {
        auto *plugin_instance =
            reinterpret_cast<class ClockPublisher *>(d->plugin_data[plugin_id]);
        plugin_instance->compute(m, d, plugin_id);
      };

  mjp_registerPlugin(&plugin);
}

ClockPublisher *ClockPublisher::Create(const mjModel *m, mjData *d,
                                       int plugin_id) {
  // topic_name
  const char *topic_name_char = mj_getPluginConfig(m, plugin_id, "topic_name");
  std::string topic_name = "";
  if (strlen(topic_name_char) > 0) {
    topic_name = std::string(topic_name_char);
  }

  // publish_rate
  const char *publish_rate_char =
      mj_getPluginConfig(m, plugin_id, "publish_rate");
  mjtNum publish_rate = 100.0;
  if (strlen(publish_rate_char) > 0) {
    publish_rate = strtod(publish_rate_char, nullptr);
  }
  if (publish_rate <= 0) {
    mju_error("[ClockPublisher] `publish_rate` must be positive.");
    return nullptr;
  }

  // use_sim_time
  const char *use_sim_time_char =
      mj_getPluginConfig(m, plugin_id, "use_sim_time");
  bool use_sim_time = true;
  if (strlen(use_sim_time_char) > 0) {
    if (!(strcmp(use_sim_time_char, "true") == 0 ||
          strcmp(use_sim_time_char, "false") == 0)) {
      mju_error("[ClockPublisher] `use_sim_time` must be `true` or `false`.");
      return nullptr;
    }
    use_sim_time = (strcmp(use_sim_time_char, "true") == 0);
  }

  if (m->body_plugin[0] != plugin_id) {
    mju_error("[ClockPublisher] This plugin must be registered in worldbody.");
    return nullptr;
  }

  std::cout << "[ClockPublisher] Created." << std::endl;

  return new ClockPublisher(m, d, topic_name, publish_rate, use_sim_time);
}

ClockPublisher::ClockPublisher(const mjModel *m,
                               mjData *, // d
                               const std::string &topic_name,
                               mjtNum publish_rate, bool use_sim_time)
    : ros_context_(RosContext::getInstance()), topic_name_(topic_name),
      publish_skip_(std::max(
          static_cast<int>(1.0 / (publish_rate * m->opt.timestep)), 1)),
      use_sim_time_(use_sim_time) {
  std::cout << "[ClockPublisher] Configuring." << std::endl;

  if (topic_name_.empty()) {
    topic_name_ = "/clock";
  }

  pub_ = ros_context_->getNode()->create_publisher<rosgraph_msgs::msg::Clock>(
      topic_name, 1);
}

void ClockPublisher::reset(const mjModel *, // m
                           int              // plugin_id
) {}

void ClockPublisher::compute(const mjModel *, // m
                             mjData *d,
                             int // plugin_id
) {
  sim_cnt_++;
  if (sim_cnt_ % publish_skip_ != 0) {
    return;
  }

  builtin_interfaces::msg::Time stamp;
  stamp.sec = static_cast<int32_t>(d->time);
  stamp.nanosec = static_cast<uint32_t>((d->time - stamp.sec) * 1e9);

  rosgraph_msgs::msg::Clock msg;
  msg.clock = stamp;

  pub_->publish(msg);
}

} // namespace RosMujoco
