#include "ClockPublisher.h"

#include <mujoco/mujoco.h>

#include <iostream>

namespace RosMujoco
{

void ClockPublisher::registerPlugin()
{
    mjpPlugin plugin;
    mjp_defaultPlugin(&plugin);

    plugin.name = "RosMujoco::ClockPublisher";
    plugin.capabilityflags |= mjPLUGIN_PASSIVE;

    const char* attributes[] = {"topic_name", "publish_rate", "use_sim_time"};

    plugin.nattribute = sizeof(attributes) / sizeof(attributes[0]);
    plugin.attributes = attributes;

    plugin.nstate = +[](const mjModel*, // m
                        int             // plugin_id
                     ) { return 0; };

    plugin.nsensordata = +[](const mjModel*, // m
                             int,            // plugin_id
                             int             // sensor_id
                          ) { return 0; };

    plugin.needstage = mjSTAGE_POS;

    plugin.init = +[](const mjModel* m, mjData* d, int plugin_id) {
        auto* plugin_instance = ClockPublisher::create(m, d, plugin_id);
        if (!plugin_instance)
        {
            return -1;
        }
        d->plugin_data[plugin_id] = reinterpret_cast<uintptr_t>(plugin_instance);
        return 0;
    };

    plugin.destroy = +[](mjData* d, int plugin_id) {
        delete reinterpret_cast<ClockPublisher*>(d->plugin_data[plugin_id]);
        d->plugin_data[plugin_id] = 0;
    };

    plugin.reset = +[](const mjModel* m, double*, // plugin_state
                       void* plugin_data, int plugin_id) {
        auto* plugin_instance =
            reinterpret_cast<class ClockPublisher*>(plugin_data);
        plugin_instance->reset(m, plugin_id);
    };

    plugin.compute =
        +[](const mjModel* m, mjData* d, int plugin_id, int // capability_bit
         ) {
            auto* plugin_instance =
                reinterpret_cast<class ClockPublisher*>(d->plugin_data[plugin_id]);
            plugin_instance->compute(m, d, plugin_id);
        };

    mjp_registerPlugin(&plugin);
}

ClockPublisher* ClockPublisher::create(const mjModel* m, mjData* d, int plugin_id)
{
    if (m->body_plugin[0] != plugin_id)
    {
        mju_error("[ClockPublisher] This plugin must be registered in worldbody.");
        return nullptr;
    }

    // Option: topic_name
    const char* topic_name_char = mj_getPluginConfig(m, plugin_id, "topic_name");
    std::string topic_name = "";
    if (strlen(topic_name_char) > 0)
    {
        topic_name = std::string(topic_name_char);
    }

    // Option: publish_rate
    const char* publish_rate_char = mj_getPluginConfig(m, plugin_id, "publish_rate");
    mjtNum publish_rate = 100.0;
    if (strlen(publish_rate_char) > 0)
    {
        publish_rate = strtod(publish_rate_char, nullptr);
    }
    if (publish_rate <= 0)
    {
        mju_error("[ClockPublisher] `publish_rate` must be positive.");
        return nullptr;
    }

    return new ClockPublisher(m, d, topic_name, publish_rate);
}

ClockPublisher::ClockPublisher(const mjModel* m, mjData*, const std::string& topic_name, const mjtNum& publish_rate)
    : ros_context_(RosContext::getInstance()), topic_name_(topic_name), publish_skip_(std::max(static_cast<int>(1. / (publish_rate * m->opt.timestep)), 1))
{
    std::cout << "[ClockPublisher] Configuring:" << "\n";
    std::cout << "  - topic_name: " << topic_name << "\n";
    std::cout << "  - publish_rate: " << publish_rate << "\n";

    if (topic_name_.empty())
    {
        topic_name_ = "/clock";
    }

    pub_ = ros_context_->getNode()->create_publisher<rosgraph_msgs::msg::Clock>(topic_name, 1);
}

void ClockPublisher::reset(const mjModel*, int)
{
    iteration_count_ = 0;
}

void ClockPublisher::compute(const mjModel*, mjData* d, int)
{
    ++iteration_count_;
    if (iteration_count_ % publish_skip_ != 0)
    {
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
