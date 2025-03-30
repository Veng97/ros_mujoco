#include "ros_mujoco/plugin/SensorPublisher.hpp"

#include <mujoco/mujoco.h>

#include <iostream>

namespace RosMujoco
{

void SensorPublisher::registerPlugin()
{
    mjpPlugin plugin;
    mjp_defaultPlugin(&plugin);

    plugin.name = "RosMujoco::SensorPublisher";
    plugin.capabilityflags |= mjPLUGIN_SENSOR;

    const char* attributes[] = {"sensor_name", "frame_id", "topic_name",
                                "publish_rate"};

    plugin.nattribute = sizeof(attributes) / sizeof(attributes[0]);
    plugin.attributes = attributes;

    plugin.nstate = +[](const mjModel*, // m
                        int             // plugin_id
                     ) { return 0; };

    plugin.nsensordata = +[](const mjModel*, // m
                             int,            // plugin_id
                             int             // sensor_id
                          ) { return 0; };

    // Can only run after forces have been computed
    plugin.needstage = mjSTAGE_ACC;

    plugin.init = +[](const mjModel* m, mjData* d, int plugin_id) {
        auto* plugin_instance = SensorPublisher::create(m, d, plugin_id);
        if (plugin_instance == nullptr)
        {
            return -1;
        }
        d->plugin_data[plugin_id] = reinterpret_cast<uintptr_t>(plugin_instance);
        return 0;
    };

    plugin.destroy = +[](mjData* d, int plugin_id) {
        delete reinterpret_cast<SensorPublisher*>(d->plugin_data[plugin_id]);
        d->plugin_data[plugin_id] = 0;
    };

    plugin.reset = +[](const mjModel* m, double*, // plugin_state
                       void* plugin_data, int plugin_id) {
        auto* plugin_instance = reinterpret_cast<class SensorPublisher*>(plugin_data);
        plugin_instance->reset(m, plugin_id);
    };

    plugin.compute = +[](const mjModel* m, mjData* d, int plugin_id,
                         int // capability_bit
                      ) {
        auto* plugin_instance = reinterpret_cast<class SensorPublisher*>(d->plugin_data[plugin_id]);
        plugin_instance->compute(m, d, plugin_id);
    };

    mjp_registerPlugin(&plugin);
}

SensorPublisher* SensorPublisher::create(const mjModel* m, mjData* d, int plugin_id)
{
    // Option: sensor_name
    const char* sensor_name_char = mj_getPluginConfig(m, plugin_id, "sensor_name");
    if (strlen(sensor_name_char) == 0)
    {
        mju_error("[SensorPublisher] `sensor_name` is missing.");
        return nullptr;
    }
    int sensor_id = 0;
    for (; sensor_id < m->nsensor; sensor_id++)
    {
        if (strcmp(sensor_name_char, mj_id2name(m, mjOBJ_SENSOR, sensor_id)) == 0)
        {
            break;
        }
    }
    if (sensor_id == m->nsensor)
    {
        mju_error("[SensorCommand] The sensor with the specified name not found.");
        return nullptr;
    }

    // Option: msg_type
    MessageType msg_type;
    int sensor_dim = m->sensor_dim[sensor_id];
    if (sensor_dim == 1)
    {
        msg_type = MessageType::Scalar;
    }
    else if (sensor_dim == 3)
    {
        if (m->sensor_type[sensor_id] == mjSENS_FRAMEPOS)
        {
            msg_type = MessageType::Point;
        }
        else
        {
            msg_type = MessageType::Vector3;
        }
    }
    else if (sensor_dim == 4)
    {
        msg_type = MessageType::Quaternion;
    }
    else
    {
        mju_error("[SensorPublisher] Unsupported sensor data dimensions: %d.", sensor_dim);
        return nullptr;
    }

    // Option: frame_id
    const char* frame_id_char = mj_getPluginConfig(m, plugin_id, "frame_id");
    std::string frame_id = "";
    if (strlen(frame_id_char) > 0)
    {
        frame_id = std::string(frame_id_char);
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
    mjtNum publish_rate = 30.0;
    if (strlen(publish_rate_char) > 0)
    {
        publish_rate = strtod(publish_rate_char, nullptr);
    }
    if (publish_rate <= 0)
    {
        mju_error("[SensorPublisher] `publish_rate` must be positive.");
        return nullptr;
    }

    return new SensorPublisher(m, d, sensor_id, msg_type, frame_id, topic_name, publish_rate);
}

SensorPublisher::SensorPublisher(const mjModel* m,
                                 mjData*, // d
                                 int sensor_id, MessageType msg_type,
                                 const std::string& frame_id,
                                 const std::string& topic_name,
                                 mjtNum publish_rate)
    : ros_context_(RosContext::getInstance()),
      sensor_id_(sensor_id),
      msg_type_(msg_type),
      frame_id_(frame_id),
      topic_name_(topic_name),
      publish_skip_(std::max(static_cast<int>(1.0 / (publish_rate * m->opt.timestep)), 1))
{
    if (frame_id_.empty())
    {
        frame_id_ = "map";
    }

    std::string sensor_name = std::string(mj_id2name(m, mjOBJ_SENSOR, sensor_id_));
    if (topic_name_.empty())
    {
        topic_name_ = "mujoco/" + sensor_name;
    }

    std::string msg_type_str;
    if (msg_type_ == MessageType::Scalar)
    {
        msg_type_str = "Scalar";
        pub_ = ros_context_->getNode()->create_publisher<ros_mujoco_interfaces::msg::ScalarStamped>(topic_name_, 1);
    }
    else if (msg_type_ == MessageType::Point)
    {
        msg_type_str = "Point";
        pub_ = ros_context_->getNode()->create_publisher<geometry_msgs::msg::PointStamped>(topic_name_, 1);
    }
    else if (msg_type_ == MessageType::Vector3)
    {
        msg_type_str = "Vector3";
        pub_ = ros_context_->getNode()->create_publisher<geometry_msgs::msg::Vector3Stamped>(topic_name_, 1);
    }
    else // if(msg_type_ == MsgQuaternion)
    {
        msg_type_str = "Quaternion";
        pub_ = ros_context_->getNode()->create_publisher<geometry_msgs::msg::QuaternionStamped>(topic_name_, 1);
    }

    std::cout << "[SensorPublisher] Configured (" << sensor_name << "):\n";
    std::cout << "  - frame_id: " << frame_id_ << "\n";
    std::cout << "  - topic_name: " << topic_name_ << "\n";
    std::cout << "  - msg_type: " << msg_type_str << "\n";
    std::cout << "  - publish_rate: " << publish_rate << " (adjusted rate " << 1. / (publish_skip_ * m->opt.timestep) << ")\n";
    std::cout << std::flush;
}

void SensorPublisher::reset(const mjModel*, // m
                            int             // plugin_id
)
{
    iteration_count_ = 0;
}

void SensorPublisher::compute(const mjModel* m, mjData* d, int // plugin_id
)
{
    ++iteration_count_;
    if (iteration_count_ % publish_skip_ != 0)
    {
        return;
    }

    builtin_interfaces::msg::Time stamp;
    stamp.sec = static_cast<int32_t>(d->time);
    stamp.nanosec = static_cast<uint32_t>((d->time - stamp.sec) * 1e9);

    std_msgs::msg::Header header;
    header.stamp = stamp;
    header.frame_id = frame_id_;

    int sensor_adr = m->sensor_adr[sensor_id_];
    if (msg_type_ == MessageType::Scalar)
    {
        ros_mujoco_interfaces::msg::ScalarStamped msg;
        msg.header = header;
        msg.value.data = d->sensordata[sensor_adr];
        std::dynamic_pointer_cast<rclcpp::Publisher<ros_mujoco_interfaces::msg::ScalarStamped>>(pub_)->publish(msg);
    }
    else if (msg_type_ == MessageType::Point)
    {
        geometry_msgs::msg::PointStamped msg;
        msg.header = header;
        msg.point.x = d->sensordata[sensor_adr + 0];
        msg.point.y = d->sensordata[sensor_adr + 1];
        msg.point.z = d->sensordata[sensor_adr + 2];
        std::dynamic_pointer_cast<rclcpp::Publisher<geometry_msgs::msg::PointStamped>>(pub_)->publish(msg);
    }
    else if (msg_type_ == MessageType::Vector3)
    {
        geometry_msgs::msg::Vector3Stamped msg;
        msg.header = header;
        msg.vector.x = d->sensordata[sensor_adr + 0];
        msg.vector.y = d->sensordata[sensor_adr + 1];
        msg.vector.z = d->sensordata[sensor_adr + 2];
        std::dynamic_pointer_cast<rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>>(pub_)->publish(msg);
    }
    else // if(msg_type_ == MsgQuaternion)
    {
        geometry_msgs::msg::QuaternionStamped msg;
        msg.header = header;
        msg.quaternion.w = d->sensordata[sensor_adr + 0];
        msg.quaternion.x = d->sensordata[sensor_adr + 1];
        msg.quaternion.y = d->sensordata[sensor_adr + 2];
        msg.quaternion.z = d->sensordata[sensor_adr + 3];
        std::dynamic_pointer_cast<rclcpp::Publisher<geometry_msgs::msg::QuaternionStamped>>(pub_)->publish(msg);
    }
}

} // namespace RosMujoco
