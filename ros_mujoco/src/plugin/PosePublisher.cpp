#include "ros_mujoco/plugin/PosePublisher.hpp"

#include <mujoco/mujoco.h>

#include <iostream>

namespace RosMujoco {

void PosePublisher::registerPlugin()
{
  mjpPlugin plugin;
  mjp_defaultPlugin(&plugin);

  plugin.name = "RosMujoco::PosePublisher";
  plugin.capabilityflags |= mjPLUGIN_SENSOR;

  const char* attributes[] = {"frame_id", "pose_topic_name", "vel_topic_name", "publish_rate", "output_tf", "tf_child_frame_id"};

  plugin.nattribute = sizeof(attributes) / sizeof(attributes[0]);
  plugin.attributes = attributes;

  plugin.nstate = +[](const mjModel*, int) { return 0; };

  plugin.nsensordata = +[](const mjModel*, int, int) { return 0; };

  plugin.needstage = mjSTAGE_VEL;

  plugin.init = +[](const mjModel* m, mjData* d, int plugin_id) {
    auto* plugin_instance = PosePublisher::create(m, d, plugin_id);
    if (plugin_instance == nullptr)
    {
      return -1;
    }
    d->plugin_data[plugin_id] = reinterpret_cast<uintptr_t>(plugin_instance);
    return 0;
  };

  plugin.destroy = +[](mjData* d, int plugin_id) {
    delete reinterpret_cast<PosePublisher*>(d->plugin_data[plugin_id]);
    d->plugin_data[plugin_id] = 0;
  };

  plugin.reset = +[](const mjModel* m, double*, void* plugin_data, int plugin_id) {
    auto* plugin_instance = reinterpret_cast<class PosePublisher*>(plugin_data);
    plugin_instance->reset(m, plugin_id);
  };

  plugin.compute = +[](const mjModel* m, mjData* d, int plugin_id, int) {
    auto* plugin_instance = reinterpret_cast<class PosePublisher*>(d->plugin_data[plugin_id]);
    plugin_instance->compute(m, d, plugin_id);
  };

  mjp_registerPlugin(&plugin);
}

PosePublisher* PosePublisher::create(const mjModel* m, mjData* d, int plugin_id)
{
  // Option: frame_id
  const char* frame_id_char = mj_getPluginConfig(m, plugin_id, "frame_id");
  std::string frame_id;
  if (strlen(frame_id_char) > 0)
  {
    frame_id = std::string(frame_id_char);
  }

  // Option: pose_topic_name
  const char* pose_topic_name_char = mj_getPluginConfig(m, plugin_id, "pose_topic_name");
  std::string pose_topic_name;
  if (strlen(pose_topic_name_char) > 0)
  {
    pose_topic_name = std::string(pose_topic_name_char);
  }

  // Option: vel_topic_name
  const char* vel_topic_name_char = mj_getPluginConfig(m, plugin_id, "vel_topic_name");
  std::string vel_topic_name;
  if (strlen(vel_topic_name_char) > 0)
  {
    vel_topic_name = std::string(vel_topic_name_char);
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
    mju_error("[PosePublisher] `publish_rate` must be positive.");
    return nullptr;
  }

  // Option: output_tf
  const char* output_tf_char = mj_getPluginConfig(m, plugin_id, "output_tf");
  bool output_tf = false;
  if (strlen(output_tf_char) > 0)
  {
    if (strcmp(output_tf_char, "true") != 0 && strcmp(output_tf_char, "false") != 0)
    {
      mju_error("[PosePublisher] `output_tf` must be `true` or `false`.");
      return nullptr;
    }
    output_tf = (strcmp(output_tf_char, "true") == 0);
  }

  // Option: tf_child_frame_id
  const char* tf_child_frame_id_char = mj_getPluginConfig(m, plugin_id, "tf_child_frame_id");
  std::string tf_child_frame_id;
  if (strlen(tf_child_frame_id_char) > 0)
  {
    tf_child_frame_id = std::string(tf_child_frame_id_char);
  }

  // Set sensor_id
  int sensor_id = 0;
  for (; sensor_id < m->nsensor; sensor_id++)
  {
    if (m->sensor_type[sensor_id] == mjSENS_PLUGIN && m->sensor_plugin[sensor_id] == plugin_id)
    {
      break;
    }
  }
  if (sensor_id == m->nsensor)
  {
    mju_error("[PosePublisher] Plugin not found in sensors.");
    return nullptr;
  }
  if (m->sensor_objtype[sensor_id] != mjOBJ_XBODY)
  {
    mju_error("[PosePublisher] Plugin must be attached to a xbody.");
    return nullptr;
  }

  return new PosePublisher(m, d, sensor_id, frame_id, pose_topic_name, vel_topic_name, publish_rate, output_tf, tf_child_frame_id);
}

PosePublisher::PosePublisher(const mjModel* m, mjData*, int sensor_id, const std::string& frame_id, const std::string& pose_topic_name, const std::string& vel_topic_name, mjtNum publish_rate, bool output_tf, const std::string& tf_child_frame_id)
    : ros_context_(RosContext::getInstance())
    , sensor_id_(sensor_id)
    , body_id_(m->sensor_objid[sensor_id])
    , frame_id_(frame_id)
    , pose_topic_name_(pose_topic_name)
    , vel_topic_name_(vel_topic_name)
    , publish_skip_(std::max(static_cast<int>(1.0 / (publish_rate * m->opt.timestep)), 1))
    , output_tf_(output_tf)
    , tf_child_frame_id_(tf_child_frame_id)
{
  std::string sensor_name = std::string(mj_id2name(m, mjOBJ_SENSOR, sensor_id_));
  std::string body_name = std::string(mj_id2name(m, mjOBJ_XBODY, body_id_));
  if (frame_id_.empty())
  {
    frame_id_ = "map";
  }
  if (pose_topic_name_.empty())
  {
    pose_topic_name_ = "mujoco/" + body_name + "/pose";
  }
  if (vel_topic_name_.empty())
  {
    vel_topic_name_ = "mujoco/" + body_name + "/vel";
  }
  if (tf_child_frame_id_.empty())
  {
    tf_child_frame_id_ = body_name;
  }

  if (output_tf_)
  {
    tf_br_ = std::make_unique<tf2_ros::TransformBroadcaster>(ros_context_->getNode());
  } else
  {
    pose_pub_ = ros_context_->getNode()->create_publisher<geometry_msgs::msg::PoseStamped>(pose_topic_name, 1);
    vel_pub_ = ros_context_->getNode()->create_publisher<geometry_msgs::msg::TwistStamped>(vel_topic_name, 1);
  }

  std::cout << "[PosePublisher] Configured (" << sensor_name << "):\n";
  std::cout << "  - frame_id: " << frame_id_ << "\n";
  if (output_tf_)
  {
    std::cout << "  - tf_child_frame_id: " << tf_child_frame_id_ << "\n";
  } else
  {
    std::cout << "  - pose_topic_name: " << pose_topic_name_ << "\n";
    std::cout << "  - vel_topic_name: " << vel_topic_name_ << "\n";
  }
  std::cout << "  - publish_rate: " << publish_rate << " (adjusted rate " << 1. / (publish_skip_ * m->opt.timestep) << ")\n";
  std::cout << std::flush;
}

void PosePublisher::reset(const mjModel*, int) { iteration_count_ = 0; }

void PosePublisher::compute(const mjModel* m, mjData* d, int)
{
  ++iteration_count_;
  if (iteration_count_ % publish_skip_ != 0)
  {
    return;
  }

  builtin_interfaces::msg::Time stamp;
  stamp.sec = static_cast<int32_t>(d->time);
  stamp.nanosec = static_cast<uint32_t>((d->time - stamp.sec) * 1e9);

  if (output_tf_)
  {
    geometry_msgs::msg::TransformStamped msg;
    msg.header.stamp = stamp;
    msg.header.frame_id = frame_id_;
    msg.child_frame_id = tf_child_frame_id_;
    msg.transform.translation.x = d->xpos[3 * body_id_ + 0];
    msg.transform.translation.y = d->xpos[3 * body_id_ + 1];
    msg.transform.translation.z = d->xpos[3 * body_id_ + 2];
    msg.transform.rotation.w = d->xquat[4 * body_id_ + 0];
    msg.transform.rotation.x = d->xquat[4 * body_id_ + 1];
    msg.transform.rotation.y = d->xquat[4 * body_id_ + 2];
    msg.transform.rotation.z = d->xquat[4 * body_id_ + 3];
    tf_br_->sendTransform(msg);
  } else
  {
    geometry_msgs::msg::PoseStamped pose_msg;
    pose_msg.header.stamp = stamp;
    pose_msg.header.frame_id = frame_id_;
    pose_msg.pose.position.x = d->xpos[3 * body_id_ + 0];
    pose_msg.pose.position.y = d->xpos[3 * body_id_ + 1];
    pose_msg.pose.position.z = d->xpos[3 * body_id_ + 2];
    pose_msg.pose.orientation.w = d->xquat[4 * body_id_ + 0];
    pose_msg.pose.orientation.x = d->xquat[4 * body_id_ + 1];
    pose_msg.pose.orientation.y = d->xquat[4 * body_id_ + 2];
    pose_msg.pose.orientation.z = d->xquat[4 * body_id_ + 3];
    pose_pub_->publish(pose_msg);

    geometry_msgs::msg::TwistStamped vel_msg;
    mjtNum vel[6];
    mj_objectVelocity(m, d, mjOBJ_XBODY, body_id_, vel, 0);
    vel_msg.header.stamp = stamp;
    vel_msg.header.frame_id = frame_id_;
    vel_msg.twist.linear.x = vel[3];
    vel_msg.twist.linear.y = vel[4];
    vel_msg.twist.linear.z = vel[5];
    vel_msg.twist.angular.x = vel[0];
    vel_msg.twist.angular.y = vel[1];
    vel_msg.twist.angular.z = vel[2];
    vel_pub_->publish(vel_msg);
  }
}

}  // namespace RosMujoco
