#include "ros_mujoco/plugin/ImagePublisher.hpp"

#include <mujoco/mujoco.h>

#include <iostream>

namespace RosMujoco {

void ImagePublisher::registerPlugin()
{
  mjpPlugin plugin;
  mjp_defaultPlugin(&plugin);

  plugin.name = "RosMujoco::ImagePublisher";
  plugin.capabilityflags |= mjPLUGIN_SENSOR;

  const char* attributes[] = {"frame_id", "color_topic_name", "depth_topic_name", "info_topic_name", "height", "width", "publish_rate"};

  plugin.nattribute = sizeof(attributes) / sizeof(attributes[0]);
  plugin.attributes = attributes;

  plugin.nstate = +[](const mjModel*, int) { return 0; };

  plugin.nsensordata = +[](const mjModel*, int, int) { return 0; };

  plugin.needstage = mjSTAGE_VEL;

  plugin.init = +[](const mjModel* m, mjData* d, int plugin_id) {
    auto* plugin_instance = ImagePublisher::create(m, d, plugin_id);
    if (plugin_instance == nullptr)
    {
      return -1;
    }
    d->plugin_data[plugin_id] = reinterpret_cast<uintptr_t>(plugin_instance);
    return 0;
  };

  plugin.destroy = +[](mjData* d, int plugin_id) {
    auto* plugin_instance = reinterpret_cast<class ImagePublisher*>(d->plugin_data[plugin_id]);
    plugin_instance->free();
    delete reinterpret_cast<ImagePublisher*>(d->plugin_data[plugin_id]);
    d->plugin_data[plugin_id] = 0;
  };

  plugin.reset = +[](const mjModel* m, double*, void* plugin_data, int plugin_id) {
    auto* plugin_instance = reinterpret_cast<class ImagePublisher*>(plugin_data);
    plugin_instance->reset(m, plugin_id);
  };

  plugin.compute = +[](const mjModel* m, mjData* d, int plugin_id, int) {
    auto* plugin_instance = reinterpret_cast<class ImagePublisher*>(d->plugin_data[plugin_id]);
    plugin_instance->compute(m, d, plugin_id);
  };

  mjp_registerPlugin(&plugin);
}

ImagePublisher* ImagePublisher::create(const mjModel* m, mjData* d, int plugin_id)
{
  // Option: frame_id
  const char* frame_id_char = mj_getPluginConfig(m, plugin_id, "frame_id");
  std::string frame_id;
  if (strlen(frame_id_char) > 0)
  {
    frame_id = std::string(frame_id_char);
  }

  // Option: color_topic_name
  const char* color_topic_name_char = mj_getPluginConfig(m, plugin_id, "color_topic_name");
  std::string color_topic_name;
  if (strlen(color_topic_name_char) > 0)
  {
    color_topic_name = std::string(color_topic_name_char);
  }

  // Option: depth_topic_name
  const char* depth_topic_name_char = mj_getPluginConfig(m, plugin_id, "depth_topic_name");
  std::string depth_topic_name;
  if (strlen(depth_topic_name_char) > 0)
  {
    depth_topic_name = std::string(depth_topic_name_char);
  }

  // Option: info_topic_name
  const char* info_topic_name_char = mj_getPluginConfig(m, plugin_id, "info_topic_name");
  std::string info_topic_name;
  if (strlen(info_topic_name_char) > 0)
  {
    info_topic_name = std::string(info_topic_name_char);
  }

  // Option: width
  const char* width_char = mj_getPluginConfig(m, plugin_id, "width");
  int width = 320;
  if (strlen(width_char) > 0)
  {
    width = static_cast<int>(strtol(width_char, nullptr, 10));
  }
  if (width <= 0)
  {
    mju_error("[ImagePublisher] `width` must be positive.");
    return nullptr;
  }

  // Option: height
  const char* height_char = mj_getPluginConfig(m, plugin_id, "height");
  int height = 240;
  if (strlen(height_char) > 0)
  {
    height = static_cast<int>(strtol(height_char, nullptr, 10));
  }
  if (height <= 0)
  {
    mju_error("[ImagePublisher] `height` must be positive.");
    return nullptr;
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
    mju_error("[ImagePublisher] `publish_rate` must be positive.");
    return nullptr;
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
    mju_error("[ImagePublisher] Plugin not found in sensors.");
    return nullptr;
  }
  if (m->sensor_objtype[sensor_id] != mjOBJ_CAMERA)
  {
    mju_error("[ImagePublisher] Plugin must be attached to a camera.");
    return nullptr;
  }

  return new ImagePublisher(m, d, sensor_id, frame_id, color_topic_name, depth_topic_name, info_topic_name, height, width, publish_rate);
}

ImagePublisher::ImagePublisher(const mjModel* m, mjData*, int sensor_id, std::string frame_id, std::string color_topic_name, std::string depth_topic_name, std::string info_topic_name, int height, int width, mjtNum publish_rate)
    : ros_context_(RosContext::getInstance())
    , sensor_id_(sensor_id)
    , camera_id_(m->sensor_objid[sensor_id])
    , frame_id_(std::move(frame_id))
    , publish_skip_(std::max(static_cast<int>(1.0 / (publish_rate * m->opt.timestep)), 1))
    , viewport_({0, 0, width, height})
{
  std::string sensor_name = std::string(mj_id2name(m, mjOBJ_SENSOR, sensor_id_));
  std::string camera_name = std::string(mj_id2name(m, mjOBJ_CAMERA, camera_id_));
  if (frame_id_.empty())
  {
    frame_id_ = camera_name;
  }
  if (color_topic_name.empty())
  {
    color_topic_name = "mujoco/" + camera_name + "/color";
  }
  if (depth_topic_name.empty())
  {
    depth_topic_name = "mujoco/" + camera_name + "/depth";
  }
  if (info_topic_name.empty())
  {
    info_topic_name = "mujoco/" + camera_name + "/camera_info";
  }

  // Init OpenGL
  if (glfwInit() == GLFW_FALSE)
  {
    mju_error("[ImagePublisher] Could not initialize GLFW.");
  }

  // Create invisible window, single-buffered
  glfwWindowHint(GLFW_VISIBLE, GLFW_FALSE);
  glfwWindowHint(GLFW_DOUBLEBUFFER, GLFW_FALSE);
  window_ = glfwCreateWindow(viewport_.width, viewport_.height, "RosMujoco::ImagePublisher", nullptr, nullptr);
  if (window_ == nullptr)
  {
    mju_error("[ImagePublisher] Could not create GLFW window.");
  }

  // Make context current
  glfwMakeContextCurrent(window_);

  // Init default data for visualization structures
  mjv_defaultCamera(&camera_);
  mjv_defaultOption(&option_);
  mjv_defaultScene(&scene_);
  mjr_defaultContext(&context_);

  // Set viewport
  context_.offWidth = viewport_.width;
  context_.offHeight = viewport_.height;

  // Create scene and context
  mjv_makeScene(m, &scene_, 1000);
  mjr_makeContext(m, &context_, mjFONTSCALE_100);

  // Init camera
  camera_.type = mjCAMERA_FIXED;
  camera_.fixedcamid = camera_id_;

  // Allocate buffers
  size_t color_buffer_size = sizeof(unsigned char) * 3 * viewport_.width * viewport_.height;
  color_buffer_ = static_cast<unsigned char*>(std::malloc(color_buffer_size));
  color_buffer_flipped_ = static_cast<unsigned char*>(std::malloc(color_buffer_size));
  size_t depth_buffer_size = sizeof(float) * viewport_.width * viewport_.height;
  depth_buffer_ = static_cast<float*>(std::malloc(depth_buffer_size));
  depth_buffer_flipped_ = static_cast<float*>(std::malloc(depth_buffer_size));

  // Create interface
  color_pub_ = ros_context_->getNode()->create_publisher<sensor_msgs::msg::Image>(color_topic_name, 1);
  depth_pub_ = ros_context_->getNode()->create_publisher<sensor_msgs::msg::Image>(depth_topic_name, 1);
  info_pub_ = ros_context_->getNode()->create_publisher<sensor_msgs::msg::CameraInfo>(info_topic_name, 1);

  std::cout << "[ImagePublisher] Configured (" << sensor_name << "):\n";
  std::cout << "  - frame_id: " << frame_id_ << "\n";
  std::cout << "  - color_topic_name: " << color_topic_name << "\n";
  std::cout << "  - depth_topic_name: " << depth_topic_name << "\n";
  std::cout << "  - info_topic_name: " << info_topic_name << "\n";
  std::cout << "  - width: " << viewport_.width << "\n";
  std::cout << "  - height: " << viewport_.height << "\n";
  std::cout << "  - publish_rate: " << publish_rate << " (adjusted rate " << 1. / (publish_skip_ * m->opt.timestep) << ")\n";
  std::cout << std::flush;
}

void ImagePublisher::reset(const mjModel*, int) { iteration_count_ = 0; }

void ImagePublisher::compute(const mjModel* m, mjData* d, int)
{
  ++iteration_count_;
  if (iteration_count_ % publish_skip_ != 0)
  {
    return;
  }

  // Make context current
  glfwMakeContextCurrent(window_);

  // Update abstract scene
  mjv_updateScene(m, d, &option_, nullptr, &camera_, mjCAT_STATIC | mjCAT_DYNAMIC, &scene_);

  // Render scene in offscreen buffer
  mjr_render(viewport_, &scene_, &context_);

  // Read rgb and depth pixels
  mjr_readPixels(color_buffer_, depth_buffer_, viewport_, &context_);

  // Convert raw depth to distance and flip images
  const auto near = static_cast<float>(m->vis.map.znear * m->stat.extent);
  const auto far = static_cast<float>(m->vis.map.zfar * m->stat.extent);
  for (int w = 0; w < viewport_.width; w++)
  {
    for (int h = 0; h < viewport_.height; h++)
    {
      int idx = h * viewport_.width + w;
      int idx_flipped = (viewport_.height - 1 - h) * viewport_.width + w;

      // See: https://github.com/deepmind/mujoco/blob/631b16e7ad192df936195658fe79f2ada85f755c/python/mujoco/renderer.py#L175-L178
      depth_buffer_[idx] = near / (1.F - depth_buffer_[idx] * (1.F - near / far));

      for (int c = 0; c < 3; c++)
      {
        color_buffer_flipped_[3 * idx_flipped + c] = color_buffer_[3 * idx + c];
      }
      depth_buffer_flipped_[idx_flipped] = depth_buffer_[idx];
    }
  }

  // Publish topic
  builtin_interfaces::msg::Time stamp;
  stamp.sec = static_cast<int32_t>(d->time);
  stamp.nanosec = static_cast<uint32_t>((d->time - stamp.sec) * 1e9);

  bool publish_color = color_pub_->get_subscription_count() > 0;
  bool publish_depth = depth_pub_->get_subscription_count() > 0;
  bool publish_info = info_pub_->get_subscription_count() > 0;

  if (publish_color)
  {
    auto color_msg = std::make_unique<sensor_msgs::msg::Image>();
    color_msg->header.stamp = stamp;
    color_msg->header.frame_id = frame_id_;
    color_msg->height = viewport_.height;
    color_msg->width = viewport_.width;
    color_msg->encoding = "rgb8";
    color_msg->is_bigendian = 0;
    color_msg->step = static_cast<unsigned int>(sizeof(unsigned char) * 3 * viewport_.width);
    color_msg->data.resize(sizeof(unsigned char) * 3 * viewport_.width * viewport_.height);
    std::memcpy(color_msg->data.data(), color_buffer_flipped_, sizeof(unsigned char) * 3 * viewport_.width * viewport_.height);
    color_pub_->publish(std::move(color_msg));
  }

  if (publish_depth)
  {
    auto depth_msg = std::make_unique<sensor_msgs::msg::Image>();
    depth_msg->header.stamp = stamp;
    depth_msg->header.frame_id = frame_id_;
    depth_msg->height = viewport_.height;
    depth_msg->width = viewport_.width;
    depth_msg->encoding = "32FC1";
    depth_msg->is_bigendian = 0;
    depth_msg->step = static_cast<unsigned int>(sizeof(float) * viewport_.width);
    depth_msg->data.resize(sizeof(float) * viewport_.width * viewport_.height);
    std::memcpy(depth_msg->data.data(), depth_buffer_flipped_, sizeof(float) * viewport_.width * viewport_.height);
    depth_pub_->publish(std::move(depth_msg));
  }

  if (publish_color || publish_depth || publish_info)
  {
    auto info_msg = std::make_unique<sensor_msgs::msg::CameraInfo>();
    info_msg->header.stamp = stamp;
    info_msg->header.frame_id = frame_id_;
    info_msg->height = viewport_.height;
    info_msg->width = viewport_.width;
    info_msg->distortion_model = "plumb_bob";
    info_msg->d.resize(5, 0.0);
    info_msg->k.fill(0.0);
    info_msg->r.fill(0.0);
    info_msg->p.fill(0.0);
    double focal_scaling = (1.0 / std::tan((m->cam_fovy[camera_id_] * M_PI / 180.0) / 2.0)) * viewport_.height / 2.0;
    info_msg->k[0] = info_msg->p[0] = focal_scaling;
    info_msg->k[2] = info_msg->p[2] = static_cast<double>(viewport_.width) / 2.0;
    info_msg->k[4] = info_msg->p[5] = focal_scaling;
    info_msg->k[5] = info_msg->p[6] = static_cast<double>(viewport_.height) / 2.0;
    info_msg->k[8] = info_msg->p[10] = 1.0;
    info_pub_->publish(std::move(info_msg));
  }
}

void ImagePublisher::free()
{
  std::free(color_buffer_);
  std::free(depth_buffer_);
  std::free(color_buffer_flipped_);
  std::free(depth_buffer_flipped_);

  mjr_freeContext(&context_);
  mjv_freeScene(&scene_);

  glfwDestroyWindow(window_);
}

}  // namespace RosMujoco
