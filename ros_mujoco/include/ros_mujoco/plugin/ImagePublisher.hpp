#pragma once

#include "ros_mujoco/plugin/RosContext.hpp"

#include <mujoco/mjdata.h>
#include <mujoco/mjmodel.h>
#include <mujoco/mjrender.h>
#include <mujoco/mjtnum.h>
#include <mujoco/mjvisualize.h>

#include <GLFW/glfw3.h>

#include <rclcpp/publisher.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <string>

namespace RosMujoco {

/** \brief Plugin to publish topics of color and depth images. */
class ImagePublisher {
 public:
  /** \brief Register plugin. */
  static void registerPlugin();

  /** \brief Create an instance.
      \param m model
      \param d data
      \param plugin_id plugin ID
   */
  static ImagePublisher* create(const mjModel* m, mjData* d, int plugin_id);

  /** \brief Copy constructor. */
  ImagePublisher(ImagePublisher&&) = default;

  /** \brief Reset.
      \param m model
      \param plugin_id plugin ID
   */
  void reset(const mjModel* m, int plugin_id);

  /** \brief Compute.
      \param m model
      \param d data
      \param plugin_id plugin ID
   */
  void compute(const mjModel* m, mjData* d, int plugin_id);

  /** \brief Free buffer. */
  void free();

 protected:
  /** \brief Constructor.
      \param m model
      \param d data
      \param sensor_id sensor ID
      \param frame_id frame ID of topics header or TF parent
      \param color_topic_name topic name of color image
      \param depth_topic_name topic name of depth image
      \param info_topic_name topic name of camera information
      \param height image height
      \param width image width
      \param publish_rate publish rate
  */
  ImagePublisher(const mjModel* m, mjData* d, int sensor_id, std::string frame_id, std::string color_topic_name, std::string depth_topic_name, std::string info_topic_name, int height, int width, mjtNum publish_rate);

  //! ROS context
  std::shared_ptr<RosContext> ros_context_;

  //! Sensor ID
  int sensor_id_ = -1;

  //! Camera ID
  int camera_id_ = -1;

  //! Frame ID of topics header or TF parent
  std::string frame_id_;

  //! Iteration interval to skip ROS publish
  int publish_skip_ = 0;

  //! Iteration count of simulation
  int iteration_count_ = 0;

  //! Data buffer
  //! @{
  unsigned char* color_buffer_;
  float* depth_buffer_;
  unsigned char* color_buffer_flipped_;
  float* depth_buffer_flipped_;
  //! @}

  //! Variables for visualization and rendering in MuJoCo
  //! @{
  mjvScene scene_;
  mjvCamera camera_;
  mjvOption option_;
  mjrContext context_;
  mjrRect viewport_;
  GLFWwindow* window_;
  //! @}

  //! ROS variables
  //! @{
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr color_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr depth_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr info_pub_;
  //! @}
};

}  // namespace RosMujoco
