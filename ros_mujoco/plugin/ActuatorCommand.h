#pragma once

#include "RosContext.h"
#include <rclcpp/subscription.hpp>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>

#include <mujoco/mjdata.h>
#include <mujoco/mjmodel.h>
#include <mujoco/mjtnum.h>
#include <mujoco/mjvisualize.h>

#include <limits>
#include <string>

namespace RosMujoco {

/** \brief Plugin to send a command to an actuator via ROS topic. */
class ActuatorCommand {
public:
  /** \brief Register plugin. */
  static void RegisterPlugin();

  /** \brief Create an instance.
      \param m model
      \param d data
      \param plugin_id plugin ID
   */
  static ActuatorCommand *Create(const mjModel *m, mjData *d, int plugin_id);

public:
  /** \brief Copy constructor. */
  ActuatorCommand(ActuatorCommand &&) = default;

  /** \brief Reset.
      \param m model
      \param plugin_id plugin ID
   */
  void reset(const mjModel *m, int plugin_id);

  /** \brief Compute.
      \param m model
      \param d data
      \param plugin_id plugin ID
   */
  void compute(const mjModel *m, mjData *d, int plugin_id);

protected:
  /** \brief Constructor.
      \param m model
      \param d data
      \param actuator_id actuator ID
      \param topic_name topic name
  */
  ActuatorCommand(const mjModel *m, mjData *d, int actuator_id,
                  std::string topic_name);

  /** \brief Constructor.
      \param msg command message
  */
  void callback(const std_msgs::msg::Float64::SharedPtr msg);

protected:
  //! ROS context
  std::shared_ptr<RosContext> ros_context_;

  //! ROS subscriber for actuator command
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr sub_;

  //! Actuator ID
  int actuator_id_ = -1;

  //! Actuator command (NaN for no command)
  mjtNum ctrl_ = std::numeric_limits<mjtNum>::quiet_NaN();
};

} // namespace RosMujoco
