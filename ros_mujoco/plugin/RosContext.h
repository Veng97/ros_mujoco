#pragma once

#include <rclcpp/executors/single_threaded_executor.hpp>
#include <rclcpp/node.hpp>

#include <mujoco/mjdata.h>

namespace RosMujoco {

class RosContext {
public:
  /**
   * \brief Get the instance of RosContext. Must be aquired by any plugin that
   * needs to publish/subscribe.
   * \note Once all instances are released/destroyed, the context will be shut
   * down.
   * \return Shared pointer to the instance.
   */
  static std::shared_ptr<RosContext> getInstance();

  /**
   * \brief Get the node to use for publishing/subscribing.
   */
  std::shared_ptr<rclcpp::Node> getNode();

  /**
   * \brief Spin node until all callbacks are complete.
   * \param d mjData pointer, used to prevent spinning multiple times in the
   * same time step.
   */
  void spinUntilComplete(const mjData *d);

  // Delete copy constructor and assignment operator to enforce singleton
  RosContext(const RosContext &) = delete;
  RosContext &operator=(const RosContext &) = delete;

private:
  // Private constructor and destructor to prevent external instantiation
  RosContext();
  ~RosContext();

  static std::weak_ptr<RosContext>
      instance_; // Use weak_ptr to prevent cyclic dependency

  std::shared_ptr<rclcpp::Node> node_;
  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
};

} // namespace RosMujoco