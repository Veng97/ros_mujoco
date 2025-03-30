#include "ros_mujoco/plugin/RosContext.hpp"

namespace RosMujoco {

std::weak_ptr<RosContext> RosContext::instance_;

RosContext::RosContext()
{
  rclcpp::init(0, nullptr);

  node_ = rclcpp::Node::make_shared("mujoco_ros");
  executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  executor_->add_node(node_);

  std::cout << "[RosContext] Initialized with node: " << node_->get_name() << "\n";
}

RosContext::~RosContext()
{
  std::cout << "[RosContext] Shutting down context\n";

  executor_.reset();
  node_.reset();

  rclcpp::shutdown();
}

std::shared_ptr<RosContext> RosContext::getInstance()
{
  std::shared_ptr<RosContext> instance = instance_.lock();
  if (!instance)
  {
    // Use a custom factory function instead of std::make_shared (which needs a public constructor)
    struct MakeSharedEnabler : public RosContext
    {
    };
    instance = std::make_shared<MakeSharedEnabler>();
    instance_ = instance;
  }
  return instance;
}

std::shared_ptr<rclcpp::Node> RosContext::getNode() { return node_; }

void RosContext::spinUntilComplete(const mjData* d)
{
  static mjtNum time_of_spin = -1.;

  // Guard against spinning multiple times in the same time step
  if (d->time == time_of_spin)
  {
    return;
  }
  time_of_spin = d->time;

  // Spin until there are no more callbacks to be executed
  if (executor_)
  {
    executor_->spin_some();
  }
}

}  // namespace RosMujoco
