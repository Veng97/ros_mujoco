#pragma once

#include "RosContext.h"

#include <builtin_interfaces/msg/time.hpp>
#include <rclcpp/publisher.hpp>
#include <rosgraph_msgs/msg/clock.hpp>

#include <mujoco/mjdata.h>
#include <mujoco/mjmodel.h>
#include <mujoco/mjtnum.h>
#include <mujoco/mjvisualize.h>

#include <memory>
#include <string>

namespace RosMujoco
{

/** \brief Plugin to publish clock topic. */
class ClockPublisher
{
  public:
    /** \brief Register plugin. */
    static void registerPlugin();

    /** \brief Create an instance.
        \param m model
        \param d data
        \param plugin_id plugin ID
     */
    static ClockPublisher* create(const mjModel* m, mjData* d, int plugin_id);

    /** \brief Copy constructor. */
    ClockPublisher(ClockPublisher&&) = default;

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

    /** \brief Constructor.
        \param m model
        \param d data
        \param topic_name topic name of clock
        \param publish_rate publish rate
        \param use_sim_time value of `use_sim_time` rosparam
    */
    ClockPublisher(const mjModel* m, mjData* d, const std::string& topic_name, const mjtNum& publish_rate);

  protected:
    //! ROS context
    std::shared_ptr<RosContext> ros_context_;

    //! ROS publisher for clock
    rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr pub_;

    //! Topic name of clock
    std::string topic_name_;

    //! Iteration interval to skip ROS publish
    int publish_skip_ = 0;

    //! Iteration count of simulation
    int iteration_count_ = 0;
};

} // namespace RosMujoco
