#pragma once

#include "ros_mujoco/plugin/RosContext.hpp"

#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/quaternion_stamped.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <rclcpp/publisher.hpp>
#include <ros_mujoco_interfaces/msg/scalar_stamped.hpp>

#include <mujoco/mjdata.h>
#include <mujoco/mjmodel.h>
#include <mujoco/mjtnum.h>
#include <mujoco/mjvisualize.h>

#include <string>

namespace RosMujoco
{

/** \brief Plugin to publish sensor data. */
class SensorPublisher
{
  public:
    /** \brief Type of ROS message. */
    using MessageType = enum class MessageType : std::uint8_t
    {
        //! Scalar
        Scalar = 0,

        //! Point
        Point,

        //! 3D vector
        Vector3,

        //! Quaternion
        Quaternion
    };

  public:
    /** \brief Register plugin. */
    static void registerPlugin();

    /** \brief Create an instance.
        \param m model
        \param d data
        \param plugin_id plugin ID
     */
    static SensorPublisher* create(const mjModel* m, mjData* d, int plugin_id);

  public:
    /** \brief Copy constructor. */
    SensorPublisher(SensorPublisher&&) = default;

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

  protected:
    /** \brief Constructor.
        \param m model
        \param d data
        \param sensor_id sensor ID
        \param msg_type type of ROS message
        \param frame_id frame ID of message header
        \param topic_name topic name
        \param publish_rate publish rate
     */
    SensorPublisher(const mjModel* m, mjData* d, int sensor_id,
                    MessageType msg_type, const std::string& frame_id,
                    const std::string& topic_name, mjtNum publish_rate);

  protected:
    //! ROS context
    std::shared_ptr<RosContext> ros_context_;

    //! Sensor ID
    int sensor_id_ = -1;

    //! Type of ROS message
    MessageType msg_type_;

    //! Frame ID of message header
    std::string frame_id_;

    //! Topic name
    std::string topic_name_;

    //! ROS publisher
    rclcpp::PublisherBase::SharedPtr pub_;

    //! Iteration interval to skip ROS publish
    int publish_skip_ = 0;

    //! Iteration count of simulation
    int iteration_count_ = 0;
};

} // namespace RosMujoco
