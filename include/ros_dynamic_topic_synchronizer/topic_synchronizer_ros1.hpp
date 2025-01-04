/// Topic synchronizer, specialized for ROS 1
#pragma once

#include <ros/ros.h>
#include <chrono> 

/// First, add the details needed for ROS 1:
namespace fsd {
namespace mf {
namespace detail {

struct ROS1Adapter {

  /// MsgPtr is the smart-pointer type ROS uses for messages. Mind that MessageT::ConstrPtr is not
  /// necessarily the same type:
  // For example, when using pcl_conversions and subscribing to a pcl::PointCloud,
  /// the pcl::PointCloud::ConstPtr is a std smart pointer instead of a boost smart pointer. (with
  /// newer PCL versions). ROS 2 uses the std-pointer
  template <typename MsgT>
  using MsgPtr = boost::shared_ptr<const MsgT>;
  using NodeHandle = ros::NodeHandle;

  /* @brief Typed subscriber, to know how to subscribe without having to specify the type of the message to the subscribe function.
  * Needed only for ROS 1 since ROS 2 does not use the type-erasing subscribers anymore.
  */
  template <typename _MessageT>
  struct Subscriber {
    using Msg = _MessageT;
    ros::Subscriber sub_;
  };

  using Clock = std::chrono::system_clock;
  using Time = std::chrono::time_point<Clock>;
  using Duration = Clock::duration;
  
  using SubOption = uint32_t; // The queue length

  static Time stamp_to_chrono(const ros::Time &stamp) {
      return Time(std::chrono::seconds(stamp.sec) + std::chrono::nanoseconds(stamp.nsec));
  }

  static void log_warn(NodeHandle &node, const std::string &msg) {
      ROS_WARN_STREAM(msg);
  }

  static void log_info(NodeHandle &node, const std::string &msg) {
      ROS_INFO_STREAM(msg);
  }

  template<typename MessageType, typename F>
  static auto subscribe(NodeHandle &node_handle, Subscriber<MessageType> &sub, const std::string &topic_name, F cb, uint32_t queue_length) {
      sub.sub_ = node_handle.subscribe<MessageType>(topic_name, queue_length, cb);
      return sub;
  }
};

}

using ROSAdapter = detail::ROS1Adapter;

}  // namespace mf
}  // namespace fsd

#include <ros_dynamic_topic_synchronizer/impl/topic_synchronizer.hpp>
