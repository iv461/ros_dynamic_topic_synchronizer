/// Topic synchronizer, specialized for ROS 1
#pragma once

#include <ros/ros.h>

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
  template <typename MessageT>
  struct Subscriber {
    void subscribe(ros::NodeHandle &node_handle, const std::string &topic_name, uint32_t queue_length,
                  std::function<void(const MsgPtr<MessageT> &)> callback) {
      sub_ = node_handle.subscribe<MessageT>(topic_name, queue_length, callback);
    }
    ros::Subscriber sub_;
  };

  template<typename Cb, typename Msg>
  auto subscribe(Subscriber<Msg> &sub, NodeHandle &nh, const std::string &topic, size_t queue_length, Cb callback) {
      sub.subscribe(node_handle_, topic_name, queue_length,
      return sub;
  }
};
}
}  // namespace mf
}  // namespace fsd

#include <ros_dynamic_topic_synchronizer/impl/topic_synchronizer.hpp>
namespace fsd {
namespace mf {

namespace ros1 {
template <typename... MessagesT>
using TopicSynchronizer = TopicSynchronizer<ROS1Adapter, MessagesT...>;
}

}  // namespace mf
}  // namespace fsd
