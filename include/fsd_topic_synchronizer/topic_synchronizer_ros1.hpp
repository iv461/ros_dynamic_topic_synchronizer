/// Topic synchronizer, specialized for ROS 1
#pragma once

#include <ros/ros.h>
#include <topic_synchronizer.hpp>

namespace fsd {
namespace mf {
namespace ros1 {

namespace detail {

/* @brief Typed subscribers, to know how to subscribe without having to specify the type of the message to the subscribe function.
* Needed only for ROS 1 since ROS 2 does not use the type-erasing subscribers anymore.
 */
template <typename MessageT>
struct Subscriber {
  void subscribe(ros::NodeHandle &node_handle, std::string topic_name, uint32_t queue_length,
                 std::function<void(const MsgPtr<MessageT> &)> callback) {
    sub_ = node_handle.subscribe<MessageT>(topic_name, queue_length, callback);
  }
  ros::Subscriber sub_;
};
}

template <typename... MessagesT>
using TopicSynchronizer = TopicSynchronizer<true, MessagesT...>;

}  // namespace mf
}  // namespace fsd
