/// Topic synchronizer, specialized for ROS 2
#pragma once 

#include <rclcpp/rclcpp.hpp>

namespace fsd {
namespace mf {

namespace detail {
template <typename MsgT>
template<typename MsgT>
using MsgPtr = std::shared_ptr<const MsgT>;

using Subscriber = rclcpp::Subscription<MsgT>;

using NodeHandle = rclcpp::Node;
}
}  // namespace mf
}  // namespace fsd

#include <ros_dynamic_topic_synchronizer/topic_synchronizer.hpp>
namespace fsd {
namespace mf {
namespace ros2 {
template <typename... MessagesT>
using TopicSynchronizer = TopicSynchronizer<false, MessagesT...>;
}
}  // namespace mf
}  // namespace fsd
