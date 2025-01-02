#pragma once 
/// Topic synchronizer, specialized for ROS 2

#include <rclcpp/rclcpp.hpp>
#include <topic_synchronizer.hpp>

namespace fsd {
namespace mf {

namespace detail {
template <typename MsgT>
using Subscriber = rclcpp::Subscription<MsgT>;
using NodeHandle = rclcpp::Node;
template<typename MsgT>
using MsgPtr = std::shared_ptr<const MsgT>;
}

namespace ros2 {
template <typename... MessagesT>
using TopicSynchronizer = TopicSynchronizer<false, MessagesT...>;
}
}  // namespace mf
}  // namespace fsd
