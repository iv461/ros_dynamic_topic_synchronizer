#pragma once 
/// Topic synchronizer, specialized for ROS 2

#include <rclcpp/rclcpp.hpp>
#include <topic_synchronizer.hpp>

namespace fsd {
namespace mf {
namespace ros2 {

namespace detail {
}

template <typename... MessagesT>
using TopicSynchronizer = TopicSynchronizer<false, MessagesT...>;
}
}  // namespace mf
}  // namespace fsd
