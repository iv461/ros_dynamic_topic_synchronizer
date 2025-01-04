/// Topic synchronizer, specialized for ROS 2
#pragma once 

#include <rclcpp/rclcpp.hpp>

namespace fsd {
namespace mf {

namespace detail {

class ROS2Adapter {
public:

    template<typename Msg>
    using MsgPtr = std::shared_ptr<Msg>;

    /// TODO should be compatible with the templates, but check
    using Clock = std::chrono::system_clock;
    using Time = std::chrono::time_point<Clock>;
    using Duration = Clock::duration;

    using Timer = rclcpp::TimerBase::SharedPtr;


    template<typename Msg>
    using Subscriber = typename rclcpp::Subscription<Msg>::SharedPtr;
    template<typename Msg>
    using Publisher = typename rclcpp::Publisher<Msg>::SharedPtr;

    using NodeHandle = rclcpp::Node; /// Underlying Node type
    /// A node interface, wrapping to some common

    static Time stamp_to_chrono(rclcpp::Header::Stamp &stamp) {
        return stamp.to_chrono();
    }

    static void log_warn(NodeHandle &node, const std::string &msg) {
      RCLCPP_WARN(node->get_logger(), msg);
    }
    static void log_info(NodeHandle &node, const std::string &msg) {
      RCLCPP_INFO(node->get_logger(), msg);
    }

    template<typename Sub, typename F>
    static auto subscribe(NodeHandle &node, Sub &sub, std::string &topic_name, F cb, rclcpp::QoS &qos) {
        using MessageType = typename Sub::ROSMessageType;
        return node->create_subscription<Msg>(topic_name, qos, cb);
    }

    
};

}
using ROSAdapter = detail::ROS2Adapter;

}  // namespace mf
}  // namespace fsd

#include <ros_dynamic_topic_synchronizer/impl/topic_synchronizer.hpp>

