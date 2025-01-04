/// Topic synchronizer, specialized for ROS 2
#pragma once 

#include <any> 
#include <map>
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

    using _Node = rclcpp::Node; /// Underlying Node type
    /// A node interface, wrapping to some common

    class Node : public rclcpp::Node {
    public:
        using Base = rclcpp::Node;
        
        Node(const std::string &name) : Base(name) {}

        template<typename Msg, typename F>
        void add_subscription(std::string topic, F cb) {
            if(my_subscribers_.count(topic) != 0) {
                /// TODO throw topic already exists
            }
            if(my_publishers_.count(topic) != 0) {
                /// TODO throw cannot subscribe on a topic that is being published at the same time
            }
            my_subscribers_[topic] = create_subscription<Msg>(topic, 1, cb);
        }

        template<typename Msg>
        auto add_publication(std::string topic, std::optional<double> max_frequency) {
            if(my_publishers_.count(topic) != 0) {
                /// TODO throw topic already exists
            }
            if(my_subscribers_.count(topic) != 0) {
                /// TODO throw cannot publish on a topic that is being subscribed at the same time
            }
            auto publisher = create_publisher<Msg>(topic, 1);
            my_publishers_[topic] = publisher;
            auto const publish = [this, publisher, topic, max_frequency](const Msg &msg) {
                auto curr_time = this->get_clock()->now();
                if(!max_frequency || !last_published_time_.count(topic) || 
                       (curr_time - last_published_time_.at(topic)).seconds() > (1. / max_frequency.value()) ) {

                    publisher->publish(msg);
                    last_published_time_[topic] = curr_time;
                }
            };
            return publish;
        }

        template<typename F>
        void add_timer(Duration time_interval, F cb) {
            my_timers_.emplace_back(create_wall_timer(time_interval, cb));
        }

        /// TODO add service 
        /// TODO add action
    private:
        std::map<std::string, rclcpp::Time> last_published_time_;
        std::vector<Timer> my_timers_;
        std::map<std::string, std::any> my_subscribers_;
        std::map<std::string, std::any> my_publishers_;
    };

    using NodeHandle = Node;
};

using ROSAdapter = ROS2Adapter;

}

}
}  // namespace mf
}  // namespace fsd

#include <ros_dynamic_topic_synchronizer/impl/topic_synchronizer.hpp>
namespace fsd {
namespace mf {
namespace ros2 {
template <typename... MessagesT>
using TopicSynchronizer = TopicSynchronizer<false, MessagesT...>;
}
}  // namespace mf
}  // namespace fsd
