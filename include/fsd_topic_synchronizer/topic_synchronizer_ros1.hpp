/****************************************************************************

 * Copyright © 2018-2020 Fraunhofer FKIE, StarkStrom Augsburg
 * Authors: Ivo Ivanov, Timo Röhling
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 ****************************************************************************/

#pragma once

#include <ros/ros.h>

#include <array>
#include <boost/functional/hash.hpp>
#include <boost/optional.hpp>
#include <boost/optional/optional_io.hpp>
#include <deque>
#include <functional>
#include <map>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

namespace fsd {
namespace mf {

/// MsgPtr is the smart-pointer type ROS uses for messages. Mind that MessageT::ConstrPtr is not
/// necessarily the same type:
// For example, when using pcl_conversions and subscribing to a pcl::PointCloud,
/// the pcl::PointCloud::ConstPtr is a std smart pointer instead of a boost smart pointer. (with
/// newer PCL versions)
template <typename MsgT>
using MsgPtr = boost::shared_ptr<const MsgT>;

/**
 * @brief The topic synchronization
 *
 * @tparam MessagesT a list of the different message types to be synchronized. No pointer types
 * should be given, instead the plain message type, for example sensor_msgs::Image.
 *
 */
template <typename... MessagesT>
class TopicSynchronizer {
  constexpr static auto NUM_MESSAGE_TYPES = sizeof...(MessagesT);

public:
  /// @brief The type of the user callback it a function receiving maps for each message type
  /// where the key of the map is the topic name and the value is a pointer to the message.
  /// The order of the messages types is the same as the order of the given template arguments.
  using UserCallbackT = std::function<void(const std::map<std::string, MsgPtr<MessagesT>> &...)>;

  /**
   * @brief Constructs the synchronizer. Sets the callback but does not subscribe yet.
   * @param node_handle The node handle used for subscription
   * @param user_callback The callback which is called when a set of synchronized messages is
   * emitted
   * @param sync_policy The already created synchronization policy with which the topics are
   * synchronized.
   */
  TopicSynchronizer(const ros::NodeHandle &node_handle, UserCallbackT user_callback,
                    std::unique_ptr<SyncPolicy> sync_policy)
      : sync_policy_(std::move(sync_policy)),
        node_handle_(node_handle),
        user_callback_(user_callback) {
    sync_policy_->removed_buffered_msg_ = [this](const auto &msg_id) {
      remove_buffered_msg(msg_id);
    };
    sync_policy_->emit_messages_ = [this](const auto &msg_ids) { emit_messages(msg_ids); };
  }

  /**
   * @brief Subscribe to all topics and start receiving messages. Should be called only
   * once.
   *
   * @param topic_names An array of the topic names for each topic type. There can be zero to an
   * arbitrary number of topics for each topic type.
   * @param queue_length The queue length of the subscriber, the same for each subscriber.
   */
  void subscribe(
      const std::array<std::vector<std::string>, NUM_MESSAGE_TYPES> &topic_names_for_topic_type,
      uint32_t queue_length) {
    /// TODO Reset

    /// First, set the number of queues and initialize the topic names
    size_t message_index = 0;
    size_t queue_index = 0;
    for (const auto &topic_names : topic_names_for_topic_type) {
      for (const auto &topic : topic_names) {
        flattened_topic_names_.push_back(topic);
        topic_name_to_queue_index_.emplace(topic, queue_index);
        queue_index_to_message_type_index_.push_back(message_index);
        queue_index++;
      }
      message_index++;
    }

    sync_policy_->set_topic_names_for_queues(flattened_topic_names_);

    /// Subscribe too all message types
    detail::call_for_index<NUM_MESSAGE_TYPES>([&](auto Index) {
      subscribe_single_message_type<Index>(std::get<Index>(topic_names_for_topic_type),
                                           queue_length);
    });
  }

private:
  using MessagesTupleT = std::tuple<std::map<std::string, MsgPtr<MessagesT>>...>;

  void emit_messages(const std::vector<MessageIdT> &message_ids) {
    MessagesTupleT msg_tuple;
    /// Fills in the message in the passed tuple of vectors of messages
    for (const auto &message_id : message_ids) {
      detail::call_for_index<NUM_MESSAGE_TYPES>(
          [&](auto Index) { add_message_to_tuple_for_index<Index>(msg_tuple, message_id); },
          queue_index_to_message_type_index_.at(message_id.queue_index));
    }
    unpack_message_tuple_and_call_user_callback(msg_tuple,
                                                std::make_index_sequence<NUM_MESSAGE_TYPES>{});
  }

  void remove_buffered_msg(const MessageIdT &message_id) {
    auto topic_name = flattened_topic_names_.at(message_id.queue_index);
    detail::call_for_index<NUM_MESSAGE_TYPES>(
        [&](auto Index) { std::get<Index>(message_buffers_).erase(message_id); },
        queue_index_to_message_type_index_.at(message_id.queue_index));
  }

  template <size_t... Index>
  void unpack_message_tuple_and_call_user_callback(const MessagesTupleT &message_tuple,
                                                   std::index_sequence<Index...>) {
    if (user_callback_) {
      user_callback_(std::get<Index>(message_tuple)...);
    }
  }

  template <size_t Index>
  void add_message_to_tuple_for_index(MessagesTupleT &msg_tuple, const MessageIdT &msg_id) {
    /// TODO remove, this likely cannot happen
    if (queue_index_to_message_type_index_.at(msg_id.queue_index) != Index) {
      throw std::invalid_argument("The message_id " + msg_id.to_string() +
                                  " does not math the Index arg: " + std::to_string(Index));
    }
    const auto &buff = std::get<Index>(message_buffers_);
    if (!buff.count(msg_id)) {
      throw std::invalid_argument(
          "The message_id " + msg_id.to_string() +
          " was not found in the buffer with this index: " + std::to_string(Index));
    }
    const auto &topic_name = flattened_topic_names_.at(msg_id.queue_index);
    std::get<Index>(msg_tuple).emplace(topic_name, buff.at(msg_id));
  }

  /// Subscribe too all topics of the same message type
  template <size_t message_index>
  void subscribe_single_message_type(const std::vector<std::string> &topic_names,
                                     uint32_t queue_length) {
    auto &subs_array = std::get<message_index>(subscribers_);
    subs_array.resize(topic_names.size());

    for (size_t i_topic = 0; i_topic < topic_names.size(); i_topic++) {
      const auto &topic_name = topic_names.at(i_topic);
      auto queue_index = topic_name_to_queue_index_.at(topic_name);
      subs_array.at(i_topic).subscribe(
          node_handle_, topic_name, queue_length,
          [this, queue_index](const auto &msg) { msg_callback<message_index>(msg, queue_index); });
    }
  }

  template <size_t message_index, typename MsgPtrT>
  void msg_callback(MsgPtrT msg, size_t queue_index) {
    MessageIdT msg_id{msg->header.stamp, queue_index};
    auto &msg_buff = std::get<message_index>(message_buffers_);
    if (msg_buff.count(msg_id) > 0) {
      ROS_WARN_STREAM(
          "Ignored trying to add a message with duplicate timestamp, timestamp of the "
          "message: "
          << msg_id.stamp);
      return;
    }
    /// Store the message
    msg_buff.emplace(msg_id, msg);
    sync_policy_->add(msg_id);
  }

  ros::NodeHandle node_handle_;
  UserCallbackT user_callback_;
  std::unique_ptr<SyncPolicy> sync_policy_;

  std::tuple<std::vector<detail::Subscriber<MessagesT>>...> subscribers_;

  /// Mapping between a queue index and topic names
  std::vector<std::string> flattened_topic_names_;
  /// Mapping between a topic name and its queue index
  std::unordered_map<std::string, size_t> topic_name_to_queue_index_;
  /// Gives the mapping from the queue index to the message type index, for example if
  /// the message types are [Image, PointCloud] and we have two subscriptions for Image
  /// and one for PointCloud, this array will be [0, 0, 1]
  std::vector<size_t> queue_index_to_message_type_index_;

  /// @brief A tuple of the buffers for the incoming messages for each message type. They do not
  /// need to be ordered as each message is uniquely identified by its
  /// MessageId. For each message type there can be multiple topics.
  std::tuple<std::unordered_map<MessageIdT, MsgPtr<MessagesT>, detail::MessageIdHash>...>
      message_buffers_;
};

}  // namespace mf
}  // namespace fsd
