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
#include <fsd_common/cpputils.hpp>
#include <functional>
#include <map>
#include <memory>
#include <string>
#include <vector>

namespace fsd {
namespace mf {

/// The smart-pointer type ROS uses for messages. Mind that MessageT::ConstrPtr is not guaranteed
/// to be the same, for example when using pcl_conversions and subscribing to a pcl::PointCloud,
/// the pcl::PointCloud::ConstPtr is a std smart pointer with newer (>1.10 I think) PCL versions,
template <typename MsgT>
using MsgPtr = boost::shared_ptr<const MsgT>;
namespace detail {
/// MessageId identifies a message uniquely. Used to decouple the synchronization policy
/// from the message content and message type. The synchronization policy needs only to know the
/// timestamp in most cases.
struct MessageId {
  MessageId(const ros::Time &stamp, size_t queue_index) : stamp(stamp), queue_index(queue_index) {}
  /// The timestamp, copied from the header. It uniquely identifies a message because duplicate
  /// timestamps on the same topic are not allowed, that means the TopicSynchronizer filters
  /// duplicates if it receives any.
  ros::Time stamp;
  /// The queue index, a queue is created for each topic.
  size_t queue_index;

  friend bool operator==(const MessageId &lhs, const MessageId &rhs) {
    return lhs.stamp == rhs.stamp && lhs.queue_index == rhs.queue_index;
  }
  friend bool operator!=(const MessageId &lhs, const MessageId &rhs) { return !(lhs == rhs); }
  std::string to_string() const {
    return "stamp: " + std::to_string(stamp.toNSec()) +
           ", queue_index: " + std::to_string(queue_index);
  }
  friend std::ostream &operator<<(std::ostream &os, const MessageId &ob) {
    return os << ob.to_string();
  }
};
struct MessageIdHash {
  size_t operator()(const MessageId &c) const noexcept {
    size_t seed = 0;
    boost::hash_combine(seed, c.stamp.toNSec());
    boost::hash_combine(seed, c.queue_index);
    return seed;
  }
};

/* @brief Typed subscriber to know how to subscribe w/o specifying the type to the subscribe
 * function
 */
template <typename MessageT>
struct Subscriber {
  void subscribe(ros::NodeHandle &node_handle, std::string topic_name, uint32_t queue_length,
                 std::function<void(const MsgPtr<MessageT> &)> callback) {
    sub_ = node_handle.subscribe<MessageT>(topic_name, queue_length, callback);
  }
  ros::Subscriber sub_;
};

template <typename F, size_t... Index>
void call_for_index_apply(F func, boost::optional<size_t> index,
                          std::index_sequence<Index...> integer_seq) {
  /// First, compile all functions and put them in an array
  std::array<std::function<void()>, integer_seq.size()> functions{
      [&]() { func(std::integral_constant<size_t, Index>{}); }...};
  /// Then, call the function at index or for all functions
  if (index) {
    functions.at(index.get())();
  } else {
    for (auto f : functions) {
      f();
    }
  }
}

/**
 * @brief Gets a function template which receives an index, and instantiates
 * it for every index from 0 to max_index. Then it calls the instantiation
 * at index passed as an argument to this function.
 *
 * @tparam F has to be a function type void f(size_t Index). The passed index is a constexpr
 * @tparam max_index > 0. If not provided, the function is called for all indices
 */
template <size_t max_index, typename F>
void call_for_index(F func, boost::optional<size_t> index = boost::none) {
  call_for_index_apply(func, index, std::make_index_sequence<max_index>{});
}
}  // namespace detail

using MessageIdT = detail::MessageId;
template <typename T>
using OptionalT = boost::optional<T>;
/**
 * @brief Base class for the different synchronization policies.
 */
struct SyncPolicy {
  SyncPolicy(std::function<void(const std::vector<MessageIdT> &message_ids)> emit_messages,
             std::function<void(const MessageIdT &message_id)> removed_buffered_msg)
      : emit_messages_(emit_messages), removed_buffered_msg_(removed_buffered_msg) {}

  virtual ~SyncPolicy() = default;

  /// Called when a new message is received
  virtual void add(MessageIdT msg_id) = 0;
  /// Set in the sync policy the topic names for each topic.
  /// Also needed to know how many queues are needed as this is only known at runtime
  virtual void set_topic_names_for_queues(const std::vector<std::string> &topic_names) = 0;

  /// Callback the policy calls when a set of synchronized messages was created and should be
  /// emitted. They are not automatically removed, as the policy may emit them multiple times, so
  /// the policy has to call removed_buffered_msg_ for messages which are not going to be emmitted
  /// again.
  std::function<void(const std::vector<MessageIdT> &message_ids)> emit_messages_;
  /// As the topic synchronizer buffers by default all incoming messages and only the policy
  /// knows which messages should be dropped, this callback should be called to free a buffered
  /// message. Messages can be freed if they are too old for example.
  std::function<void(const MessageIdT &message_id)> removed_buffered_msg_;
};

// Implementation of the policy which synchronizes the topics by approximately matching the header
// timestamps. It is an reimplementation of the one in
// [fkie_message_filters](https://github.com/fkie/message_filters/blob/master/fkie_message_filters/include/fkie_message_filters/combiner_policies/approximate_time_impl.h)
// It has the additional feature of a timeout: When a topic does not receive a message a subset of
// synchronized messages is still emitted, but does not contain messages from that topic.
/// TODO Ivo: The maximum allowed time difference between messages should be implemented
struct ApproxTimePolicy : public SyncPolicy {
  /**
   * @brief Construct the ApproxTimePolicy
   *
   * @param max_msg_age The maximum age a message can have until it is dropped.
   * @param timeout Optionally, a timeout period can be given. When the time difference
   * between the timestamp of the latest received message
   * and the latest received message of another topic exceeds this time, this topic is said to be
   * timed out. In this case, a subset of synchronized messages is still emmitted, but does not
   * contain messages from this topic. If no timeout is given, no topic is allowed to time out.
   * In this case, no set of synchronized messages is emmitted as long as no every topic sends
   * messages.
   *
   */
  ApproxTimePolicy(
      const ros::Duration &max_msg_age, const OptionalT<ros::Duration> &timeout,
      std::function<void(const std::vector<MessageIdT> &message_ids)> emit_messages = nullptr,
      std::function<void(const MessageIdT &message_id)> removed_buffered_ms = nullptr);

  void add(MessageIdT msg_id) override;

  void set_topic_names_for_queues(const std::vector<std::string> &topic_names) override {
    topic_names_for_queues_ = topic_names;
    set_number_of_topics(topic_names.size());
  }

private:
  using QueueT = std::deque<MessageIdT>;

  /// Returns the absolute time difference between two timestamps
  static ros::Duration abs_difference(ros::Time t1, ros::Time t2) {
    return t1 > t2 ? t1 - t2 : t2 - t1;
  }

  void set_number_of_topics(uint32_t number_of_topics);

  void remove_too_old_msgs(ros::Time cutoff);

  /// returns whether more can be emitted
  bool try_to_emit();

  void remove_last_emmitted_msgs();

  bool can_still_find_better_match_in_queue(uint32_t queue_index);

  bool can_still_find_better_match_in_any_queue();

  void reset();

  void select_pivot();

  void determine_timeouts(ros::Time current_timestamp);

  uint32_t number_of_topics_{0};

  std::vector<std::string> topic_names_for_queues_;
  ros::Duration max_msg_age_;
  OptionalT<ros::Duration> max_allowed_difference_;
  /// If no timeout is set, then all topics are required always
  OptionalT<ros::Duration> timeout_{1.};
  /// @brief the pivot is a queue index
  OptionalT<uint32_t> pivot_;
  /// The pivot timestamp identifies the message in the queue that we selected as the pivot. Note
  /// that the pivot queue can still receive new messages, so we need to remember this exact
  /// message.
  OptionalT<ros::Time> pivot_timestamp_;

  std::vector<QueueT> queues_;
  std::vector<OptionalT<MessageIdT>> heads_;
  /// For each topic the timestamps of the last received messages. Not initialized when no messages
  /// were received yet.
  std::vector<OptionalT<ros::Time>> last_timestamps_;
  /// For each topic a truth value whether we this topic timed out
  std::vector<bool> topics_timed_out_;
};

/**
 * @brief The topic synchronization
 *
 * @tparam MessagesT a list of the different message types to be synchronized. No pointer types
 * should be given, instead the message type, for example sensor_msgs::Image.
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
    /// TODO remove, cannot happen
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
};  // namespace mf

}  // namespace mf
}  // namespace fsd
