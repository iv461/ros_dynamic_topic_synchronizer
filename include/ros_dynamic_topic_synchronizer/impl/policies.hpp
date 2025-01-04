
/// Copyright 2018-2020 Fraunhofer FKIE, 
/// Authors: Ivo Ivanov, Timo Röhling
/// Licensed under the Apache License, Version 2.0.

/// Implementation of the approximate time synchronizer algorithm, adapted from 
// the package fkie_message_filters
#pragma once 

#include <chrono> 
#include <string> 

#include <boost/functional/hash.hpp>

#include <boost/optional/optional_io.hpp>
#include <boost/optional.hpp>

#include <functional>
#include <array>
#include <deque>

namespace fsd {
namespace mf {
using Clock = std::chrono::system_clock;
using Time = std::chrono::time_point<Clock>;
using Duration = Clock::duration;

namespace detail {

/// MessageId uniquely identifies a message. Used to decouple the synchronization policy from the
/// message content and message type. In most cases, the synchronization policy only needs to know
/// the timestamp.
struct MessageId {
  MessageId(const Time &stamp, size_t queue_index) : stamp(stamp), queue_index(queue_index) {}
  /// The timestamp, copied from the header. It uniquely identifies a message on a topic because duplicate
  /// timestamps on the same topic are not allowed. (The TopicSynchronizer filters duplicates if it receives any.)
  Time stamp;

  /// The queue index, a queue is created for each topic.
  size_t queue_index;

  /// Are two IDs the same ?
  friend bool operator==(const MessageId &lhs, const MessageId &rhs) {
    return lhs.stamp == rhs.stamp && lhs.queue_index == rhs.queue_index;
  }
  friend bool operator!=(const MessageId &lhs, const MessageId &rhs) { return !(lhs == rhs); }

  std::string to_string() const {
    return "stamp: " + std::to_string(0) + // TODO print stamp
           ", queue_index: " + std::to_string(queue_index);
  }

  friend std::ostream &operator<<(std::ostream &os, const MessageId &ob) {
    return os << ob.to_string();
  }
};
struct MessageIdHash {
  size_t operator()(const MessageId &c) const noexcept {
    size_t seed = 0;
    /// TODO std timestamps can already be hashed, use it maybe
    auto nsec_since = std::chrono::duration_cast<std::chrono::nanoseconds>(
                     c.stamp.time_since_epoch()).count();

    boost::hash_combine(seed, nsec_since);
    boost::hash_combine(seed, c.queue_index);
    return seed;
  }
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
 * @brief This function receives a function template that has a single index parameter, and instantiates
 * this function template for every index in the range of 0 to max_index. Then it calls all these instantiations
 * and additionally passesa to them their index as a regular function argument.
 *
 * @tparam F has to be a function template of type void f<size_t>(size_t Index). The passed index is a constexpr.
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
  /// Since the topic synchronizer buffers all incoming messages by default and only the policy
  /// knows which messages should be dropped, this callback should be called to release a buffered
  /// message. Messages can be released after after they have been emitted or if they are too old.
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
   * @param print_verbose Prints which messages arrived and on which we are still waiting, for easier debugging
   *
   */
  ApproxTimePolicy(
      const Duration &max_msg_age, const OptionalT<Duration> &timeout,
      bool print_verbose,
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
  static Duration abs_difference(Time t1, Time t2) {
    return t1 > t2 ? t1 - t2 : t2 - t1;
  }

  void set_number_of_topics(uint32_t number_of_topics);

  void remove_too_old_msgs(Time cutoff);

  /// returns whether more can be emitted
  bool try_to_emit();

  void remove_last_emmitted_msgs();

  bool can_still_find_better_match_in_queue(uint32_t queue_index);

  bool can_still_find_better_match_in_any_queue();

  void reset();

  void select_pivot();

  void determine_timeouts(Time current_timestamp);

  uint32_t number_of_topics_{0};

  std::vector<std::string> topic_names_for_queues_;
  Duration max_msg_age_;
  OptionalT<Duration> max_allowed_difference_;
  /// If no timeout is set, then all topics are required always
  OptionalT<Duration> timeout_{std::chrono::seconds(1)};

  bool print_verbose_{false};
  
  /// @brief the pivot is a queue index
  OptionalT<uint32_t> pivot_;

  /// The pivot timestamp identifies the message in the queue that we selected as the pivot. Note
  /// that the pivot queue can still receive new messages, so we need to remember this exact
  /// message.
  OptionalT<Time> pivot_timestamp_;

  std::vector<QueueT> queues_;
  std::vector<OptionalT<MessageIdT>> heads_;
  /// For each topic the timestamps of the last received messages. Not initialized when no messages
  /// were received yet.
  std::vector<OptionalT<Time>> last_timestamps_;
  /// For each topic a truth value whether we this topic timed out
  std::vector<bool> topics_timed_out_;
};
}
}
