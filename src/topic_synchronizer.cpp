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

#include <ros_dynamic_topic_synchronizer/policies.hpp>
#include <iostream>

namespace fsd {
namespace mf {

ApproxTimePolicy::ApproxTimePolicy(
    const Duration &max_msg_age, const OptionalT<Duration> &timeout,
    bool print_verbose,
    std::function<void(const std::vector<MessageIdT> &message_ids)> emit_messages,
    std::function<void(const MessageIdT &message_id)> removed_buffered_ms)
    : SyncPolicy(emit_messages, removed_buffered_ms) {
  max_msg_age_ = max_msg_age;
  timeout_ = timeout;
  print_verbose_ = print_verbose;
}

void ApproxTimePolicy::add(MessageIdT msg_id) {
  last_timestamps_.at(msg_id.queue_index) = msg_id.stamp;
  auto cutoff = msg_id.stamp - max_msg_age_;
  remove_too_old_msgs(cutoff);
  determine_timeouts(msg_id.stamp);
  auto &head = heads_.at(msg_id.queue_index);
  auto &queue = queues_.at(msg_id.queue_index);
  if (head) {
    auto newest_timestamp = queue.empty() ? head.get().stamp : queue.back().stamp;
    if (msg_id.stamp < newest_timestamp) {
      /*
      std::cout <<
          "WARNING: Trying to add a timestamp which is older than the newest, messages are "
          "arriving "
          "in non-chronological order. Resetting the syncrhonizer... (message in queue has timestamp: "
          << newest_timestamp << ", timestamp of the new message: " << msg_id.stamp << ")";
      */
      reset();
    }
    if (newest_timestamp == msg_id.stamp) {
      /*std::cout <<
          "WARNING: Ignored trying to add a duplicate timestamp, timestamp of the "
          "message: "
          << msg_id.stamp;
          */
      removed_buffered_msg_(msg_id);
      return;
    }
    queue.push_back(msg_id);
  } else {
    head = msg_id;
  }
  while (try_to_emit()) /// Now emit while there is something to emit
    ;
}

void ApproxTimePolicy::set_number_of_topics(uint32_t number_of_topics) {
  number_of_topics_ = number_of_topics;
  queues_.resize(number_of_topics_);
  heads_.resize(number_of_topics_);
  last_timestamps_.resize(number_of_topics_);
  topics_timed_out_.resize(number_of_topics_);
}

void ApproxTimePolicy::remove_too_old_msgs(Time cutoff) {
  for (size_t i_q = 0; i_q < queues_.size(); i_q++) {
    auto &head = heads_.at(i_q);
    auto &queue = queues_.at(i_q);
    if (head) {
      if (head.get().stamp < cutoff) {
        removed_buffered_msg_(head.get());
        head = {};
        pivot_ = {};
      }
    }
    while (!queue.empty()) {
      if (queue.front().stamp <= cutoff) {
        removed_buffered_msg_(queue.front());
        queue.pop_front();
      } else {
        break;
      }
    }
    if (!head && !queue.empty()) {
      head = queue.front();
      queue.pop_front();
    }
  }
}

bool ApproxTimePolicy::try_to_emit() {
  /// All heads which are not timeouted should be initialized
  bool all_heads_initialized = true;
  for (size_t i = 0; i < heads_.size(); i++) {
    all_heads_initialized &= heads_.at(i) != boost::none || topics_timed_out_.at(i);
  }

  if (!all_heads_initialized) {
    return false;
  }
  select_pivot();
  /// Happens only when no head is initialized in case of timeout
  if (!pivot_) {
    return false;
  }

  bool can_improve = can_still_find_better_match_in_any_queue();
  if (can_improve) {
    return false;
  }

  std::vector<MessageIdT> messages_to_be_emitted;
  for (auto &head : heads_) {
    /// head is only missing in case of a timeout
    if (head) {
      messages_to_be_emitted.push_back(head.get());
    }
  }

  emit_messages_(messages_to_be_emitted);
  pivot_ = {};
  remove_last_emmitted_msgs();

  return true;
}

void ApproxTimePolicy::remove_last_emmitted_msgs() {
  for (size_t i = 0; i < queues_.size(); i++) {
    auto &head = heads_.at(i);
    auto &q = queues_.at(i);
    if (head) {
      removed_buffered_msg_(head.get());
    }
    if (!q.empty()) {
      head = q.front();
      q.pop_front();
    } else {
      head = {};
    }
  }
}

bool ApproxTimePolicy::can_still_find_better_match_in_queue(uint32_t queue_index) {
  if (queue_index == pivot_.get()) {
    return false;
  }
  if (topics_timed_out_.at(queue_index)) {
    return false;
  }
  auto &queue = queues_.at(queue_index);
  auto &head = heads_.at(queue_index);
  auto stamp = head.get().stamp;
  while (!queue.empty()) {
    auto &next_stamp = queue.front().stamp;

    if (abs_difference(pivot_timestamp_.get(), next_stamp) <
        abs_difference(pivot_timestamp_.get(), stamp)) {
      removed_buffered_msg_(head.get());
      head = queue.front();
      stamp = next_stamp;
      queue.pop_front();
    } else {
      return false;
    }
  }
  return stamp < pivot_timestamp_.get();
}

bool ApproxTimePolicy::can_still_find_better_match_in_any_queue() {
  bool can_improve = false;
  for (size_t i_q = 0; i_q < queues_.size(); i_q++) {
    auto res = can_still_find_better_match_in_queue(i_q);
    if (res) {
      can_improve = true;
    }
  }
  return can_improve;
}

void ApproxTimePolicy::reset() {
  for (size_t i_q = 0; i_q < queues_.size(); i_q++) {
    if (heads_.at(i_q)) {
      removed_buffered_msg_(heads_.at(i_q).get());
      heads_.at(i_q) = {};
    }
    for (auto &msg : queues_.at(i_q)) {
      removed_buffered_msg_(msg);
    }
    queues_.at(i_q).clear();
  }
  pivot_ = {};
}

void ApproxTimePolicy::select_pivot() {
  if (pivot_ != boost::none) {
    return;
  }
  for (size_t i = 0; i < heads_.size(); i++) {
    if (!heads_.at(i)) {
      continue;
    }
    auto stamp = heads_.at(i).get().stamp;
    if (!pivot_ || stamp > pivot_timestamp_.get()) {
      pivot_timestamp_ = stamp;
      pivot_ = i;
    }
  }
}

void ApproxTimePolicy::determine_timeouts(Time current_timestamp) {
  if (!timeout_) {
    return;
  }
  for (size_t i = 0; i < number_of_topics_; i++) {
    bool last_timeout = topics_timed_out_.at(i);
    if (last_timestamps_.at(i)) {
      auto time_diff = current_timestamp - last_timestamps_.at(i).get();
      bool timed_out = time_diff > timeout_;
      topics_timed_out_.at(i) = timed_out;
    } else {
      topics_timed_out_.at(i) = true;
    }
    if (topics_timed_out_.at(i) && !last_timeout) {
      /// TODO maybe add callback
      //ROS_WARN_STREAM("Topic " << topic_names_for_queues_.at(i) << " has timed out");
    }
  }
}

}  // namespace mf
}  // namespace fsd
