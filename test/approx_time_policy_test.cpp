#include <gtest/gtest.h>

#include <ros_dynamic_topic_synchronizer/topic_synchronizer.hpp>
#include <iostream>
#include <unordered_set>

using fsd::mf::ApproxTimePolicy;
using fsd::mf::MessageIdT;
using fsd::mf::detail::MessageIdHash;

const auto default_base_time = 1676145467;

std::vector<MessageIdT> create_message_sequence_single_queue(size_t message_count,
                                                             size_t queue_index,
                                                             double time_delta = .1,
                                                             size_t base_time = default_base_time) {
  std::vector<MessageIdT> messages;
  for (size_t i = 0; i < message_count; i++) {
    messages.emplace_back(ros::Time(base_time + time_delta * i), queue_index);
  }
  return messages;
}

std::vector<MessageIdT> create_message_sequence(size_t message_count, size_t queue_count,
                                                double time_delta = .1,
                                                size_t base_time = default_base_time) {
  std::vector<MessageIdT> messages;
  for (size_t queue_i = 0; queue_i < queue_count; queue_i++) {
    auto new_seq =
        create_message_sequence_single_queue(message_count, queue_i, time_delta, base_time);
    messages.insert(messages.end(), new_seq.begin(), new_seq.end());
  }
  return messages;
}

class FixtureTwoTopics : public ::testing::Test {
public:
  void init(const ros::Duration &max_msg_age, const boost::optional<ros::Duration> &timeout) {
    syncer_ = std::make_unique<ApproxTimePolicy>(
        max_msg_age, timeout,
        [this](const std::vector<MessageIdT> &message_ids) { on_message_emit(message_ids); },
        [this](const MessageIdT &message_id) { on_message_remove(message_id); });
    std::vector<std::string> topic_names{"topic1", "topic2"};
    syncer_->set_topic_names_for_queues(topic_names);
  }

  void SetUp() override { init(ros::Duration(5.), boost::none); }

  void on_message_emit(const std::vector<MessageIdT> &message_ids) {
    received_messages_ += message_ids.size();
    message_inserted_calls_.push_back(message_ids);
  }
  void on_message_remove(const MessageIdT &message_id) {
    message_removed_calls_.push_back(message_id);
  }

  size_t received_messages_{0};
  std::vector<std::vector<MessageIdT>> message_inserted_calls_;
  std::vector<MessageIdT> message_removed_calls_;
  std::unique_ptr<ApproxTimePolicy> syncer_;
};

// fixture where for two topics 10 messages are added each and and all messages should get emitted
class FixtureTwoTopicsAddedMessages : public FixtureTwoTopics {
public:
  size_t number_of_messages = 10;
  size_t number_of_topics = 2;

  void SetUp() override {
    FixtureTwoTopics::SetUp();

    auto msg_sequence = create_message_sequence(number_of_messages, number_of_topics, .1);
    for (const auto &message_id : msg_sequence) {
      syncer_->add(message_id);
    }
  }
};

class FixtureTwoTopicsWithTimeout : public FixtureTwoTopics {
public:
  ros::Duration timeout_period_{1.};
  void SetUp() override { init(ros::Duration(5.), timeout_period_); }
};

TEST_F(FixtureTwoTopics, Negative) {
  EXPECT_TRUE(message_inserted_calls_.empty()) << "Messages were inserted without adding any.";
  EXPECT_TRUE(message_removed_calls_.empty()) << "Messages were removed without adding any.";
}

TEST_F(FixtureTwoTopics, SingleTopicNoneEmittedOrRemoved) {
  auto number_of_messages = 10;
  auto msg_sequence = create_message_sequence(number_of_messages, 1, .1);
  for (const auto &message_id : msg_sequence) {
    syncer_->add(message_id);
  }

  /// none should be added as we added only messages of one topic
  EXPECT_EQ(message_inserted_calls_.size(), 0);
  // no messages should get removed as the max age is .1s * 10 = 1s and the max age is set to 5s
  EXPECT_EQ(message_removed_calls_.size(), 0) << "Messages were removed";
}

TEST_F(FixtureTwoTopics, MultipleTopicsNoneEmittedOrRemoved) {
  // first, put in a sequence of ten messages
  // | | | | | ...
  //             | | | | | ...
  auto number_of_messages = 10;
  auto msg_sequence = create_message_sequence_single_queue(number_of_messages, 0, .1);

  for (const auto &message_id : msg_sequence) {
    syncer_->add(message_id);
  }
  EXPECT_EQ(message_inserted_calls_.size(), 0) << "Messages were inserted after first sequence";
  EXPECT_EQ(message_removed_calls_.size(), 0) << "Messages were removed after first sequence";

  // add another sequence for the second topic, but the first message starts 1s after the last
  // message of the first topic.
  auto msg_sequence2 =
      create_message_sequence_single_queue(number_of_messages, 1, .1, default_base_time + 2);
  for (const auto &message_id : msg_sequence2) {
    syncer_->add(message_id);
  }
  EXPECT_EQ(message_inserted_calls_.size(), 0) << "Messages were inserted after second sequence";
  /// now
  EXPECT_EQ(message_removed_calls_.size(), number_of_messages - 1)
      << "Messages were removed after second sequence";
}

TEST_F(FixtureTwoTopicsAddedMessages, TwoTopicsSimple) {
  EXPECT_EQ(message_inserted_calls_.size(), number_of_messages);
  // as all messages should be emitted, every message should get removed
  EXPECT_EQ(message_removed_calls_.size(), number_of_topics * number_of_messages);
}

TEST_F(FixtureTwoTopicsAddedMessages, EmittedMessagesAreUnique) {
  // first, flatten vector
  std::unordered_set<MessageIdT, MessageIdHash> message_id_set;
  size_t msg_inserted_cnt{0};
  for (const auto &msg_tuple : message_inserted_calls_) {
    for (const auto &msg : msg_tuple) {
      message_id_set.insert(msg);
      msg_inserted_cnt++;
    }
  }

  EXPECT_EQ(msg_inserted_cnt, message_id_set.size()) << "Found duplicate messages";
}

TEST_F(FixtureTwoTopics, TwoTopicsSmallPhaseShift) {
  auto number_of_messages = 50;
  auto msg_sequence =
      create_message_sequence_single_queue(number_of_messages, 0, .1, default_base_time + .01);
  auto msg_sequence2 =
      create_message_sequence_single_queue(number_of_messages, 1, .1, default_base_time);

  for (const auto &message_id : msg_sequence) {
    syncer_->add(message_id);
  }
  for (const auto &message_id : msg_sequence2) {
    syncer_->add(message_id);
  }

  // for the last message which is the most recent
  // TODO why not 49 ??
  ASSERT_EQ(message_inserted_calls_.size(), number_of_messages);
  /// TODO check that all emitted messages were also removed
}

TEST_F(FixtureTwoTopics, DifferentFrequencies) {
  /// Test whether with topics which have slightly different frequencies and a small phase shift, we
  /// get all the expected messages
  auto number_of_messages = 50;
  const auto first_topic_period_time = .1;
  const auto second_topic_period_time = .08;
  const auto phase_shift = .01;
  auto msg_sequence = create_message_sequence_single_queue(
      number_of_messages, 0, first_topic_period_time, default_base_time + phase_shift);
  auto msg_sequence2 = create_message_sequence_single_queue(
      number_of_messages, 1, second_topic_period_time, default_base_time);

  for (const auto &message_id : msg_sequence) {
    syncer_->add(message_id);
  }
  for (const auto &message_id : msg_sequence2) {
    syncer_->add(message_id);
  }

  size_t expected_num_msgs =
      std::floor((number_of_messages * second_topic_period_time) / first_topic_period_time);
  // for the last message which is the most recent
  EXPECT_EQ(message_inserted_calls_.size(), expected_num_msgs);
  /// TODO check that all emitted messages where also removed
}

/// Test that the set of emitted messages is always complete, i.e it contains a message from all
/// topics and never an incomplete set of messages is emitted in case no timeout is set and thus the
/// timeout functionality is disabled.
TEST_F(FixtureTwoTopicsWithTimeout, TimeoutNegativeTest) {}

/// Test if a subset of the messages is received after the timeout period, if one is set
TEST_F(FixtureTwoTopics, TimeoutTest) {}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
