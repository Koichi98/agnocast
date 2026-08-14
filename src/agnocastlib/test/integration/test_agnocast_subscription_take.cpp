#include "agnocast/agnocast.hpp"
#include "agnocast/node/agnocast_node.hpp"

#include "std_msgs/msg/string.hpp"

#include <gtest/gtest.h>

#include <memory>
#include <stdexcept>
#include <string>

using StringMsg = std_msgs::msg::String;

class SubscriptionTakeIntegrationTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    agnocast::init(0, nullptr);
    rclcpp::NodeOptions options;
    options.start_parameter_services(false);
    node_ = std::make_shared<agnocast::Node>("test_subscription_take", options);
  }

  void TearDown() override
  {
    node_.reset();
    if (agnocast::ok()) {
      agnocast::shutdown();
    }
  }

  void publish(const agnocast::Publisher<StringMsg>::SharedPtr & pub, const std::string & data)
  {
    auto message = pub->borrow_loaned_message();
    message->data = data;
    pub->publish(std::move(message));
  }

  std::shared_ptr<agnocast::Node> node_;
};

TEST_F(SubscriptionTakeIntegrationTest, take_returns_empty_before_anything_is_published)
{
  agnocast::Subscription<StringMsg> sub(node_.get(), "/test_take_empty", rclcpp::QoS{1});

  EXPECT_FALSE(sub.take());
}

TEST_F(SubscriptionTakeIntegrationTest, take_returns_the_published_message)
{
  const std::string topic = "/test_take_roundtrip";
  auto pub = node_->create_publisher<StringMsg>(topic, 1);
  agnocast::Subscription<StringMsg> sub(node_.get(), topic, rclcpp::QoS{1});

  publish(pub, "hello");

  const auto taken = sub.take();
  ASSERT_TRUE(taken);
  EXPECT_EQ(taken->data, "hello");
}

TEST_F(SubscriptionTakeIntegrationTest, take_without_allow_same_message_does_not_repeat)
{
  const std::string topic = "/test_take_no_repeat";
  auto pub = node_->create_publisher<StringMsg>(topic, 1);
  agnocast::Subscription<StringMsg> sub(node_.get(), topic, rclcpp::QoS{1});

  publish(pub, "once");

  ASSERT_TRUE(sub.take());
  EXPECT_FALSE(sub.take());
}

TEST_F(SubscriptionTakeIntegrationTest, take_with_allow_same_message_returns_the_same_entry_again)
{
  const std::string topic = "/test_take_allow_same";
  auto pub = node_->create_publisher<StringMsg>(topic, 1);
  agnocast::Subscription<StringMsg> sub(node_.get(), topic, rclcpp::QoS{1});

  publish(pub, "same");

  const auto first = sub.take(true);
  const auto second = sub.take(true);
  ASSERT_TRUE(first);
  ASSERT_TRUE(second);
  EXPECT_EQ(first.get_entry_id(), second.get_entry_id());
  EXPECT_EQ(second->data, "same");
}

TEST_F(SubscriptionTakeIntegrationTest, take_throws_on_a_subscription_constructed_with_a_callback)
{
  auto sub = node_->create_subscription<StringMsg>(
    "/test_take_with_callback", rclcpp::QoS{1},
    [](const agnocast::ipc_shared_ptr<const StringMsg> &) {});

  EXPECT_THROW(sub->take(), std::runtime_error);
  EXPECT_THROW(sub->take(true), std::runtime_error);
}

TEST_F(SubscriptionTakeIntegrationTest, take_subscription_still_polls_through_its_new_base)
{
  const std::string topic = "/test_take_subscription_compat";
  auto pub = node_->create_publisher<StringMsg>(topic, 1);
  agnocast::TakeSubscription<StringMsg> sub(node_.get(), topic, rclcpp::QoS{1});

  publish(pub, "compat");

  const auto taken = sub.take();
  ASSERT_TRUE(taken);
  EXPECT_EQ(taken->data, "compat");
}

TEST_F(SubscriptionTakeIntegrationTest, polling_subscriber_keeps_returning_the_latest_message)
{
  const std::string topic = "/test_polling_subscriber";
  auto pub = node_->create_publisher<StringMsg>(topic, 1);
  agnocast::PollingSubscriber<StringMsg> sub(node_.get(), topic);

  publish(pub, "latest");

  const auto first = sub.take_data();
  ASSERT_TRUE(first);
  EXPECT_EQ(first->data, "latest");

  const auto second = sub.take_data();
  ASSERT_TRUE(second);
  EXPECT_EQ(second->data, "latest");
}
