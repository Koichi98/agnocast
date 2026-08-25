#include "agnocast/agnocast_service_event_publisher.hpp"

#include <gtest/gtest.h>

#if AGNOCAST_HAS_SERVICE_INTROSPECTION

#include "rclcpp/rclcpp.hpp"

#include <service_msgs/msg/service_event_info.hpp>

#include <stdexcept>

// configure() rejects its arguments before it creates the event publisher, so these need no
// kernel module, unlike the rest of the introspection coverage.
class ServiceEventPublisherTest : public ::testing::Test
{
protected:
  std::shared_ptr<rclcpp::Node> node_;
  std::unique_ptr<agnocast::ServiceEventPublisher> event_publisher_;

  void SetUp() override
  {
    rclcpp::init(0, nullptr);
    node_ = std::make_shared<rclcpp::Node>("test_service_event_publisher_node");
    event_publisher_ = std::make_unique<agnocast::ServiceEventPublisher>(
      node_.get(), "/test_service", "std_srvs/srv/SetBool", agnocast::PublisherRole::Default);
  }

  void TearDown() override
  {
    event_publisher_.reset();
    node_.reset();
    if (rclcpp::ok()) {
      rclcpp::shutdown();
    }
  }
};

TEST_F(ServiceEventPublisherTest, ConfigureRejectsANullClock)
{
  EXPECT_THROW(
    event_publisher_->configure(nullptr, rclcpp::ServicesQoS(), RCL_SERVICE_INTROSPECTION_CONTENTS),
    std::invalid_argument);
}

TEST_F(ServiceEventPublisherTest, ConfigureRejectsANullClockWhenDisabling)
{
  // rcl validates the clock before it looks at the requested state; this matches that.
  EXPECT_THROW(
    event_publisher_->configure(nullptr, rclcpp::ServicesQoS(), RCL_SERVICE_INTROSPECTION_OFF),
    std::invalid_argument);
}

TEST_F(ServiceEventPublisherTest, ConfigureRejectsAQosAgnocastCannotUse)
{
  // Reaching GenericPublisher with this QoS would call exit(), taking the node down on what is
  // only a diagnostic toggle.
  EXPECT_THROW(
    event_publisher_->configure(
      node_->get_clock(), rclcpp::QoS(rclcpp::KeepAll()), RCL_SERVICE_INTROSPECTION_CONTENTS),
    std::invalid_argument);
}

TEST_F(ServiceEventPublisherTest, ARejectedConfigureLeavesIntrospectionOff)
{
  EXPECT_THROW(
    event_publisher_->configure(nullptr, rclcpp::ServicesQoS(), RCL_SERVICE_INTROSPECTION_CONTENTS),
    std::invalid_argument);

  // Asserted rather than probed by publishing: a state committed before the throw would make
  // publish_service_event_message() dereference a null clock, which is a fault gtest cannot
  // contain, and that would discard the results of every other case in this binary.
  EXPECT_EQ(event_publisher_->introspection_state(), RCL_SERVICE_INTROSPECTION_OFF);
}

TEST_F(ServiceEventPublisherTest, DisablingDoesNotRejectAQosItWillNotUse)
{
  // The QoS is only used when a publisher is created, so turning introspection off must work
  // whatever QoS the caller happens to be passing along.
  EXPECT_NO_THROW(event_publisher_->configure(
    node_->get_clock(), rclcpp::QoS(rclcpp::KeepAll()), RCL_SERVICE_INTROSPECTION_OFF));
  EXPECT_EQ(event_publisher_->introspection_state(), RCL_SERVICE_INTROSPECTION_OFF);
}

#endif  // AGNOCAST_HAS_SERVICE_INTROSPECTION
