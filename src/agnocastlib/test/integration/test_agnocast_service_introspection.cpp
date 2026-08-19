#include "agnocast/agnocast.hpp"
#include "rclcpp/rclcpp.hpp"

#include <gtest/gtest.h>

#if AGNOCAST_HAS_SERVICE_INTROSPECTION

#include "std_srvs/srv/set_bool.hpp"
#include <service_msgs/msg/service_event_info.hpp>

#include <chrono>
#include <memory>
#include <mutex>
#include <thread>
#include <vector>

using namespace std::chrono_literals;
using service_msgs::msg::ServiceEventInfo;

namespace
{
constexpr const char * kServiceName = "test_introspected_service";
constexpr const char * kEventTopicName = "/test_introspected_service/_service_event";
}  // namespace

class ServiceIntrospectionTest : public ::testing::Test
{
  using Request = std_srvs::srv::SetBool::Request;
  using Response = std_srvs::srv::SetBool::Response;
  using Event = std_srvs::srv::SetBool_Event;

  std::shared_ptr<rclcpp::Node> node_;
  std::shared_ptr<agnocast::MultiThreadedAgnocastExecutor> executor_;
  std::thread spin_thread_;

  std::mutex events_mtx_;
  std::vector<Event> events_;
  agnocast::Subscription<Event>::SharedPtr event_subscriber_;

protected:
  agnocast::Service<std_srvs::srv::SetBool>::SharedPtr service_;
  agnocast::Client<std_srvs::srv::SetBool>::SharedPtr client_;

  void SetUp() override
  {
    rclcpp::init(0, nullptr);
    node_ = std::make_shared<rclcpp::Node>("test_introspection_node");
    executor_ = std::make_shared<agnocast::MultiThreadedAgnocastExecutor>();
    executor_->add_node(node_);

    // The event subscription is created first so that no event can be published before it is
    // able to observe it.
    event_subscriber_ = agnocast::create_subscription<Event>(
      node_.get(), kEventTopicName, rclcpp::ServicesQoS(),
      [this](const agnocast::ipc_shared_ptr<const Event> & event) {
        std::lock_guard<std::mutex> lock(events_mtx_);
        events_.push_back(*event);
      },
      agnocast::SubscriptionOptions{
        node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive)});

    service_ = agnocast::create_service<std_srvs::srv::SetBool>(
      node_.get(), kServiceName,
      [](
        agnocast::ipc_shared_ptr<Request> && request,
        agnocast::ipc_shared_ptr<Response> && response) {
        response->success = request->data;
        response->message = "ok";
      },
      rclcpp::ServicesQoS(),
      node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive));

    client_ = agnocast::create_client<std_srvs::srv::SetBool>(
      node_.get(), kServiceName, rclcpp::ServicesQoS(),
      node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive));

    spin_thread_ = std::thread([this]() { executor_->spin(); });
    ASSERT_TRUE(client_->wait_for_service(5s));
  }

  void TearDown() override
  {
    executor_->cancel();
    if (spin_thread_.joinable()) {
      spin_thread_.join();
    }
    if (rclcpp::ok()) {
      rclcpp::shutdown();
    }
  }

  void enable_introspection(rcl_service_introspection_state_t state)
  {
    service_->configure_introspection(node_->get_clock(), rclcpp::ServicesQoS(), state);
  }

  /// Issues one call and returns once it has completed. Events may still be in flight.
  void call_service(bool data)
  {
    auto request = client_->borrow_loaned_request();
    request->data = data;
    auto future = client_->async_send_request(std::move(request));
    ASSERT_EQ(future.wait_for(5s), std::future_status::ready);
  }

  /// Waits until at least `expected` events have arrived, then returns everything seen. Returns
  /// early only on timeout, so a test expecting zero events still waits the full duration.
  std::vector<Event> collect_events(size_t expected, std::chrono::milliseconds timeout = 2s)
  {
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (std::chrono::steady_clock::now() < deadline) {
      {
        std::lock_guard<std::mutex> lock(events_mtx_);
        if (events_.size() >= expected) {
          return events_;
        }
      }
      std::this_thread::sleep_for(20ms);
    }
    std::lock_guard<std::mutex> lock(events_mtx_);
    return events_;
  }

  void clear_events()
  {
    std::lock_guard<std::mutex> lock(events_mtx_);
    events_.clear();
  }
};

TEST_F(ServiceIntrospectionTest, PublishesNoEventsWhileIntrospectionIsOff)
{
  call_service(true);

  EXPECT_TRUE(collect_events(1).empty())
    << "no service event should be published before configure_introspection() enables it";
}

TEST_F(ServiceIntrospectionTest, ContentsPublishesRequestReceivedAndResponseSentWithPayload)
{
  enable_introspection(RCL_SERVICE_INTROSPECTION_CONTENTS);

  call_service(true);
  auto events = collect_events(2);

  ASSERT_EQ(events.size(), 2u);
  EXPECT_EQ(events[0].info.event_type, ServiceEventInfo::REQUEST_RECEIVED);
  EXPECT_EQ(events[1].info.event_type, ServiceEventInfo::RESPONSE_SENT);

  EXPECT_EQ(events[0].info.sequence_number, events[1].info.sequence_number)
    << "both events of one call must carry the request's sequence number";
  EXPECT_EQ(events[0].info.client_gid, events[1].info.client_gid);

  ASSERT_EQ(events[0].request.size(), 1u) << "Contents must carry the request payload";
  EXPECT_TRUE(events[0].request[0].data);

  ASSERT_EQ(events[1].response.size(), 1u) << "Contents must carry the response payload";
  EXPECT_TRUE(events[1].response[0].success);
  EXPECT_EQ(events[1].response[0].message, "ok");
}

TEST_F(ServiceIntrospectionTest, MetadataPublishesEventsWithoutPayload)
{
  enable_introspection(RCL_SERVICE_INTROSPECTION_METADATA);

  call_service(true);
  auto events = collect_events(2);

  ASSERT_EQ(events.size(), 2u);
  EXPECT_TRUE(events[0].request.empty()) << "Metadata must not carry the request payload";
  EXPECT_TRUE(events[1].response.empty()) << "Metadata must not carry the response payload";
}

TEST_F(ServiceIntrospectionTest, SwitchingBackToOffStopsEvents)
{
  enable_introspection(RCL_SERVICE_INTROSPECTION_CONTENTS);
  call_service(true);
  ASSERT_EQ(collect_events(2).size(), 2u);

  enable_introspection(RCL_SERVICE_INTROSPECTION_OFF);
  clear_events();

  call_service(false);

  EXPECT_TRUE(collect_events(1).empty())
    << "no service event should be published after introspection is switched off";
}

TEST_F(ServiceIntrospectionTest, ReEnablingAfterOffPublishesEventsAgain)
{
  enable_introspection(RCL_SERVICE_INTROSPECTION_CONTENTS);
  enable_introspection(RCL_SERVICE_INTROSPECTION_OFF);
  clear_events();

  enable_introspection(RCL_SERVICE_INTROSPECTION_CONTENTS);
  call_service(true);

  EXPECT_EQ(collect_events(2).size(), 2u)
    << "the event publisher must be recreated when introspection is enabled a second time";
}

#endif  // AGNOCAST_HAS_SERVICE_INTROSPECTION
