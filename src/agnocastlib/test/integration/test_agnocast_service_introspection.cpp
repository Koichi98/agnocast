#include "agnocast/agnocast.hpp"
#include "rclcpp/rclcpp.hpp"

#include <gtest/gtest.h>

// The gate reaches here only through agnocast/agnocast.hpp, so assert it arrived: #if would
// otherwise read an undefined macro as 0 and silently drop every case below.
#ifndef AGNOCAST_SERVICE_INTROSPECTION_GATE_INCLUDED
#error "agnocast/agnocast_service_event_publisher.hpp must be included before the gate is used"
#endif

#if AGNOCAST_HAS_SERVICE_INTROSPECTION

#include "std_srvs/srv/set_bool.hpp"
#include <service_msgs/msg/service_event_info.hpp>

#include <algorithm>
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
protected:
  using Request = std_srvs::srv::SetBool::Request;
  using Response = std_srvs::srv::SetBool::Response;
  using Event = std_srvs::srv::SetBool_Event;

  std::shared_ptr<rclcpp::Node> node_;
  std::shared_ptr<agnocast::MultiThreadedAgnocastExecutor> executor_;
  std::thread spin_thread_;

  std::mutex events_mtx_;
  std::vector<Event> events_;
  agnocast::Subscription<Event>::SharedPtr event_subscriber_;

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

    create_service();

    client_ = agnocast::create_client<std_srvs::srv::SetBool>(
      node_.get(), kServiceName, rclcpp::ServicesQoS(),
      node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive));

    spin_thread_ = std::thread([this]() { executor_->spin(); });
    ASSERT_TRUE(client_->wait_for_service(5s));
  }

  /// Creates the service under test. Overridden to cover the deferred-response path, which
  /// publishes RESPONSE_SENT from send_response() instead of from the subscription callback.
  virtual void create_service()
  {
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
  }

  void TearDown() override
  {
    executor_->cancel();
    if (spin_thread_.joinable()) {
      spin_thread_.join();
    }
    // Release the endpoints while the context is still up, as test_agnocast_client.cpp does by
    // keeping them test-local.
    client_.reset();
    service_.reset();
    event_subscriber_.reset();
    executor_.reset();
    node_.reset();
    if (rclcpp::ok()) {
      rclcpp::shutdown();
    }
  }

  void enable_introspection(rcl_service_introspection_state_t state)
  {
    service_->configure_introspection(node_->get_clock(), rclcpp::ServicesQoS(), state);
  }

  void enable_client_introspection(rcl_service_introspection_state_t state)
  {
    client_->configure_introspection(node_->get_clock(), rclcpp::ServicesQoS(), state);
  }

  /// Issues one call and returns the request id the caller was handed, once the call has
  /// completed. Events may still be in flight. Call through ASSERT_NO_FATAL_FAILURE.
  void call_service(bool data, int64_t * request_id = nullptr)
  {
    auto request = client_->borrow_loaned_request();
    request->data = data;
    auto future = client_->async_send_request(std::move(request));
    if (request_id != nullptr) {
      *request_id = future.request_id;
    }
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
  ASSERT_NO_FATAL_FAILURE(call_service(true));

  EXPECT_TRUE(collect_events(1).empty())
    << "no service event should be published before configure_introspection() enables it";
}

TEST_F(ServiceIntrospectionTest, ContentsPublishesRequestReceivedAndResponseSentWithPayload)
{
  enable_introspection(RCL_SERVICE_INTROSPECTION_CONTENTS);

  int64_t request_id = -1;
  ASSERT_NO_FATAL_FAILURE(call_service(true, &request_id));
  auto events = collect_events(2);

  ASSERT_EQ(events.size(), 2u);
  EXPECT_EQ(events[0].info.event_type, ServiceEventInfo::REQUEST_RECEIVED);
  EXPECT_EQ(events[1].info.event_type, ServiceEventInfo::RESPONSE_SENT);

  EXPECT_EQ(events[0].info.sequence_number, events[1].info.sequence_number)
    << "both events of one call must carry the request's sequence number";
  EXPECT_EQ(events[0].info.sequence_number, request_id)
    << "the sequence number must be the request id the caller was handed";

  // Both events read the gid from the same request, so equality alone would hold even if the
  // gid were never populated.
  EXPECT_EQ(events[0].info.client_gid, events[1].info.client_gid);
  EXPECT_NE(events[0].info.client_gid, decltype(events[0].info.client_gid){})
    << "the client gid must actually be filled in";

  ASSERT_EQ(events[0].request.size(), 1u) << "Contents must carry the request payload";
  EXPECT_TRUE(events[0].request[0].data);

  ASSERT_EQ(events[1].response.size(), 1u) << "Contents must carry the response payload";
  EXPECT_TRUE(events[1].response[0].success);
  EXPECT_EQ(events[1].response[0].message, "ok");
}

TEST_F(ServiceIntrospectionTest, MetadataPublishesEventsWithoutPayload)
{
  enable_introspection(RCL_SERVICE_INTROSPECTION_METADATA);

  ASSERT_NO_FATAL_FAILURE(call_service(true));
  auto events = collect_events(2);

  ASSERT_EQ(events.size(), 2u);
  EXPECT_TRUE(events[0].request.empty()) << "Metadata must not carry the request payload";
  EXPECT_TRUE(events[1].response.empty()) << "Metadata must not carry the response payload";
}

TEST_F(ServiceIntrospectionTest, SwitchingBackToOffStopsEvents)
{
  enable_introspection(RCL_SERVICE_INTROSPECTION_CONTENTS);
  ASSERT_NO_FATAL_FAILURE(call_service(true));
  ASSERT_EQ(collect_events(2).size(), 2u);

  enable_introspection(RCL_SERVICE_INTROSPECTION_OFF);
  clear_events();

  ASSERT_NO_FATAL_FAILURE(call_service(false));

  EXPECT_TRUE(collect_events(1).empty())
    << "no service event should be published after introspection is switched off";
}

TEST_F(ServiceIntrospectionTest, ReEnablingAfterOffPublishesEventsAgain)
{
  enable_introspection(RCL_SERVICE_INTROSPECTION_CONTENTS);
  enable_introspection(RCL_SERVICE_INTROSPECTION_OFF);
  clear_events();

  enable_introspection(RCL_SERVICE_INTROSPECTION_CONTENTS);
  ASSERT_NO_FATAL_FAILURE(call_service(true));

  EXPECT_EQ(collect_events(2).size(), 2u)
    << "the event publisher must be recreated when introspection is enabled a second time";
}

TEST_F(ServiceIntrospectionTest, ClientPublishesRequestSentAndResponseReceived)
{
  enable_client_introspection(RCL_SERVICE_INTROSPECTION_CONTENTS);

  ASSERT_NO_FATAL_FAILURE(call_service(true));
  auto events = collect_events(2);

  ASSERT_EQ(events.size(), 2u);
  EXPECT_EQ(events[0].info.event_type, ServiceEventInfo::REQUEST_SENT);
  EXPECT_EQ(events[1].info.event_type, ServiceEventInfo::RESPONSE_RECEIVED);

  ASSERT_EQ(events[0].request.size(), 1u);
  EXPECT_TRUE(events[0].request[0].data);
  ASSERT_EQ(events[1].response.size(), 1u);
  EXPECT_EQ(events[1].response[0].message, "ok");
}

TEST_F(ServiceIntrospectionTest, BothSidesTogetherCoverTheWholeExchange)
{
  enable_introspection(RCL_SERVICE_INTROSPECTION_METADATA);
  enable_client_introspection(RCL_SERVICE_INTROSPECTION_METADATA);

  ASSERT_NO_FATAL_FAILURE(call_service(true));
  auto events = collect_events(4);

  ASSERT_EQ(events.size(), 4u);

  // One transaction, so every event must carry the same correlation key.
  for (const auto & e : events) {
    EXPECT_EQ(e.info.sequence_number, events[0].info.sequence_number);
    EXPECT_EQ(e.info.client_gid, events[0].info.client_gid);
  }

  std::vector<uint8_t> types;
  types.reserve(events.size());
  for (const auto & e : events) {
    types.push_back(e.info.event_type);
  }
  std::sort(types.begin(), types.end());
  EXPECT_EQ(
    types, std::vector<uint8_t>(
             {ServiceEventInfo::REQUEST_SENT, ServiceEventInfo::REQUEST_RECEIVED,
              ServiceEventInfo::RESPONSE_SENT, ServiceEventInfo::RESPONSE_RECEIVED}));
}

TEST_F(ServiceIntrospectionTest, SwitchingBetweenMetadataAndContentsChangesOnlyThePayload)
{
  enable_introspection(RCL_SERVICE_INTROSPECTION_CONTENTS);
  ASSERT_NO_FATAL_FAILURE(call_service(true));
  ASSERT_EQ(collect_events(2).size(), 2u);
  clear_events();

  // Events keep flowing across the transition; only the payload appears and disappears. That
  // the publisher is not recreated is not observable from here.
  enable_introspection(RCL_SERVICE_INTROSPECTION_METADATA);
  ASSERT_NO_FATAL_FAILURE(call_service(true));
  auto metadata_events = collect_events(2);
  ASSERT_EQ(metadata_events.size(), 2u);
  EXPECT_TRUE(metadata_events[0].request.empty());
  EXPECT_TRUE(metadata_events[1].response.empty());
  clear_events();

  enable_introspection(RCL_SERVICE_INTROSPECTION_CONTENTS);
  ASSERT_NO_FATAL_FAILURE(call_service(true));
  auto contents_events = collect_events(2);
  ASSERT_EQ(contents_events.size(), 2u);
  EXPECT_EQ(contents_events[0].request.size(), 1u);
  EXPECT_EQ(contents_events[1].response.size(), 1u);
}

/// Exercises the deferred-response path, where RESPONSE_SENT comes from send_response().
class DeferredServiceIntrospectionTest : public ServiceIntrospectionTest
{
protected:
  void create_service() override
  {
    service_ = agnocast::create_service<std_srvs::srv::SetBool>(
      node_.get(), kServiceName,
      [](
        agnocast::Service<std_srvs::srv::SetBool>::SharedPtr service,
        agnocast::ipc_shared_ptr<Request> && request) {
        auto response = service->borrow_loaned_response(request);
        response->success = request->data;
        response->message = "ok";
        service->send_response(std::move(request), std::move(response));
      },
      rclcpp::ServicesQoS(),
      node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive));
  }
};

TEST_F(DeferredServiceIntrospectionTest, DeferredResponsePublishesBothEvents)
{
  enable_introspection(RCL_SERVICE_INTROSPECTION_CONTENTS);

  int64_t request_id = -1;
  ASSERT_NO_FATAL_FAILURE(call_service(true, &request_id));
  auto events = collect_events(2);

  ASSERT_EQ(events.size(), 2u);
  EXPECT_EQ(events[0].info.event_type, ServiceEventInfo::REQUEST_RECEIVED);
  EXPECT_EQ(events[1].info.event_type, ServiceEventInfo::RESPONSE_SENT);
  EXPECT_EQ(events[1].info.sequence_number, request_id);

  ASSERT_EQ(events[1].response.size(), 1u);
  EXPECT_TRUE(events[1].response[0].success);
  EXPECT_EQ(events[1].response[0].message, "ok");
}

#else

// Without this the binary reports "0 tests from 0 test suites" and passes, which is
// indistinguishable in CI from the suite having been dropped by mistake.
TEST(ServiceIntrospectionTest, SkippedBecauseTheDistroHasNoServiceIntrospection)
{
  GTEST_SKIP() << "service introspection requires rclcpp 21 or newer";
}

#endif  // AGNOCAST_HAS_SERVICE_INTROSPECTION
