#include "agnocast/agnocast.hpp"
#include "agnocast/internal/service_introspection.hpp"
#include "rclcpp/rclcpp.hpp"

#include "std_srvs/srv/empty.hpp"
#include "std_srvs/srv/set_bool.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <chrono>
#include <future>
#include <memory>
#include <mutex>
#include <thread>
#include <utility>
#include <vector>

using namespace std::chrono_literals;

class ClientTest : public ::testing::Test
{
  using Request = std_srvs::srv::Empty::Request;
  using Response = std_srvs::srv::Empty::Response;

  std::shared_ptr<rclcpp::Node> node_;
  std::shared_ptr<agnocast::SingleThreadedAgnocastExecutor> executor_;
  std::thread spin_thread_;

protected:
  void SetUp() override
  {
    rclcpp::init(0, nullptr);
    node_ = std::make_shared<rclcpp::Node>("test_node");
    executor_ = std::make_shared<agnocast::SingleThreadedAgnocastExecutor>();
    executor_->add_node(node_);
    spin_thread_ = std::thread([this]() { executor_->spin(); });
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

  auto create_service()
  {
    auto cb_group = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    return agnocast::create_service<std_srvs::srv::Empty>(
      node_.get(), "test_service",
      [](agnocast::ipc_shared_ptr<Request> &&, agnocast::ipc_shared_ptr<Response> &&) { return; },
      rclcpp::ServicesQoS(), cb_group);
  }

  auto create_client()
  {
    auto cb_group = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    return agnocast::create_client<std_srvs::srv::Empty>(
      node_.get(), "test_service", rclcpp::ServicesQoS(), cb_group);
  }
};

TEST_F(ClientTest, ServiceIsReadyReturnsCorrectValue)
{
  auto client = create_client();

  EXPECT_FALSE(client->service_is_ready())
    << "service_is_ready() should return false before the service is created";

  auto service = create_service();

  EXPECT_TRUE(client->service_is_ready())
    << "service_is_ready() should return true after the service is created";
}

TEST_F(ClientTest, WaitForServiceReturnsWhenServiceIsReady)
{
  auto client = create_client();
  auto future = std::async(std::launch::async, [&client]() { return client->wait_for_service(); });
  // Give wait_for_service() a moment to start blocking.
  std::this_thread::sleep_for(100ms);

  EXPECT_NE(future.wait_for(0ms), std::future_status::ready)
    << "wait_for_service() (indefinite) should block while the service is not ready";

  auto service = create_service();

  ASSERT_EQ(future.wait_for(1s), std::future_status::ready)
    << "wait_for_service() should return promptly after the service is created";
  EXPECT_TRUE(future.get()) << "wait_for_service() should return true after the service is created";
}

TEST_F(ClientTest, WaitForServiceTimesOutWhenNoService)
{
  auto client = create_client();
  auto future =
    std::async(std::launch::async, [&client]() { return client->wait_for_service(100ms); });

  EXPECT_FALSE(future.get()) << "wait_for_service() should return false after the timeout";
}

TEST_F(ClientTest, WaitForServiceReturnsOnShutdown)
{
  auto client = create_client();
  auto future = std::async(std::launch::async, [&client]() { return client->wait_for_service(); });
  // Give wait_for_service() a moment to start blocking.
  std::this_thread::sleep_for(100ms);

  rclcpp::shutdown();

  ASSERT_EQ(future.wait_for(1s), std::future_status::ready)
    << "wait_for_service() should return promptly on shutdown";
  EXPECT_FALSE(future.get()) << "wait_for_service() should return false on shutdown";
}

TEST_F(ClientTest, RequestIdIsUnique)
{
  auto client = create_client();
  auto request1 = client->borrow_loaned_request();
  auto request2 = client->borrow_loaned_request();
  auto future_and_request_id2 = client->async_send_request(std::move(request2));
  auto future_and_request_id1 = client->async_send_request(std::move(request1));
  EXPECT_NE(future_and_request_id1.request_id, future_and_request_id2.request_id)
    << "Request IDs should be unique";
}

/* --- ClientTest: end --- */

class AgnocastNodeClientTest : public ::testing::Test
{
  using Request = std_srvs::srv::Empty::Request;
  using Response = std_srvs::srv::Empty::Response;

  std::shared_ptr<agnocast::Node> node_;
  std::shared_ptr<agnocast::AgnocastOnlySingleThreadedExecutor> executor_;
  std::thread spin_thread_;

protected:
  void SetUp() override
  {
    agnocast::init(0, nullptr);
    node_ = std::make_shared<agnocast::Node>("test_node");
    executor_ = std::make_shared<agnocast::AgnocastOnlySingleThreadedExecutor>();
    executor_->add_node(node_->get_node_base_interface());
    spin_thread_ = std::thread([this]() { executor_->spin(); });
  }

  void TearDown() override
  {
    executor_->cancel();
    if (spin_thread_.joinable()) {
      spin_thread_.join();
    }
    if (agnocast::ok()) {
      agnocast::shutdown();
    }
  }

  auto create_client()
  {
    return agnocast::create_client<std_srvs::srv::Empty>(node_.get(), "test_service");
  }
};

TEST_F(AgnocastNodeClientTest, WaitForServiceReturnsOnShutdown)
{
  auto client = create_client();
  auto future = std::async(std::launch::async, [&client]() { return client->wait_for_service(); });
  // Give wait_for_service() a moment to start blocking.
  std::this_thread::sleep_for(100ms);

  agnocast::shutdown();

  ASSERT_EQ(future.wait_for(1s), std::future_status::ready)
    << "wait_for_service() should return promptly on shutdown";
  EXPECT_FALSE(future.get()) << "wait_for_service() should return false on shutdown";
}

/* --- AgnocastNodeClientTest: end --- */

#if AGNOCAST_SERVICE_INTROSPECTION_SUPPORTED

// Service introspection is only available from ROS 2 Iron (rclcpp 21) onwards, since the
// ServiceT::Event message and rcl_service_introspection_state_t do not exist before that.
class ServiceIntrospectionTest : public ::testing::Test
{
protected:
  using SetBool = std_srvs::srv::SetBool;
  using Event = SetBool::Event;
  using EventInfo = decltype(std::declval<Event>().info);

  static constexpr const char * k_service_name = "introspection_service";
  static constexpr const char * k_event_topic = "/introspection_service/_service_event";

  std::shared_ptr<rclcpp::Node> node_;
  std::shared_ptr<agnocast::SingleThreadedAgnocastExecutor> executor_;
  std::thread spin_thread_;

  std::mutex events_mtx_;
  std::vector<Event> events_;
  agnocast::Subscription<Event>::SharedPtr event_sub_;

  void SetUp() override
  {
    rclcpp::init(0, nullptr);
    node_ = std::make_shared<rclcpp::Node>("introspection_test_node");
    executor_ = std::make_shared<agnocast::SingleThreadedAgnocastExecutor>();
    executor_->add_node(node_);
    spin_thread_ = std::thread([this]() { executor_->spin(); });
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

  void subscribe_events()
  {
    event_sub_ = agnocast::create_subscription<Event>(
      node_.get(), k_event_topic, rclcpp::QoS(10),
      [this](const agnocast::ipc_shared_ptr<const Event> & msg) {
        std::lock_guard<std::mutex> lock(events_mtx_);
        events_.push_back(*msg);
      });
  }

  auto create_service()
  {
    auto cb_group = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    return agnocast::create_service<SetBool>(
      node_.get(), k_service_name,
      [](
        agnocast::ipc_shared_ptr<SetBool::Request> && request,
        agnocast::ipc_shared_ptr<SetBool::Response> && response) {
        response->success = request->data;
        response->message = "ok";
      },
      rclcpp::ServicesQoS(), cb_group);
  }

  auto create_client()
  {
    auto cb_group = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    return agnocast::create_client<SetBool>(
      node_.get(), k_service_name, rclcpp::ServicesQoS(), cb_group);
  }

  std::vector<Event> collected_events()
  {
    std::lock_guard<std::mutex> lock(events_mtx_);
    return events_;
  }

  size_t count_of(uint8_t event_type)
  {
    const auto events = collected_events();
    return static_cast<size_t>(std::count_if(
      events.begin(), events.end(),
      [event_type](const Event & e) { return e.info.event_type == event_type; }));
  }

  // Issue one call and give the executor time to deliver the response and all four events.
  void call_once(bool data)
  {
    auto client = create_client();
    auto service = create_service();
    ASSERT_TRUE(client->wait_for_service(1s));

    auto request = client->borrow_loaned_request();
    request->data = data;
    auto future_and_id = client->async_send_request(std::move(request));
    ASSERT_EQ(future_and_id.future.wait_for(5s), std::future_status::ready);

    // The events are published on the same executor, so give them a moment to arrive.
    std::this_thread::sleep_for(500ms);
  }
};

TEST_F(ServiceIntrospectionTest, NoEventsWhenIntrospectionIsOff)
{
  subscribe_events();
  call_once(true);

  EXPECT_TRUE(collected_events().empty())
    << "introspection defaults to off, so no service events should be published";
}

TEST_F(ServiceIntrospectionTest, MetadataModePublishesAllFourEventsWithoutPayload)
{
  subscribe_events();

  auto client = create_client();
  auto service = create_service();
  client->configure_introspection(
    node_->get_clock(), rclcpp::QoS(1), RCL_SERVICE_INTROSPECTION_METADATA);
  service->configure_introspection(
    node_->get_clock(), rclcpp::QoS(1), RCL_SERVICE_INTROSPECTION_METADATA);
  ASSERT_TRUE(client->wait_for_service(1s));

  auto request = client->borrow_loaned_request();
  request->data = true;
  auto future_and_id = client->async_send_request(std::move(request));
  ASSERT_EQ(future_and_id.future.wait_for(5s), std::future_status::ready);
  std::this_thread::sleep_for(500ms);

  EXPECT_EQ(count_of(EventInfo::REQUEST_SENT), 1u);
  EXPECT_EQ(count_of(EventInfo::REQUEST_RECEIVED), 1u);
  EXPECT_EQ(count_of(EventInfo::RESPONSE_SENT), 1u);
  EXPECT_EQ(count_of(EventInfo::RESPONSE_RECEIVED), 1u);

  for (const auto & event : collected_events()) {
    EXPECT_TRUE(event.request.empty()) << "metadata mode must not carry the request payload";
    EXPECT_TRUE(event.response.empty()) << "metadata mode must not carry the response payload";
  }
}

TEST_F(ServiceIntrospectionTest, ContentsModeCarriesPayload)
{
  subscribe_events();

  auto client = create_client();
  auto service = create_service();
  client->configure_introspection(
    node_->get_clock(), rclcpp::QoS(1), RCL_SERVICE_INTROSPECTION_CONTENTS);
  service->configure_introspection(
    node_->get_clock(), rclcpp::QoS(1), RCL_SERVICE_INTROSPECTION_CONTENTS);
  ASSERT_TRUE(client->wait_for_service(1s));

  auto request = client->borrow_loaned_request();
  request->data = true;
  auto future_and_id = client->async_send_request(std::move(request));
  ASSERT_EQ(future_and_id.future.wait_for(5s), std::future_status::ready);
  std::this_thread::sleep_for(500ms);

  const auto events = collected_events();
  ASSERT_EQ(events.size(), 4u);

  for (const auto & event : events) {
    const bool is_request_event = event.info.event_type == EventInfo::REQUEST_SENT ||
                                  event.info.event_type == EventInfo::REQUEST_RECEIVED;
    if (is_request_event) {
      ASSERT_EQ(event.request.size(), 1u);
      EXPECT_TRUE(event.request[0].data);
      EXPECT_TRUE(event.response.empty());
    } else {
      ASSERT_EQ(event.response.size(), 1u);
      EXPECT_TRUE(event.response[0].success);
      EXPECT_EQ(event.response[0].message, "ok");
      EXPECT_TRUE(event.request.empty());
    }
  }
}

TEST_F(ServiceIntrospectionTest, ClientAndServerAgreeOnGidAndSequenceNumber)
{
  subscribe_events();

  auto client = create_client();
  auto service = create_service();
  client->configure_introspection(
    node_->get_clock(), rclcpp::QoS(1), RCL_SERVICE_INTROSPECTION_METADATA);
  service->configure_introspection(
    node_->get_clock(), rclcpp::QoS(1), RCL_SERVICE_INTROSPECTION_METADATA);
  ASSERT_TRUE(client->wait_for_service(1s));

  auto request = client->borrow_loaned_request();
  request->data = false;
  auto future_and_id = client->async_send_request(std::move(request));
  ASSERT_EQ(future_and_id.future.wait_for(5s), std::future_status::ready);
  std::this_thread::sleep_for(500ms);

  const auto events = collected_events();
  ASSERT_EQ(events.size(), 4u);

  const auto expected_gid =
    agnocast::internal::make_service_client_gid(node_->get_fully_qualified_name());
  for (const auto & event : events) {
    EXPECT_EQ(event.info.client_gid, expected_gid)
      << "all four events of one call must carry the caller's GID";
    EXPECT_EQ(event.info.sequence_number, future_and_id.request_id)
      << "all four events of one call must carry the call's sequence number";
  }
}

TEST_F(ServiceIntrospectionTest, ReconfiguringToOffStopsEvents)
{
  subscribe_events();

  auto client = create_client();
  auto service = create_service();
  client->configure_introspection(
    node_->get_clock(), rclcpp::QoS(1), RCL_SERVICE_INTROSPECTION_METADATA);
  service->configure_introspection(
    node_->get_clock(), rclcpp::QoS(1), RCL_SERVICE_INTROSPECTION_METADATA);
  ASSERT_TRUE(client->wait_for_service(1s));

  {
    auto request = client->borrow_loaned_request();
    request->data = true;
    auto future_and_id = client->async_send_request(std::move(request));
    ASSERT_EQ(future_and_id.future.wait_for(5s), std::future_status::ready);
  }
  std::this_thread::sleep_for(500ms);
  const size_t events_while_on = collected_events().size();
  ASSERT_EQ(events_while_on, 4u);

  client->configure_introspection(
    node_->get_clock(), rclcpp::QoS(1), RCL_SERVICE_INTROSPECTION_OFF);
  service->configure_introspection(
    node_->get_clock(), rclcpp::QoS(1), RCL_SERVICE_INTROSPECTION_OFF);

  {
    auto request = client->borrow_loaned_request();
    request->data = true;
    auto future_and_id = client->async_send_request(std::move(request));
    ASSERT_EQ(future_and_id.future.wait_for(5s), std::future_status::ready);
  }
  std::this_thread::sleep_for(500ms);

  EXPECT_EQ(collected_events().size(), events_while_on)
    << "no further events should be published after introspection is turned off";
}

#endif  // AGNOCAST_SERVICE_INTROSPECTION_SUPPORTED
