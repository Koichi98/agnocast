#include "agnocast/agnocast.hpp"
#include "agnocast/internal/service_introspection.hpp"
#include "rclcpp/rclcpp.hpp"

#include "std_srvs/srv/empty.hpp"
#include "std_srvs/srv/set_bool.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <future>
#include <memory>
#include <mutex>
#include <stdexcept>
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

// Every client of one service on one node reads the same response topic and tells responses apart
// by sequence number alone.
class SharedResponseTopicTest : public ::testing::Test
{
protected:
  using SetBool = std_srvs::srv::SetBool;

  std::shared_ptr<rclcpp::Node> node_;
  std::shared_ptr<agnocast::SingleThreadedAgnocastExecutor> executor_;
  std::thread spin_thread_;

  void SetUp() override
  {
    rclcpp::init(0, nullptr);
    node_ = std::make_shared<rclcpp::Node>("shared_response_topic_test_node");
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
    return agnocast::create_service<SetBool>(
      node_.get(), "shared_response_topic_service",
      [](
        agnocast::ipc_shared_ptr<SetBool::Request> && request,
        agnocast::ipc_shared_ptr<SetBool::Response> && response) {
        response->success = request->data;
      },
      rclcpp::ServicesQoS(), cb_group);
  }

  auto create_client()
  {
    auto cb_group = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    return agnocast::create_client<SetBool>(
      node_.get(), "shared_response_topic_service", rclcpp::ServicesQoS(), cb_group);
  }
};

TEST_F(SharedResponseTopicTest, SequenceNumbersDoNotCollideBetweenClientsOnTheSameNode)
{
  auto client1 = create_client();
  auto client2 = create_client();

  auto request1 = client1->borrow_loaned_request();
  auto request2 = client2->borrow_loaned_request();
  auto id1 = client1->async_send_request(std::move(request1)).request_id;
  auto id2 = client2->async_send_request(std::move(request2)).request_id;

  EXPECT_NE(id1, id2);
}

TEST_F(SharedResponseTopicTest, EachClientResolvesItsOwnResponse)
{
  auto client1 = create_client();
  auto client2 = create_client();
  auto service = create_service();
  ASSERT_TRUE(client1->wait_for_service(1s));

  auto request1 = client1->borrow_loaned_request();
  request1->data = true;
  auto request2 = client2->borrow_loaned_request();
  request2->data = false;

  auto future1 = client1->async_send_request(std::move(request1));
  auto future2 = client2->async_send_request(std::move(request2));

  ASSERT_EQ(future1.future.wait_for(5s), std::future_status::ready);
  ASSERT_EQ(future2.future.wait_for(5s), std::future_status::ready);

  EXPECT_TRUE(future1.future.get()->success);
  EXPECT_FALSE(future2.future.get()->success);
}

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
  // Spelled out, so the test pins the ROS 2 convention independently of the helper.
  static constexpr const char * k_event_topic = "/introspection_service/_service_event";
  static constexpr const char * k_deferred_service_name = "introspection_deferred_service";
  static constexpr const char * k_deferred_event_topic =
    "/introspection_deferred_service/_service_event";

  static constexpr auto k_event_timeout = 5s;
  // Paid in full by every test asserting an absence, so it is kept short.
  static constexpr auto k_quiescence = 500ms;

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

  void subscribe_events(const char * topic = k_event_topic)
  {
    event_sub_ = agnocast::create_subscription<Event>(
      node_.get(), topic, rclcpp::QoS(10),
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

  auto create_deferred_service()
  {
    auto cb_group = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    return agnocast::create_service<SetBool>(
      node_.get(), k_deferred_service_name,
      [](
        std::shared_ptr<agnocast::Service<SetBool>> service,
        agnocast::ipc_shared_ptr<SetBool::Request> && request) {
        auto response = service->borrow_loaned_response(request);
        response->success = request->data;
        response->message = "ok";
        service->send_response(std::move(request), std::move(response));
      },
      rclcpp::ServicesQoS(), cb_group);
  }

  auto create_client(const char * service_name = k_service_name)
  {
    auto cb_group = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    return agnocast::create_client<SetBool>(
      node_.get(), service_name, rclcpp::ServicesQoS(), cb_group);
  }

  std::vector<Event> collected_events()
  {
    std::lock_guard<std::mutex> lock(events_mtx_);
    return events_;
  }

  size_t event_count()
  {
    std::lock_guard<std::mutex> lock(events_mtx_);
    return events_.size();
  }

  bool wait_for_events(size_t expected)
  {
    const auto deadline = std::chrono::steady_clock::now() + k_event_timeout;
    while (std::chrono::steady_clock::now() < deadline) {
      if (event_count() >= expected) {
        return true;
      }
      std::this_thread::sleep_for(10ms);
    }
    return event_count() >= expected;
  }

  size_t count_of(uint8_t event_type)
  {
    const auto events = collected_events();
    return static_cast<size_t>(std::count_if(
      events.begin(), events.end(),
      [event_type](const Event & e) { return e.info.event_type == event_type; }));
  }

  void call_once(const agnocast::Client<SetBool>::SharedPtr & client, bool data, int64_t * seqno)
  {
    auto request = client->borrow_loaned_request();
    request->data = data;
    auto future_and_id = client->async_send_request(std::move(request));
    ASSERT_EQ(future_and_id.future.wait_for(5s), std::future_status::ready);
    if (seqno != nullptr) {
      *seqno = future_and_id.request_id;
    }
  }

  void enable_introspection(
    const agnocast::Client<SetBool>::SharedPtr & client,
    const agnocast::Service<SetBool>::SharedPtr & service, rcl_service_introspection_state_t state)
  {
    client->configure_introspection(node_->get_clock(), rclcpp::QoS(1), state);
    service->configure_introspection(node_->get_clock(), rclcpp::QoS(1), state);
  }
};

TEST_F(ServiceIntrospectionTest, NoEventsWhenIntrospectionIsOff)
{
  subscribe_events();

  auto client = create_client();
  auto service = create_service();
  ASSERT_TRUE(client->wait_for_service(1s));

  call_once(client, true, nullptr);
  std::this_thread::sleep_for(k_quiescence);

  ASSERT_TRUE(collected_events().empty())
    << "introspection defaults to off, so no service events should be published";

  // Positive control: an absence assertion alone would also pass if the subscription never worked.
  enable_introspection(client, service, RCL_SERVICE_INTROSPECTION_METADATA);
  call_once(client, true, nullptr);

  EXPECT_TRUE(wait_for_events(4))
    << "the same subscription must see events once introspection is enabled, otherwise the "
       "assertion above proves nothing";
}

TEST_F(ServiceIntrospectionTest, MetadataModePublishesAllFourEventsWithoutPayload)
{
  subscribe_events();

  auto client = create_client();
  auto service = create_service();
  enable_introspection(client, service, RCL_SERVICE_INTROSPECTION_METADATA);
  ASSERT_TRUE(client->wait_for_service(1s));

  call_once(client, true, nullptr);
  ASSERT_TRUE(wait_for_events(4)) << "one call must produce all four events";

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
  enable_introspection(client, service, RCL_SERVICE_INTROSPECTION_CONTENTS);
  ASSERT_TRUE(client->wait_for_service(1s));

  call_once(client, true, nullptr);
  ASSERT_TRUE(wait_for_events(4)) << "one call must produce all four events";

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
  enable_introspection(client, service, RCL_SERVICE_INTROSPECTION_METADATA);
  ASSERT_TRUE(client->wait_for_service(1s));

  int64_t seqno = -1;
  call_once(client, false, &seqno);
  ASSERT_TRUE(wait_for_events(4)) << "one call must produce all four events";

  const auto events = collected_events();
  ASSERT_EQ(events.size(), 4u);

  const auto expected_gid =
    agnocast::internal::make_service_client_gid(node_->get_fully_qualified_name());
  for (const auto & event : events) {
    EXPECT_EQ(event.info.client_gid, expected_gid)
      << "all four events of one call must carry the caller's GID";
    EXPECT_EQ(event.info.sequence_number, seqno)
      << "all four events of one call must carry the call's sequence number";
  }
}

// The deferred path emits RESPONSE_SENT from send_response(), a separate emission site.
TEST_F(ServiceIntrospectionTest, DeferredCallbackPathEmitsAllFourEvents)
{
  subscribe_events(k_deferred_event_topic);

  auto client = create_client(k_deferred_service_name);
  auto service = create_deferred_service();
  enable_introspection(client, service, RCL_SERVICE_INTROSPECTION_CONTENTS);
  ASSERT_TRUE(client->wait_for_service(1s));

  int64_t seqno = -1;
  call_once(client, true, &seqno);
  ASSERT_TRUE(wait_for_events(4))
    << "a deferred-callback service must emit the same four events as a basic one";

  EXPECT_EQ(count_of(EventInfo::REQUEST_SENT), 1u);
  EXPECT_EQ(count_of(EventInfo::REQUEST_RECEIVED), 1u);
  EXPECT_EQ(count_of(EventInfo::RESPONSE_SENT), 1u)
    << "RESPONSE_SENT must be emitted exactly once, by send_response()";
  EXPECT_EQ(count_of(EventInfo::RESPONSE_RECEIVED), 1u);

  for (const auto & event : collected_events()) {
    EXPECT_EQ(event.info.sequence_number, seqno);
  }
}

TEST_F(ServiceIntrospectionTest, ReconfiguringToOffStopsEvents)
{
  subscribe_events();

  auto client = create_client();
  auto service = create_service();
  enable_introspection(client, service, RCL_SERVICE_INTROSPECTION_METADATA);
  ASSERT_TRUE(client->wait_for_service(1s));

  call_once(client, true, nullptr);
  ASSERT_TRUE(wait_for_events(4)) << "one call must produce all four events";
  const size_t events_while_on = event_count();
  ASSERT_EQ(events_while_on, 4u);

  enable_introspection(client, service, RCL_SERVICE_INTROSPECTION_OFF);

  call_once(client, true, nullptr);
  std::this_thread::sleep_for(k_quiescence);

  EXPECT_EQ(event_count(), events_while_on)
    << "no further events should be published after introspection is turned off";
}

TEST_F(ServiceIntrospectionTest, ConfigureIntrospectionRejectsNullClock)
{
  auto client = create_client();
  auto service = create_service();

  EXPECT_THROW(
    client->configure_introspection(nullptr, rclcpp::QoS(1), RCL_SERVICE_INTROSPECTION_METADATA),
    std::invalid_argument)
    << "a null clock would silently drop every event, so it must be rejected up front";
  EXPECT_THROW(
    service->configure_introspection(nullptr, rclcpp::QoS(1), RCL_SERVICE_INTROSPECTION_METADATA),
    std::invalid_argument);

  // OFF needs no clock, so it must stay accepted.
  EXPECT_NO_THROW(
    client->configure_introspection(nullptr, rclcpp::QoS(1), RCL_SERVICE_INTROSPECTION_OFF));
}

#endif  // AGNOCAST_SERVICE_INTROSPECTION_SUPPORTED
