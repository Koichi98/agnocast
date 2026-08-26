#include "agnocast/agnocast.hpp"
#include "rclcpp/rclcpp.hpp"

#include "std_srvs/srv/empty.hpp"

#include <gtest/gtest.h>

#include <chrono>
#include <future>
#include <memory>
#include <thread>
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

// No service exists in these tests, so nothing ever answers. That is the state a caller has to be
// able to get out of: only a response releases a pending request, so without these calls the entry
// and its promise would outlive every hope of being completed.

TEST_F(ClientTest, RemovePendingRequestReleasesTheEntry)
{
  auto client = create_client();
  auto sent = client->async_send_request(client->borrow_loaned_request());

  EXPECT_TRUE(client->remove_pending_request(sent.request_id))
    << "remove_pending_request() should report removing the pending request";
  EXPECT_FALSE(client->remove_pending_request(sent.request_id))
    << "remove_pending_request() should report nothing to remove the second time";
}

TEST_F(ClientTest, RemovedRequestFutureStopsWaiting)
{
  auto client = create_client();
  auto sent = client->async_send_request(client->borrow_loaned_request());

  ASSERT_NE(sent.future.wait_for(0ms), std::future_status::ready)
    << "the future should still be waiting while the request is pending";

  client->remove_pending_request(sent);

  ASSERT_EQ(sent.future.wait_for(0ms), std::future_status::ready)
    << "removing the pending request should destroy its promise and release the future";
  EXPECT_THROW(sent.future.get(), std::future_error)
    << "the released future should throw rather than hand back a response that never came";
}

TEST_F(ClientTest, RemovingACallbackRequestReleasesItsSharedFuture)
{
  auto client = create_client();
  auto sent = client->async_send_request(client->borrow_loaned_request(), [](auto) {});

  ASSERT_TRUE(client->remove_pending_request(sent))
    << "remove_pending_request() should accept the callback overload's return value too";

  ASSERT_EQ(sent.future.wait_for(0ms), std::future_status::ready)
    << "removing the pending request should destroy its promise and release the shared future";
  EXPECT_THROW(sent.future.get(), std::future_error)
    << "the released shared future should throw rather than wait for a callback that cannot run";
}

TEST_F(ClientTest, PrunePendingRequestsReleasesEveryEntry)
{
  auto client = create_client();
  for (int i = 0; i < 3; ++i) {
    client->async_send_request(client->borrow_loaned_request());
  }

  EXPECT_EQ(client->prune_pending_requests(), 3u)
    << "prune_pending_requests() should remove every pending request";
  EXPECT_EQ(client->prune_pending_requests(), 0u)
    << "prune_pending_requests() should report nothing left the second time";
}

TEST_F(ClientTest, PruneRequestsOlderThanKeepsNewerRequests)
{
  auto client = create_client();
  auto old_request = client->async_send_request(client->borrow_loaned_request());

  // system_clock has no guaranteed resolution, so separate the two sends by an observable gap.
  std::this_thread::sleep_for(10ms);
  const auto cutoff = std::chrono::system_clock::now();
  std::this_thread::sleep_for(10ms);

  auto new_request = client->async_send_request(client->borrow_loaned_request());

  std::vector<int64_t> pruned;
  EXPECT_EQ(client->prune_requests_older_than(cutoff, &pruned), 1u)
    << "only the request sent before the cutoff should be removed";
  EXPECT_EQ(pruned, std::vector<int64_t>{old_request.request_id})
    << "the removed request ID should be reported back";
  EXPECT_TRUE(client->remove_pending_request(new_request))
    << "the request sent after the cutoff should still be pending";
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
