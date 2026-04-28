#include "agnocast_cie_thread_configurator/non_ros_thread_info_ipc.hpp"

#include <rclcpp/logger.hpp>

#include <gtest/gtest.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <cstring>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <system_error>
#include <thread>
#include <vector>

using namespace std::chrono_literals;

using agnocast_cie_thread_configurator::fill_abstract_sockaddr;
using agnocast_cie_thread_configurator::kNonRosThreadInfoSocketName;
using agnocast_cie_thread_configurator::NonRosThreadInfoIpcServer;
using agnocast_cie_thread_configurator::NonRosThreadInfoMsg;
using agnocast_cie_thread_configurator::open_sender_socket;
using agnocast_cie_thread_configurator::send_thread_info;

namespace
{

class ReceivedMessages
{
public:
  void push(const NonRosThreadInfoMsg & msg)
  {
    std::lock_guard<std::mutex> lock(m_);
    msgs_.push_back(msg);
    cv_.notify_all();
  }

  bool wait_for_count(size_t n, std::chrono::milliseconds timeout)
  {
    std::unique_lock<std::mutex> lock(m_);
    return cv_.wait_for(lock, timeout, [this, n] { return msgs_.size() >= n; });
  }

  std::vector<NonRosThreadInfoMsg> snapshot()
  {
    std::lock_guard<std::mutex> lock(m_);
    return msgs_;
  }

private:
  std::mutex m_;
  std::condition_variable cv_;
  std::vector<NonRosThreadInfoMsg> msgs_;
};

}  // namespace

TEST(IpcServer, RoundTripDeliversThreadIdAndName)
{
  auto received = std::make_shared<ReceivedMessages>();
  NonRosThreadInfoIpcServer server(
    rclcpp::get_logger("test"), [received](const NonRosThreadInfoMsg & m) { received->push(m); });

  const int fd = open_sender_socket("test_thread");
  ASSERT_NE(fd, -1);

  NonRosThreadInfoMsg msg = {};
  msg.thread_id = 42;
  std::memcpy(msg.thread_name, "test_thread", 11);

  ASSERT_TRUE(send_thread_info(fd, msg, "test_thread"));
  ::close(fd);

  ASSERT_TRUE(received->wait_for_count(1, 1s));
  auto got = received->snapshot();
  ASSERT_EQ(got.size(), 1u);
  EXPECT_EQ(got[0].thread_id, 42);
  EXPECT_STREQ(got[0].thread_name, "test_thread");
}

TEST(IpcServer, BindCollisionThrowsEaddrInUse)
{
  NonRosThreadInfoIpcServer first(rclcpp::get_logger("test"), [](const NonRosThreadInfoMsg &) {});

  try {
    NonRosThreadInfoIpcServer second(
      rclcpp::get_logger("test"), [](const NonRosThreadInfoMsg &) {});
    FAIL() << "expected std::system_error from second bind";
  } catch (const std::system_error & e) {
    EXPECT_EQ(e.code().value(), EADDRINUSE);
  }
}

TEST(IpcServer, EmptyCallbackRejected)
{
  EXPECT_THROW(NonRosThreadInfoIpcServer(rclcpp::get_logger("test"), {}), std::invalid_argument);
}

TEST(IpcServer, MalformedDatagramsAreDropped)
{
  auto received = std::make_shared<ReceivedMessages>();
  NonRosThreadInfoIpcServer server(
    rclcpp::get_logger("test"), [received](const NonRosThreadInfoMsg & m) { received->push(m); });

  const int fd = ::socket(AF_UNIX, SOCK_DGRAM | SOCK_CLOEXEC, 0);
  ASSERT_NE(fd, -1);
  sockaddr_un addr;
  const socklen_t addrlen = fill_abstract_sockaddr(addr, kNonRosThreadInfoSocketName);

  uint8_t small[4] = {1, 2, 3, 4};
  ASSERT_NE(
    ::sendto(fd, small, sizeof(small), 0, reinterpret_cast<const sockaddr *>(&addr), addrlen), -1);

  std::vector<uint8_t> big(1024, 0xAB);
  ASSERT_NE(
    ::sendto(fd, big.data(), big.size(), 0, reinterpret_cast<const sockaddr *>(&addr), addrlen),
    -1);

  NonRosThreadInfoMsg good = {};
  good.thread_id = 7;
  std::memcpy(good.thread_name, "alive", 5);
  ASSERT_NE(
    ::sendto(fd, &good, sizeof(good), 0, reinterpret_cast<const sockaddr *>(&addr), addrlen), -1);

  ::close(fd);

  ASSERT_TRUE(received->wait_for_count(1, 1s));
  auto got = received->snapshot();
  ASSERT_EQ(got.size(), 1u);
  EXPECT_EQ(got[0].thread_id, 7);
  EXPECT_STREQ(got[0].thread_name, "alive");
}

TEST(IpcServer, CallbackExceptionDoesNotKillReceiver)
{
  std::atomic<int> dispatch_count{0};
  auto received = std::make_shared<ReceivedMessages>();

  NonRosThreadInfoIpcServer server(
    rclcpp::get_logger("test"), [&dispatch_count, received](const NonRosThreadInfoMsg & m) {
      const int n = ++dispatch_count;
      if (n == 1) {
        throw std::runtime_error("boom");
      }
      received->push(m);
    });

  const int fd = open_sender_socket("first");
  ASSERT_NE(fd, -1);

  NonRosThreadInfoMsg msg1 = {};
  msg1.thread_id = 1;
  std::memcpy(msg1.thread_name, "first", 5);
  ASSERT_TRUE(send_thread_info(fd, msg1, "first"));

  // Wait briefly so the throw is processed before the second send.
  std::this_thread::sleep_for(50ms);

  NonRosThreadInfoMsg msg2 = {};
  msg2.thread_id = 2;
  std::memcpy(msg2.thread_name, "second", 6);
  ASSERT_TRUE(send_thread_info(fd, msg2, "second"));
  ::close(fd);

  ASSERT_TRUE(received->wait_for_count(1, 1s));
  auto got = received->snapshot();
  ASSERT_EQ(got.size(), 1u);
  EXPECT_EQ(got[0].thread_id, 2);
  EXPECT_STREQ(got[0].thread_name, "second");
  EXPECT_EQ(dispatch_count.load(), 2);
}

TEST(IpcServer, ShutdownJoinsPromptly)
{
  const auto start = std::chrono::steady_clock::now();
  {
    NonRosThreadInfoIpcServer server(
      rclcpp::get_logger("test"), [](const NonRosThreadInfoMsg &) {});
    std::this_thread::sleep_for(50ms);  // let receiver enter epoll_wait
  }
  const auto elapsed = std::chrono::steady_clock::now() - start;
  EXPECT_LT(elapsed, 1s);
}
