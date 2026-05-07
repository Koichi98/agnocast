// Specification tests for agnocast timer internals.
//
// These tests document and lock in the observable behavior of timer-related
// helper functions defined in agnocast_timer_info.cpp:
//   - handle_pre_time_jump
//   - handle_post_time_jump
//   - register_timer_info / unregister_timer_info
//   - create_timer_fd
//
// Behavioral expectations are derived from the integration tests in
// test/integration/test_agnocast_create_timer.cpp.
//
// Each test follows the Arrange / Act / Assert pattern.

#include "agnocast/agnocast_timer_info.hpp"

#include <gtest/gtest.h>
#include <rcl/time.h>
#include <sys/eventfd.h>
#include <unistd.h>

#include <chrono>
#include <memory>

using namespace std::chrono_literals;

// Forward declarations for internal helpers (file-local in agnocast_timer_info.cpp
// but with external linkage; declared here so tests can call them directly).
namespace agnocast
{
void handle_pre_time_jump(TimerInfo & timer_info);
void handle_post_time_jump(TimerInfo & timer_info, const rcl_time_jump_t & jump);
}  // namespace agnocast

namespace
{

constexpr int64_t kPeriodNs = 100'000'000;  // 100ms

// Creates a Clock (RCL_ROS_TIME) with ROS time override enabled and set to `time_ns`.
rclcpp::Clock::SharedPtr make_ros_clock_at(int64_t time_ns)
{
  auto clock = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
  rcl_clock_t * rcl_clock = clock->get_clock_handle();
  std::lock_guard<std::mutex> lock(clock->get_clock_mutex());
  EXPECT_EQ(rcl_enable_ros_time_override(rcl_clock), RCL_RET_OK);
  EXPECT_EQ(rcl_set_ros_time_override(rcl_clock, time_ns), RCL_RET_OK);
  return clock;
}

// Builds a TimerInfo with the minimum fields required by the time-jump handlers.
std::shared_ptr<agnocast::TimerInfo> make_timer_info(
  rclcpp::Clock::SharedPtr clock, int64_t now_ns, int64_t period_ns = kPeriodNs)
{
  auto info = std::make_shared<agnocast::TimerInfo>();
  info->timer_id = 1;
  info->period = std::chrono::nanoseconds{period_ns};
  info->clock = std::move(clock);
  info->last_call_time_ns.store(now_ns, std::memory_order_relaxed);
  info->next_call_time_ns.store(now_ns + period_ns, std::memory_order_relaxed);
  info->time_credit.store(0, std::memory_order_relaxed);
  return info;
}

// Reads up to one u64 event from `fd` non-destructively for assertion.
// Returns true if an event was pending and successfully consumed.
bool consume_eventfd(int fd)
{
  uint64_t value = 0;
  const ssize_t ret = read(fd, &value, sizeof(value));
  return ret == sizeof(value) && value > 0;
}

}  // namespace

// =====================================================================
// Category: Pre-jump callback (handle_pre_time_jump)
// =====================================================================

TEST(TimerHandlePreJump, SavesRemainingPeriodAsTimeCreditWhenClockInitialized)
{
  // Arrange
  const int64_t now_ns = 500'000'000;  // 0.5s on the new epoch
  auto clock = make_ros_clock_at(now_ns);
  auto info = make_timer_info(clock, /*now_ns=*/now_ns);
  // Manually set next_call so the credit is deterministic: now + 30ms.
  const int64_t next_call_ns = now_ns + 30'000'000;
  info->next_call_time_ns.store(next_call_ns, std::memory_order_relaxed);

  // Act
  agnocast::handle_pre_time_jump(*info);

  // Assert
  EXPECT_EQ(info->time_credit.load(std::memory_order_relaxed), next_call_ns - now_ns);
}

TEST(TimerHandlePreJump, IsNoopWhenClockUninitialized)
{
  // Arrange — ROS clock with override at t=0 represents an uninitialized clock.
  auto clock = make_ros_clock_at(0);
  auto info = make_timer_info(clock, /*now_ns=*/0);
  info->time_credit.store(42, std::memory_order_relaxed);  // sentinel

  // Act
  agnocast::handle_pre_time_jump(*info);

  // Assert — pre-jump bails out without overwriting time_credit.
  EXPECT_EQ(info->time_credit.load(std::memory_order_relaxed), 42);
}

// =====================================================================
// Category: Post-jump — ROS time activation
// =====================================================================

TEST(TimerHandlePostJump, RosTimeActivationClosesTimerFd)
{
  // Arrange
  auto clock = make_ros_clock_at(1'000'000'000);  // 1s
  auto info = make_timer_info(clock, /*now_ns=*/1'000'000'000);
  // Open a real timerfd so we can observe it being closed.
  info->timer_fd = eventfd(0, EFD_NONBLOCK | EFD_CLOEXEC);
  ASSERT_GE(info->timer_fd, 0);
  rcl_time_jump_t jump = {};
  jump.clock_change = RCL_ROS_TIME_ACTIVATED;

  // Act
  agnocast::handle_post_time_jump(*info, jump);

  // Assert — the field is reset to -1 so subsequent accesses skip it.
  EXPECT_EQ(info->timer_fd, -1);
}

TEST(TimerHandlePostJump, RosTimeActivationConsumesAndAppliesTimeCredit)
{
  // Arrange — 30ms of credit was saved in the pre-jump callback.
  const int64_t now_ns = 2'000'000'000;
  auto clock = make_ros_clock_at(now_ns);
  auto info = make_timer_info(clock, /*now_ns=*/now_ns);
  const int64_t credit = 30'000'000;
  info->time_credit.store(credit, std::memory_order_relaxed);
  rcl_time_jump_t jump = {};
  jump.clock_change = RCL_ROS_TIME_ACTIVATED;

  // Act
  agnocast::handle_post_time_jump(*info, jump);

  // Assert — credit is consumed (set to 0) and applied to call-time anchors.
  EXPECT_EQ(info->time_credit.load(std::memory_order_relaxed), 0);
  EXPECT_EQ(info->last_call_time_ns.load(std::memory_order_relaxed), now_ns - credit);
  EXPECT_EQ(info->next_call_time_ns.load(std::memory_order_relaxed), now_ns - credit + kPeriodNs);
}

// =====================================================================
// Category: Post-jump — ROS time deactivation
// =====================================================================

TEST(TimerHandlePostJump, RosTimeDeactivationDoesNotMutateState)
{
  // Arrange — Deactivation is unsupported and should only emit a warning.
  const int64_t now_ns = 3'000'000'000;
  auto clock = make_ros_clock_at(now_ns);
  auto info = make_timer_info(clock, /*now_ns=*/now_ns);
  const int64_t snapshot_next = info->next_call_time_ns.load(std::memory_order_relaxed);
  const int64_t snapshot_last = info->last_call_time_ns.load(std::memory_order_relaxed);
  rcl_time_jump_t jump = {};
  jump.clock_change = RCL_ROS_TIME_DEACTIVATED;

  // Act
  agnocast::handle_post_time_jump(*info, jump);

  // Assert
  EXPECT_EQ(info->next_call_time_ns.load(std::memory_order_relaxed), snapshot_next);
  EXPECT_EQ(info->last_call_time_ns.load(std::memory_order_relaxed), snapshot_last);
  EXPECT_EQ(info->timer_fd, -1);  // unchanged from default
}

// =====================================================================
// Category: Post-jump — Forward jump (no clock change)
// =====================================================================

TEST(TimerHandlePostJump, ForwardJumpWritesClockEventfdWhenTimerIsReady)
{
  // Arrange — now has advanced past next_call_time, so the timer is ready.
  const int64_t now_ns = 1'200'000'000;
  auto clock = make_ros_clock_at(now_ns);
  auto info = make_timer_info(clock, /*now_ns=*/now_ns - 200'000'000);
  info->next_call_time_ns.store(now_ns - 100'000'000, std::memory_order_relaxed);
  info->clock_eventfd = eventfd(0, EFD_NONBLOCK | EFD_CLOEXEC);
  ASSERT_GE(info->clock_eventfd, 0);
  rcl_time_jump_t jump = {};
  jump.clock_change = RCL_ROS_TIME_NO_CHANGE;

  // Act
  agnocast::handle_post_time_jump(*info, jump);

  // Assert — exactly one wakeup is delivered to clock_eventfd.
  EXPECT_TRUE(consume_eventfd(info->clock_eventfd));
}

TEST(TimerHandlePostJump, ForwardJumpDoesNotWriteWhenTimerNotReady)
{
  // Arrange — next_call_time is still in the future after the jump.
  const int64_t now_ns = 1'000'000'000;
  auto clock = make_ros_clock_at(now_ns);
  auto info = make_timer_info(clock, /*now_ns=*/now_ns);
  info->next_call_time_ns.store(now_ns + 100'000'000, std::memory_order_relaxed);
  info->clock_eventfd = eventfd(0, EFD_NONBLOCK | EFD_CLOEXEC);
  ASSERT_GE(info->clock_eventfd, 0);
  rcl_time_jump_t jump = {};
  jump.clock_change = RCL_ROS_TIME_NO_CHANGE;

  // Act
  agnocast::handle_post_time_jump(*info, jump);

  // Assert — nothing is delivered to clock_eventfd.
  EXPECT_FALSE(consume_eventfd(info->clock_eventfd));
}

// =====================================================================
// Category: Post-jump — Backward jump (no clock change)
// =====================================================================

TEST(TimerHandlePostJump, BackwardJumpResetsCallTimesWhenJumpedPastLastCall)
{
  // Arrange — now is earlier than the recorded last_call_time (backward jump).
  const int64_t last_call_ns = 1'000'000'000;
  const int64_t now_ns = 200'000'000;  // 800ms backwards
  auto clock = make_ros_clock_at(now_ns);
  auto info = make_timer_info(clock, /*now_ns=*/last_call_ns);
  rcl_time_jump_t jump = {};
  jump.clock_change = RCL_ROS_TIME_NO_CHANGE;

  // Act
  agnocast::handle_post_time_jump(*info, jump);

  // Assert — call-time anchors are reset relative to the new "now".
  EXPECT_EQ(info->last_call_time_ns.load(std::memory_order_relaxed), now_ns);
  EXPECT_EQ(info->next_call_time_ns.load(std::memory_order_relaxed), now_ns + kPeriodNs);
}

TEST(TimerHandlePostJump, BackwardJumpWithinOnePeriodLeavesStateUntouched)
{
  // Arrange — now is still after last_call (jump is small / forward).
  const int64_t last_call_ns = 1'000'000'000;
  const int64_t now_ns = 1'050'000'000;  // 50ms after last_call
  auto clock = make_ros_clock_at(now_ns);
  auto info = make_timer_info(clock, /*now_ns=*/last_call_ns);
  // next_call still in the future, so the forward-ready branch is not entered either.
  info->next_call_time_ns.store(now_ns + 50'000'000, std::memory_order_relaxed);
  rcl_time_jump_t jump = {};
  jump.clock_change = RCL_ROS_TIME_NO_CHANGE;

  // Act
  agnocast::handle_post_time_jump(*info, jump);

  // Assert
  EXPECT_EQ(info->last_call_time_ns.load(std::memory_order_relaxed), last_call_ns);
  EXPECT_EQ(info->next_call_time_ns.load(std::memory_order_relaxed), now_ns + 50'000'000);
}

// =====================================================================
// Category: Timer fd creation (create_timer_fd)
// =====================================================================

TEST(CreateTimerFd, ReturnsValidFdForSteadyClock)
{
  // Arrange / Act
  const int fd = agnocast::create_timer_fd(1, 100ms, RCL_STEADY_TIME);

  // Assert
  EXPECT_GE(fd, 0);
  if (fd >= 0) {
    close(fd);
  }
}

TEST(CreateTimerFd, ReturnsValidFdForRosClock)
{
  // Arrange / Act — ROS_TIME also creates a timerfd (used while sim time is inactive).
  const int fd = agnocast::create_timer_fd(2, 100ms, RCL_ROS_TIME);

  // Assert
  EXPECT_GE(fd, 0);
  if (fd >= 0) {
    close(fd);
  }
}

TEST(CreateTimerFd, AcceptsZeroPeriod)
{
  // Arrange / Act — Zero period uses a workaround so timerfd_settime does not disarm.
  const int fd = agnocast::create_timer_fd(3, 0ms, RCL_STEADY_TIME);

  // Assert
  EXPECT_GE(fd, 0);
  if (fd >= 0) {
    close(fd);
  }
}

// =====================================================================
// Category: TimerInfo registry (register_timer_info / unregister_timer_info)
// =====================================================================

class TimerInfoRegistry : public ::testing::Test
{
protected:
  void TearDown() override
  {
    std::lock_guard<std::mutex> lock(agnocast::id2_timer_info_mtx);
    agnocast::id2_timer_info.clear();
  }
};

TEST_F(TimerInfoRegistry, UnregisterRemovesEntry)
{
  // Arrange
  const uint32_t id = 99;
  {
    std::lock_guard<std::mutex> lock(agnocast::id2_timer_info_mtx);
    agnocast::id2_timer_info[id] = std::make_shared<agnocast::TimerInfo>();
  }

  // Act
  agnocast::unregister_timer_info(id);

  // Assert
  std::lock_guard<std::mutex> lock(agnocast::id2_timer_info_mtx);
  EXPECT_EQ(agnocast::id2_timer_info.count(id), 0u);
}

TEST_F(TimerInfoRegistry, AllocateTimerIdProducesMonotonicallyIncreasingIds)
{
  // Arrange
  const uint32_t first = agnocast::allocate_timer_id();

  // Act
  const uint32_t second = agnocast::allocate_timer_id();
  const uint32_t third = agnocast::allocate_timer_id();

  // Assert
  EXPECT_LT(first, second);
  EXPECT_LT(second, third);
}
