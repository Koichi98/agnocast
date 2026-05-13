#include "agnocast/agnocast.hpp"
#include "agnocast/agnocast_timer_info.hpp"
#include "rclcpp/rclcpp.hpp"

#include <gtest/gtest.h>
#include <rcl/time.h>

#include <chrono>
#include <memory>
#include <mutex>
#include <thread>

using namespace agnocast;

// =========================================
// create_timer free function tests
// =========================================

class CreateTimerFreeFunctionTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    rclcpp::NodeOptions options;
    options.start_parameter_services(false);
    node = std::make_shared<agnocast::Node>("test_timer_node", options);
  }

  void TearDown() override { node.reset(); }

  std::shared_ptr<agnocast::Node> node;
};

TEST_F(CreateTimerFreeFunctionTest, creates_timer_and_registers_info)
{
  // Arrange
  auto clock = std::make_shared<rclcpp::Clock>(RCL_STEADY_TIME);
  const auto period = rclcpp::Duration(std::chrono::milliseconds(100));

  // Act
  auto timer = agnocast::create_timer(node.get(), clock, period, []() {});

  // Assert
  ASSERT_NE(timer, nullptr);
  EXPECT_EQ(timer->get_clock()->get_clock_type(), RCL_STEADY_TIME);
}

TEST_F(CreateTimerFreeFunctionTest, uses_default_callback_group_when_nullptr)
{
  // Arrange
  auto clock = std::make_shared<rclcpp::Clock>(RCL_STEADY_TIME);
  const auto period = rclcpp::Duration(std::chrono::milliseconds(100));
  auto expected_group = node->get_node_base_interface()->get_default_callback_group();

  // Act
  auto timer = agnocast::create_timer(node.get(), clock, period, []() {}, nullptr);

  // Assert: verify the timer was registered with the default callback group
  ASSERT_NE(timer, nullptr);
  // Check via id2_timer_info that the callback group matches the default
  std::lock_guard<std::mutex> lock(agnocast::id2_timer_info_mtx);
  bool found = false;
  for (const auto & [id, info] : agnocast::id2_timer_info) {
    if (info->callback_group == expected_group && info->timer.lock() == timer) {
      found = true;
      break;
    }
  }
  EXPECT_TRUE(found) << "Timer should be registered with the default callback group";
}

TEST_F(CreateTimerFreeFunctionTest, uses_explicit_callback_group)
{
  // Arrange
  auto clock = std::make_shared<rclcpp::Clock>(RCL_STEADY_TIME);
  const auto period = rclcpp::Duration(std::chrono::milliseconds(50));
  auto group = node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  // Act
  auto timer = agnocast::create_timer(node.get(), clock, period, []() {}, group);

  // Assert
  ASSERT_NE(timer, nullptr);
  std::lock_guard<std::mutex> lock(agnocast::id2_timer_info_mtx);
  bool found = false;
  for (const auto & [id, info] : agnocast::id2_timer_info) {
    if (info->callback_group == group && info->timer.lock() == timer) {
      found = true;
      break;
    }
  }
  EXPECT_TRUE(found) << "Timer should be registered with the explicit callback group";
}

TEST_F(CreateTimerFreeFunctionTest, callback_is_invoked)
{
  // Arrange
  auto clock = std::make_shared<rclcpp::Clock>(RCL_STEADY_TIME);
  const auto period = rclcpp::Duration(std::chrono::milliseconds(10));
  bool called = false;
  auto timer = agnocast::create_timer(node.get(), clock, period, [&called]() { called = true; });

  // Act
  timer->execute_callback();

  // Assert
  EXPECT_TRUE(called);
}

// =========================================
// cancel and reset and time_until_trigger function tests
// =========================================

TEST_F(CreateTimerFreeFunctionTest, new_timer_starts_running)
{
  // Arrange
  auto clock = std::make_shared<rclcpp::Clock>(RCL_STEADY_TIME);
  const auto period = rclcpp::Duration(std::chrono::milliseconds(100));

  // Act: agnocast does not support autostart=false, so a freshly created timer
  // must already be running (not canceled) with the next trigger within one period.
  auto timer = agnocast::create_timer(node.get(), clock, period, []() {});

  // Assert
  EXPECT_FALSE(timer->is_canceled());
  const auto tut = timer->time_until_trigger();
  EXPECT_GT(tut, std::chrono::nanoseconds(0));
  EXPECT_LE(tut, period.to_chrono<std::chrono::nanoseconds>());
}

TEST_F(CreateTimerFreeFunctionTest, cancel_timer)
{
  // Arrange
  auto clock = std::make_shared<rclcpp::Clock>(RCL_STEADY_TIME);
  const auto period = rclcpp::Duration(std::chrono::milliseconds(100));
  bool called = false;
  auto timer = agnocast::create_timer(node.get(), clock, period, [&called]() { called = true; });

  // Act
  timer->cancel();

  // Assert
  EXPECT_TRUE(timer->is_canceled());
  EXPECT_FALSE(called);
}

TEST_F(CreateTimerFreeFunctionTest, time_until_trigger_cancel_and_reset)
{
  // Arrange
  auto clock = std::make_shared<rclcpp::Clock>(RCL_STEADY_TIME);
  const auto period = rclcpp::Duration(std::chrono::milliseconds(100));
  bool called = false;
  auto timer = agnocast::create_timer(node.get(), clock, period, [&called]() { called = true; });

  // Act
  auto tut_before_cancel = timer->time_until_trigger();
  timer->cancel();
  auto tut_after_cancel = timer->time_until_trigger();
  timer->reset();
  auto tut_after_reset = timer->time_until_trigger();
  std::this_thread::sleep_for(std::chrono::milliseconds(200));
  auto tut_after_wait = timer->time_until_trigger();

  // Assert
  EXPECT_TRUE(tut_before_cancel < std::chrono::milliseconds(100));
  EXPECT_EQ(tut_after_cancel, std::chrono::nanoseconds::max());
  EXPECT_TRUE(tut_after_reset < std::chrono::milliseconds(100));
  EXPECT_LT(tut_after_wait, std::chrono::nanoseconds(0));
  EXPECT_FALSE(called);
}

TEST_F(CreateTimerFreeFunctionTest, time_until_trigger_cancel_and_reset_ros_time)
{
  // Arrange — activate ROS time so the clock is fully controlled (no wall-clock noise).
  constexpr int64_t kT0Ns = 1'000'000'000;     // 1s
  constexpr int64_t kPeriodNs = 100'000'000;   // 100ms
  constexpr int64_t kAdvanceNs = 200'000'000;  // 200ms past t0
  auto clock = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
  rcl_clock_t * rcl_clock = clock->get_clock_handle();
  {
    std::lock_guard<std::mutex> lock(clock->get_clock_mutex());
    ASSERT_EQ(rcl_enable_ros_time_override(rcl_clock), RCL_RET_OK);
    ASSERT_EQ(rcl_set_ros_time_override(rcl_clock, kT0Ns), RCL_RET_OK);
  }
  const auto period = rclcpp::Duration(std::chrono::nanoseconds(kPeriodNs));
  bool called = false;
  auto timer = agnocast::create_timer(node.get(), clock, period, [&called]() { called = true; });

  // Act
  auto tut_before_cancel = timer->time_until_trigger();
  timer->cancel();
  auto tut_after_cancel = timer->time_until_trigger();
  timer->reset();
  auto tut_after_reset = timer->time_until_trigger();
  {
    std::lock_guard<std::mutex> lock(clock->get_clock_mutex());
    ASSERT_EQ(rcl_set_ros_time_override(rcl_clock, kT0Ns + kAdvanceNs), RCL_RET_OK);
  }
  auto tut_after_wait = timer->time_until_trigger();

  // Assert — values are exact because ROS time advances only when we say so.
  EXPECT_EQ(tut_before_cancel, std::chrono::nanoseconds(kPeriodNs));
  EXPECT_EQ(tut_after_cancel, std::chrono::nanoseconds::max());
  EXPECT_EQ(tut_after_reset, std::chrono::nanoseconds(kPeriodNs));
  EXPECT_EQ(tut_after_wait, std::chrono::nanoseconds(kPeriodNs - kAdvanceNs));
  EXPECT_FALSE(called);
}

// =========================================
// ID overflow tests
// =========================================

TEST(AllocateTimerIdTest, throws_when_id_has_reserved_epoll_flag_bits)
{
  // Arrange: Set next_timer_id to MAX_TIMER_ID so the next allocation overflows the boundary.
  const uint32_t original = next_timer_id.load();
  next_timer_id.store(MAX_TIMER_ID);

  // Act & Assert
  EXPECT_THROW(allocate_timer_id(), std::runtime_error);

  // Cleanup
  next_timer_id.store(original);
}

TEST(AllocateTimerIdTest, succeeds_just_below_reserved_range)
{
  // Arrange: Set next_timer_id to the maximum valid value.
  const uint32_t original = next_timer_id.load();
  next_timer_id.store(MAX_TIMER_ID - 1);

  // Act & Assert
  EXPECT_EQ(allocate_timer_id(), MAX_TIMER_ID - 1);

  // Cleanup
  next_timer_id.store(original);
}
