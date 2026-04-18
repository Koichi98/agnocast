#include <agnocast/agnocast.hpp>
#include <agnocast/agnocast_callback_isolated_executor.hpp>

#include <agnocast_cie_config_msgs/msg/callback_group_info.hpp>
#include <std_msgs/msg/bool.hpp>

#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <memory>
#include <mutex>
#include <thread>
#include <vector>

class DummyNode : public rclcpp::Node
{
public:
  DummyNode() : Node("dummy_node")
  {
    rclcpp::CallbackGroup::SharedPtr callback_group_1 =
      this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    rclcpp::PublisherOptions pub_options_1;
    pub_options_1.callback_group = callback_group_1;
    ros2_pub_1_ = this->create_publisher<std_msgs::msg::Bool>("/test_topic_1", 10, pub_options_1);

    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(100),
      [this]() {
        auto msg = std_msgs::msg::Bool();
        msg.data = true;
        ros2_pub_1_->publish(msg);
        published_ = true;
      },
      callback_group_1);

    rclcpp::CallbackGroup::SharedPtr callback_group_2 =
      this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::SubscriptionOptions sub_options_1;
    sub_options_1.callback_group = callback_group_2;
    ros2_sub_1_ = this->create_subscription<std_msgs::msg::Bool>(
      "/test_topic_1", 10,
      [this](const std_msgs::msg::Bool::SharedPtr msg) {
        (void)msg;
        subscribed_ = true;
      },
      sub_options_1);
  }

  bool is_published() const { return published_.load(); }
  bool is_subscribed() const { return subscribed_.load(); }

private:
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr ros2_pub_1_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr ros2_sub_1_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::atomic<bool> published_{false};
  std::atomic<bool> subscribed_{false};
};

class CallbackGroupInfoReceiverNode : public rclcpp::Node
{
public:
  CallbackGroupInfoReceiverNode()
  : Node("callback_group_info_receiver", "/agnocast_cie_thread_configurator")
  {
    subscription_ = this->create_subscription<agnocast_cie_config_msgs::msg::CallbackGroupInfo>(
      "/agnocast_cie_thread_configurator/callback_group_info", rclcpp::QoS(1000).keep_all(),
      [this](const agnocast_cie_config_msgs::msg::CallbackGroupInfo::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        received_messages_.push_back(*msg);
      });
  }

  std::vector<agnocast_cie_config_msgs::msg::CallbackGroupInfo> get_received_messages()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return received_messages_;
  }

private:
  rclcpp::Subscription<agnocast_cie_config_msgs::msg::CallbackGroupInfo>::SharedPtr subscription_;
  std::mutex mutex_;
  std::vector<agnocast_cie_config_msgs::msg::CallbackGroupInfo> received_messages_;
};

// Node that creates a ROS callback group dynamically after construction.
class DynamicRosGroupNode : public rclcpp::Node
{
public:
  DynamicRosGroupNode() : Node("dynamic_ros_group_node") {}

  void create_ros_callback_group()
  {
    // Must store the callback group as a member: NodeBase::callback_groups_ holds WeakPtr,
    // so the group would be destroyed if only a local SharedPtr existed.
    cbg_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    timer_ = create_wall_timer(
      std::chrono::milliseconds(100), [this]() { timer_fired_.store(true); }, cbg_);
  }

  bool is_timer_fired() const { return timer_fired_.load(); }

private:
  rclcpp::CallbackGroup::SharedPtr cbg_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::atomic<bool> timer_fired_{false};
};

// Node that creates an agnocast callback group dynamically after construction.
// A timer in the default group sends messages to the mqueue, which triggers the
// agnocast subscription callback registered in the dynamically created group.
class DynamicAgnocastGroupNode : public rclcpp::Node
{
public:
  DynamicAgnocastGroupNode() : Node("dynamic_agnocast_group_node")
  {
    timer_ = create_wall_timer(std::chrono::milliseconds(100), [this]() {
      if (mq_receiver_ == static_cast<mqd_t>(-1)) return;

      if (mq_sender_ == static_cast<mqd_t>(-1)) {
        mq_sender_ = mq_open(mq_name_.c_str(), O_WRONLY | O_NONBLOCK);
        if (mq_sender_ == static_cast<mqd_t>(-1)) return;
      }

      agnocast::MqMsgAgnocast mq_msg = {};
      mq_send(mq_sender_, reinterpret_cast<char *>(&mq_msg), 0 /*msg_len*/, 0);
    });
  }

  ~DynamicAgnocastGroupNode()
  {
    if (mq_sender_ != static_cast<mqd_t>(-1)) {
      mq_close(mq_sender_);
    }
    if (mq_receiver_ != static_cast<mqd_t>(-1)) {
      mq_close(mq_receiver_);
      mq_unlink(mq_name_.c_str());
    }
  }

  void create_agnocast_callback_group()
  {
    // Must store the callback group as a member: NodeBase::callback_groups_ holds WeakPtr,
    // so the group would be destroyed if only a local SharedPtr existed.
    cbg_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    struct mq_attr attr = {};
    attr.mq_flags = 0;
    attr.mq_msgsize = sizeof(agnocast::MqMsgAgnocast);
    attr.mq_curmsgs = 0;
    attr.mq_maxmsg = 1;

    const int mq_mode = 0666;
    mq_receiver_ = mq_open(mq_name_.c_str(), O_CREAT | O_RDONLY | O_NONBLOCK, mq_mode, &attr);
    if (mq_receiver_ == static_cast<mqd_t>(-1)) {
      std::cerr << "mq_open failed for mq_name='" << mq_name_ << "': " << strerror(errno)
                << std::endl;
      FAIL();
    }

    std::function<void(const agnocast::ipc_shared_ptr<std_msgs::msg::Bool> &)> callback =
      [this]([[maybe_unused]] const agnocast::ipc_shared_ptr<std_msgs::msg::Bool> & msg) {
        agnocast_cb_called_.store(true);
      };
    const bool is_transient_local = false;
    agnocast::register_callback<std_msgs::msg::Bool>(
      callback, agnocast_topic_name_, 0, is_transient_local, mq_receiver_, cbg_);
  }

  bool is_agnocast_cb_called() const { return agnocast_cb_called_.load(); }

private:
  rclcpp::CallbackGroup::SharedPtr cbg_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::string agnocast_topic_name_ = "/test_dynamic_agnocast";
  std::string mq_name_ = "/test_dynamic_agnocast@0";
  mqd_t mq_receiver_{static_cast<mqd_t>(-1)};
  mqd_t mq_sender_{static_cast<mqd_t>(-1)};
  std::atomic<bool> agnocast_cb_called_{false};
};

class CallbackIsolatedExecutorTest : public ::testing::Test
{
protected:
  void SetUp() override { rclcpp::init(0, nullptr); }
  void TearDown() override { rclcpp::shutdown(); }
};

TEST_F(CallbackIsolatedExecutorTest, test_spin_publishes_callback_group_info)
{
  // Arrange
  auto receiver_node = std::make_shared<CallbackGroupInfoReceiverNode>();
  rclcpp::executors::SingleThreadedExecutor receiver_executor;
  receiver_executor.add_node(receiver_node);
  std::thread receiver_thread([&receiver_executor]() { receiver_executor.spin(); });

  auto test_node = std::make_shared<DummyNode>();
  auto callback_isolated_executor = std::make_shared<agnocast::CallbackIsolatedAgnocastExecutor>();
  callback_isolated_executor->add_node(test_node);

  // Act
  std::thread callback_isolated_thread(
    [&callback_isolated_executor]() { callback_isolated_executor->spin(); });

  while (test_node->is_published() == false || test_node->is_subscribed() == false) {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
  callback_isolated_executor->cancel();
  if (callback_isolated_thread.joinable()) {
    callback_isolated_thread.join();
  }
  receiver_executor.cancel();
  if (receiver_thread.joinable()) {
    receiver_thread.join();
  }

  // Assert
  ASSERT_EQ(receiver_node->get_received_messages().size(), 3u);  // 1 default + 2 created
}

// Verify that the CIE monitoring loop detects a ROS callback group created after spin() starts,
// spawns a child executor for it, and publishes its CallbackGroupInfo.
TEST_F(CallbackIsolatedExecutorTest, test_ros2_callback_group_added_after_spin)
{
  // Arrange: set up a receiver to capture CallbackGroupInfo messages
  auto receiver_node = std::make_shared<CallbackGroupInfoReceiverNode>();
  rclcpp::executors::SingleThreadedExecutor receiver_executor;
  receiver_executor.add_node(receiver_node);
  std::thread receiver_thread([&receiver_executor]() { receiver_executor.spin(); });

  // Create a node with only the default callback group
  auto test_node = std::make_shared<DynamicRosGroupNode>();
  auto cie = std::make_shared<agnocast::CallbackIsolatedAgnocastExecutor>();
  cie->add_node(test_node);

  // Act: start spinning - only the default callback group should be detected initially
  std::thread cie_thread([&cie]() { cie->spin(); });

  // Helper to ensure cleanup always happens
  auto cleanup = [&]() {
    cie->cancel();
    if (cie_thread.joinable()) cie_thread.join();
    receiver_executor.cancel();
    if (receiver_thread.joinable()) receiver_thread.join();
  };

  constexpr auto timeout = std::chrono::seconds(10);

  // Wait for the initial callback group info to be published (1 default group)
  auto start_time = std::chrono::steady_clock::now();
  while (receiver_node->get_received_messages().size() < 1u) {
    if (std::chrono::steady_clock::now() - start_time >= timeout) {
      cleanup();
      FAIL() << "Timed out waiting for initial callback group info";
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  // Create a new ROS callback group after spin() has started
  test_node->create_ros_callback_group();

  // Wait for the dynamically added group to be detected and the timer to fire
  start_time = std::chrono::steady_clock::now();
  while (!test_node->is_timer_fired() || receiver_node->get_received_messages().size() < 2u) {
    if (std::chrono::steady_clock::now() - start_time >= timeout) {
      auto msgs = receiver_node->get_received_messages().size();
      auto fired = test_node->is_timer_fired();
      cleanup();
      FAIL() << "Timed out: timer_fired=" << fired << ", callback_group_info_count=" << msgs;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  cleanup();

  // Assert: 1 default + 1 dynamically added = 2 callback group info messages
  ASSERT_EQ(receiver_node->get_received_messages().size(), 2u);
  EXPECT_TRUE(test_node->is_timer_fired());
}

// Verify that the CIE monitoring loop detects an agnocast callback group created after spin()
// starts, spawns a SingleThreadedAgnocastExecutor for it, and the agnocast callback executes.
TEST_F(CallbackIsolatedExecutorTest, test_agnocast_callback_group_added_after_spin)
{
  // Arrange: set up a receiver to capture CallbackGroupInfo messages
  auto receiver_node = std::make_shared<CallbackGroupInfoReceiverNode>();
  rclcpp::executors::SingleThreadedExecutor receiver_executor;
  receiver_executor.add_node(receiver_node);
  std::thread receiver_thread([&receiver_executor]() { receiver_executor.spin(); });

  // Create a node with a timer (default group) that will send to mqueue after the agnocast
  // callback group is created
  auto test_node = std::make_shared<DynamicAgnocastGroupNode>();
  auto cie = std::make_shared<agnocast::CallbackIsolatedAgnocastExecutor>();
  cie->add_node(test_node);

  // Act: start spinning - only the default callback group should be detected initially
  std::thread cie_thread([&cie]() { cie->spin(); });

  // Helper to ensure cleanup always happens
  auto cleanup = [&]() {
    cie->cancel();
    if (cie_thread.joinable()) cie_thread.join();
    receiver_executor.cancel();
    if (receiver_thread.joinable()) receiver_thread.join();
  };

  constexpr auto timeout = std::chrono::seconds(10);

  // Wait for the initial callback group info to be published (1 default group)
  auto start_time = std::chrono::steady_clock::now();
  while (receiver_node->get_received_messages().size() < 1u) {
    if (std::chrono::steady_clock::now() - start_time >= timeout) {
      cleanup();
      FAIL() << "Timed out waiting for initial callback group info";
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  // Create a new agnocast callback group after spin() has started
  test_node->create_agnocast_callback_group();

  // Wait for the dynamically added group to be detected and the agnocast callback to execute
  start_time = std::chrono::steady_clock::now();
  while (!test_node->is_agnocast_cb_called() ||
         receiver_node->get_received_messages().size() < 2u) {
    if (std::chrono::steady_clock::now() - start_time >= timeout) {
      auto msgs = receiver_node->get_received_messages().size();
      auto called = test_node->is_agnocast_cb_called();
      cleanup();
      FAIL() << "Timed out: agnocast_cb_called=" << called
             << ", callback_group_info_count=" << msgs;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  cleanup();

  // Assert: 1 default + 1 dynamically added = 2 callback group info messages
  ASSERT_EQ(receiver_node->get_received_messages().size(), 2u);
  EXPECT_TRUE(test_node->is_agnocast_cb_called());
}
