#include "node_for_executor_test.hpp"

#include <sys/eventfd.h>
#include <unistd.h>

NodeForExecutorTest::NodeForExecutorTest(
  const int64_t num_agnocast_sub_cbs, const int64_t num_ros2_sub_cbs,
  const int64_t num_agnocast_cbs_to_be_added, const std::chrono::milliseconds pub_period,
  const std::chrono::milliseconds cb_exec_time, const std::string cbg_type)
: Node("node_for_executor_test")
{
  if (cbg_type == "mutually_exclusive") {
    agnocast_common_cbg_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    ros2_common_cbg_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  } else if (cbg_type == "reentrant") {
    agnocast_common_cbg_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    ros2_common_cbg_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  }
  cb_exec_time_ = cb_exec_time;

  // For Agnocast
  agnocast_timer_ =
    create_wall_timer(pub_period, std::bind(&NodeForExecutorTest::agnocast_timer_cb, this));
  for (int64_t i = 0; i < num_agnocast_sub_cbs; i++) {
    add_agnocast_sub_cb();
  }
  num_total_agnocast_sub_cbs_ = num_agnocast_sub_cbs + num_agnocast_cbs_to_be_added;
  agnocast_sub_cbs_called_ = std::make_unique<std::atomic<bool>[]>(num_total_agnocast_sub_cbs_);
  for (size_t i = 0; i < num_total_agnocast_sub_cbs_; i++) {
    agnocast_sub_cbs_called_[i].store(false, std::memory_order_relaxed);
  }

  // For ROS 2
  ros2_pub_ = create_publisher<std_msgs::msg::Bool>(ros2_topic_name_, 1);
  ros2_timer_ = create_wall_timer(pub_period, std::bind(&NodeForExecutorTest::ros2_timer_cb, this));
  for (int64_t i = 0; i < num_ros2_sub_cbs; i++) {
    rclcpp::SubscriptionOptions options;
    if (ros2_common_cbg_) {
      options.callback_group = ros2_common_cbg_;
    } else {  // individual
      options.callback_group = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    }
    ros2_subs_.push_back(create_subscription<std_msgs::msg::Bool>(
      ros2_topic_name_, 1,
      [this, i](const std::shared_ptr<const std_msgs::msg::Bool> msg) { ros2_sub_cb(msg, i); },
      options));
  }
  num_ros2_sub_cbs_ = num_ros2_sub_cbs;
  ros2_sub_cbs_called_ = std::make_unique<std::atomic<bool>[]>(num_ros2_sub_cbs_);
  for (size_t i = 0; i < num_ros2_sub_cbs_; i++) {
    ros2_sub_cbs_called_[i].store(false, std::memory_order_relaxed);
  }
}

NodeForExecutorTest::~NodeForExecutorTest()
{
  for (auto & efd : eventfds_) {
    if (efd >= 0) {
      close(efd);
    }
  }
}

void NodeForExecutorTest::add_agnocast_sub_cb()
{
  int64_t cb_i = eventfds_.size();
  rclcpp::SubscriptionOptions options;
  auto cbg = (agnocast_common_cbg_)
               ? agnocast_common_cbg_
               : create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  options.callback_group = cbg;
  auto efd = open_eventfd_for_receiver();
  std::function<void(const agnocast::ipc_shared_ptr<std_msgs::msg::Bool> &)> callback =
    [this, cb_i](const agnocast::ipc_shared_ptr<std_msgs::msg::Bool> & msg) {
      agnocast_sub_cb(msg, cb_i);
    };
  const bool is_transient_local = false;
  agnocast::register_callback<std_msgs::msg::Bool>(
    callback, agnocast_topic_name_, cb_i, is_transient_local, efd, cbg);
}

// NOTE: If the implementation of agnocast is changed, this function does not
// necessarily have to be changed as well, because this test is for the Executor.
int NodeForExecutorTest::open_eventfd_for_receiver()
{
  int efd = eventfd(0, EFD_NONBLOCK | EFD_CLOEXEC);
  if (efd == -1) {
    std::cerr << "eventfd creation failed: " << strerror(errno) << std::endl;
    exit(EXIT_FAILURE);
  }
  eventfds_.push_back(efd);

  return efd;
}

void NodeForExecutorTest::dummy_work(std::chrono::milliseconds exec_time)
{
  auto start = std::chrono::high_resolution_clock::now();
  while (std::chrono::high_resolution_clock::now() - start < exec_time) {
  }
}

// NOTE: If the implementation of agnocast is changed, this function does not
// necessarily have to be changed as well, because this test is for the Executor.
void NodeForExecutorTest::agnocast_timer_cb()
{
  // Write to eventfds to trigger callbacks
  for (auto & efd : eventfds_) {
    uint64_t val = 1;
    if (write(efd, &val, sizeof(val)) == -1) {
      if (errno != EAGAIN) {
        std::cerr << "eventfd write failed: " << strerror(errno) << std::endl;
        exit(EXIT_FAILURE);
      }
    }
  }

  // Add new agnocast sub callbacks
  if (eventfds_.size() < num_total_agnocast_sub_cbs_) {
    add_agnocast_sub_cb();
  }
}

void NodeForExecutorTest::agnocast_sub_cb(
  [[maybe_unused]] const agnocast::ipc_shared_ptr<std_msgs::msg::Bool> & msg, int64_t cb_i)
{
  std::unique_lock<std::mutex> lock(mutex_for_agnocast_cbg_, std::try_to_lock);
  if (!lock.owns_lock()) {
    is_mutually_exclusive_agnocast_ = false;
  }

  agnocast_sub_cbs_called_[cb_i].store(true, std::memory_order_release);
  dummy_work(cb_exec_time_);
}

void NodeForExecutorTest::ros2_timer_cb()
{
  std_msgs::msg::Bool msg;
  msg.data = true;
  ros2_pub_->publish(msg);
}

void NodeForExecutorTest::ros2_sub_cb(
  [[maybe_unused]] const std::shared_ptr<const std_msgs::msg::Bool> & msg, int64_t cb_i)
{
  std::unique_lock<std::mutex> lock(mutex_for_ros2_cbg_, std::try_to_lock);
  if (!lock.owns_lock()) {
    is_mutually_exclusive_ros2_ = false;
  }

  ros2_sub_cbs_called_[cb_i].store(true, std::memory_order_release);
  dummy_work(cb_exec_time_);
}

bool NodeForExecutorTest::is_all_ros2_sub_cbs_called() const
{
  for (size_t i = 0; i < num_ros2_sub_cbs_; i++) {
    if (!ros2_sub_cbs_called_[i].load(std::memory_order_acquire)) {
      return false;
    }
  }
  return true;
}

bool NodeForExecutorTest::is_all_agnocast_sub_cbs_called() const
{
  for (size_t i = 0; i < num_total_agnocast_sub_cbs_; i++) {
    if (!agnocast_sub_cbs_called_[i].load(std::memory_order_acquire)) {
      return false;
    }
  }
  return true;
}

bool NodeForExecutorTest::is_mutually_exclusive_agnocast() const
{
  return is_mutually_exclusive_agnocast_;
}

bool NodeForExecutorTest::is_mutually_exclusive_ros2() const
{
  return is_mutually_exclusive_ros2_;
}
