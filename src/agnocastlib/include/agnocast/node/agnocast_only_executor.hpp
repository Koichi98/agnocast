#pragma once

#include "agnocast/agnocast_epoll.hpp"
#include "agnocast/agnocast_executable.hpp"
#include "rclcpp/callback_group.hpp"
#include "rclcpp/future_return_code.hpp"
#include "rclcpp/node_interfaces/node_base_interface.hpp"
#include "rcpputils/scope_exit.hpp"
#include "rcpputils/thread_safety_annotations.hpp"

#include <atomic>
#include <chrono>
#include <future>
#include <list>
#include <map>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <vector>

namespace agnocast
{

using WeakCallbackGroupsToNodesMap = std::map<
  rclcpp::CallbackGroup::WeakPtr, rclcpp::node_interfaces::NodeBaseInterface::WeakPtr,
  std::owner_less<rclcpp::CallbackGroup::WeakPtr>>;

class Node;

class AgnocastOnlyExecutor
{
  static constexpr int kMaxSpinOnceTimeoutMs = 50;

protected:
  std::atomic_bool spinning_;
  int epoll_fd_;
  int shutdown_event_fd_;
  pid_t my_pid_;

  // Lock ordering: When both mutexes are needed, always acquire
  // ready_agnocast_executables_mutex_ before mutex_ to prevent deadlocks.
  std::mutex ready_agnocast_executables_mutex_;
  std::vector<AgnocastExecutable> ready_agnocast_executables_;

  mutable std::mutex mutex_;
  WeakCallbackGroupsToNodesMap weak_groups_associated_with_executor_to_nodes_
    RCPPUTILS_TSA_GUARDED_BY(mutex_);
  WeakCallbackGroupsToNodesMap weak_groups_to_nodes_associated_with_executor_
    RCPPUTILS_TSA_GUARDED_BY(mutex_);
  std::list<rclcpp::node_interfaces::NodeBaseInterface::WeakPtr> weak_nodes_
    RCPPUTILS_TSA_GUARDED_BY(mutex_);

  bool get_next_agnocast_executable(
    AgnocastExecutable & agnocast_executable, const int timeout_ms, bool & shutdown_detected);
  bool get_next_ready_agnocast_executable(AgnocastExecutable & agnocast_executable);
  void execute_agnocast_executable(AgnocastExecutable & agnocast_executable);

  /// Returns true if the callback group is associated with this executor.
  /// Returns false if no groups or nodes have been explicitly associated.
  bool is_callback_group_associated(const rclcpp::CallbackGroup::SharedPtr & group);

  /// Discover callback groups created on associated nodes after add_node() was called.
  void add_callback_groups_from_nodes_associated_to_executor();

public:
  explicit AgnocastOnlyExecutor();

  virtual ~AgnocastOnlyExecutor();

  virtual void spin() = 0;
  virtual void cancel();

  /// Spin (blocking) until the future is complete, it times out waiting, or shutdown is detected.
  /**
   * \param[in] future The future to wait on. If this function returns SUCCESS, the future can be
   *   accessed without blocking (though it may still throw an exception).
   * \param[in] timeout Optional timeout parameter.
   *   `-1` is block forever, `0` is non-blocking.
   *   If the time spent inside the blocking loop exceeds this timeout, return a TIMEOUT return
   *   code.
   * \return The return code, one of `SUCCESS`, `INTERRUPTED`, or `TIMEOUT`.
   *
   * Differences from rclcpp::Executor::spin_until_future_complete():
   *   1. rclcpp checks `rclcpp::ok(this->context_) && spinning` as the loop condition.
   *      AgnocastOnlyExecutor checks `spinning_` and `shutdown_detected` (via epoll/SignalHandler).
   *   2. rclcpp calls `spin_once_impl(timeout_left)` to process one work item.
   *      AgnocastOnlyExecutor uses `need_epoll_updates` check +
   *      `get_next_agnocast_executable` + `execute_agnocast_executable`.
   *   3. rclcpp passes the full remaining timeout to `spin_once_impl`.
   *      AgnocastOnlyExecutor caps the per-iteration epoll timeout to ensure
   *      `need_epoll_updates` is checked periodically.
   */
  template<typename FutureT, typename TimeRepT = int64_t, typename TimeT = std::milli>
  rclcpp::FutureReturnCode spin_until_future_complete(
    const FutureT & future,
    std::chrono::duration<TimeRepT, TimeT> timeout = std::chrono::duration<TimeRepT, TimeT>(-1))
  {
    // Check the future before entering the while loop.
    // If the future is already complete, don't try to spin.
    std::future_status status = future.wait_for(std::chrono::seconds(0));
    if (status == std::future_status::ready) {
      return rclcpp::FutureReturnCode::SUCCESS;
    }

    auto end_time = std::chrono::steady_clock::now();
    std::chrono::nanoseconds timeout_ns =
      std::chrono::duration_cast<std::chrono::nanoseconds>(timeout);
    if (timeout_ns > std::chrono::nanoseconds::zero()) {
      end_time += timeout_ns;
    }
    std::chrono::nanoseconds timeout_left = timeout_ns;

    if (spinning_.exchange(true)) {
      throw std::runtime_error("spin_until_future_complete() called while already spinning");
    }
    RCPPUTILS_SCOPE_EXIT(this->spinning_.store(false););
    while (spinning_.load()) {
      // Handle epoll updates for newly registered subscriptions/timers.
      if (need_epoll_updates.load()) {
        add_callback_groups_from_nodes_associated_to_executor();
        agnocast::prepare_epoll_impl(
          epoll_fd_, my_pid_, ready_agnocast_executables_mutex_, ready_agnocast_executables_,
          [this](const rclcpp::CallbackGroup::SharedPtr & group) {
            return is_callback_group_associated(group);
          });
      }

      // Determine per-iteration timeout.
      // Cap to kMaxSpinOnceTimeoutMs to ensure need_epoll_updates is checked periodically.
      int iter_timeout_ms = kMaxSpinOnceTimeoutMs;
      if (timeout_ns >= std::chrono::nanoseconds::zero()) {
        auto remaining_ms =
          std::chrono::duration_cast<std::chrono::milliseconds>(timeout_left).count();
        iter_timeout_ms =
          static_cast<int>(std::min(static_cast<int64_t>(kMaxSpinOnceTimeoutMs), remaining_ms));
      }

      // Do one item of work.
      AgnocastExecutable agnocast_executable;
      bool shutdown_detected = false;
      if (get_next_agnocast_executable(agnocast_executable, iter_timeout_ms, shutdown_detected)) {
        execute_agnocast_executable(agnocast_executable);
      }

      if (shutdown_detected) {
        break;
      }

      // Check if the future is set, return SUCCESS if it is.
      status = future.wait_for(std::chrono::seconds(0));
      if (status == std::future_status::ready) {
        return rclcpp::FutureReturnCode::SUCCESS;
      }
      // If the original timeout is < 0, then this is blocking, never TIMEOUT.
      if (timeout_ns < std::chrono::nanoseconds::zero()) {
        continue;
      }
      // Otherwise check if we still have time to wait, return TIMEOUT if not.
      auto now = std::chrono::steady_clock::now();
      if (now >= end_time) {
        return rclcpp::FutureReturnCode::TIMEOUT;
      }
      // Subtract the elapsed time from the original timeout.
      timeout_left = std::chrono::duration_cast<std::chrono::nanoseconds>(end_time - now);
    }

    // The future did not complete before spinning was stopped, return INTERRUPTED.
    return rclcpp::FutureReturnCode::INTERRUPTED;
  }

  void add_callback_group(
    rclcpp::CallbackGroup::SharedPtr group_ptr,
    rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_ptr, bool notify = true);

  void remove_callback_group(rclcpp::CallbackGroup::SharedPtr group_ptr, bool notify = true);

  std::vector<rclcpp::CallbackGroup::WeakPtr> get_all_callback_groups();

  std::vector<rclcpp::CallbackGroup::WeakPtr> get_manually_added_callback_groups();

  std::vector<rclcpp::CallbackGroup::WeakPtr> get_automatically_added_callback_groups_from_nodes();

  void add_node(rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_ptr, bool notify = true);

  void add_node(const std::shared_ptr<agnocast::Node> & node, bool notify = true);

  void remove_node(
    rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_ptr, bool notify = true);

  void remove_node(const std::shared_ptr<agnocast::Node> & node, bool notify = true);
};

}  // namespace agnocast
