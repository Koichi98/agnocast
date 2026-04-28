#pragma once

#include "agnocast_cie_thread_configurator/non_ros_thread_info_ipc.hpp"
#include "rclcpp/rclcpp.hpp"

#include <sys/syscall.h>
#include <unistd.h>

#include <cstdio>
#include <cstring>
#include <map>
#include <string>
#include <thread>
#include <tuple>
#include <utility>

namespace agnocast_cie_thread_configurator
{

// Get hardware information from lscpu command
std::map<std::string, std::string> get_hardware_info();

// Get default domain ID from ROS_DOMAIN_ID environment variable
size_t get_default_domain_id();

// Create a node for a different domain
rclcpp::Node::SharedPtr create_node_for_domain(size_t domain_id);

// Spawn a thread whose scheduling policy can be managed by cie_thread_configurator.
//
// Contract:
// - thread_name: non-null, NUL-terminated, unique among threads managed by
//   cie_thread_configurator, <= kNonRosThreadNameMax bytes, no embedded NUL.
// - The daemon (prerun_node / thread_configurator_node) must be up within
//   ~5 s of this call. If not, registration is silently skipped (WARN) and
//   the user function still runs unconfigured.
// - The calling process must share the daemon's network namespace (abstract
//   UDS names are netns-scoped).
// - Thread-safe: concurrent calls are permitted.
template <class F, class... Args>
std::thread spawn_non_ros2_thread(const char * thread_name, F && f, Args &&... args)
{
  // Reject nullptr at the API boundary: passing it through to the lambda's
  // std::string(thread_name) capture below would invoke undefined behaviour,
  // while every other contract violation is reported as a clean WARN.
  if (thread_name == nullptr) {
    std::fprintf(
      stderr,
      "[cie_thread_client] [WARN] spawn_non_ros2_thread called with nullptr thread_name; "
      "skipping NonRosThreadInfo registration.\n");
    thread_name = "";
  }
  std::thread t([thread_name = std::string(thread_name), func = std::forward<F>(f),
                 captured_args = std::make_tuple(std::forward<Args>(args)...)]() mutable {
    switch (validate_thread_name(thread_name)) {
      case ThreadNameValidation::kTooLong:
        std::fprintf(
          stderr,
          "[cie_thread_client] [WARN] thread name '%s' exceeds %zu bytes; skipping "
          "NonRosThreadInfo registration.\n",
          thread_name.c_str(), kNonRosThreadNameMax);
        break;
      case ThreadNameValidation::kEmbeddedNul:
        std::fprintf(
          stderr,
          "[cie_thread_client] [WARN] thread name contains an embedded NUL; skipping "
          "NonRosThreadInfo registration.\n");
        break;
      case ThreadNameValidation::kOk: {
        // Empty thread_name only reaches here from the nullptr->"" remap above,
        // which already logged a WARN and decided to skip; sending an empty-name
        // datagram would just produce confusing "yaml does not contain configuration
        // for name=" noise on the daemon side.
        if (thread_name.empty()) break;
        const int fd = open_sender_socket(thread_name.c_str());
        if (fd != -1) {
          // Zero-init covers the tail of msg.thread_name so uninitialized stack
          // bytes do not leak onto the wire after the memcpy of thread_name.size()
          // bytes; the trailing NUL is supplied by the same zero-init.
          NonRosThreadInfoMsg msg = {};
          msg.thread_id = static_cast<int64_t>(syscall(SYS_gettid));
          std::memcpy(msg.thread_name, thread_name.c_str(), thread_name.size());
          send_thread_info(fd, msg, thread_name.c_str());
          ::close(fd);
        }
        break;
      }
    }

    std::apply(std::move(func), std::move(captured_args));
  });
  return t;
}

}  // namespace agnocast_cie_thread_configurator
