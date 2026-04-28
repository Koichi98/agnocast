#pragma once

#include "agnocast_cie_thread_configurator/non_ros_thread_info_ipc.hpp"
#include "rclcpp/rclcpp.hpp"

#include <sys/socket.h>
#include <sys/syscall.h>
#include <sys/un.h>
#include <unistd.h>

#include <cerrno>
#include <cstdio>
#include <cstring>
#include <map>
#include <string>
#include <string_view>
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

enum class ThreadNameValidation {
  kOk,
  kTooLong,
  kEmbeddedNul,
};

// Validates that `thread_name` can be safely embedded in a NonRosThreadInfoMsg wire frame.
// Empty strings are valid (kOk). Callers should invoke this before constructing a
// NonRosThreadInfoMsg directly.
inline ThreadNameValidation validate_thread_name(std::string_view thread_name) noexcept
{
  if (thread_name.size() > kNonRosThreadNameMax) {
    return ThreadNameValidation::kTooLong;
  }
  if (thread_name.find('\0') != std::string_view::npos) {
    return ThreadNameValidation::kEmbeddedNul;
  }
  return ThreadNameValidation::kOk;
}

// Implementation detail of `spawn_non_ros2_thread`; not part of the public API.
// Opens a SOCK_DGRAM AF_UNIX socket and connects it to the daemon's abstract-namespace
// address. Waits up to kSenderMaxConnectWaitIters * kSenderRetryDelay for the daemon to
// start (connect returns ECONNREFUSED until something is bound to that abstract name).
// Returns a connected fd on success (caller must close it) or -1 on permanent failure /
// timeout. `thread_name` is borrowed and used only for log messages; it must remain valid
// for the duration of the call.
inline int open_sender_socket(const char * thread_name) noexcept
{
  const int fd = ::socket(AF_UNIX, SOCK_DGRAM | SOCK_CLOEXEC, 0);
  if (fd == -1) {
    std::fprintf(
      stderr, "[cie_thread_client] [WARN] socket(AF_UNIX) failed for thread '%s': %s\n",
      thread_name, strerror(errno));
    return -1;
  }

  sockaddr_un addr;
  const socklen_t addrlen = fill_abstract_sockaddr(addr, kNonRosThreadInfoSocketName);

  for (int attempt = 0; attempt < kSenderMaxConnectWaitIters;) {
    if (::connect(fd, reinterpret_cast<const sockaddr *>(&addr), addrlen) == 0) {
      return fd;
    }
    const int err = errno;
    // ECONNREFUSED: daemon not yet bound (the common startup race we are polling against).
    // EAGAIN/EWOULDBLOCK: AF_UNIX connect() can hit this on autobind-table exhaustion under
    // burst load. EINTR: retried here too -- under the present design every retryable errno
    // consumes one slot of the kSenderMaxConnectWaitIters budget and sleeps kSenderRetryDelay
    // before the next attempt; a signal storm is therefore bounded by the same wall-clock
    // budget as ECONNREFUSED rather than spinning the loop hot.
    if (err != ECONNREFUSED && err != EINTR && err != EAGAIN && err != EWOULDBLOCK) {
      std::fprintf(
        stderr, "[cie_thread_client] [WARN] connect to '@%s' failed for thread '%s': %s\n",
        kNonRosThreadInfoSocketName, thread_name, strerror(err));
      ::close(fd);
      return -1;
    }
    std::this_thread::sleep_for(kSenderRetryDelay);
    ++attempt;
  }
  std::fprintf(
    stderr,
    "[cie_thread_client] [WARN] No NonRosThreadInfo daemon listening for thread '%s'. "
    "Please run thread_configurator_node if you want to configure thread scheduling.\n",
    thread_name);
  ::close(fd);
  return -1;
}

// Implementation detail of `spawn_non_ros2_thread`; not part of the public API.
// Sends `msg` on the connected `fd` (MSG_DONTWAIT), retrying transient errors up to
// kSenderMaxSendAttempts * kSenderRetryDelay. EAGAIN here means the receiver's SO_RCVBUF
// is full; the default Linux rcvbuf (~200 KiB) holds hundreds of these messages, so the
// retry budget is overwhelmingly defensive. ENOBUFS is also retried: AF_UNIX SOCK_DGRAM
// can return it under transient kernel-memory pressure when allocating an sk_buff fails,
// and dropping the registration outright over a momentary squeeze would silently leave
// the thread unconfigured for its lifetime. EINTR consumes a retry slot too; see
// open_sender_socket for the rationale. Returns true on success, false on permanent error
// or timeout (a WARN is logged in either failure case).
inline bool send_thread_info(
  int fd, const NonRosThreadInfoMsg & msg, const char * thread_name) noexcept
{
  for (int attempt = 0; attempt < kSenderMaxSendAttempts;) {
    const ssize_t n = ::send(fd, &msg, sizeof(msg), MSG_DONTWAIT);
    if (n == static_cast<ssize_t>(sizeof(msg))) {
      return true;
    }
    if (n != -1) {
      // SOCK_DGRAM is atomic per datagram; the kernel never produces partial sends. Treat
      // any other non-(-1) return as a hard failure -- and crucially, do NOT read errno
      // here, it is unspecified when the syscall did not return -1.
      std::fprintf(
        stderr,
        "[cie_thread_client] [WARN] send returned unexpected size %zd for NonRosThreadInfo "
        "(thread '%s'); not retrying.\n",
        n, thread_name);
      return false;
    }
    const int err = errno;
    if (err != EAGAIN && err != EWOULDBLOCK && err != EINTR && err != ENOBUFS) {
      std::fprintf(
        stderr, "[cie_thread_client] [WARN] send failed for NonRosThreadInfo (thread '%s'): %s\n",
        thread_name, strerror(err));
      return false;
    }
    std::this_thread::sleep_for(kSenderRetryDelay);
    ++attempt;
  }
  std::fprintf(
    stderr,
    "[cie_thread_client] [WARN] Timed out sending NonRosThreadInfo for thread '%s' "
    "(receiver buffer full); thread will run unconfigured.\n",
    thread_name);
  return false;
}

// Spawn a thread whose scheduling policy can be managed by cie_thread_configurator.
//
// Contract:
// - thread_name: non-null, NUL-terminated, unique, <= kNonRosThreadNameMax bytes, no
//   embedded NUL. A null pointer is treated as a contract violation: registration is
//   skipped (WARN) and the user function still runs unconfigured.
// - The daemon (prerun_node / thread_configurator_node) must be up within
//   kSenderMaxConnectWaitIters * kSenderRetryDelay (5 s by default). On success, the
//   registration datagram is delivered with up to kSenderMaxSendAttempts *
//   kSenderRetryDelay (500 ms by default) of additional retries if the daemon's receive
//   buffer is momentarily full.
// - The calling process must share the daemon's network namespace (abstract UDS names are
//   netns-scoped; cross-namespace registration is intentionally unreachable).
// - On any of the above failing, registration is silently skipped (WARN) and the user
//   function still runs unconfigured.
// - The user function `f` does not start until after the registration IPC completes (or
//   times out): in the worst case (no daemon, full receive buffer) the spawned thread
//   blocks for up to (kSenderMaxConnectWaitIters + kSenderMaxSendAttempts) * kSenderRetryDelay
//   (~5.5 s by default) before invoking `f`. Callers with hard start-time budgets must
//   ensure the daemon is up before calling.
// - Thread-safe: concurrent calls are permitted.
template <class F, class... Args>
std::thread spawn_non_ros2_thread(const char * thread_name, F && f, Args &&... args)
{
  // Reject nullptr at the API boundary: passing it through to the lambda's
  // `std::string(thread_name)` capture below would invoke undefined behaviour, while every
  // other contract violation (oversize, embedded NUL) is reported as a clean WARN. Convert
  // to an empty string so validate_thread_name's kOk path is reached and the user's `f`
  // still runs (matching the documented "registration silently skipped" semantics).
  if (thread_name == nullptr) {
    std::fprintf(
      stderr,
      "[cie_thread_client] [WARN] spawn_non_ros2_thread called with nullptr thread_name; "
      "skipping NonRosThreadInfo publish.\n");
    thread_name = "";
  }
  std::thread t([thread_name = std::string(thread_name), func = std::forward<F>(f),
                 captured_args = std::make_tuple(std::forward<Args>(args)...)]() mutable {
    switch (validate_thread_name(thread_name)) {
      case ThreadNameValidation::kTooLong:
        std::fprintf(
          stderr,
          "[cie_thread_client] [WARN] Thread name '%s' exceeds %zu bytes; skipping "
          "NonRosThreadInfo publish.\n",
          thread_name.c_str(), kNonRosThreadNameMax);
        break;
      case ThreadNameValidation::kEmbeddedNul:
        std::fprintf(
          stderr,
          "[cie_thread_client] [WARN] Thread name contains an embedded NUL; skipping "
          "NonRosThreadInfo publish.\n");
        break;
      case ThreadNameValidation::kOk: {
        const int fd = open_sender_socket(thread_name.c_str());
        if (fd != -1) {
          // Zero-init is load-bearing: we only memcpy `thread_name.size()` bytes plus a
          // single trailing NUL, so the tail of `msg.thread_name` (and any trailing
          // padding) would otherwise leak uninitialized stack bytes onto the wire.
          NonRosThreadInfoMsg msg = {};
          msg.thread_id = static_cast<int64_t>(syscall(SYS_gettid));
          std::memcpy(msg.thread_name, thread_name.c_str(), thread_name.size());
          msg.thread_name[thread_name.size()] = '\0';
          send_thread_info(fd, msg, thread_name.c_str());
          if (::close(fd) == -1) {
            std::fprintf(stderr, "[cie_thread_client] [WARN] close failed: %s\n", strerror(errno));
          }
        }
        break;
      }
    }

    std::apply(std::move(func), std::move(captured_args));
  });
  return t;
}

}  // namespace agnocast_cie_thread_configurator
