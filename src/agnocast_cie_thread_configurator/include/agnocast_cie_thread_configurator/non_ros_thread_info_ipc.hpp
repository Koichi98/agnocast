#pragma once

#if !defined(__linux__)
#error \
  "agnocast_cie_thread_configurator requires Linux: AF_UNIX abstract-namespace addressing, " \
  "epoll(7), and eventfd(2) are Linux-specific."
#endif

#include <rclcpp/clock.hpp>
#include <rclcpp/logger.hpp>

#include <sys/socket.h>
#include <sys/un.h>

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <functional>
#include <string_view>
#include <thread>
#include <type_traits>

namespace agnocast_cie_thread_configurator
{

// Wire-format constants: the sender (spawn_non_ros2_thread) and the receiver
// (NonRosThreadInfoIpcServer) must agree on these byte-for-byte. Changing any
// of them is an ABI break that requires both sides to be rebuilt together.
inline constexpr char kNonRosThreadInfoSocketName[] = "agnocast_cie_non_ros_thread_info";
inline constexpr size_t kNonRosThreadNameMax = 63;  // bytes excluding trailing NUL

// Guarantee the abstract address fits sun_path so fill_abstract_sockaddr's
// memcpy cannot overflow. The +1 accounts for the leading '\0' marker that
// pins an address to the abstract namespace.
static_assert(
  sizeof(kNonRosThreadInfoSocketName) <= sizeof(sockaddr_un::sun_path),
  "abstract socket name does not fit in sockaddr_un::sun_path");

// SOCK_DGRAM preserves record boundaries, so each send/recv carries exactly
// one struct. thread_id is the Linux TID (SYS_gettid), not the PID.
struct NonRosThreadInfoMsg
{
  int64_t thread_id;
  char thread_name[kNonRosThreadNameMax + 1];
};
static_assert(std::is_trivially_copyable<NonRosThreadInfoMsg>::value);
// Pin the on-wire layout: a future addition / reorder would silently break
// the size check in the receiver loop while looking compatible on both sides
// if rebuilt together.
static_assert(
  offsetof(NonRosThreadInfoMsg, thread_name) == sizeof(int64_t),
  "NonRosThreadInfoMsg has unexpected padding before thread_name");
static_assert(
  sizeof(NonRosThreadInfoMsg) == sizeof(int64_t) + (kNonRosThreadNameMax + 1),
  "NonRosThreadInfoMsg wire size changed; bump the protocol on both sides");

// Fill `addr` to address the abstract-namespace UDS named `name` and return
// the addrlen the kernel expects. The kernel keys abstract names by the exact
// byte range [sun_path, sun_path + addrlen - offsetof(sun_path)], so do NOT
// NUL-terminate.
// Precondition: strlen(name) + 1 <= sizeof(sockaddr_un::sun_path).
inline socklen_t fill_abstract_sockaddr(sockaddr_un & addr, const char * name) noexcept
{
  std::memset(&addr, 0, sizeof(addr));
  addr.sun_family = AF_UNIX;
  const size_t name_len = std::strlen(name);
  // sun_path[0] = '\0' marks an abstract address; the visible name starts at sun_path[1].
  std::memcpy(addr.sun_path + 1, name, name_len);
  return static_cast<socklen_t>(offsetof(sockaddr_un, sun_path) + 1 + name_len);
}

enum class ThreadNameValidation { kOk, kTooLong, kEmbeddedNul };

// Validates that `thread_name` can be safely embedded in a
// NonRosThreadInfoMsg wire frame. Returns kOk if the name fits in
// kNonRosThreadNameMax bytes and contains no embedded NUL bytes; otherwise
// the relevant error code. An empty string is kOk.
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

// Sender-side retry budgets. Increasing kSenderMaxConnectWaitIters tolerates
// slower daemon startup (e.g., loaded CI hosts) at the cost of delaying the
// user function in the "daemon absent" failure mode. kSenderMaxSendAttempts
// is overwhelmingly defensive: the default Linux rcvbuf holds hundreds of
// these messages.
inline constexpr int kSenderMaxConnectWaitIters = 500;  // 5 s daemon-up wait
inline constexpr int kSenderMaxSendAttempts = 50;       // 500 ms EAGAIN budget
inline constexpr std::chrono::milliseconds kSenderRetryDelay{10};

// Opens a SOCK_DGRAM AF_UNIX socket and connects to the daemon's
// abstract-namespace address. Waits up to
// kSenderMaxConnectWaitIters * kSenderRetryDelay for the daemon to start
// (connect returns ECONNREFUSED until something is bound).
// Returns a connected fd on success (caller must close it) or -1 on
// permanent failure / timeout. `thread_name` is borrowed and used only for
// log messages; it must remain valid for the duration of the call.
int open_sender_socket(const char * thread_name) noexcept;

// Sends `msg` on the connected `fd` (MSG_DONTWAIT), retrying transient
// errors up to kSenderMaxSendAttempts * kSenderRetryDelay. Returns true on
// success, false on permanent error or timeout (a WARN is logged on failure).
bool send_thread_info(int fd, const NonRosThreadInfoMsg & msg, const char * thread_name) noexcept;

// Receives NonRosThreadInfoMsg datagrams on the abstract-namespace UDS named
// kNonRosThreadInfoSocketName. At most one server per network namespace per
// host (abstract names are netns-scoped).
//
// Callbacks fire on the receiver thread, so callers MUST synchronize any
// state shared with other threads. Malformed datagrams (wrong size),
// transient recv errors, and exceptions escaping the user callback are all
// logged (throttled) and dropped without tearing down the receiver thread.
//
// Construction throws std::system_error on any kernel-resource failure
// (socket/bind/eventfd/epoll_create1/epoll_ctl). The most operationally
// relevant case is EADDRINUSE on bind, which means another agnocast_cie
// thread-info daemon is already bound in this network namespace.
//
// Non-copyable and non-movable: owns a std::thread and three raw fds. Hold
// instances via std::unique_ptr (as both daemon nodes do) when reseat-ability
// is needed.
class NonRosThreadInfoIpcServer
{
public:
  // Invoked on the internal receiver thread, never on an executor thread.
  using Callback = std::function<void(const NonRosThreadInfoMsg &)>;

  // `logger` is stored by value; it must be associated with a live rcl
  // context for the lifetime of this object because the receiver thread
  // emits RCLCPP_FATAL / RCLCPP_ERROR_THROTTLE through it.
  NonRosThreadInfoIpcServer(rclcpp::Logger logger, Callback cb);
  ~NonRosThreadInfoIpcServer();

  NonRosThreadInfoIpcServer(const NonRosThreadInfoIpcServer &) = delete;
  NonRosThreadInfoIpcServer & operator=(const NonRosThreadInfoIpcServer &) = delete;

private:
  void run();
  bool epoll_add(int fd) noexcept;
  void cleanup_fds() noexcept;

  rclcpp::Logger logger_;
  Callback cb_;
  int sock_fd_ = -1;
  int shutdown_fd_ = -1;
  int epoll_fd_ = -1;
  rclcpp::Clock throttle_clock_{RCL_STEADY_TIME};
  std::thread thread_;
};

}  // namespace agnocast_cie_thread_configurator
