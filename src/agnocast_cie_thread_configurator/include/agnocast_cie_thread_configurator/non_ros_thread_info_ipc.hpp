#pragma once

#if !defined(__linux__)
#error \
  "agnocast_cie_thread_configurator requires Linux: AF_UNIX abstract-namespace addressing, epoll(7), and eventfd(2) are Linux-specific."
#endif

#include <rclcpp/clock.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>

#include <rcl/time.h>
#include <sys/epoll.h>
#include <sys/eventfd.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>

#include <cerrno>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <exception>
#include <functional>
#include <stdexcept>
#include <string>
#include <system_error>
#include <thread>
#include <type_traits>
#include <utility>

namespace agnocast_cie_thread_configurator
{

// Wire-format constants: the sender (spawn_non_ros2_thread) and the receiver
// (NonRosThreadInfoIpcServer) must agree on these values byte-for-byte. Changing either
// is an ABI break that requires both sides to be rebuilt together.
//
// Abstract-namespace AF_UNIX address (Linux extension): sun_path[0] is set to '\0' and the
// visible name follows starting at sun_path[1]. Abstract names live in the network namespace,
// not the filesystem, so they leave no stale entry behind when a process exits -- closing the
// bound socket releases the name immediately.
inline constexpr char kNonRosThreadInfoSocketName[] = "agnocast_cie_non_ros_thread_info";
// Maximum thread-name length in bytes, excluding the NUL terminator. NonRosThreadInfoMsg
// reserves `kNonRosThreadNameMax + 1` bytes for the name field so a full-width name still
// has room for the trailing NUL pinned by the receiver in run().
inline constexpr size_t kNonRosThreadNameMax = 255;

// Guarantee the abstract address used here fits sun_path so fill_abstract_sockaddr's memcpy
// cannot overflow. The +1 accounts for the leading '\0' byte that marks an abstract name;
// sizeof(kNonRosThreadInfoSocketName) already includes the C-string trailing NUL, which is
// not transmitted on the wire but matches the +1 we add in the address layout.
static_assert(
  sizeof(kNonRosThreadInfoSocketName) <= sizeof(sockaddr_un::sun_path),
  "abstract socket name does not fit in sockaddr_un::sun_path");

// Sender-side retry budgets. Tuning trade-off: increasing kSenderMaxConnectWaitIters tolerates
// slower daemon startup (e.g., loaded CI hosts) at the cost of delaying the user function in
// the "daemon absent" failure mode; decreasing it surfaces "no daemon" sooner but risks
// false-negative startup races. kSenderMaxSendAttempts is overwhelmingly defensive (the
// default rcvbuf holds hundreds of these messages); shrinking it is generally safe.
inline constexpr int kSenderMaxConnectWaitIters = 500;  // 5 s daemon-up wait
inline constexpr int kSenderMaxSendAttempts = 50;       // 500 ms EAGAIN budget
// Shared polling interval used by both retry budgets above; tuning this scales both
// the daemon-up wait and the EAGAIN retry budget proportionally.
inline constexpr std::chrono::milliseconds kSenderRetryDelay{10};

// Implementation detail; not part of the public API. Callers outside this header should use
// `kNonRosThreadInfoSocketName` together with NonRosThreadInfoIpcServer / open_sender_socket
// rather than building their own sockaddr.
//
// Fill `addr` to address the abstract-namespace UDS named `name` and return the addrlen the
// kernel expects. The kernel keys abstract names by the exact byte range
// [sun_path, sun_path + addrlen - offsetof(sun_path)], so do NOT NUL-terminate.
// Precondition: strlen(name) + 1 <= sizeof(sockaddr_un::sun_path) (~108 bytes on Linux);
// behaviour is undefined if the name is longer.
inline socklen_t fill_abstract_sockaddr(sockaddr_un & addr, const char * name) noexcept
{
  std::memset(&addr, 0, sizeof(addr));
  addr.sun_family = AF_UNIX;
  const size_t name_len = std::strlen(name);
  // sun_path[0] = '\0' marks an abstract address; the visible name starts at sun_path[1].
  std::memcpy(addr.sun_path + 1, name, name_len);
  return static_cast<socklen_t>(offsetof(sockaddr_un, sun_path) + 1 + name_len);
}

// SOCK_DGRAM preserves record boundaries, so each send/recv carries exactly one struct.
// thread_id is the Linux TID (SYS_gettid), not the PID.
struct NonRosThreadInfoMsg
{
  int64_t thread_id;
  char thread_name[kNonRosThreadNameMax + 1];
};
static_assert(std::is_trivially_copyable<NonRosThreadInfoMsg>::value);
// Pin the on-wire layout: a future addition / reorder would silently break the size check
// in the receiver loop while looking compatible on both sides if rebuilt together. Bumping
// any of these constants is an ABI break that requires both sender and receiver to be
// rebuilt and the network namespace to be drained of older peers.
static_assert(
  offsetof(NonRosThreadInfoMsg, thread_name) == sizeof(int64_t),
  "NonRosThreadInfoMsg has unexpected padding before thread_name");
static_assert(
  sizeof(NonRosThreadInfoMsg) == sizeof(int64_t) + (kNonRosThreadNameMax + 1),
  "NonRosThreadInfoMsg wire size changed; bump the protocol on both sides");

// At most one server per network namespace per host (abstract names are netns-scoped).
// Callbacks fire on the receiver thread, so callers MUST synchronize any state shared with
// the executor. Malformed datagrams (wrong size), transient recv errors, and exceptions
// escaping the user callback are all logged (throttled) and dropped without tearing down
// the receiver thread, so neither a bad sender nor a buggy callback can disable the IPC
// server. Linux-only: relies on AF_UNIX abstract-namespace addressing.
//
// Non-copyable and non-movable: the class owns a std::thread and three raw fds, and copy is
// explicitly deleted (move would also need an explicit definition because of std::thread's
// implicit deletion of the move constructor when copy is deleted). Hold instances via
// std::unique_ptr (as both daemon nodes do) when reseat-ability is needed.
//
// Construction throws std::system_error on any kernel-resource failure
// (socket/bind/eventfd/epoll_create1/epoll_ctl). The most operationally relevant case is
// EADDRINUSE on bind, which indicates that another agnocast_cie thread-info daemon is
// already bound in this network namespace; the error message spells this out so launch
// tooling can report it usefully.
class NonRosThreadInfoIpcServer
{
public:
  // Invoked on the internal receiver thread, never on the executor thread. Implementations
  // must synchronize any state shared with the executor and should be quick. Exceptions
  // escaping the callback are caught (throttled-logged) and dropped so the receiver thread
  // stays alive; do not rely on them for control flow.
  using Callback = std::function<void(const NonRosThreadInfoMsg &)>;

  // `logger` is stored by value; it must be associated with a live rcl context for the
  // lifetime of this object because the receiver thread emits `RCLCPP_FATAL` /
  // `RCLCPP_ERROR_THROTTLE` through it.
  NonRosThreadInfoIpcServer(rclcpp::Logger logger, Callback cb)
  : logger_(std::move(logger)), cb_(std::move(cb))
  {
    // An empty std::function would silently swallow every datagram on the receiver thread
    // (caught by the catch-all in run()) and the daemon would appear to function while
    // configuring nothing -- much harder to diagnose than a constructor-time error.
    if (!cb_) {
      throw std::invalid_argument("NonRosThreadInfoIpcServer: callback must not be empty");
    }

    sock_fd_ = ::socket(AF_UNIX, SOCK_DGRAM | SOCK_CLOEXEC, 0);
    if (sock_fd_ == -1) {
      throw std::system_error(errno, std::generic_category(), "socket(AF_UNIX, SOCK_DGRAM) failed");
    }

    sockaddr_un addr;
    const socklen_t addrlen = fill_abstract_sockaddr(addr, kNonRosThreadInfoSocketName);
    if (::bind(sock_fd_, reinterpret_cast<const sockaddr *>(&addr), addrlen) == -1) {
      const int err = errno;
      cleanup_fds();
      const std::string what =
        err == EADDRINUSE
          ? std::string("bind(@") + kNonRosThreadInfoSocketName +
              ") failed: another agnocast_cie thread-info daemon is already running in this "
              "network namespace; stop it before starting a new one"
          : std::string("bind(@") + kNonRosThreadInfoSocketName + ") failed";
      throw std::system_error(err, std::generic_category(), what);
    }

    shutdown_fd_ = ::eventfd(0, EFD_CLOEXEC);
    if (shutdown_fd_ == -1) {
      const int err = errno;
      cleanup_fds();
      throw std::system_error(err, std::generic_category(), "eventfd() failed");
    }

    epoll_fd_ = ::epoll_create1(EPOLL_CLOEXEC);
    if (epoll_fd_ == -1) {
      const int err = errno;
      cleanup_fds();
      throw std::system_error(err, std::generic_category(), "epoll_create1() failed");
    }

    if (!epoll_add(sock_fd_) || !epoll_add(shutdown_fd_)) {
      const int err = errno;
      cleanup_fds();
      throw std::system_error(err, std::generic_category(), "epoll_ctl(ADD) failed");
    }

    // std::thread's constructor can throw std::system_error (e.g., EAGAIN from pthread_create
    // when the process hits a thread/resource limit). Without this guard, the partially
    // constructed object's fds would leak and the abstract-namespace name would stay bound
    // until process exit, blocking subsequent bind retries with EADDRINUSE.
    try {
      thread_ = std::thread(&NonRosThreadInfoIpcServer::run, this);
    } catch (...) {
      cleanup_fds();
      throw;
    }
  }

  ~NonRosThreadInfoIpcServer()
  {
    if (thread_.joinable()) {
      const uint64_t v = 1;
      // EINTR-retry: write(2) is technically interruptible by signals; treating EINTR as
      // fatal here would let an asynchronously-delivered signal at destruction time
      // terminate an otherwise-healthy process during normal teardown.
      ssize_t w;
      do {
        w = ::write(shutdown_fd_, &v, sizeof(v));
      } while (w == -1 && errno == EINTR);
      if (w != static_cast<ssize_t>(sizeof(v))) {
        // The receiver is parked in epoll_wait(-1) and only the shutdown_fd_ event makes
        // run() return -- waking sock_fd_ (e.g., via shutdown(SHUT_RD)) would still leave
        // the loop spinning on EAGAIN. eventfd write effectively cannot fail under normal
        // operation (the fd is owned by us and freshly created); if it does, the IPC
        // subsystem is corrupt and continuing would hang ~NonRosThreadInfoIpcServer forever
        // in thread_.join(). Match the epoll_wait failure path: surface and terminate.
        // Distinguish the two failure modes: w == -1 means errno is meaningful, while a
        // non-(-1) short write leaves errno unspecified per POSIX so we must not strerror it
        // (mirrors the discipline applied in send_thread_info).
        if (w == -1) {
          const int err = errno;
          RCLCPP_FATAL(
            logger_, "eventfd write failed (%s); cannot wake receiver thread", strerror(err));
          // Also write to stderr in case rclcpp's logger backend is asynchronous and does
          // not flush before std::terminate(); operators rely on this diagnostic to react.
          std::fprintf(
            stderr,
            "[NonRosThreadInfoIpcServer] [FATAL] eventfd write failed (%s); cannot wake "
            "receiver thread\n",
            strerror(err));
        } else {
          RCLCPP_FATAL(
            logger_,
            "eventfd write returned unexpected size %zd (expected %zu); cannot wake "
            "receiver thread",
            w, sizeof(v));
          std::fprintf(
            stderr,
            "[NonRosThreadInfoIpcServer] [FATAL] eventfd write returned unexpected size %zd "
            "(expected %zu); cannot wake receiver thread\n",
            w, sizeof(v));
        }
        std::terminate();
      }
      thread_.join();
    }
    cleanup_fds();
    // Closing sock_fd_ released the abstract-namespace name; nothing to unlink.
  }

  NonRosThreadInfoIpcServer(const NonRosThreadInfoIpcServer &) = delete;
  NonRosThreadInfoIpcServer & operator=(const NonRosThreadInfoIpcServer &) = delete;

private:
  bool epoll_add(int fd) noexcept
  {
    if (fd == -1 || epoll_fd_ == -1) {
      errno = EBADF;
      return false;
    }
    epoll_event ev = {};
    ev.events = EPOLLIN;
    ev.data.fd = fd;
    return ::epoll_ctl(epoll_fd_, EPOLL_CTL_ADD, fd, &ev) == 0;
  }

  void cleanup_fds() noexcept
  {
    if (epoll_fd_ != -1) {
      ::close(epoll_fd_);
      epoll_fd_ = -1;
    }
    if (shutdown_fd_ != -1) {
      ::close(shutdown_fd_);
      shutdown_fd_ = -1;
    }
    if (sock_fd_ != -1) {
      ::close(sock_fd_);
      sock_fd_ = -1;
    }
  }

  void run()
  {
    constexpr int kMaxEvents = 2;
    epoll_event events[kMaxEvents];
    NonRosThreadInfoMsg msg;  // recv overwrites every byte on the only path that reads msg.

    while (true) {
      const int ret = ::epoll_wait(epoll_fd_, events, kMaxEvents, -1);
      if (ret < 0) {
        if (errno == EINTR) {
          continue;
        }
        // epoll_wait failing for anything other than EINTR is unrecoverable: the daemon's
        // registration channel would silently go mute for the rest of the process lifetime.
        // Surface it loudly so launch tooling can react.
        const int err = errno;
        RCLCPP_FATAL(logger_, "epoll_wait() failed: %s", strerror(err));
        // Also write to stderr in case rclcpp's logger backend does not flush before
        // std::terminate(); operators rely on this diagnostic to react.
        std::fprintf(
          stderr, "[NonRosThreadInfoIpcServer] [FATAL] epoll_wait() failed: %s\n", strerror(err));
        std::terminate();
      }

      for (int i = 0; i < ret; ++i) {
        if (events[i].data.fd == shutdown_fd_) {
          return;
        }
        if (events[i].data.fd != sock_fd_) {
          continue;
        }
        // MSG_TRUNC: when the incoming datagram is larger than `sizeof(msg)`, the kernel
        // returns the *actual* datagram size instead of the buffer length. Without this
        // flag, oversize datagrams from a buggy or malicious local-netns peer would be
        // silently truncated, the buffer would still look like a well-formed
        // NonRosThreadInfoMsg, and the callback would dispatch on attacker-controlled
        // bytes -- violating the class-level "malformed datagrams are dropped" guarantee.
        const ssize_t n = ::recv(sock_fd_, &msg, sizeof(msg), MSG_TRUNC);
        if (n == static_cast<ssize_t>(sizeof(msg))) {
          msg.thread_name[kNonRosThreadNameMax] = '\0';
          // Catch-all so a transient std::bad_alloc / logging hiccup / user-callback bug
          // cannot escape the std::thread entry function and call std::terminate, which
          // would silently kill the daemon. Matches the resilience guarantee documented
          // in the class-level comment.
          try {
            cb_(msg);
          } catch (const std::exception & e) {
            RCLCPP_ERROR_THROTTLE(
              logger_, throttle_clock_, kThrottleMs, "callback threw (continuing): %s", e.what());
          } catch (...) {
            RCLCPP_ERROR_THROTTLE(
              logger_, throttle_clock_, kThrottleMs, "callback threw (continuing): unknown");
          }
          continue;
        }
        if (n == -1) {
          if (errno == EINTR) {
            continue;
          }
          // Stay alive on transient recv errors: any local netns peer can ship a malformed
          // datagram, and tearing the thread down would silently disable all future
          // registrations for the daemon's lifetime. Throttle to keep a buggy peer from
          // saturating the log.
          RCLCPP_ERROR_THROTTLE(
            logger_, throttle_clock_, kThrottleMs, "recv failed (continuing): %s", strerror(errno));
          continue;
        }
        // SOCK_DGRAM: n == 0 is a zero-length datagram, n > 0 && n < sizeof(msg) is a
        // short datagram, and (with MSG_TRUNC) n > sizeof(msg) is an oversize datagram
        // whose payload was truncated into the buffer. All indicate a malformed/buggy
        // peer; drop and keep serving.
        RCLCPP_ERROR_THROTTLE(
          logger_, throttle_clock_, kThrottleMs,
          "recv returned unexpected size %zd (expected %zu); dropping", n, sizeof(msg));
      }
    }
  }

  static constexpr int kThrottleMs = 5000;

  rclcpp::Logger logger_;
  Callback cb_;
  int sock_fd_ = -1;
  int shutdown_fd_ = -1;
  int epoll_fd_ = -1;
  rclcpp::Clock throttle_clock_{RCL_STEADY_TIME};
  std::thread thread_;
};

}  // namespace agnocast_cie_thread_configurator
