#ifndef _GNU_SOURCE
#define _GNU_SOURCE
#endif

#include "agnocast_cie_thread_configurator/non_ros_thread_info_ipc.hpp"

#include <rclcpp/logging.hpp>

#include <sys/epoll.h>
#include <sys/eventfd.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>

#include <cerrno>
#include <chrono>
#include <cstdio>
#include <cstring>
#include <stdexcept>
#include <string>
#include <system_error>
#include <thread>
#include <utility>

namespace
{
// Wire-format socket name: the sender and the receiver must agree on this
// byte-for-byte. Changing it is an ABI break that requires both sides to be
// rebuilt together.
constexpr char kNonRosThreadInfoSocketName[] = "agnocast_cie_non_ros_thread_info";

// Guarantee the abstract address fits sun_path so fill_abstract_sockaddr's
// memcpy cannot overflow. The +1 accounts for the leading '\0' marker that
// pins an address to the abstract namespace.
static_assert(
  sizeof(kNonRosThreadInfoSocketName) <= sizeof(sockaddr_un::sun_path),
  "abstract socket name does not fit in sockaddr_un::sun_path");

// Sender-side retry budgets. Increasing kSenderMaxConnectWaitIters tolerates
// slower daemon startup (e.g., loaded CI hosts) at the cost of delaying the
// user function in the "daemon absent" failure mode. kSenderMaxSendAttempts
// is overwhelmingly defensive: the default Linux rcvbuf holds hundreds of
// these messages.
constexpr int kSenderMaxConnectWaitIters = 500;  // 5 s daemon-up wait
constexpr int kSenderMaxSendAttempts = 50;       // 500 ms EAGAIN budget
constexpr std::chrono::milliseconds kSenderRetryDelay{10};

// Thread-safe replacement for std::strerror. strerror itself is MT-Unsafe
// (it returns a pointer to a static buffer), so concurrent senders can
// clobber each other's diagnostic strings. strerrordesc_np (glibc 2.32+) is
// MT-Safe and returns nullptr only for unknown errnos.
const char * safe_strerror(int err) noexcept
{
  const char * msg = ::strerrordesc_np(err);
  return msg != nullptr ? msg : "Unknown error";
}
}  // namespace

namespace agnocast_cie_thread_configurator
{

int open_sender_socket(const char * thread_name) noexcept
{
  const int fd = ::socket(AF_UNIX, SOCK_DGRAM | SOCK_CLOEXEC, 0);
  if (fd == -1) {
    std::fprintf(
      stderr, "[cie_thread_client] [WARN] socket(AF_UNIX) failed for thread '%s': %s\n",
      thread_name, safe_strerror(errno));
    return -1;
  }

  sockaddr_un addr;
  const socklen_t addrlen = fill_abstract_sockaddr(addr, kNonRosThreadInfoSocketName);

  for (int attempt = 0; attempt < kSenderMaxConnectWaitIters; ++attempt) {
    if (::connect(fd, reinterpret_cast<const sockaddr *>(&addr), addrlen) == 0) {
      return fd;
    }
    const int err = errno;
    // ECONNREFUSED: daemon not yet bound (the common startup race we are
    // polling against). EAGAIN/EWOULDBLOCK: AF_UNIX connect can hit autobind
    // table exhaustion under burst load. EINTR: also consumes a slot so a
    // signal storm cannot become a hot spin.
    if (err != ECONNREFUSED && err != EINTR && err != EAGAIN && err != EWOULDBLOCK) {
      std::fprintf(
        stderr, "[cie_thread_client] [WARN] connect to '@%s' failed for thread '%s': %s\n",
        kNonRosThreadInfoSocketName, thread_name, safe_strerror(err));
      ::close(fd);
      return -1;
    }
    std::this_thread::sleep_for(kSenderRetryDelay);
  }
  std::fprintf(
    stderr,
    "[cie_thread_client] [WARN] No NonRosThreadInfo daemon listening for thread '%s'. "
    "Please run thread_configurator_node if you want to configure thread scheduling.\n",
    thread_name);
  ::close(fd);
  return -1;
}

bool send_thread_info(int fd, const NonRosThreadInfoMsg & msg, const char * thread_name) noexcept
{
  for (int attempt = 0; attempt < kSenderMaxSendAttempts; ++attempt) {
    const ssize_t n = ::send(fd, &msg, sizeof(msg), MSG_DONTWAIT);
    if (n == static_cast<ssize_t>(sizeof(msg))) {
      return true;
    }
    if (n != -1) {
      // SOCK_DGRAM is per-datagram atomic: the kernel never produces partial
      // sends. Treat any other non-(-1) return as a hard failure -- and
      // crucially, do NOT read errno, it is unspecified when the syscall did
      // not return -1.
      std::fprintf(
        stderr,
        "[cie_thread_client] [WARN] send returned unexpected size %zd for NonRosThreadInfo "
        "(thread '%s'); not retrying.\n",
        n, thread_name);
      return false;
    }
    const int err = errno;
    // ENOBUFS: AF_UNIX SOCK_DGRAM can return it under transient kernel-memory
    // pressure when allocating an sk_buff fails; treating it as fatal would
    // silently leave the thread unconfigured for its lifetime.
    if (err != EAGAIN && err != EWOULDBLOCK && err != EINTR && err != ENOBUFS) {
      std::fprintf(
        stderr, "[cie_thread_client] [WARN] send failed for NonRosThreadInfo (thread '%s'): %s\n",
        thread_name, safe_strerror(err));
      return false;
    }
    std::this_thread::sleep_for(kSenderRetryDelay);
  }
  std::fprintf(
    stderr,
    "[cie_thread_client] [WARN] Timed out sending NonRosThreadInfo for thread '%s' "
    "(receiver buffer full); thread will run unconfigured.\n",
    thread_name);
  return false;
}

namespace
{
constexpr int kThrottleMs = 5000;
}

NonRosThreadInfoIpcServer::NonRosThreadInfoIpcServer(rclcpp::Logger logger, Callback cb)
: logger_(std::move(logger)), cb_(std::move(cb))
{
  // An empty std::function would silently swallow every datagram (caught by
  // the catch-all in run()) and the daemon would appear to function while
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

  // std::thread's constructor can throw std::system_error (e.g., EAGAIN from
  // pthread_create). Without this guard the partially constructed object's
  // fds would leak and the abstract name would stay bound until process
  // exit, blocking subsequent bind retries with EADDRINUSE.
  try {
    thread_ = std::thread(&NonRosThreadInfoIpcServer::run, this);
  } catch (...) {
    cleanup_fds();
    throw;
  }
}

NonRosThreadInfoIpcServer::~NonRosThreadInfoIpcServer()
{
  if (thread_.joinable()) {
    const uint64_t v = 1;
    // EINTR-retry: write(2) is technically interruptible by signals; treating
    // EINTR as fatal here would let an asynchronously-delivered signal at
    // destruction time terminate an otherwise-healthy process.
    ssize_t w;
    do {
      w = ::write(shutdown_fd_, &v, sizeof(v));
    } while (w == -1 && errno == EINTR);
    if (w != static_cast<ssize_t>(sizeof(v))) {
      // The receiver is parked in epoll_wait(-1) and only the shutdown_fd_
      // event makes run() return. eventfd write effectively cannot fail
      // under normal operation; if it does, continuing would hang the
      // destructor forever in thread_.join(). Surface and terminate.
      if (w == -1) {
        const int err = errno;
        RCLCPP_FATAL(
          logger_, "eventfd write failed (%s); cannot wake receiver thread", safe_strerror(err));
        std::fprintf(
          stderr,
          "[NonRosThreadInfoIpcServer] [FATAL] eventfd write failed (%s); cannot wake "
          "receiver thread\n",
          safe_strerror(err));
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

bool NonRosThreadInfoIpcServer::epoll_add(int fd) noexcept
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

void NonRosThreadInfoIpcServer::cleanup_fds() noexcept
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

void NonRosThreadInfoIpcServer::run()
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
      // epoll_wait failing for anything other than EINTR is unrecoverable:
      // the daemon's registration channel would silently go mute for the
      // rest of the process lifetime. Surface it loudly so launch tooling
      // can react.
      const int err = errno;
      RCLCPP_FATAL(logger_, "epoll_wait() failed: %s", safe_strerror(err));
      std::fprintf(
        stderr, "[NonRosThreadInfoIpcServer] [FATAL] epoll_wait() failed: %s\n",
        safe_strerror(err));
      std::terminate();
    }

    for (int i = 0; i < ret; ++i) {
      if (events[i].data.fd == shutdown_fd_) {
        return;
      }
      if (events[i].data.fd != sock_fd_) {
        continue;
      }
      // MSG_TRUNC: when the incoming datagram is larger than `sizeof(msg)`,
      // the kernel returns the *actual* datagram size instead of the buffer
      // length. Without this flag, oversized datagrams would be silently
      // truncated and the callback would dispatch on attacker-controlled bytes.
      const ssize_t n = ::recv(sock_fd_, &msg, sizeof(msg), MSG_TRUNC);
      if (n == static_cast<ssize_t>(sizeof(msg))) {
        msg.thread_name[kNonRosThreadNameMax] = '\0';
        // Catch-all so a transient std::bad_alloc / logging hiccup / user-
        // callback bug cannot escape the std::thread entry function and call
        // std::terminate, which would silently kill the daemon.
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
        // Stay alive on transient recv errors: any local netns peer can ship
        // a malformed datagram, and tearing the thread down would silently
        // disable all future registrations.
        RCLCPP_ERROR_THROTTLE(
          logger_, throttle_clock_, kThrottleMs, "recv failed (continuing): %s",
          safe_strerror(errno));
        continue;
      }
      // SOCK_DGRAM: n == 0 is a zero-length datagram, n > 0 && n < sizeof(msg)
      // is a short datagram, and (with MSG_TRUNC) n > sizeof(msg) is an
      // oversize datagram whose payload was truncated. Drop and keep serving.
      RCLCPP_ERROR_THROTTLE(
        logger_, throttle_clock_, kThrottleMs,
        "recv returned unexpected size %zd (expected %zu); dropping", n, sizeof(msg));
    }
  }
}

}  // namespace agnocast_cie_thread_configurator
