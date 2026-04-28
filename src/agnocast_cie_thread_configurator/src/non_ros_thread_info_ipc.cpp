#ifndef _GNU_SOURCE
#define _GNU_SOURCE
#endif

#include "agnocast_cie_thread_configurator/non_ros_thread_info_ipc.hpp"

#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>

#include <cerrno>
#include <chrono>
#include <cstdio>
#include <cstring>
#include <thread>

namespace
{
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

}  // namespace agnocast_cie_thread_configurator
