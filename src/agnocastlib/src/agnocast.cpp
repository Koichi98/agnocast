#include "agnocast/agnocast.hpp"

#include "agnocast/agnocast_ioctl.hpp"
#include "agnocast/agnocast_mq.hpp"
#include "agnocast/agnocast_version.hpp"
#include "agnocast/bridge/performance/agnocast_performance_bridge_manager.hpp"

#include <dlfcn.h>
#include <strings.h>
#include <sys/stat.h>
#include <sys/types.h>

#include <algorithm>
#include <atomic>
#include <cerrno>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <mutex>
#include <span>
#include <string>
#include <string_view>
#include <vector>

extern "C" {
const char * agnocast_get_version()
{
  return agnocastlib::VERSION;
}
}

namespace agnocast
{

int agnocast_fd = -1;
std::vector<int> shm_fds;
std::mutex shm_fds_mtx;
std::mutex mmap_mtx;
// mmap_mtx: Prevents a race condition and segfault between two threads
// in a multithreaded executor using the same mqueue_fd.
//
// Race Scenario:
// 1. Thread 1 (T1):
//    - Calls epoll_wait(), mq_receive(), then ioctl(RECEIVE_CMD), initially obtaining
//      publisher info (PID, shared memory address `shm_addr`).
//    - Critical: OS context switch occurs *after* ioctl() but *before* T1 fully
//      processes/maps `shm_addr`.
// 2. Thread 2 (T2):
//    - Calls epoll_wait(), mq_receive(), then ioctl(RECEIVE_CMD) on the same mqueue_fd,
//      but does *not* receive publisher info (assuming it's already set up).
//    - Proceeds to a callback which attempts to use `shm_addr`, leading to a SEGFAULT.
//
// Root Cause: T2's callback uses `shm_addr` that T1 fetched but hadn't initialized/mapped yet.
// This mutex ensures atomicity for T1's critical section: from ioctl fetching publisher
// info through to completing shared memory setup.

void * map_area(
  const pid_t pid, const uint64_t shm_addr, const uint64_t shm_size, const bool writable)
{
  const std::string shm_name = create_shm_name(pid);

  int oflag = writable ? O_CREAT | O_EXCL | O_RDWR : O_RDONLY;
  const int shm_mode = 0666;
  int shm_fd = shm_open(shm_name.c_str(), oflag, shm_mode);
  if (shm_fd == -1) {
    RCLCPP_ERROR(logger, "shm_open failed: %s", strerror(errno));
    close(agnocast_fd);
    return nullptr;
  }

  {
    std::lock_guard<std::mutex> lock(shm_fds_mtx);
    shm_fds.push_back(shm_fd);
  }

  auto cleanup_shm_fd = [&]() {
    {
      std::lock_guard<std::mutex> lock(shm_fds_mtx);
      shm_fds.erase(std::remove(shm_fds.begin(), shm_fds.end(), shm_fd), shm_fds.end());
    }
    close(shm_fd);
    if (writable) {
      shm_unlink(shm_name.c_str());
    }
  };

  if (writable) {
    if (ftruncate(shm_fd, static_cast<off_t>(shm_size)) == -1) {
      RCLCPP_ERROR(logger, "ftruncate failed: %s", strerror(errno));
      cleanup_shm_fd();
      close(agnocast_fd);
      return nullptr;
    }

    const int new_shm_mode = 0444;
    if (fchmod(shm_fd, new_shm_mode) == -1) {
      RCLCPP_ERROR(logger, "fchmod failed: %s", strerror(errno));
      cleanup_shm_fd();
      close(agnocast_fd);
      return nullptr;
    }
  }

  int prot = writable ? PROT_READ | PROT_WRITE : PROT_READ;
  void * ret = mmap(
    reinterpret_cast<void *>(shm_addr), shm_size, prot, MAP_SHARED | MAP_FIXED_NOREPLACE, shm_fd,
    0);

  if (ret == MAP_FAILED) {
    RCLCPP_ERROR(logger, "mmap failed: %s", strerror(errno));
    cleanup_shm_fd();
    close(agnocast_fd);
    return nullptr;
  }

  return ret;
}

void * map_writable_area(const pid_t pid, const uint64_t shm_addr, const uint64_t shm_size)
{
  return map_area(pid, shm_addr, shm_size, true);
}

void map_read_only_area(const pid_t pid, const uint64_t shm_addr, const uint64_t shm_size)
{
  if (map_area(pid, shm_addr, shm_size, false) == nullptr) {
    exit(EXIT_FAILURE);
  }
}

// Initializes the child allocator for bridge functionality.
// Note: This function must only be called in a forked child process before TLSF initialization.
// Calling it after initialization will result in double initialization.
void initialize_bridge_allocator(void * mempool_ptr, size_t mempool_size)
{
  void * handle = dlopen(nullptr, RTLD_NOW);
  if (handle == nullptr) {
    const char * err_msg = dlerror();
    throw std::runtime_error(
      std::string("dlopen failed: ") + (err_msg != nullptr ? err_msg : "Unknown"));
  }

  using InitFunc = bool (*)(void *, size_t);
  auto init_func = reinterpret_cast<InitFunc>(dlsym(handle, "init_child_allocator"));

  const char * dlsym_error = dlerror();
  if ((dlsym_error != nullptr) || (init_func == nullptr)) {
    dlclose(handle);
    throw std::runtime_error(
      std::string("dlsym 'init_child_allocator' failed: ") +
      (dlsym_error != nullptr ? dlsym_error : "Symbol is null"));
  }

  bool success = init_func(mempool_ptr, mempool_size);

  if (!success) {
    throw std::runtime_error("init_child_allocator returned false.");
  }
}

initialize_agnocast_result acquire_agnocast_resources_for_bridge()
{
  union ioctl_add_process_args add_process_args = {};
  add_process_args.is_performance_bridge_manager = true;
  add_process_args.domain_id = get_ros_domain_id();
  if (ioctl(agnocast_fd, AGNOCAST_ADD_PROCESS_CMD, &add_process_args) < 0) {
    throw std::runtime_error(std::string("AGNOCAST_ADD_PROCESS_CMD failed: ") + strerror(errno));
  }

  if (add_process_args.ret_performance_bridge_daemon_exist) {
    close(agnocast_fd);
    exit(EXIT_SUCCESS);
  }

  void * mempool_ptr =
    map_writable_area(getpid(), add_process_args.ret_addr, add_process_args.ret_shm_size);

  if (mempool_ptr == nullptr) {
    throw std::runtime_error("map_writable_area failed.");
  }

  return {
    mempool_ptr,
    add_process_args.ret_shm_size,
  };
}

void poll_for_unlink()
{
  std::vector<exit_subscription_mq_info> mq_info_buf(MAX_SUBSCRIPTION_NUM_PER_PROCESS);

  while (true) {
    sleep(1);

    struct ioctl_get_exit_process_args get_exit_process_args = {};
    do {
      get_exit_process_args = {};
      get_exit_process_args.subscription_mq_info_buffer_addr =
        reinterpret_cast<uint64_t>(mq_info_buf.data());
      get_exit_process_args.subscription_mq_info_buffer_size =
        static_cast<uint32_t>(mq_info_buf.size());
      if (ioctl(agnocast_fd, AGNOCAST_GET_EXIT_PROCESS_CMD, &get_exit_process_args) < 0) {
        RCLCPP_ERROR(logger, "AGNOCAST_GET_EXIT_PROCESS_CMD failed: %s", strerror(errno));
        close(agnocast_fd);
        exit(EXIT_FAILURE);
      }

      if (get_exit_process_args.ret_pid > 0) {
        const std::string shm_name = create_shm_name(get_exit_process_args.ret_pid);
        shm_unlink(shm_name.c_str());

        // Unlink subscription MQs that the exited process owned
        for (uint32_t i = 0; i < get_exit_process_args.ret_subscription_mq_info_num; i++) {
          // NOLINTNEXTLINE(cppcoreguidelines-pro-bounds-array-to-pointer-decay,hicpp-no-array-decay)
          const std::string topic_name(mq_info_buf[i].topic_name);
          const std::string sub_mq_name =
            create_mq_name_for_agnocast_publish(topic_name, mq_info_buf[i].subscriber_id);
          mq_unlink(sub_mq_name.c_str());
        }
      }
    } while (get_exit_process_args.ret_pid > 0);

    if (get_exit_process_args.ret_daemon_should_exit) {
      break;
    }
  }

  exit(0);
}

void poll_for_bridge_manager()
{
  try {
    const auto resources = acquire_agnocast_resources_for_bridge();
    initialize_bridge_allocator(resources.mempool_ptr, resources.mempool_size);
    PerformanceBridgeManager manager;
    manager.run();
  } catch (const std::exception & e) {
    RCLCPP_ERROR(logger, "BridgeManager crashed: %s", e.what());
    exit(EXIT_FAILURE);
  }
  exit(0);
}

// Forked into the pre-allocator child: must not allocate. execlp() with a literal
// argv is malloc-free, unlike setenv() which may allocate before agnocast's TLSF
// allocator is ready.
void exec_discovery_agent()
{
  execlp(
    "ros2", "ros2", "run", "ros2agnocast_discovery_agent", "agnocast_discovery_agent",
    "--exit-when-idle", static_cast<char *>(nullptr));
  // execlp only returns on failure. This still runs in the pre-allocator forked child, so the
  // failure path must be async-signal-safe and allocation-free: RCLCPP_ERROR / strerror / exit()
  // may allocate or run atexit handlers before the TLSF allocator is ready. Use write() + _exit().
  constexpr std::string_view err_msg = "[ERROR] [Agnocast] Failed to exec the discovery agent\n";
  // Best-effort diagnostic -- we _exit next regardless; the assignment + cast just
  // consume write()'s warn_unused_result without allocating or logging.
  const ssize_t written = write(STDERR_FILENO, err_msg.data(), err_msg.size());
  static_cast<void>(written);
  _exit(EXIT_FAILURE);
}

struct semver
{
  int major;
  int minor;
  int patch;
};

bool parse_semver(const char * version, struct semver * out_ver)
{
  if (version == nullptr || out_ver == nullptr) {
    return false;
  }

  out_ver->major = 0;
  out_ver->minor = 0;
  out_ver->patch = 0;

  std::string version_str(version);
  std::stringstream ss(version_str);

  int64_t major = 0;
  int64_t minor = 0;
  int64_t patch = 0;

  if (!(ss >> major) || ss.get() != '.') {
    return false;
  }

  if (!(ss >> minor) || ss.get() != '.') {
    return false;
  }

  if (!(ss >> patch)) {
    return false;
  }

  if (!ss.eof()) {
    char remaining = '\0';
    if (ss >> remaining) {
      return false;
    }
  }

  if (major < 0 || minor < 0 || patch < 0) {
    return false;
  }

  out_ver->major = static_cast<int>(major);
  out_ver->minor = static_cast<int>(minor);
  out_ver->patch = static_cast<int>(patch);

  return true;
}

bool compare_to_minor_version(const struct semver * v1, const struct semver * v2)
{
  if (v1 == nullptr || v2 == nullptr) {
    return false;
  }

  return (v1->major == v2->major && v1->minor == v2->minor);
}

bool compare_to_patch_version(const struct semver * v1, const struct semver * v2)
{
  if (v1 == nullptr || v2 == nullptr) {
    return false;
  }

  return (v1->major == v2->major && v1->minor == v2->minor && v1->patch == v2->patch);
}

bool is_version_consistent(
  const unsigned char * heaphook_version_ptr, const size_t heaphook_version_str_len,
  struct ioctl_get_version_args kmod_version)
{
  std::array<char, VERSION_BUFFER_LEN> heaphook_version_arr{};
  struct semver lib_ver
  {
  };
  struct semver heaphook_ver
  {
  };
  struct semver kmod_ver
  {
  };

  size_t copy_len = heaphook_version_str_len < (VERSION_BUFFER_LEN - 1) ? heaphook_version_str_len
                                                                        : (VERSION_BUFFER_LEN - 1);
  std::memcpy(heaphook_version_arr.data(), heaphook_version_ptr, copy_len);
  heaphook_version_arr[copy_len] = '\0';

  bool parse_lib_result = parse_semver(agnocastlib::VERSION, &lib_ver);
  bool parse_heaphook_result = parse_semver(heaphook_version_arr.data(), &heaphook_ver);
  bool parse_kmod_result =
    parse_semver(static_cast<const char *>(&kmod_version.ret_version[0]), &kmod_ver);

  if (!parse_lib_result || !parse_heaphook_result || !parse_kmod_result) {
    RCLCPP_ERROR(logger, "Failed to parse one or more version strings");
    return false;
  }

  if (!compare_to_patch_version(&lib_ver, &heaphook_ver)) {
    RCLCPP_ERROR(
      logger,
      "Agnocast Heaphook and Agnocastlib versions must match exactly: Major, Minor, and Patch "
      "versions must all be identical. (agnocast-heaphook(%d.%d.%d), agnocast(%d.%d.%d))",
      heaphook_ver.major, heaphook_ver.minor, heaphook_ver.patch, lib_ver.major, lib_ver.minor,
      lib_ver.patch);
    return false;
  }

  if (!compare_to_minor_version(&lib_ver, &kmod_ver)) {
    RCLCPP_ERROR(
      logger,
      "Agnocast Kernel Module and Agnocastlib must be compatible: Major and Minor versions must "
      "match. (agnocast-kmod(%d.%d.%d), agnocast(%d.%d.%d))",
      kmod_ver.major, kmod_ver.minor, kmod_ver.patch, lib_ver.major, lib_ver.minor, lib_ver.patch);
    return false;
  }

  return true;
}

// getenv() does not allocate, so this is safe before agnocast's allocator is ready.
bool env_is_truthy(const char * name)
{
  const char * v = getenv(name);
  return v != nullptr &&
         (strcmp(v, "1") == 0 || strcasecmp(v, "true") == 0 || strcasecmp(v, "yes") == 0);
}

// Opt-out for deployments that manage the discovery agent themselves.
bool discovery_agent_auto_fork_disabled()
{
  return env_is_truthy("AGNOCAST_NO_DISCOVERY_AGENT");
}

// The fallback chain mirrors the one rcl and launch use, so daemon logs land next to node logs.
std::string resolve_daemon_log_dir()
{
  const char * override_dir = getenv("AGNOCAST_DAEMON_LOG_DIR");
  if (override_dir != nullptr && *override_dir != '\0') {
    RCLCPP_WARN_ONCE(
      logger, "AGNOCAST_DAEMON_LOG_DIR is experimental and may change or be removed.");
    return override_dir;
  }

  const char * ros_log_dir = getenv("ROS_LOG_DIR");
  if (ros_log_dir != nullptr && *ros_log_dir != '\0') {
    return ros_log_dir;
  }

  const char * ros_home = getenv("ROS_HOME");
  if (ros_home != nullptr && *ros_home != '\0') {
    return std::string(ros_home) + "/log";
  }

  const char * home = getenv("HOME");
  if (home != nullptr && *home != '\0') {
    return std::string(home) + "/.ros/log";
  }

  return "";
}

bool make_directories(const std::string & path)
{
  for (size_t i = 1; i <= path.size(); i++) {
    if (i != path.size() && path[i] != '/') {
      continue;
    }
    const std::string component = path.substr(0, i);
    if (mkdir(component.c_str(), 0755) != 0 && errno != EEXIST) {
      return false;
    }
  }
  return true;
}

// The name carries the daemon's scope so that daemons which legitimately coexist do not share a
// file. No pid or timestamp: a duplicate daemon that loses its race and exits immediately would
// then leave an empty file behind on every node start.
std::string resolve_daemon_log_path(const char * daemon_name, const uint32_t * domain_id)
{
  const std::string dir = resolve_daemon_log_dir();
  if (dir.empty() || !make_directories(dir)) {
    // Without a log file the daemon falls back to /dev/null, so say so rather than silently
    // discarding its output.
    RCLCPP_WARN_ONCE(
      logger,
      "No writable daemon log directory (tried ROS_LOG_DIR, $ROS_HOME/log, ~/.ros/log), so "
      "Agnocast daemon output is discarded.");
    return "";
  }

  struct stat ipc_ns_st = {};
  const unsigned long ipc_ns_inode =
    (stat("/proc/self/ns/ipc", &ipc_ns_st) == 0) ? ipc_ns_st.st_ino : 0;

  std::string path = dir + "/agnocast_" + daemon_name + "_ns" + std::to_string(ipc_ns_inode);
  if (domain_id != nullptr) {
    path += "_d" + std::to_string(*domain_id);
  }
  return path + ".log";
}

// Must run before setsid(), which drops the controlling terminal that /dev/tty needs.
//
// dup2 implicitly closes the inherited descriptors, which is the point: a daemon outlives its
// parent, and ros2 launch or CTest only completes once every pipe it set up is released -- it waits
// for EOF on stdout/stderr, and on stdin for the last reader to close. fd 0 gets /dev/null rather
// than out_fd, which is write-only, and closing it outright would let the next open() claim it.
void redirect_stdio_for_daemon(const char * daemon_name, const std::string & log_path)
{
  const int devnull = open("/dev/null", O_RDWR);
  if (devnull >= 0 && dup2(devnull, STDIN_FILENO) < 0) {
    _exit(EXIT_FAILURE);
  }

  int out_fd = -1;
  bool opened_log_file = false;
  // Opt-in only; the log file is the default target.
  if (env_is_truthy("AGNOCAST_DAEMON_LOG_TO_TTY")) {
    out_fd = open("/dev/tty", O_WRONLY);
  } else {
    // rcutils colours per message from isatty(), which already says no for a log file, unless
    // RCUTILS_COLORIZED_OUTPUT forces it on. Override that: escapes in the file spoil grep.
    setenv("RCUTILS_COLORIZED_OUTPUT", "0", 1);
  }
  if (out_fd < 0 && !log_path.empty()) {
    out_fd = open(log_path.c_str(), O_WRONLY | O_CREAT | O_APPEND, 0644);
    opened_log_file = out_fd >= 0;
  }
  if (out_fd < 0) {
    out_fd = devnull;
  }
  if (out_fd < 0) {
    return;
  }

  // Instances append to one file, so mark where this one starts.
  if (opened_log_file) {
    char header[128];
    const int len = snprintf(
      header, sizeof(header), "--- agnocast %s daemon started (pid %d) ---\n", daemon_name,
      static_cast<int>(getpid()));
    if (len > 0) {
      // snprintf reports what it would have written, which can exceed the buffer.
      const size_t header_len = std::min(static_cast<size_t>(len), sizeof(header) - 1);
      const ssize_t written = write(out_fd, header, header_len);
      static_cast<void>(written);
    }
  }

  // A failed dup2 leaves the daemon holding the parent's pipe, which is the hang this function
  // exists to prevent. A missing daemon is the lesser problem, so give up instead.
  if (dup2(out_fd, STDOUT_FILENO) < 0 || dup2(out_fd, STDERR_FILENO) < 0) {
    constexpr char msg[] = "[ERROR] [Agnocast] Failed to redirect the daemon's stdio\n";
    const ssize_t written = write(STDERR_FILENO, msg, sizeof(msg) - 1);
    static_cast<void>(written);
    _exit(EXIT_FAILURE);
  }
  if (out_fd > STDERR_FILENO) {
    close(out_fd);
  }
  if (devnull > STDERR_FILENO && devnull != out_fd) {
    close(devnull);
  }
  // stdout to a file is fully buffered, so INFO would be lost if the daemon is killed.
  setvbuf(stdout, nullptr, _IOLBF, 0);
}

template <typename Func>
pid_t spawn_daemon_process(const char * daemon_name, const std::string & log_path, Func && func)
{
  auto fail = [](const char * err_fmt) {
    RCLCPP_ERROR(logger, err_fmt, strerror(errno));
    close(agnocast_fd);
    exit(EXIT_FAILURE);
  };

  if (env_is_truthy("AGNOCAST_DAEMON_LOG_TO_TTY")) {
    RCLCPP_WARN_ONCE(
      logger, "AGNOCAST_DAEMON_LOG_TO_TTY is experimental and may change or be removed.");
  }

  // Buffered data would otherwise be duplicated into the child and flushed twice.
  fflush(nullptr);

  pid_t pid = fork();
  if (pid < 0) {
    fail("fork failed: %s");
  }
  if (pid == 0) {
    agnocast::is_bridge_process = true;
    unsetenv("LD_PRELOAD");

    redirect_stdio_for_daemon(daemon_name, log_path);

    if (setsid() == -1) {
      fail("setsid failed: %s");
    }

    func();
    exit(0);
  }

  return pid;
}

// NOTE: Avoid heap allocation inside initialize_agnocast. TLSF is not initialized yet.
struct initialize_agnocast_result initialize_agnocast(
  const unsigned char * heaphook_version_ptr, const size_t heaphook_version_str_len)
{
  if (agnocast_fd >= 0) {
    RCLCPP_ERROR(logger, "Agnocast is already open");
    exit(EXIT_FAILURE);
  }

  agnocast_fd = open("/dev/agnocast", O_RDWR);
  if (agnocast_fd < 0) {
    if (errno == ENOENT) {
      RCLCPP_ERROR(logger, "%s", AGNOCAST_DEVICE_NOT_FOUND_MSG);
    } else {
      RCLCPP_ERROR(logger, "Failed to open /dev/agnocast: %s", strerror(errno));
    }
    exit(EXIT_FAILURE);
  }

  struct ioctl_get_version_args get_version_args = {};
  if (ioctl(agnocast_fd, AGNOCAST_GET_VERSION_CMD, &get_version_args) < 0) {
    RCLCPP_ERROR(logger, "AGNOCAST_GET_VERSION_CMD failed: %s", strerror(errno));
    close(agnocast_fd);
    exit(EXIT_FAILURE);
  }

  if (!is_version_consistent(heaphook_version_ptr, heaphook_version_str_len, get_version_args)) {
    close(agnocast_fd);
    exit(EXIT_FAILURE);
  }

  // Kept in a local because add_process_args is a union: ADD_PROCESS overwrites the input
  // domain_id with its ret_* fields, so it cannot be read back afterwards.
  const uint32_t domain_id = get_ros_domain_id();

  union ioctl_add_process_args add_process_args = {};
  add_process_args.domain_id = domain_id;
  if (ioctl(agnocast_fd, AGNOCAST_ADD_PROCESS_CMD, &add_process_args) < 0) {
    RCLCPP_ERROR(logger, "AGNOCAST_ADD_PROCESS_CMD failed: %s", strerror(errno));
    close(agnocast_fd);
    exit(EXIT_FAILURE);
  }

  bool should_spawn_bridge = false;
  auto bridge_mode = get_bridge_mode();

  // Create a shm_unlink daemon process if it doesn't exist in its ipc namespace.
  if (!add_process_args.ret_unlink_daemon_exist) {
    // Namespace-scoped, hence no domain in the log path.
    spawn_daemon_process(
      "unlink", resolve_daemon_log_path("unlink", nullptr), []() { poll_for_unlink(); });
  }
  if (bridge_mode == BridgeMode::On && !add_process_args.ret_performance_bridge_daemon_exist) {
    should_spawn_bridge = true;
  }

  if (should_spawn_bridge) {
    spawn_daemon_process(
      "bridge_manager", resolve_daemon_log_path("bridge_manager", &domain_id),
      []() { poll_for_bridge_manager(); });
  }

  // The forked agent inherits this process's IPC namespace and ROS_DOMAIN_ID, and
  // self-exits when the scope empties. A missing or unstartable agent is not fatal
  // (the data plane does not depend on the observer); a fork() failure still is, as
  // for the other daemons spawned here -- it means system-wide resource exhaustion.
  if (!add_process_args.ret_discovery_agent_exist && !discovery_agent_auto_fork_disabled()) {
    spawn_daemon_process(
      "discovery_agent", resolve_daemon_log_path("discovery_agent", &domain_id),
      []() { exec_discovery_agent(); });
  }

  void * mempool_ptr =
    map_writable_area(getpid(), add_process_args.ret_addr, add_process_args.ret_shm_size);
  if (mempool_ptr == nullptr) {
    close(agnocast_fd);
    exit(EXIT_FAILURE);
  }

  struct initialize_agnocast_result result = {};
  result.mempool_ptr = mempool_ptr;
  result.mempool_size = add_process_args.ret_shm_size;
  return result;
}

static void shutdown_agnocast()
{
  std::lock_guard<std::mutex> lock(shm_fds_mtx);
  for (int fd : shm_fds) {
    if (close(fd) == -1) {
      perror("[ERROR] [Agnocast] close shm_fd failed");
    }
  }
}

class Cleanup
{
public:
  Cleanup(const Cleanup &) = delete;
  Cleanup & operator=(const Cleanup &) = delete;
  Cleanup(Cleanup &&) = delete;
  Cleanup & operator=(Cleanup &&) = delete;

  Cleanup() = default;
  ~Cleanup() { shutdown_agnocast(); }
};

static Cleanup cleanup;

}  // namespace agnocast
