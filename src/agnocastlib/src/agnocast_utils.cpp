#include "agnocast/agnocast_utils.hpp"

#include "agnocast/agnocast_mq.hpp"
#include "agnocast/node/agnocast_node.hpp"

#include <sys/stat.h>

#include <cerrno>
#include <cstdlib>
#include <cstring>
#include <mutex>
#include <regex>
#include <string>
#include <system_error>
#include <unordered_map>

namespace agnocast
{
rclcpp::Logger logger = rclcpp::get_logger("Agnocast");
bool is_bridge_process = false;

namespace
{
struct DbgTopicFilter
{
  bool configured = false;  // AGNOCAST_DBG_TOPIC_REGEX was present in the environment
  std::regex regex;
};

const DbgTopicFilter & dbg_topic_filter()
{
  static const DbgTopicFilter filter = [] {
    DbgTopicFilter f;
    const char * env = std::getenv("AGNOCAST_DBG_TOPIC_REGEX");
    if (env == nullptr) {
      RCLCPP_INFO(
        logger,
        "[agnocast-dbg] AGNOCAST_DBG_TOPIC_REGEX is unset: per-message pub/sub logs cover every "
        "topic, throttled to entry #1 and every 100th message.");
      return f;
    }
    f.configured = true;
    try {
      f.regex = std::regex(env, std::regex::ECMAScript);
    } catch (const std::regex_error & e) {
      // A bad regex must not silently disable the logs the user came here for.
      RCLCPP_ERROR(
        logger,
        "[agnocast-dbg] AGNOCAST_DBG_TOPIC_REGEX='%s' is not a valid regex (%s). Falling back to "
        "logging every topic with the 1/100 throttle.",
        env, e.what());
      f.configured = false;
      return f;
    }
    RCLCPP_INFO(
      logger,
      "[agnocast-dbg] AGNOCAST_DBG_TOPIC_REGEX='%s': only matching topics emit per-message pub/sub "
      "logs, and they emit one line per message (no throttle).",
      env);
    return f;
  }();
  return filter;
}
}  // namespace

bool dbg_topic_filter_is_unset()
{
  return !dbg_topic_filter().configured;
}

bool is_dbg_target_topic(const std::string & topic_name)
{
  const DbgTopicFilter & filter = dbg_topic_filter();
  if (!filter.configured) {
    return true;
  }

  // Called once per received/published message, so memoize the regex result per topic.
  static std::mutex cache_mutex;
  static std::unordered_map<std::string, bool> cache;
  std::lock_guard<std::mutex> lock(cache_mutex);
  const auto it = cache.find(topic_name);
  if (it != cache.end()) {
    return it->second;
  }
  const bool matched = std::regex_search(topic_name, filter.regex);
  cache.emplace(topic_name, matched);
  return matched;
}

void validate_ld_preload()
{
  if (is_bridge_process) {
    // The bridge process is spawned with an empty LD_PRELOAD to avoid loading the heaphook library
    // in its descendant processes.
    return;
  }

  const char * ld_preload_cstr = getenv("LD_PRELOAD");
  if (
    ld_preload_cstr == nullptr ||
    std::strstr(ld_preload_cstr, "libagnocast_heaphook.so") == nullptr) {
    RCLCPP_ERROR(logger, "libagnocast_heaphook.so not found in LD_PRELOAD.");
    exit(EXIT_FAILURE);
  }

  std::string ld_preload(ld_preload_cstr);
  std::vector<std::string> paths;
  std::string::size_type start = 0;
  std::string::size_type end = 0;

  while ((end = ld_preload.find(':', start)) != std::string::npos) {
    paths.push_back(ld_preload.substr(start, end - start));
    start = end + 1;
  }
  paths.push_back(ld_preload.substr(start));

  if (paths.size() == 1) {
    RCLCPP_WARN(
      logger,
      "Pre-existing shared libraries in LD_PRELOAD may have been overwritten by "
      "libagnocast_heaphook.so");
  }
}

static std::string create_mq_name(
  const std::string & header, const std::string & topic_name, const topic_local_id_t id)
{
  if (topic_name.length() == 0 || topic_name[0] != '/') {
    RCLCPP_ERROR(logger, "create_mq_name failed");
    close(agnocast_fd);
    exit(EXIT_FAILURE);
  }

  std::string mq_name = topic_name;
  mq_name[0] = '@';
  mq_name = header + mq_name + "@" + std::to_string(id);

  // As a mq_name, '/' cannot be used
  for (size_t i = 1; i < mq_name.size(); i++) {
    if (mq_name[i] == '/') {
      mq_name[i] = '_';
    }
  }

  return mq_name;
}

std::string create_mq_name_for_agnocast_publish(
  const std::string & topic_name, const topic_local_id_t id)
{
  return create_mq_name("/agnocast", topic_name, id);
}

std::string create_mq_name_for_bridge(const pid_t pid)
{
  std::string name = "/agnocast_bridge_manager@" + std::to_string(pid);
  if (pid == PERFORMANCE_BRIDGE_VIRTUAL_PID) {
    const char * domain_id = getenv("ROS_DOMAIN_ID");
    if (domain_id != nullptr) {
      name += "_d" + std::string(domain_id);
    }
  }
  return name;
}

uint64_t get_self_ipc_ns_inode()
{
  struct stat st
  {
  };
  if (stat("/proc/self/ns/ipc", &st) != 0) {
    throw std::system_error(errno, std::generic_category(), "stat(/proc/self/ns/ipc)");
  }
  return static_cast<uint64_t>(st.st_ino);
}

std::string create_shm_name(const pid_t pid)
{
  return "/agnocast@" + std::to_string(pid);
}

std::string create_service_request_topic_name(const std::string & service_name)
{
  return "/AGNOCAST_SRV_REQUEST" + service_name;
}

std::string create_service_response_topic_name(
  const std::string & service_name, const std::string & client_node_name)
{
  return "/AGNOCAST_SRV_RESPONSE" + service_name + "_SEP_" + client_node_name;
}

uint64_t agnocast_get_timestamp()
{
  auto now = std::chrono::system_clock::now();
  return std::chrono::duration_cast<std::chrono::nanoseconds>(now.time_since_epoch()).count();
}

const void * get_node_base_address(agnocast::Node * node)
{
  return static_cast<const void *>(node->get_node_base_interface().get());
}

}  // namespace agnocast
