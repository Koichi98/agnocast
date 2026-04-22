#include "agnocast/agnocast_utils.hpp"

#include "agnocast/agnocast_mq.hpp"
#include "agnocast/node/agnocast_node.hpp"

#include <unistd.h>

#include <array>
#include <climits>
#include <cstdlib>
#include <cstring>

namespace agnocast
{
rclcpp::Logger logger = rclcpp::get_logger("Agnocast");
bool is_bridge_process = false;

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

void validate_not_rclcpp_component_container()
{
  // Detect the case where an Agnocast-enabled component is loaded into a stock rclcpp_components
  // container. Such a container uses rclcpp::executors::{Single,Multi}ThreadedExecutor, which does
  // not poll Agnocast callbacks, so subscription callbacks would silently never fire.
  //
  // /proc/self/exe resolves to the real executable regardless of argv[0] manipulation or symlink
  // name, and is readable without extra privileges because it refers to the calling process.
  std::array<char, PATH_MAX> exe_path_buf{};
  ssize_t len = readlink("/proc/self/exe", exe_path_buf.data(), exe_path_buf.size() - 1);
  if (len <= 0) {
    // If we cannot resolve the executable path, skip the check rather than failing spuriously.
    return;
  }
  exe_path_buf[len] = '\0';

  const std::string exe_path(exe_path_buf.data());
  const std::string exe_name =
    exe_path.substr(exe_path.find_last_of('/') + 1);

  static constexpr std::array<const char *, 3> kStockContainers = {
    "component_container",
    "component_container_mt",
    "component_container_isolated",
  };

  bool matched_by_name = false;
  for (const char * name : kStockContainers) {
    if (exe_name == name) {
      matched_by_name = true;
      break;
    }
  }
  const bool matched_by_path = exe_path.find("/rclcpp_components/") != std::string::npos;

  if (!matched_by_name && !matched_by_path) {
    return;
  }

  const char * escape = getenv("AGNOCAST_ALLOW_NON_AGNOCAST_CONTAINER");
  if (escape != nullptr && std::strcmp(escape, "1") == 0) {
    RCLCPP_WARN(
      logger,
      "Running an Agnocast pub/sub inside stock rclcpp_components container '%s' "
      "(AGNOCAST_ALLOW_NON_AGNOCAST_CONTAINER=1 set, continuing).",
      exe_path.c_str());
    return;
  }

  RCLCPP_ERROR(
    logger,
    "Agnocast pub/sub is being created inside a stock rclcpp_components container ('%s'). "
    "Stock containers use rclcpp executors and do not dispatch Agnocast callbacks. "
    "Use agnocast_component_container / agnocast_component_container_mt / "
    "agnocast_component_container_cie instead. "
    "Set AGNOCAST_ALLOW_NON_AGNOCAST_CONTAINER=1 to bypass this check.",
    exe_path.c_str());
  close(agnocast_fd);
  exit(EXIT_FAILURE);
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
