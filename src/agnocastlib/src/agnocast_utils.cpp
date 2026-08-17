#include "agnocast/agnocast_utils.hpp"

#include "agnocast/node/agnocast_node.hpp"

#include <sys/stat.h>

#include <array>
#include <cerrno>
#include <cstdlib>
#include <cstring>
#include <limits>
#include <memory>
#include <system_error>

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

uint32_t get_ros_domain_id()
{
  const char * domain_id_env = getenv("ROS_DOMAIN_ID");
  if (domain_id_env == nullptr || *domain_id_env == '\0') {
    return 0;
  }
  char * end = nullptr;
  errno = 0;
  const uint64_t value = std::strtoul(domain_id_env, &end, 10);
  // Out-of-range values would silently wrap into an unintended domain (e.g. 0),
  // breaking isolation, so reject them rather than truncate.
  if (*end != '\0' || errno != 0 || value > std::numeric_limits<uint32_t>::max()) {
    return 0;
  }
  return static_cast<uint32_t>(value);
}

// UDS-address suffix that scopes the per-IPC-namespace bridge listener by domain.
// The kmod keys the bridge manager on the *parsed* domain, so this must use
// get_ros_domain_id() and not the raw env string. Domain 0 takes no suffix,
// matching the Python discovery agent (bridge_decider._bridge_uds_addr).
static std::string bridge_domain_suffix()
{
  const uint32_t domain_id = get_ros_domain_id();
  if (domain_id == 0) {
    return "";
  }
  return "_d" + std::to_string(domain_id);
}

std::string create_uds_addr_for_bridge()
{
  // Abstract-namespace UDS address is prefixed with '\0' and its length is
  // scoped by the socklen_t passed to bind()/sendto() (no trailing NUL).
  std::string addr;
  addr.push_back('\0');
  addr += "agnocast_bridge_manager_";
  addr += std::to_string(get_self_ipc_ns_inode());
  addr += bridge_domain_suffix();
  return addr;
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
  const std::string & service_name, const std::string & client_node_name, const uint32_t domain_id)
{
  // Keep the "/AGNOCAST_SRV_RESPONSE<service>_SEP_" head in sync with the prefix that
  // register_domain_bridge registers as a domain bridge rule for a bridged service.
  return "/AGNOCAST_SRV_RESPONSE" + service_name + "_SEP_" + client_node_name + "_D" +
         std::to_string(domain_id);
}

std::optional<std::pair<uint32_t, std::string>> query_domain_rule(
  const std::string & topic_name, const uint32_t domain_id)
{
  ioctl_get_domain_rule_args args = {};
  args.topic_name = {topic_name.c_str(), topic_name.size()};
  args.domain_id = domain_id;
  if (ioctl(agnocast_fd, AGNOCAST_GET_DOMAIN_RULE_CMD, &args) < 0) {
    RCLCPP_ERROR(logger, "AGNOCAST_GET_DOMAIN_RULE_CMD failed: %s", strerror(errno));
    close(agnocast_fd);
    exit(EXIT_FAILURE);
  }
  if (!args.ret_found) return std::nullopt;
  return std::make_pair(
    args.ret_peer_domain, std::string(static_cast<const char *>(args.ret_peer_topic_name)));
}

uint32_t get_agnocast_sub_count(const std::string & topic_name, const uint32_t domain_id)
{
  auto topic_info_buffer = std::make_unique<std::array<topic_info_ret, MAX_TOPIC_INFO_RET_NUM>>();

  ioctl_topic_info_args topic_info_args = {};
  topic_info_args.topic_name = {topic_name.c_str(), topic_name.size()};
  topic_info_args.topic_info_ret_buffer_addr =
    reinterpret_cast<uint64_t>(topic_info_buffer->data());
  topic_info_args.topic_info_ret_buffer_size = MAX_TOPIC_INFO_RET_NUM;
  topic_info_args.domain_id = domain_id;
  if (ioctl(agnocast_fd, AGNOCAST_GET_TOPIC_SUBSCRIBER_INFO_CMD, &topic_info_args) < 0) {
    RCLCPP_ERROR(logger, "AGNOCAST_GET_TOPIC_SUBSCRIBER_INFO_CMD failed: %s", strerror(errno));
    close(agnocast_fd);
    exit(EXIT_FAILURE);
  }

  return topic_info_args.ret_topic_info_ret_num;
}

uint32_t count_agnocast_subscribers_across_bridge(const std::string & topic_name)
{
  const uint32_t own_domain = get_ros_domain_id();
  uint32_t count = get_agnocast_sub_count(topic_name, own_domain);

  const std::optional<std::pair<uint32_t, std::string>> peer =
    query_domain_rule(topic_name, own_domain);
  if (peer.has_value()) {
    count += get_agnocast_sub_count(peer->second, peer->first);
  }
  return count;
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

const void * get_node_base_address(rclcpp::Node * node)
{
  return static_cast<const void *>(
    node->get_node_base_interface()->get_shared_rcl_node_handle().get());
}

}  // namespace agnocast
