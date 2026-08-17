// Copyright 2026
// SPDX-License-Identifier: Apache-2.0

#include "agnocast/internal/ros2_node_registry_reader.hpp"

#include <rclcpp/logging.hpp>

#include <sys/stat.h>

#include <cerrno>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <sstream>

namespace agnocast::internal
{

namespace
{
// Same tmpfs root and override as `TypeRegistryWriter`, so a hardened container needs to point
// only one environment variable at a writable directory.
std::string default_base_dir()
{
  const char * env = std::getenv("AGNOCAST_TMPFS_DIR");
  const std::string root = (env != nullptr && *env != '\0') ? env : "/dev/shm";
  return root + "/agnocast_ros2_nodes";
}

std::string g_base_dir = default_base_dir();  // NOLINT(runtime/string)

// Composes the fully qualified name the way rclcpp does, so the result can be compared with the
// node names the kmod holds (those come from `get_fully_qualified_name()`).
std::string to_fully_qualified_name(const std::string & ns, const std::string & name)
{
  if (ns.empty() || ns == "/") {
    return "/" + name;
  }
  return ns + "/" + name;
}
}  // namespace

void set_ros2_node_registry_base_dir_for_test(const std::string & dir)
{
  g_base_dir = dir;
}

void reset_ros2_node_registry_base_dir_for_test()
{
  g_base_dir = default_base_dir();
}

std::optional<std::vector<std::string>> read_ros2_node_names(
  const uint64_t ipc_ns_inode, const uint32_t domain_id)
{
  const std::string path =
    g_base_dir + "/" + std::to_string(ipc_ns_inode) + "/" + std::to_string(domain_id);

  struct stat st = {};
  if (::stat(path.c_str(), &st) != 0) {
    // No agent in this (namespace, domain): the common case in a pure-Agnocast deployment, so it
    // is not worth a log line on every call.
    return std::nullopt;
  }

  const auto age =
    std::chrono::system_clock::now() - std::chrono::system_clock::from_time_t(st.st_mtim.tv_sec);
  if (age > kStaleAfter) {
    RCLCPP_WARN_ONCE(
      rclcpp::get_logger("Agnocast"),
      "The discovery agent's ROS 2 node list ('%s') has not been updated for more than %lds, so it "
      "is ignored. get_node_names() reports Agnocast nodes only until the agent resumes.",
      path.c_str(), static_cast<long>(kStaleAfter.count()));
    return std::nullopt;
  }

  std::ifstream file(path);
  if (!file) {
    RCLCPP_WARN_ONCE(
      rclcpp::get_logger("Agnocast"), "Failed to open the ROS 2 node list ('%s'): %s", path.c_str(),
      std::strerror(errno));
    return std::nullopt;
  }

  std::vector<std::string> node_names;
  std::string line;
  // getline() strips the delimiter, so an unterminated tail is indistinguishable from a complete
  // last line here; eof() after the read tells them apart.
  while (std::getline(file, line)) {
    if (file.eof()) {
      break;  // writer was interrupted mid-line
    }
    const size_t tab = line.find('\t');
    if (tab == std::string::npos) {
      continue;  // missing the node name field
    }
    const std::string ns = line.substr(0, tab);
    std::string name = line.substr(tab + 1);
    // Ignore any additional fields a newer writer appends.
    const size_t next_tab = name.find('\t');
    if (next_tab != std::string::npos) {
      name = name.substr(0, next_tab);
    }
    if (name.empty()) {
      continue;
    }
    node_names.push_back(to_fully_qualified_name(ns, name));
  }

  return node_names;
}

}  // namespace agnocast::internal
