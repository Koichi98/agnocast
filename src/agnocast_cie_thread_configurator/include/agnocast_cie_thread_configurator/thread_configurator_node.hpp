#pragma once

#include "agnocast_cie_thread_configurator/non_ros_thread_info_ipc.hpp"
#include "rclcpp/rclcpp.hpp"
#include "yaml-cpp/yaml.h"

#include "agnocast_cie_config_msgs/msg/callback_group_info.hpp"

#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

class ThreadConfiguratorNode : public rclcpp::Node
{
  struct ThreadConfig
  {
    std::string thread_str;  // callback_group_id or thread_name
    size_t domain_id = 0;
    int64_t thread_id = -1;
    std::vector<int> affinity;
    std::string policy;
    int priority = 0;

    // For SCHED_DEADLINE
    unsigned int runtime = 0;
    unsigned int period = 0;
    unsigned int deadline = 0;

    bool applied = false;
  };

public:
  explicit ThreadConfiguratorNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~ThreadConfiguratorNode();
  void print_all_unapplied();

  const std::vector<rclcpp::Node::SharedPtr> & get_domain_nodes() const;

private:
  void validate_hardware_info(const YAML::Node & yaml);
  void validate_rt_throttling(const YAML::Node & yaml);
  bool set_affinity_by_cgroup(int64_t thread_id, const std::vector<int> & cpus);
  bool issue_syscalls(const ThreadConfig & config);
  void callback_group_callback(
    size_t domain_id, const agnocast_cie_config_msgs::msg::CallbackGroupInfo::SharedPtr msg);
  // Runs on the IPC receiver thread (not the executor); acquires callbacks_mutex_.
  void non_ros_thread_callback(const agnocast_cie_thread_configurator::NonRosThreadInfoMsg & msg);
  void on_all_configured();

  std::vector<rclcpp::Node::SharedPtr> nodes_for_each_domain_;
  std::vector<rclcpp::Subscription<agnocast_cie_config_msgs::msg::CallbackGroupInfo>::SharedPtr>
    subs_for_each_domain_;

  std::vector<ThreadConfig> callback_group_configs_;
  // (domain_id, callback_group_id) -> ThreadConfig*
  std::map<std::pair<size_t, std::string>, ThreadConfig *> id_to_callback_group_config_;

  std::vector<ThreadConfig> non_ros_thread_configs_;
  // thread_name -> ThreadConfig*. std::less<> enables heterogeneous lookup so the receiver
  // callback can find the entry from a string_view without allocating a std::string copy.
  std::map<std::string, ThreadConfig *, std::less<>> id_to_non_ros_thread_config_;

  // Serializes callback_group_callback (executor) and non_ros_thread_callback (receiver),
  // plus the shared counters and per-config mutable fields (thread_id, applied).
  std::mutex callbacks_mutex_;

  int unapplied_num_;
  int cgroup_num_;
  bool configured_at_least_once_ = false;

  // Declared last so it is destroyed first, before the state its callback touches.
  std::unique_ptr<agnocast_cie_thread_configurator::NonRosThreadInfoIpcServer> non_ros_thread_ipc_;
};
