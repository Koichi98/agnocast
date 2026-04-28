#pragma once

#include "agnocast_cie_thread_configurator/non_ros_thread_info_ipc.hpp"
#include "rclcpp/rclcpp.hpp"
#include "yaml-cpp/yaml.h"

#include "agnocast_cie_config_msgs/msg/callback_group_info.hpp"

#include <atomic>
#include <map>
#include <memory>
#include <string>
#include <vector>

class ThreadConfiguratorNode : public rclcpp::Node
{
  // Each ThreadConfig is mutated by exactly one writer: a MutuallyExclusive
  // callback-group thread for callback-group configs, and the IpcServer
  // receiver thread for non-ROS thread configs. So thread_id and applied
  // stay non-atomic. print_all_unapplied resets the IpcServer before reading
  // non-ROS configs to ensure the receiver thread is joined.
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
  void non_ros_thread_callback(const agnocast_cie_thread_configurator::NonRosThreadInfoMsg & msg);

  std::vector<rclcpp::Node::SharedPtr> nodes_for_each_domain_;
  std::vector<rclcpp::Subscription<agnocast_cie_config_msgs::msg::CallbackGroupInfo>::SharedPtr>
    subs_for_each_domain_;

  std::vector<ThreadConfig> callback_group_configs_;
  // (domain_id, callback_group_id) -> ThreadConfig*
  std::map<std::pair<size_t, std::string>, ThreadConfig *> id_to_callback_group_config_;

  std::vector<ThreadConfig> non_ros_thread_configs_;
  // thread_name -> ThreadConfig*
  std::map<std::string, ThreadConfig *> id_to_non_ros_thread_config_;

  std::atomic<int> unapplied_num_{0};
  std::atomic<int> cgroup_num_{0};
  std::atomic<bool> configured_at_least_once_{false};

  // Declared LAST so reverse-order destruction joins the receiver thread
  // before any data it reads/writes (the maps and atomics above) is destroyed.
  std::unique_ptr<agnocast_cie_thread_configurator::NonRosThreadInfoIpcServer>
    non_ros_thread_ipc_server_;
};
