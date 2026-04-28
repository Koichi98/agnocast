#pragma once

#include "agnocast_cie_thread_configurator/non_ros_thread_info_ipc.hpp"
#include "rclcpp/rclcpp.hpp"
#include "yaml-cpp/yaml.h"

#include "agnocast_cie_config_msgs/msg/callback_group_info.hpp"

#include <filesystem>
#include <memory>
#include <mutex>
#include <set>
#include <string>
#include <vector>

class PrerunNode : public rclcpp::Node
{
public:
  explicit PrerunNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  void dump_yaml_config(std::filesystem::path path);

  const std::vector<rclcpp::Node::SharedPtr> & get_domain_nodes() const;

private:
  void topic_callback(
    size_t domain_id, const agnocast_cie_config_msgs::msg::CallbackGroupInfo::SharedPtr msg);
  void non_ros_thread_callback(const agnocast_cie_thread_configurator::NonRosThreadInfoMsg & msg);

  std::vector<rclcpp::Node::SharedPtr> nodes_for_each_domain_;
  std::vector<rclcpp::Subscription<agnocast_cie_config_msgs::msg::CallbackGroupInfo>::SharedPtr>
    subs_for_each_domain_;

  // (domain_id, callback_group_id) pairs. Guarded by domain_and_cbg_ids_mutex_
  // during callbacks; dump_yaml_config reads it post-spin without the lock.
  std::set<std::pair<size_t, std::string>> domain_and_cbg_ids_;
  std::mutex domain_and_cbg_ids_mutex_;
  // non-ROS thread names. Mutated only on the IpcServer receiver thread;
  // dump_yaml_config reads it post-spin. No concurrent access, no mutex needed.
  std::set<std::string> non_ros_thread_names_;

  // Declared LAST so reverse-order destruction joins the receiver thread
  // before any data it reads/writes (non_ros_thread_names_ above) is destroyed.
  std::unique_ptr<agnocast_cie_thread_configurator::NonRosThreadInfoIpcServer>
    non_ros_thread_ipc_server_;
};
