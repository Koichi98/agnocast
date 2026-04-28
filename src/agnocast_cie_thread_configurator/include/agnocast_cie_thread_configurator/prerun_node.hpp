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
  // Runs on the IPC receiver thread (not the executor); acquires non_ros_thread_mutex_.
  void non_ros_thread_callback(const agnocast_cie_thread_configurator::NonRosThreadInfoMsg & msg);

  std::vector<rclcpp::Node::SharedPtr> nodes_for_each_domain_;
  std::vector<rclcpp::Subscription<agnocast_cie_config_msgs::msg::CallbackGroupInfo>::SharedPtr>
    subs_for_each_domain_;

  // (domain_id, callback_group_id); only touched by the executor thread.
  std::set<std::pair<size_t, std::string>> domain_and_cbg_ids_;

  // Written by the IPC receiver thread, read by the executor when dumping YAML.
  // std::less<> enables heterogeneous lookup so the receiver callback can probe membership
  // from a string_view without allocating a std::string copy on the duplicate path.
  std::mutex non_ros_thread_mutex_;
  std::set<std::string, std::less<>> non_ros_thread_names_;

  // Declared last so it is destroyed first, before the state its callback touches.
  std::unique_ptr<agnocast_cie_thread_configurator::NonRosThreadInfoIpcServer> non_ros_thread_ipc_;
};
