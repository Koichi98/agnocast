#include "agnocast_cie_thread_configurator/prerun_node.hpp"

#include "agnocast_cie_thread_configurator/cie_thread_configurator.hpp"
#include "rclcpp/rclcpp.hpp"
#include "yaml-cpp/yaml.h"

#include "agnocast_cie_config_msgs/msg/callback_group_info.hpp"

#include <filesystem>
#include <fstream>
#include <iostream>
#include <set>
#include <string>
#include <string_view>

PrerunNode::PrerunNode(const rclcpp::NodeOptions & options) : Node("prerun_node", options)
{
  // https://docs.ros.org/en/rolling/Concepts/Intermediate/About-Domain-ID.html#choosing-a-domain-id-short-version
  constexpr size_t max_domain_id = 101;

  const auto domains =
    this->declare_parameter<std::vector<int64_t>>("domains", std::vector<int64_t>{});
  std::set<size_t> domain_ids;
  for (const auto raw_domain_id : domains) {
    if (raw_domain_id < 0) {
      RCLCPP_WARN(
        this->get_logger(), "Negative domain ID %lld is invalid. Skipping.",
        static_cast<long long>(raw_domain_id));
      continue;
    }

    const size_t domain_id = static_cast<size_t>(raw_domain_id);
    if (domain_id > max_domain_id) {
      RCLCPP_WARN(
        this->get_logger(), "Domain ID %zu exceeds maximum valid value (%zu). Skipping.", domain_id,
        max_domain_id);
      continue;
    }

    domain_ids.insert(domain_id);
  }

  size_t default_domain_id = agnocast_cie_thread_configurator::get_default_domain_id();

  auto cbg_qos = rclcpp::QoS(rclcpp::KeepAll()).reliable().transient_local();

  // Create subscription for default domain on this node
  subs_for_each_domain_.push_back(
    this->create_subscription<agnocast_cie_config_msgs::msg::CallbackGroupInfo>(
      "/agnocast_cie_thread_configurator/callback_group_info", cbg_qos,
      [this,
       default_domain_id](const agnocast_cie_config_msgs::msg::CallbackGroupInfo::SharedPtr msg) {
        this->topic_callback(default_domain_id, msg);
      }));

  // Create nodes and subscriptions for other domain IDs
  for (size_t domain_id : domain_ids) {
    if (domain_id == default_domain_id) {
      continue;
    }

    auto node = agnocast_cie_thread_configurator::create_node_for_domain(domain_id);
    nodes_for_each_domain_.push_back(node);

    auto sub = node->create_subscription<agnocast_cie_config_msgs::msg::CallbackGroupInfo>(
      "/agnocast_cie_thread_configurator/callback_group_info", cbg_qos,
      [this, domain_id](const agnocast_cie_config_msgs::msg::CallbackGroupInfo::SharedPtr msg) {
        this->topic_callback(domain_id, msg);
      });
    subs_for_each_domain_.push_back(sub);

    RCLCPP_INFO(this->get_logger(), "Created subscription for domain ID: %zu", domain_id);
  }

  // Start last: the callback can fire as soon as the receiver thread is up.
  non_ros_thread_ipc_ =
    std::make_unique<agnocast_cie_thread_configurator::NonRosThreadInfoIpcServer>(
      this->get_logger(),
      [this](const agnocast_cie_thread_configurator::NonRosThreadInfoMsg & msg) {
        this->non_ros_thread_callback(msg);
      });
}

void PrerunNode::topic_callback(
  size_t domain_id, const agnocast_cie_config_msgs::msg::CallbackGroupInfo::SharedPtr msg)
{
  auto key = std::make_pair(domain_id, msg->callback_group_id);
  if (domain_and_cbg_ids_.find(key) != domain_and_cbg_ids_.end()) {
    return;
  }

  RCLCPP_INFO(
    this->get_logger(), "Received CallbackGroupInfo: domain=%zu | tid=%ld | %s", domain_id,
    msg->thread_id, msg->callback_group_id.c_str());

  domain_and_cbg_ids_.insert(key);
}

void PrerunNode::non_ros_thread_callback(
  const agnocast_cie_thread_configurator::NonRosThreadInfoMsg & msg)
{
  std::lock_guard<std::mutex> lock(non_ros_thread_mutex_);

  // Heterogeneous lookup avoids allocating a std::string for the duplicate-check probe.
  // msg.thread_name is NUL-terminated (the receiver pins thread_name[kNonRosThreadNameMax]).
  const std::string_view thread_name_view{msg.thread_name};
  if (non_ros_thread_names_.find(thread_name_view) != non_ros_thread_names_.end()) {
    RCLCPP_ERROR(
      this->get_logger(), "Duplicate thread_name received: tid=%ld | %s", msg.thread_id,
      msg.thread_name);
    return;
  }

  RCLCPP_INFO(
    this->get_logger(), "Received NonRosThreadInfo: tid=%ld | %s", msg.thread_id, msg.thread_name);

  non_ros_thread_names_.emplace(thread_name_view);
}

const std::vector<rclcpp::Node::SharedPtr> & PrerunNode::get_domain_nodes() const
{
  return nodes_for_each_domain_;
}

void PrerunNode::dump_yaml_config(std::filesystem::path path)
{
  YAML::Emitter out;

  out << YAML::BeginMap;

  // Add hardware information section
  out << YAML::Key << "hardware_info";
  out << YAML::Value << YAML::BeginMap;

  auto hw_info = agnocast_cie_thread_configurator::get_hardware_info();

  for (const auto & [key, value] : hw_info) {
    out << YAML::Key << key << YAML::Value << value;
  }

  out << YAML::EndMap;

  // Add rt_throttling section
  out << YAML::Key << "rt_throttling";
  out << YAML::Value << YAML::BeginMap;
  out << YAML::Key << "runtime_us" << YAML::Value << 950000;
  out << YAML::Key << "period_us" << YAML::Value << 1000000;
  out << YAML::EndMap;

  // Add callback_groups section
  out << YAML::Key << "callback_groups";
  out << YAML::Value << YAML::BeginSeq;

  for (const auto & [domain_id, callback_group_id] : domain_and_cbg_ids_) {
    out << YAML::BeginMap;
    out << YAML::Key << "id" << YAML::Value << callback_group_id;
    out << YAML::Key << "domain_id" << YAML::Value << domain_id;
    out << YAML::Key << "affinity" << YAML::Value << YAML::Null;
    out << YAML::Key << "policy" << YAML::Value << "SCHED_OTHER";
    out << YAML::Key << "priority" << YAML::Value << 0;
    out << YAML::EndMap;
    out << YAML::Newline;
  }

  out << YAML::EndSeq;

  // Add non_ros_threads section
  out << YAML::Key << "non_ros_threads";
  out << YAML::Value << YAML::BeginSeq;

  // The receiver thread may still be alive; snapshot under the lock and release it
  // before YAML emission. The set is small (one entry per registered non-ROS thread)
  // and yaml-cpp emission heap-allocates per entry, so doing it under the lock would
  // unnecessarily block the receiver from inserting new registrations.
  const auto snapshot = [this] {
    std::lock_guard<std::mutex> lock(non_ros_thread_mutex_);
    return non_ros_thread_names_;
  }();
  for (const auto & thread_name : snapshot) {
    out << YAML::BeginMap;
    out << YAML::Key << "name" << YAML::Value << thread_name;
    out << YAML::Key << "affinity" << YAML::Value << YAML::Null;
    out << YAML::Key << "policy" << YAML::Value << "SCHED_OTHER";
    out << YAML::Key << "priority" << YAML::Value << 0;
    out << YAML::EndMap;
    out << YAML::Newline;
  }

  out << YAML::EndSeq;
  out << YAML::EndMap;

  std::ofstream fout(path / "template.yaml");
  fout << out.c_str();
  fout.close();

  std::cout << "template.yaml is created in the current directory" << std::endl;
}
