#pragma once

#include "agnocast/agnocast_callback_isolated_executor.hpp"
#include "agnocast/agnocast_mq.hpp"
#include "agnocast/bridge/performance/agnocast_performance_bridge_loader.hpp"
#include "agnocast/bridge/standard/agnocast_standard_bridge_loader.hpp"

#include <rclcpp/rclcpp.hpp>

#include <memory>
#include <optional>
#include <string>
#include <unordered_map>
#include <utility>
#include <variant>

namespace agnocast
{

struct BridgeManagerContext
{
  rclcpp::Node::SharedPtr container_node;
  std::shared_ptr<CallbackIsolatedAgnocastExecutor> executor;
  rclcpp::Logger logger;
  std::shared_ptr<PerformanceBridgeLoader> performance_loader;
  std::shared_ptr<StandardBridgeLoader> standard_loader;
};

enum class ServiceBridgeState { NONE, PENDING, A2R, R2A };

class ServiceBridgeItem
{
  // Stateful members.
  ServiceBridgeState state_ = ServiceBridgeState::NONE;
  ServiceBridgeEntity entity_ = {nullptr, nullptr, nullptr};
  std::shared_ptr<rcl_node_t> shadow_node_ = nullptr;

  // Configuration members; set once and never modified.
  std::string service_name_;
  std::optional<BridgeFactorySpec> factory_spec_ = std::nullopt;  // only for standard bridges
  std::optional<std::string> service_type_ = std::nullopt;        // only for performance bridges
  std::optional<std::pair<std::string, std::string>> shadow_node_identity_ = std::nullopt;
  bool may_start_r2a_bridge_ = false;
  bool may_start_a2r_bridge_ = false;

  static void set_error_string(const std::string & error_string);
  static const char * get_error_string();

  static std::shared_ptr<rcl_node_t> find_or_create_shadow_node(
    const std::unordered_map<std::string, ServiceBridgeItem> & parent_map, const char * ns,
    const char * name);

  int get_agno_service_qos(rclcpp::QoS & qos);

  bool ros2_service_exists(const BridgeManagerContext & ctx);
  bool agno_service_exists();
  bool agno_client_exists();

  int start_r2a_bridge(
    const std::unordered_map<std::string, ServiceBridgeItem> & parent_map,
    const BridgeManagerContext & ctx);
  int start_a2r_bridge(const BridgeManagerContext & ctx);

  void update_configuration(const MqMsgBridge & msg);
  void update_configuration(const MqMsgPerformanceBridge & msg);

  void check_and_update_r2a(const BridgeManagerContext & ctx);
  void check_and_update_a2r(const BridgeManagerContext & ctx);
  void check_and_update_pending(
    const std::unordered_map<std::string, ServiceBridgeItem> & parent_map,
    const BridgeManagerContext & ctx);

public:
  ServiceBridgeState state() const { return state_; }
  const std::string & service_name() const { return service_name_; }

  void check_and_update(
    const std::unordered_map<std::string, ServiceBridgeItem> & parent_map,
    const BridgeManagerContext & ctx);

  void handle_request(
    const MqMsgBridge & msg, const std::unordered_map<std::string, ServiceBridgeItem> & parent_map,
    const BridgeManagerContext & ctx);
  void handle_request(
    const MqMsgPerformanceBridge & msg,
    const std::unordered_map<std::string, ServiceBridgeItem> & parent_map,
    const BridgeManagerContext & ctx);
};

}  // namespace agnocast
