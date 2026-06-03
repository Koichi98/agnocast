#include "agnocast/bridge/agnocast_service_bridge.hpp"

#include "agnocast/agnocast_utils.hpp"
#include "agnocast/bridge/agnocast_bridge_utils.hpp"

#include <sys/ioctl.h>

#include <algorithm>
#include <array>
#include <cassert>
#include <cerrno>
#include <cstring>
#include <utility>

namespace
{

// The global error string used by ServiceBridgeItem.
std::string error_string;

}  // namespace

namespace agnocast
{

void ServiceBridgeItem::set_error_string(const std::string & error_string)
{
  ::error_string = error_string;
}
const char * ServiceBridgeItem::get_error_string()
{
  return ::error_string.c_str();
}

// Returns nullptr if an error occurs while creating the shadow node (the error string will be set).
std::shared_ptr<rcl_node_t> ServiceBridgeItem::find_or_create_shadow_node(
  const std::unordered_map<std::string, ServiceBridgeItem> & parent_map, const char * ns,
  const char * name)
{
  for (const auto & [_, item] : parent_map) {
    const std::shared_ptr<rcl_node_t> & shadow_node = item.shadow_node_;
    if (
      shadow_node != nullptr && strcmp(rcl_node_get_name(shadow_node.get()), name) == 0 &&
      strcmp(rcl_node_get_namespace(shadow_node.get()), ns) == 0) {
      return shadow_node;
    }
  }

  rcl_context_t * rcl_ctx = rclcpp::contexts::get_global_default_context()->get_rcl_context().get();

  rcl_node_options_t options = rcl_node_get_default_options();
  options.enable_rosout = false;
  options.use_global_arguments = false;
  if (rcl_parse_arguments(0, nullptr, options.allocator, &(options.arguments)) != RCL_RET_OK) {
    rcl_reset_error();
    set_error_string("Failed to parse arguments while creating shadow node");
    return nullptr;
  }

  auto del = [](rcl_node_t * node) {
    if (rcl_node_is_valid(node)) {
      if (rcl_node_fini(node) != RCL_RET_OK) {
        RCUTILS_LOG_ERROR_NAMED(
          "agnocast_bridge", "Error in destruction of shadow node: %s", rcl_get_error_string().str);
        rcl_reset_error();
      }
    }
    delete node;
  };
  auto node = std::shared_ptr<rcl_node_t>(new rcl_node_t{}, del);

  if (rcl_node_init(node.get(), name, ns, rcl_ctx, &options) != RCL_RET_OK) {
    rcl_reset_error();
    set_error_string("Failed to initialize shadow node");
    return nullptr;
  }

  return node;
}

// Returns 0 on success, -1 on error (the error string will be set).
int ServiceBridgeItem::get_agno_service_qos(rclcpp::QoS & qos)
{
  const std::string request_topic_name = create_service_request_topic_name(service_name_);

  auto topic_info_buffer = std::make_unique<std::array<topic_info_ret, 1>>();
  ioctl_topic_info_args topic_info_args = {};
  topic_info_args.topic_name = {request_topic_name.c_str(), request_topic_name.size()};
  topic_info_args.topic_info_ret_buffer_addr =
    reinterpret_cast<uint64_t>(topic_info_buffer->data());
  topic_info_args.topic_info_ret_buffer_size = 1;

  if (ioctl(agnocast_fd, AGNOCAST_GET_TOPIC_SUBSCRIBER_INFO_CMD, &topic_info_args) < 0) {
    if (errno == ENOBUFS) {
      set_error_string("Multiple target agnocast services found");
    } else {
      set_error_string("Failed to fetch target service information from agnocast kernel module");
    }
    return -1;
  }

  if (topic_info_args.ret_topic_info_ret_num <= 0) {
    set_error_string("No target agnocast service found");
    return -1;
  }

  const topic_info_ret & info = (*topic_info_buffer)[0];

  // We know the durability policy is set to Volatile because this is a service.
  qos.keep_last(info.qos_depth)
    .durability(rclcpp::DurabilityPolicy::Volatile)
    .reliability(
      info.qos_is_reliable ? rclcpp::ReliabilityPolicy::Reliable
                           : rclcpp::ReliabilityPolicy::BestEffort);
  return 0;
}

// Returns false if the target ROS 2 service does not exist or if an exception occurs while checking
// it (the reason will be set in the error string).
bool ServiceBridgeItem::ros2_service_exists(const BridgeManagerContext & ctx)
{
  try {
    const auto services = ctx.container_node->get_service_names_and_types();
    bool exists = std::any_of(services.begin(), services.end(), [this](const auto & s) {
      return s.first == this->service_name_;
    });
    if (!exists) {
      set_error_string("No target ROS 2 service found");
    }
    return exists;
  } catch (const std::exception & e) {
    set_error_string(e.what());
    return false;
  } catch (...) {
    set_error_string("Unknown error");
    return false;
  }
}

// Returns false if the target Agnocast service does not exist or if an error occurs while checking
// it (the reason will be set in the error string).
bool ServiceBridgeItem::agno_service_exists()
{
  // TODO(bdm-k): Add a dedicated service-liveness ioctl so we can validate target service state
  // directly without using get_service_qos() as a probe.
  rclcpp::QoS qos{10};
  if (get_agno_service_qos(qos) != 0) {
    return false;
  }
  return true;
}

// Returns false if there is no target Agnocast client or if an error occurs while checking it (the
// reason will be set in the error string).
bool ServiceBridgeItem::agno_client_exists()
{
  const std::string request_topic_name = create_service_request_topic_name(service_name_);

  PublisherCountResult result = get_agnocast_publisher_count(request_topic_name);

  if (result.count == -1) {
    set_error_string("Failed to fetch publisher count from agnocast kernel module");
    return false;
  }
  if (result.count == 0) {
    set_error_string("No target Agnocast client found");
    return false;
  }
  return true;
}

// Creates and starts the R2A bridge. Relevant configuration members must be set beforehand.
// Returns 0 on success, -1 on error (the error string will be set). On error, it is guaranteed
// that the stateful members are not modified.
int ServiceBridgeItem::start_r2a_bridge(
  const std::unordered_map<std::string, ServiceBridgeItem> & parent_map,
  const BridgeManagerContext & ctx)
{
  // Warn if the target service already exists in ROS 2.
  if (ros2_service_exists(ctx)) {
    RCLCPP_WARN(
      ctx.logger,
      "Found a ROS 2 service with the same name while creating the R2A service bridge: '%s'",
      service_name_.c_str());
  }

  rclcpp::QoS service_qos{10};
  if (get_agno_service_qos(service_qos) != 0) {
    return -1;
  }

  std::shared_ptr<rcl_node_t> shadow_node;
  if (shadow_node_identity_.has_value()) {
    const char * ns = shadow_node_identity_->first.c_str();
    const char * name = shadow_node_identity_->second.c_str();
    if ((shadow_node = find_or_create_shadow_node(parent_map, ns, name)) == nullptr) {
      return -1;
    }
  }

  ServiceBridgeEntity entity;
  if (service_type_.has_value() && ctx.performance_loader != nullptr) {
    entity = ctx.performance_loader->create_r2a_service_bridge(
      ctx.container_node, service_name_, *service_type_, service_qos);
  } else if (factory_spec_.has_value() && ctx.standard_loader != nullptr) {
    // TODO(bdm-k): Populate this once the standard bridge loader is updated.
    set_error_string("Standard bridge support is not yet implemented");
    return -1;
  } else {
    assert(false && "missing configuration members or bridge loader");
  }

  if (entity.entity_handle == nullptr) {
    set_error_string("Bridge loader returned nullptr");
    return -1;
  }

  state_ = ServiceBridgeState::R2A;
  entity_ = std::move(entity);
  shadow_node_ = std::move(shadow_node);
  return 0;
}

// Creates and starts the A2R bridge. Relevant configuration members must be set beforehand.
// Returns 0 on success, -1 on error (the error string will be set). On error, it is guaranteed
// that the stateful members are not modified.
int ServiceBridgeItem::start_a2r_bridge(const BridgeManagerContext & ctx)
{
  ServiceBridgeEntity entity;
  if (service_type_.has_value() && ctx.performance_loader != nullptr) {
    entity = ctx.performance_loader->create_a2r_service_bridge(
      ctx.container_node, service_name_, *service_type_, rclcpp::ServicesQoS());
  } else if (factory_spec_.has_value() && ctx.standard_loader != nullptr) {
    // TODO(bdm-k): Populate this once the standard bridge loader is updated.
    set_error_string("Standard bridge support is not yet implemented");
    return -1;
  } else {
    assert(false && "missing configuration members or bridge loader");
  }

  if (entity.entity_handle == nullptr) {
    set_error_string("Bridge loader returned nullptr");
    return -1;
  }

  state_ = ServiceBridgeState::A2R;
  entity_ = entity;
  shadow_node_ = nullptr;
  return 0;
}

void ServiceBridgeItem::update_configuration(const MqMsgBridge & msg)
{
  if (service_name_.empty()) {
    service_name_ = static_cast<const char *>(msg.srv_target.service_name);
  }
  if (!factory_spec_.has_value()) {
    BridgeFactorySpec factory_spec;
    if (msg.factory.in_main_executable) {
      factory_spec.shared_lib_path = std::nullopt;
    } else {
      factory_spec.shared_lib_path =
        std::string(static_cast<const char *>(msg.factory.shared_lib_path));
    }
    factory_spec.fn_offset_r2a = msg.factory.fn_offset_r2a;
    factory_spec.fn_offset_a2r = msg.factory.fn_offset_a2r;
    factory_spec_ = std::move(factory_spec);
  }
  if (!shadow_node_identity_.has_value() && msg.srv_target.create_shadow_node) {
    shadow_node_identity_ = {
      static_cast<const char *>(msg.srv_target.shadow_node_namespace),
      static_cast<const char *>(msg.srv_target.shadow_node_name)};
  }

  if (msg.direction == BridgeDirection::ROS2_TO_AGNOCAST) {
    may_start_r2a_bridge_ = true;
  } else {
    may_start_a2r_bridge_ = true;
  }
}

void ServiceBridgeItem::update_configuration(const MqMsgPerformanceBridge & msg)
{
  if (service_name_.empty()) {
    service_name_ = static_cast<const char *>(msg.srv_target.service_name);
  }
  if (!service_type_.has_value()) {
    service_type_ = static_cast<const char *>(msg.srv_target.service_type);
  }
  if (!shadow_node_identity_.has_value() && msg.srv_target.create_shadow_node) {
    shadow_node_identity_ = {
      static_cast<const char *>(msg.srv_target.shadow_node_namespace),
      static_cast<const char *>(msg.srv_target.shadow_node_name)};
  }

  if (msg.direction == BridgeDirection::ROS2_TO_AGNOCAST) {
    may_start_r2a_bridge_ = true;
  } else {
    may_start_a2r_bridge_ = true;
  }
}

void ServiceBridgeItem::check_and_update_r2a(const BridgeManagerContext & ctx)
{
  if (agno_service_exists()) {
    return;
  }

  RCLCPP_WARN(
    ctx.logger, "Removing R2A service bridge for '%s': %s", service_name_.c_str(),
    get_error_string());

  if (entity_.srv_cb_group) {
    ctx.executor->stop_callback_group(entity_.srv_cb_group);
  }
  if (entity_.client_cb_group) {
    ctx.executor->stop_callback_group(entity_.client_cb_group);
  }

  state_ = ServiceBridgeState::PENDING;
  entity_ = {nullptr, nullptr, nullptr};
  shadow_node_ = nullptr;
}

void ServiceBridgeItem::check_and_update_a2r(const BridgeManagerContext & ctx)
{
  if (ros2_service_exists(ctx)) {
    return;
  }

  RCLCPP_WARN(
    ctx.logger, "Removing A2R service bridge for '%s': %s", service_name_.c_str(),
    get_error_string());

  if (entity_.srv_cb_group) {
    ctx.executor->stop_callback_group(entity_.srv_cb_group);
  }
  if (entity_.client_cb_group) {
    ctx.executor->stop_callback_group(entity_.client_cb_group);
  }

  state_ = ServiceBridgeState::PENDING;
  entity_ = {nullptr, nullptr, nullptr};
  shadow_node_ = nullptr;
}

void ServiceBridgeItem::check_and_update_pending(
  const std::unordered_map<std::string, ServiceBridgeItem> & parent_map,
  const BridgeManagerContext & ctx)
{
  if (may_start_r2a_bridge_ && agno_service_exists()) {
    if (start_r2a_bridge(parent_map, ctx) != 0) {
      RCLCPP_WARN(
        ctx.logger, "Failed to start R2A service bridge for '%s': %s", service_name_.c_str(),
        get_error_string());
    }
    return;
  }

  if (may_start_a2r_bridge_ && ros2_service_exists(ctx)) {
    if (start_a2r_bridge(ctx) != 0) {
      RCLCPP_WARN(
        ctx.logger, "Failed to start A2R service bridge for '%s': %s", service_name_.c_str(),
        get_error_string());
    }
    return;
  }

  if (!agno_client_exists()) {
    RCLCPP_WARN(
      ctx.logger, "Removing service bridge state-machine for '%s': %s", service_name_.c_str(),
      get_error_string());

    state_ = ServiceBridgeState::NONE;
  }
}

void ServiceBridgeItem::check_and_update(
  const std::unordered_map<std::string, ServiceBridgeItem> & parent_map,
  const BridgeManagerContext & ctx)
{
  switch (state_) {
    case ServiceBridgeState::PENDING:
      check_and_update_pending(parent_map, ctx);
      break;
    case ServiceBridgeState::R2A:
      check_and_update_r2a(ctx);
      break;
    case ServiceBridgeState::A2R:
      check_and_update_a2r(ctx);
      break;
    default:
      break;
  }
}

void ServiceBridgeItem::handle_request(
  const MqMsgBridge & msg, const std::unordered_map<std::string, ServiceBridgeItem> & parent_map,
  const BridgeManagerContext & ctx)
{
  update_configuration(msg);

  switch (msg.direction) {
    case BridgeDirection::ROS2_TO_AGNOCAST:
      if (state_ == ServiceBridgeState::NONE || state_ == ServiceBridgeState::PENDING) {
        if (start_r2a_bridge(parent_map, ctx) != 0) {
          RCLCPP_WARN(
            ctx.logger, "Failed to start R2A service bridge for '%s': %s", service_name_.c_str(),
            get_error_string());
        }
      }
      break;
    case BridgeDirection::AGNOCAST_TO_ROS2:
      if (state_ == ServiceBridgeState::NONE) {
        state_ = ServiceBridgeState::PENDING;
      }
      break;
  }
}

void ServiceBridgeItem::handle_request(
  const MqMsgPerformanceBridge & msg,
  const std::unordered_map<std::string, ServiceBridgeItem> & parent_map,
  const BridgeManagerContext & ctx)
{
  update_configuration(msg);

  switch (msg.direction) {
    case BridgeDirection::ROS2_TO_AGNOCAST:
      if (state_ == ServiceBridgeState::NONE || state_ == ServiceBridgeState::PENDING) {
        if (start_r2a_bridge(parent_map, ctx) != 0) {
          RCLCPP_WARN(
            ctx.logger, "Failed to start R2A service bridge for '%s': %s", service_name_.c_str(),
            get_error_string());
        }
      }
      break;
    case BridgeDirection::AGNOCAST_TO_ROS2:
      if (state_ == ServiceBridgeState::NONE) {
        state_ = ServiceBridgeState::PENDING;
      }
      break;
  }
}

}  // namespace agnocast
