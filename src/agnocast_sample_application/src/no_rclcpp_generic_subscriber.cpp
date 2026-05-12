// Sample subscriber: agnocast::Node-derived counterpart to GenericSubscriber.
//
// Same logic as src/generic_subscriber.cpp (the rclcpp::Node version) but uses
// agnocast::Node as the base class. Built into a separate executable via
// agnocast_components_register_node(... EXECUTOR AgnocastOnlySingleThreadedExecutor),
// matching the relationship between MinimalSubscriber (rclcpp::Node) and
// NoRclcppSubscriber (agnocast::Node).
//
// The class shows that agnocast::GenericSubscription's `agnocast::Node *` constructor
// works exactly like its `rclcpp::Node *` constructor; the only practical differences in
// this file are the base class, the executor (via the macro), and the launch-file form
// (a plain <node> instead of a composable_node inside agnocast_component_container).

#include "agnocast/agnocast.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rosidl_typesupport_introspection_cpp/field_types.hpp"

#include <rclcpp_components/register_node_macro.hpp>

#include <mqueue.h>
#include <poll.h>

#include <atomic>
#include <cstdint>
#include <cstring>
#include <memory>
#include <optional>
#include <string>
#include <thread>

namespace
{
namespace tsi = rosidl_typesupport_introspection_cpp;

std::optional<int64_t> read_integer_field(const uint8_t * base, const agnocast::FieldInfo & f)
{
  if (f.is_array) return std::nullopt;
  const uint8_t * p = base + f.offset;
  switch (f.type_id) {
    case tsi::ROS_TYPE_INT8:
      return static_cast<int64_t>(*reinterpret_cast<const int8_t *>(p));
    case tsi::ROS_TYPE_UINT8:
    case tsi::ROS_TYPE_OCTET:
      return static_cast<int64_t>(*reinterpret_cast<const uint8_t *>(p));
    case tsi::ROS_TYPE_INT16:
      return static_cast<int64_t>(*reinterpret_cast<const int16_t *>(p));
    case tsi::ROS_TYPE_UINT16:
      return static_cast<int64_t>(*reinterpret_cast<const uint16_t *>(p));
    case tsi::ROS_TYPE_INT32:
      return static_cast<int64_t>(*reinterpret_cast<const int32_t *>(p));
    case tsi::ROS_TYPE_UINT32:
      return static_cast<int64_t>(*reinterpret_cast<const uint32_t *>(p));
    case tsi::ROS_TYPE_INT64:
      return *reinterpret_cast<const int64_t *>(p);
    case tsi::ROS_TYPE_UINT64:
      return static_cast<int64_t>(*reinterpret_cast<const uint64_t *>(p));
    default:
      return std::nullopt;
  }
}

}  // namespace

class NoRclcppGenericSubscriber : public agnocast::Node
{
  std::unique_ptr<agnocast::GenericSubscription> sub_;
  std::optional<agnocast::FieldInfo> id_field_;
  std::string field_name_;
  std::thread worker_;
  std::atomic<bool> stop_{false};

  void worker_loop()
  {
    while (!stop_.load(std::memory_order_relaxed)) {
      struct pollfd pfd
      {
      };
      pfd.fd = sub_->mq_fd();
      pfd.events = POLLIN;
      const int pret = ::poll(&pfd, 1, 500);
      if (pret <= 0 || !(pfd.revents & POLLIN)) {
        continue;
      }
      char buf[64];
      while (true) {
        const ssize_t r = ::mq_receive(sub_->mq_fd(), buf, sizeof(buf), nullptr);
        if (r < 0) {
          if (errno == EAGAIN) break;
          RCLCPP_ERROR(get_logger(), "mq_receive failed: %s", std::strerror(errno));
          break;
        }
      }
      sub_->drain([this](const void * payload, int64_t entry_id) {
        if (id_field_.has_value()) {
          const auto value =
            read_integer_field(static_cast<const uint8_t *>(payload), id_field_.value());
          if (value.has_value()) {
            RCLCPP_INFO(
              get_logger(), "subscribe message: %s=%ld", field_name_.c_str(),
              static_cast<long>(value.value()));
            return;
          }
        }
        RCLCPP_INFO(
          get_logger(), "subscribe message: entry_id=%ld", static_cast<long>(entry_id));
      });
    }
  }

public:
  explicit NoRclcppGenericSubscriber(const rclcpp::NodeOptions & options)
  : agnocast::Node("no_rclcpp_generic_subscriber", options)
  {
    const std::string topic_name = declare_parameter<std::string>("topic", "/my_topic");
    const std::string type_name = declare_parameter<std::string>(
      "type", "agnocast_sample_interfaces/msg/DynamicSizeArray");
    field_name_ = declare_parameter<std::string>("field", "id");

    id_field_ = agnocast::resolve_field_offset(type_name, field_name_);
    if (!id_field_.has_value()) {
      RCLCPP_WARN(
        get_logger(), "field '%s' not found on type '%s'; will log entry_id instead",
        field_name_.c_str(), type_name.c_str());
    }

    sub_ = std::make_unique<agnocast::GenericSubscription>(this, topic_name, rclcpp::QoS(10));
    worker_ = std::thread(&NoRclcppGenericSubscriber::worker_loop, this);
  }

  ~NoRclcppGenericSubscriber()
  {
    stop_.store(true, std::memory_order_relaxed);
    if (worker_.joinable()) {
      worker_.join();
    }
  }
};

RCLCPP_COMPONENTS_REGISTER_NODE(NoRclcppGenericSubscriber)
