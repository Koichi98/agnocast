// Sample component-style subscriber built on agnocast::GenericSubscription.
//
// Mirrors the structure of MinimalSubscriber (src/minimal_subscriber.cpp): a class derived
// from rclcpp::Node, registered as a composable node via RCLCPP_COMPONENTS_REGISTER_NODE,
// loaded into agnocast_component_container by launch/generic_listener.launch.xml.
//
// The difference from MinimalSubscriber: the message type is NOT linked at compile time.
// It is supplied as a ROS parameter (`type`) and resolved at runtime via
// rosidl_typesupport_introspection_cpp.
//
// Per arrival, prints one line matching the typed listener output:
//   [INFO] [...] [generic_subscriber]: subscribe message: id=N
// where N is the value of the field named `field` (default "id") if it exists, otherwise
// the kernel-side entry id.

#include "agnocast/agnocast.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rosidl_typesupport_introspection_cpp/field_types.hpp"

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

// Read an integer-typed field from `base + offset` and widen it to int64. Returns
// std::nullopt for non-integer field types (so the caller can fall back gracefully).
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

class GenericSubscriber : public rclcpp::Node
{
  std::unique_ptr<agnocast::GenericSubscription> sub_;
  std::optional<agnocast::FieldInfo> id_field_;
  std::string field_name_;
  std::thread worker_;
  std::atomic<bool> stop_{false};

  // GenericSubscription is not callback-driven (it exposes mq_fd() + drain()), so we run a
  // dedicated worker thread that wakes on MQ events. This is independent of the rclcpp
  // executor that spins the Node itself.
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
  explicit GenericSubscriber(const rclcpp::NodeOptions & options)
  : Node("generic_subscriber", options)
  {
    const std::string topic_name =
      declare_parameter<std::string>("topic", "/my_topic");
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
    worker_ = std::thread(&GenericSubscriber::worker_loop, this);
  }

  ~GenericSubscriber() override
  {
    stop_.store(true, std::memory_order_relaxed);
    if (worker_.joinable()) {
      worker_.join();
    }
  }
};

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(GenericSubscriber)
