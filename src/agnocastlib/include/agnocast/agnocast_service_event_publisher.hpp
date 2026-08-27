#pragma once

#include <rclcpp/version.h>

// Service introspection needs the event typesupport hooks in rosidl_service_type_support_t and the
// service_msgs package, neither of which exists before Iron (rclcpp 21).
#define AGNOCAST_HAS_SERVICE_INTROSPECTION (RCLCPP_VERSION_MAJOR >= 21)

#if AGNOCAST_HAS_SERVICE_INTROSPECTION

#include "agnocast/agnocast_publisher.hpp"
#include "agnocast/internal/service_typesupport.hpp"

#include <rcl/service_introspection.h>

#include <memory>
#include <mutex>
#include <string>
#include <utility>
#include <variant>

namespace agnocast
{

class ServiceEventPublisher
{
  const std::variant<rclcpp::Node *, agnocast::Node *> node_;
  const std::string service_type_;
  const std::string event_topic_name_;
  const std::string event_topic_type_;
  const rclcpp::QoS event_publisher_qos_;
  const rclcpp::Clock::SharedPtr clock_;

  mutable std::mutex mtx_;
  rcl_service_introspection_state_t state_ = RCL_SERVICE_INTROSPECTION_OFF;
  GenericPublisher::SharedPtr publisher_ = nullptr;
  std::shared_ptr<const ServiceTsBundle> ts_bundle_;

  struct Snapshot
  {
    rcl_service_introspection_state_t state;
    GenericPublisher::SharedPtr publisher;
    std::shared_ptr<const ServiceTsBundle> ts_bundle;
  };

  Snapshot snapshot() const;
  void commit(const Snapshot & next);

public:
  explicit ServiceEventPublisher(
    std::variant<rclcpp::Node *, agnocast::Node *> node, const std::string & service_name,
    const std::string & service_type, const rclcpp::QoS & qos,
    const rclcpp::Clock::SharedPtr & clock);

  /// @brief Changes the state of the service event publisher (thread-safe).
  /// @param new_state The new state to set.
  /// @throws std::runtime_error if the typesupport libraries cannot be loaded. Only the first
  /// transition out of OFF loads them.
  void change_state(rcl_service_introspection_state_t new_state);

  /// @brief Publishes a service event message (thread-safe).
  /// @param event_type The event type.
  /// @param payload A pointer to the request/response payload.
  /// @param sequence_number The sequence number of the (corresponding) request.
  /// @param client_gid The GID of the client that triggered the request.
  /// @return A pair consisting of a success flag and an error message (if any).
  std::pair<bool, std::string> publish_service_event_message(
    const uint8_t event_type, const void * payload, int64_t sequence_number,
    const uint8_t (&client_gid)[RMW_GID_STORAGE_SIZE]);
};

}  // namespace agnocast

#endif  // AGNOCAST_HAS_SERVICE_INTROSPECTION
