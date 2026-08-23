#pragma once

#include <rclcpp/version.h>

// Service introspection needs the event typesupport hooks in rosidl_service_type_support_t,
// rcl_service_introspection_state_t and the service_msgs package, none of which exist before
// Iron (rclcpp 21). Humble is still a supported target, so the whole feature is gated here.
// Keep this distinct from the RCLCPP_VERSION_MAJOR >= 28 checks elsewhere: those track when
// rclcpp::get_service_typesupport_handle appeared, which is an unrelated boundary.
#define AGNOCAST_HAS_SERVICE_INTROSPECTION (RCLCPP_VERSION_MAJOR >= 21)

#if AGNOCAST_HAS_SERVICE_INTROSPECTION

#include "agnocast/agnocast_publisher.hpp"
#include "agnocast/internal/service_typesupport.hpp"

#include <rcl/service_introspection.h>
#include <rmw/types.h>

#include <memory>
#include <mutex>
#include <string>
#include <utility>
#include <variant>

namespace agnocast
{

/// @brief Publishes `<service>/_service_event` messages for one service, mirroring
/// rclcpp::Service::configure_introspection.
///
/// The typesupport libraries are loaded on the first transition out of OFF, so a service that
/// never enables introspection loads nothing.
class ServiceEventPublisher
{
  const std::variant<rclcpp::Node *, agnocast::Node *> node_;
  const std::string service_type_;
  const std::string event_topic_name_;
  const std::string event_topic_type_;

  // Serializes whole transitions. Without it the read-decide-write in configure() could
  // interleave with another transition and lose an update; mtx_ alone cannot prevent that
  // because the publisher is created outside it.
  std::mutex transition_mtx_;

  mutable std::mutex mtx_;
  rcl_service_introspection_state_t state_ = RCL_SERVICE_INTROSPECTION_OFF;
  GenericPublisher::SharedPtr publisher_;
  rclcpp::Clock::SharedPtr clock_;
  std::shared_ptr<const ServiceTsBundle> ts_bundle_;

  struct Snapshot
  {
    rcl_service_introspection_state_t state;
    GenericPublisher::SharedPtr publisher;
    rclcpp::Clock::SharedPtr clock;
    std::shared_ptr<const ServiceTsBundle> ts_bundle;
  };

  Snapshot snapshot() const;
  void commit(const Snapshot & next);

public:
  ServiceEventPublisher(
    std::variant<rclcpp::Node *, agnocast::Node *> node, const std::string & service_name,
    const std::string & service_type);

  /// @brief Sets the introspection state, creating or destroying the event publisher as needed
  /// (thread-safe).
  /// @param clock The clock used to generate introspection timestamps.
  /// @param qos_service_event_pub The QoS to use when creating the event publisher.
  /// @param state The state to set introspection to.
  void configure(
    const rclcpp::Clock::SharedPtr & clock, const rclcpp::QoS & qos_service_event_pub,
    rcl_service_introspection_state_t state);

  /// @brief Publishes a service event message (thread-safe). A no-op while introspection is off.
  /// @param event_type The event type.
  /// @param payload A pointer to the request/response payload. Must point at the payload itself,
  /// not at the enclosing ServiceRequestWrapper/ServiceResponseWrapper.
  /// @param sequence_number The sequence number of the (corresponding) request.
  /// @param client_gid The GID of the client that triggered the request.
  /// @return A pair consisting of a success flag and an error message (if any).
  std::pair<bool, std::string> publish_service_event_message(
    const uint8_t event_type, const void * payload, int64_t sequence_number,
    const uint8_t (&client_gid)[RMW_GID_STORAGE_SIZE]);
};

}  // namespace agnocast

#endif  // AGNOCAST_HAS_SERVICE_INTROSPECTION
