#pragma once

#include "agnocast/agnocast_publisher.hpp"
#include "agnocast/agnocast_utils.hpp"

#include <memory>
#include <string>
#include <variant>

namespace agnocast
{

enum class ServiceIntrospectionState : uint8_t {
  // Introspection disabled.
  Off,
  // Publish metadata only.
  Metadata,
  // Publish metadata and request/response payloads.
  Contents,
};

class ServiceEventPublisher
{
  const std::variant<rclcpp::Node *, agnocast::Node *> node_;
  const std::string event_topic_name_;
  const std::string event_topic_type_;
  const rclcpp::QoS event_publisher_qos_;
  const rclcpp::Clock::SharedPtr clock_;
  const ServiceTsBundle ts_bundle_;

  const char * error_msg_ = nullptr;
  ServiceIntrospectionState state_ = ServiceIntrospectionState::Off;
  GenericPublisher::SharedPtr publisher_ = nullptr;

  /// @brief Sets the error message.
  /// @param msg Must be a string literal.
  void set_error_msg(const char * msg);
  void reset_error();

public:
  explicit ServiceEventPublisher(
    std::variant<rclcpp::Node *, agnocast::Node *> node, const std::string & service_name,
    const std::string & service_type, const rclcpp::QoS & qos,
    const rclcpp::Clock::SharedPtr & clock);

  const char * get_error_msg() const;

  /// @brief Changes the state of the service event publisher.
  /// @param new_state The new state to set.
  void change_state(ServiceIntrospectionState new_state);

  /// @brief Publishes a service event message.
  /// @param event_type The event type.
  /// @param payload A pointer to the request/response payload.
  /// @param sequence_number The sequence number of the (corresponding) request.
  /// @param client_gid The GID of the client that triggered the request.
  /// @return true if the message was successfully published, false otherwise with an error message
  ///         set.
  bool publish_service_event_message(
    const uint8_t event_type, const void * payload, int64_t sequence_number,
    const uint8_t (&client_gid)[RMW_GID_STORAGE_SIZE]);
};

}  // namespace agnocast
