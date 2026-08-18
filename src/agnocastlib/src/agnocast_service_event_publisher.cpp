#include "agnocast/agnocast_service_event_publisher.hpp"

#include "builtin_interfaces/msg/time.hpp"
#include "rclcpp/serialization.hpp"
#include "rclcpp/serialized_message.hpp"
#include "rcutils/allocator.h"
#include "rosidl_runtime_c/service_type_support_struct.h"

#include <service_msgs/msg/service_event_info.hpp>

using service_msgs::msg::ServiceEventInfo;

namespace agnocast
{

ServiceEventPublisher::ServiceEventPublisher(
  std::variant<rclcpp::Node *, agnocast::Node *> node, const std::string & service_name,
  const std::string & service_type, const rclcpp::QoS & qos, const rclcpp::Clock::SharedPtr & clock)
: node_(std::move(node)),
  event_topic_name_(service_name + "/_service_event"),
  event_topic_type_(service_type + "_Event"),
  event_publisher_qos_(qos),
  clock_(clock),
  ts_bundle_(load_service_typesupport(service_type))
{
}

void ServiceEventPublisher::set_error_msg(const char * msg)
{
  error_msg_ = msg;
}
void ServiceEventPublisher::reset_error()
{
  error_msg_ = nullptr;
}
const char * ServiceEventPublisher::get_error_msg() const
{
  return error_msg_;
}

void ServiceEventPublisher::change_state(ServiceIntrospectionState new_state)
{
  if (state_ == new_state) {
    return;
  }

  // If the new state is Off, the current state is either Metadata or Contents, so destroy the
  // existing publisher. If the current state is Off, it's changing to either Metadata or Contents,
  // so create a new publisher. The remaining state transitions are between Metadata and Contents,
  // which require no action.
  if (new_state == ServiceIntrospectionState::Off) {
    publisher_.reset();
  } else if (state_ == ServiceIntrospectionState::Off) {
    std::visit(
      [this](auto * n) {
        publisher_ = std::make_shared<GenericPublisher>(
          n, event_topic_name_, event_topic_type_, event_publisher_qos_);
      },
      node_);
  }

  state_ = new_state;
}

bool ServiceEventPublisher::publish_service_event_message(
  const uint8_t event_type, const void * payload, int64_t sequence_number,
  const uint8_t (&client_gid)[RMW_GID_STORAGE_SIZE])
{
  if (state_ == ServiceIntrospectionState::Off) {
    return true;
  }

  // Prepare the introspection info (metadata).
  builtin_interfaces::msg::Time stamp = clock_->now();

  rosidl_service_introspection_info_t info = {};
  info.event_type = event_type;
  info.stamp_sec = stamp.sec;
  info.stamp_nanosec = stamp.nanosec;
  info.sequence_number = sequence_number;

  static_assert(sizeof(info.client_gid) == RMW_GID_STORAGE_SIZE);
  std::memcpy(info.client_gid, static_cast<const void *>(client_gid), RMW_GID_STORAGE_SIZE);

  // Prepare the default allocator.
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  // Construct the event message.
  void * event_msg;
  if (state_ == ServiceIntrospectionState::Metadata) {
    payload = nullptr;
  }
  switch (event_type) {
    case ServiceEventInfo::REQUEST_RECEIVED:
    case ServiceEventInfo::REQUEST_SENT:
      event_msg = ts_bundle_.service_ts_introspection->event_message_create_handle_function(
        &info, &allocator, payload, nullptr);
      break;
    case ServiceEventInfo::RESPONSE_RECEIVED:
    case ServiceEventInfo::RESPONSE_SENT:
      event_msg = ts_bundle_.service_ts_introspection->event_message_create_handle_function(
        &info, &allocator, nullptr, payload);
      break;
    default:
      set_error_msg("unsupported event type");
      return false;
  }
  if (event_msg == nullptr) {
    set_error_msg("event_message_create_handle_function() failed to create event message");
    return false;
  }

  // Serialize the event message and publish it.
  rclcpp::SerializedMessage serialized_msg;
  rclcpp::SerializationBase serialization(ts_bundle_.service_ts->event_typesupport);
  try {
    serialization.serialize_message(event_msg, &serialized_msg);
  } catch (const std::exception & e) {
    set_error_msg("serialize_message() failed to serialize event message");
    return false;
  }
  publisher_->publish(serialized_msg);

  return true;
}

}  // namespace agnocast
