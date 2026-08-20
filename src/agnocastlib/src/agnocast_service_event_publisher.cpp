#include "agnocast/agnocast_service_event_publisher.hpp"

#include "builtin_interfaces/msg/time.hpp"
#include "rclcpp/serialization.hpp"
#include "rclcpp/serialized_message.hpp"
#include "rcutils/allocator.h"
#include "rosidl_runtime_c/service_type_support_struct.h"

#include <service_msgs/msg/service_event_info.hpp>

#include <cstring>

using service_msgs::msg::ServiceEventInfo;

namespace agnocast
{

std::pair<ServiceIntrospectionState, GenericPublisher::SharedPtr> ServiceEventPublisher::snapshot()
  const
{
  std::lock_guard<std::mutex> lock(mtx_);
  return std::make_pair(state_, publisher_);
}

void ServiceEventPublisher::commit(
  ServiceIntrospectionState state, GenericPublisher::SharedPtr publisher)
{
  std::lock_guard<std::mutex> lock(mtx_);
  state_ = state;
  publisher_ = publisher;
}

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

void ServiceEventPublisher::change_state(ServiceIntrospectionState new_state)
{
  auto [state, publisher] = snapshot();

  if (state == new_state) {
    return;
  }

  // If the new state is Off, the current state is either Metadata or Contents, so destroy the
  // existing publisher. If the current state is Off, it's changing to either Metadata or Contents,
  // so create a new publisher. The remaining state transitions are between Metadata and Contents,
  // which require no action.
  if (new_state == ServiceIntrospectionState::Off) {
    commit(new_state, nullptr);
  } else if (state == ServiceIntrospectionState::Off) {
    std::visit(
      [this, new_state](auto * n) {
        auto new_publisher = std::make_shared<GenericPublisher>(
          n, event_topic_name_, event_topic_type_, event_publisher_qos_);
        commit(new_state, new_publisher);
      },
      node_);
  }

  // NOTE: The publisher is destroyed when this function returns, not in the commit(), minimizing
  // the lock window. This is because we've got the snapshot.
}

std::pair<bool, std::string> ServiceEventPublisher::publish_service_event_message(
  const uint8_t event_type, const void * payload, int64_t sequence_number,
  const uint8_t (&client_gid)[RMW_GID_STORAGE_SIZE])
{
  auto [state, publisher] = snapshot();

  if (state == ServiceIntrospectionState::Off) {
    return std::make_pair(true, "");
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
  if (state == ServiceIntrospectionState::Metadata) {
    payload = nullptr;
  }
  switch (event_type) {
    case ServiceEventInfo::REQUEST_RECEIVED:
    case ServiceEventInfo::REQUEST_SENT:
      event_msg = ts_bundle_.service_ts->event_message_create_handle_function(
        &info, &allocator, payload, nullptr);
      break;
    case ServiceEventInfo::RESPONSE_RECEIVED:
    case ServiceEventInfo::RESPONSE_SENT:
      event_msg = ts_bundle_.service_ts->event_message_create_handle_function(
        &info, &allocator, nullptr, payload);
      break;
    default:
      return std::make_pair(false, "unsupported event type");
  }
  if (event_msg == nullptr) {
    return std::make_pair(
      false, "event_message_create_handle_function() failed to create event message");
  }

  // Serialize the event message and publish it.
  rclcpp::SerializedMessage serialized_msg;
  rclcpp::SerializationBase serialization(ts_bundle_.service_ts->event_typesupport);
  try {
    serialization.serialize_message(event_msg, &serialized_msg);
  } catch (const std::exception & e) {
    ts_bundle_.service_ts->event_message_destroy_handle_function(event_msg, &allocator);
    return std::make_pair(false, "serialize_message() failed to serialize event message");
  }
  publisher->publish(serialized_msg);

  ts_bundle_.service_ts->event_message_destroy_handle_function(event_msg, &allocator);
  return std::make_pair(true, "");
}

}  // namespace agnocast
