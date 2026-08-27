#include "agnocast/agnocast_service_event_publisher.hpp"

#if AGNOCAST_HAS_SERVICE_INTROSPECTION

#include "builtin_interfaces/msg/time.hpp"
#include "rclcpp/serialization.hpp"
#include "rclcpp/serialized_message.hpp"
#include "rcutils/allocator.h"
#include "rosidl_runtime_c/service_type_support_struct.h"

#include <service_msgs/msg/service_event_info.hpp>

#include <cstring>
#include <memory>
#include <stdexcept>

using service_msgs::msg::ServiceEventInfo;

namespace agnocast
{

ServiceEventPublisher::Snapshot ServiceEventPublisher::snapshot() const
{
  std::lock_guard<std::mutex> lock(mtx_);
  return Snapshot{state_, publisher_, clock_, ts_bundle_};
}

void ServiceEventPublisher::commit(const Snapshot & next)
{
  std::lock_guard<std::mutex> lock(mtx_);
  state_ = next.state;
  publisher_ = next.publisher;
  clock_ = next.clock;
  ts_bundle_ = next.ts_bundle;
}

ServiceEventPublisher::ServiceEventPublisher(
  std::variant<rclcpp::Node *, agnocast::Node *> node, const std::string & service_name,
  const std::string & service_type)
: node_(std::move(node)),
  service_type_(service_type),
  event_topic_name_(service_name + "/_service_event"),
  event_topic_type_(service_type + "_Event")
{
}

void ServiceEventPublisher::configure(
  const rclcpp::Clock::SharedPtr & clock, const rclcpp::QoS & qos_service_event_pub,
  rcl_service_introspection_state_t state)
{
  if (clock == nullptr) {
    throw std::invalid_argument("a clock is required to configure service introspection");
  }

  std::lock_guard<std::mutex> transition_lock(transition_mtx_);

  Snapshot current = snapshot();

  if (current.state == state) {
    return;
  }

  if (state == RCL_SERVICE_INTROSPECTION_OFF) {
    // The bundle is carried over so that re-enabling does not load the libraries again.
    commit(Snapshot{state, nullptr, nullptr, current.ts_bundle});
    // The publisher is destroyed here rather than inside commit(), keeping mtx_ off the teardown
    // path.
    return;
  }

  // The clock and the QoS only take effect when the publisher is created, matching
  // rcl_service_configure_service_introspection: a later call that only moves between METADATA and
  // CONTENTS keeps the publisher and leaves them alone.
  if (current.publisher) {
    commit(Snapshot{state, current.publisher, current.clock, current.ts_bundle});
    return;
  }

  Snapshot next{state, nullptr, clock, current.ts_bundle};

  if (!next.ts_bundle) {
    next.ts_bundle =
      std::make_shared<const ServiceTsBundle>(load_service_typesupport(service_type_));
  }

  std::visit(
    [this, &next, &qos_service_event_pub](auto * n) {
      next.publisher = std::make_shared<GenericPublisher>(
        n, event_topic_name_, event_topic_type_, qos_service_event_pub);
    },
    node_);

  commit(next);
}

std::pair<bool, std::string> ServiceEventPublisher::publish_service_event_message(
  const uint8_t event_type, const void * payload, int64_t sequence_number,
  const uint8_t (&client_gid)[RMW_GID_STORAGE_SIZE])
{
  Snapshot active = snapshot();

  if (active.state == RCL_SERVICE_INTROSPECTION_OFF) {
    return std::make_pair(true, "");
  }

  // Prepare the introspection info (metadata).
  builtin_interfaces::msg::Time stamp = active.clock->now();

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
  if (active.state == RCL_SERVICE_INTROSPECTION_METADATA) {
    payload = nullptr;
  }
  switch (event_type) {
    case ServiceEventInfo::REQUEST_RECEIVED:
    case ServiceEventInfo::REQUEST_SENT:
      event_msg = active.ts_bundle->service_ts->event_message_create_handle_function(
        &info, &allocator, payload, nullptr);
      break;
    case ServiceEventInfo::RESPONSE_RECEIVED:
    case ServiceEventInfo::RESPONSE_SENT:
      event_msg = active.ts_bundle->service_ts->event_message_create_handle_function(
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
  rclcpp::SerializationBase serialization(active.ts_bundle->service_ts->event_typesupport);
  try {
    serialization.serialize_message(event_msg, &serialized_msg);
  } catch (const std::exception & e) {
    active.ts_bundle->service_ts->event_message_destroy_handle_function(event_msg, &allocator);
    return std::make_pair(false, "serialize_message() failed to serialize event message");
  }
  active.publisher->publish(serialized_msg);

  active.ts_bundle->service_ts->event_message_destroy_handle_function(event_msg, &allocator);
  return std::make_pair(true, "");
}

}  // namespace agnocast

#endif  // AGNOCAST_HAS_SERVICE_INTROSPECTION
