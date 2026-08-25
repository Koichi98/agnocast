#include "agnocast/agnocast_service_event_publisher.hpp"

#if AGNOCAST_HAS_SERVICE_INTROSPECTION

#include "agnocast/node/agnocast_node.hpp"
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
  published_state_.store(next.state, std::memory_order_relaxed);
  publisher_ = next.publisher;
  clock_ = next.clock;
  ts_bundle_ = next.ts_bundle;
}

ServiceEventPublisher::ServiceEventPublisher(
  std::variant<rclcpp::Node *, agnocast::Node *> node, const std::string & service_name,
  const std::string & service_type, PublisherRole publisher_role)
: node_(std::move(node)),
  service_type_(service_type),
  // Matches RCL_SERVICE_INTROSPECTION_TOPIC_POSTFIX and the `<Srv>_Event` message rosidl
  // generates, so ros2 service echo can find the topic.
  event_topic_name_(service_name + RCL_SERVICE_INTROSPECTION_TOPIC_POSTFIX),
  event_topic_type_(service_type + "_Event"),
  publisher_role_(publisher_role)
{
}

void ServiceEventPublisher::configure(
  const rclcpp::Clock::SharedPtr & clock, const rclcpp::QoS & qos_service_event_pub,
  rcl_service_introspection_state_t state)
{
  // Checked before anything is committed. publish_service_event_message() dereferences the
  // clock, and a null one is undefined behaviour rather than an exception, so its try/catch
  // would not contain it. Checked for every state, as rcl does.
  if (clock == nullptr) {
    throw std::invalid_argument("a clock is required to configure service introspection");
  }

  // validate_publisher_qos() calls exit() on a QoS Agnocast cannot use. Enabling introspection
  // is a runtime toggle, typically from a parameter callback, so that would kill the node.
  if (qos_service_event_pub.history() == rclcpp::HistoryPolicy::KeepAll) {
    throw std::invalid_argument(
      "Agnocast does not support the KeepAll history policy for the service event publisher");
  }

  std::lock_guard<std::mutex> transition_lock(transition_mtx_);

  Snapshot current = snapshot();

  if (state == RCL_SERVICE_INTROSPECTION_OFF) {
    if (current.state == RCL_SERVICE_INTROSPECTION_OFF) {
      return;
    }
    // ts_bundle is carried over so re-enabling does not dlopen the service typesupport again.
    // The event publisher holds its own libraries for <Srv>_Event, and those are dropped here
    // and reloaded on the next enable.
    commit(Snapshot{state, nullptr, nullptr, current.ts_bundle});
    // The publisher is destroyed here rather than inside commit(), keeping mtx_ off the
    // teardown path.
    return;
  }

  // Both the clock and the QoS only take effect when the publisher is created, matching
  // rcl_service_configure_service_introspection: a later call that only moves between
  // METADATA and CONTENTS leaves them alone.
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
        n, event_topic_name_, event_topic_type_, qos_service_event_pub, PublisherOptions{},
        publisher_role_);
    },
    node_);

  commit(next);
}

namespace
{

const char * event_type_name(const uint8_t event_type)
{
  switch (event_type) {
    case ServiceEventInfo::REQUEST_SENT:
      return "request sent";
    case ServiceEventInfo::REQUEST_RECEIVED:
      return "request received";
    case ServiceEventInfo::RESPONSE_SENT:
      return "response sent";
    case ServiceEventInfo::RESPONSE_RECEIVED:
      return "response received";
    default:
      return "unknown";
  }
}

}  // namespace

void ServiceEventPublisher::log_failure(
  const uint8_t event_type, const char * reason) const noexcept
{
  std::visit(
    [this, event_type, reason](auto * n) {
      RCLCPP_ERROR(
        n->get_logger(), "Failed to publish the %s event on '%s': %s", event_type_name(event_type),
        event_topic_name_.c_str(), reason);
    },
    node_);
}

void ServiceEventPublisher::publish_service_event_message(
  const uint8_t event_type, const void * payload, int64_t sequence_number,
  const uint8_t (&client_gid)[RMW_GID_STORAGE_SIZE]) noexcept
{
  if (published_state_.load(std::memory_order_relaxed) == RCL_SERVICE_INTROSPECTION_OFF) {
    return;
  }

  Snapshot active = snapshot();

  // configure() may have turned introspection off between the two reads.
  if (active.state == RCL_SERVICE_INTROSPECTION_OFF) {
    return;
  }

  const bool with_payload = active.state != RCL_SERVICE_INTROSPECTION_METADATA;
  const void * request_payload = nullptr;
  const void * response_payload = nullptr;
  switch (event_type) {
    case ServiceEventInfo::REQUEST_RECEIVED:
    case ServiceEventInfo::REQUEST_SENT:
      request_payload = with_payload ? payload : nullptr;
      break;
    case ServiceEventInfo::RESPONSE_RECEIVED:
    case ServiceEventInfo::RESPONSE_SENT:
      response_payload = with_payload ? payload : nullptr;
      break;
    default:
      log_failure(event_type, "unsupported event type");
      return;
  }

  // Nothing below may escape: this runs on the request/response path, before the payload is
  // handed to the kmod, so an exception here would stop the call itself from being delivered.
  try {
    builtin_interfaces::msg::Time stamp = active.clock->now();

    rosidl_service_introspection_info_t info = {};
    info.event_type = event_type;
    info.stamp_sec = stamp.sec;
    info.stamp_nanosec = stamp.nanosec;
    info.sequence_number = sequence_number;

    static_assert(sizeof(info.client_gid) == RMW_GID_STORAGE_SIZE);
    std::memcpy(info.client_gid, static_cast<const void *>(client_gid), RMW_GID_STORAGE_SIZE);

    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    const auto * service_ts = active.ts_bundle->service_ts;

    auto destroy = [service_ts, &allocator](void * msg) {
      service_ts->event_message_destroy_handle_function(msg, &allocator);
    };
    std::unique_ptr<void, decltype(destroy)> event_msg(
      service_ts->event_message_create_handle_function(
        &info, &allocator, request_payload, response_payload),
      destroy);
    if (!event_msg) {
      log_failure(event_type, "event_message_create_handle_function() returned null");
      return;
    }

    rclcpp::SerializedMessage serialized_msg;
    rclcpp::SerializationBase serialization(service_ts->event_typesupport);
    serialization.serialize_message(event_msg.get(), &serialized_msg);
    active.publisher->publish(serialized_msg);
  } catch (const std::exception & e) {
    log_failure(event_type, e.what());
  } catch (...) {
    log_failure(event_type, "unknown exception");
  }
}

}  // namespace agnocast

#endif  // AGNOCAST_HAS_SERVICE_INTROSPECTION
