// ROS 2 service introspection for Agnocast services.
//
// rclcpp implements service introspection inside rcl: Service/Client own an rcl handle, and
// configure_introspection() asks rcl to publish a `<service name>/_service_event` topic. Agnocast
// services never touch rcl (agnocast::Node does not even own an rcl_node_t), so the same feature is
// re-implemented here on top of an ordinary Agnocast publisher. The event topic is created with
// PublisherRole::Default, so the ROS 2 bridge exposes it to DDS and `ros2 service echo` sees the
// events just like it would for an rclcpp service.
//
// The `ServiceT::Event` message and rcl/service_introspection.h only exist from ROS 2 Iron
// (rclcpp 21) onwards, so everything here is gated on AGNOCAST_SERVICE_INTROSPECTION_SUPPORTED.
// On Humble the gate is 0 and Service/Client do not declare configure_introspection() at all,
// matching rclcpp.

#pragma once

#include <rclcpp/version.h>

#if RCLCPP_VERSION_GTE(21, 0, 0)
#include <rcl/service_introspection.h>
#define AGNOCAST_SERVICE_INTROSPECTION_SUPPORTED 1
#else
#define AGNOCAST_SERVICE_INTROSPECTION_SUPPORTED 0
#endif

#include "agnocast/agnocast_publisher.hpp"
#include "agnocast/agnocast_utils.hpp"
#include "rclcpp/rclcpp.hpp"

#include <array>
#include <atomic>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <utility>
#include <variant>

namespace agnocast
{

class Node;

namespace internal
{

/// Node handle held by a Service/Client, used to create the event publisher lazily.
using ServiceNodeVariant = std::variant<rclcpp::Node *, agnocast::Node *>;

/// Return the topic name ROS 2 uses for service introspection events, i.e. the resolved service
/// name with "/_service_event" appended.
std::string create_service_event_topic_name(const std::string & resolved_service_name);

/// Derive the 16-byte `client_gid` reported in service events from the calling node's
/// fully-qualified name.
///
/// ROS 2 uses the client's rmw GID here. Agnocast has no rmw entity for a service client, and the
/// only client identity carried on the wire is the node name (see
/// agnocast/internal/service_wire_type.hpp), so the GID is derived from that name instead. Both
/// ends of a call compute it independently and agree, without changing the wire format.
///
/// Limitation: two Client instances for the same service inside one node share a GID, while their
/// sequence numbers are counted per instance, so (client_gid, sequence_number) can collide between
/// them. Distinguishing them requires carrying an identifier on the wire.
std::array<uint8_t, 16> make_service_client_gid(const std::string & node_fqn);

#if AGNOCAST_SERVICE_INTROSPECTION_SUPPORTED

/// Introspection state shared by BasicService and Client.
///
/// Holds the configured state, the clock used for event timestamps, and the event publisher.
/// `is_enabled()` is a single relaxed atomic load, so a service that never enables introspection
/// pays nothing beyond that on the request/response path.
template <typename ServiceT>
class ServiceIntrospection
{
  using EventT = typename ServiceT::Event;
  using EventPublisher = Publisher<EventT>;
  // service_msgs::msg::ServiceEventInfo, reached through the event message so that this header
  // needs no service_msgs include and agnocastlib needs no service_msgs dependency.
  using EventInfoT = decltype(std::declval<EventT>().info);

  // Stored as the underlying integer so it can live in an atomic.
  std::atomic<int> state_{RCL_SERVICE_INTROSPECTION_OFF};
  std::mutex mtx_;
  rclcpp::Clock::SharedPtr clock_;
  typename EventPublisher::SharedPtr publisher_;

  /// @brief Publish one service event.
  ///
  /// `request` and `response` are only read in RCL_SERVICE_INTROSPECTION_CONTENTS mode and either
  /// may be null; ROS 2 fills at most one of them per event. Callers must pass payloads that are
  /// still valid, which on the publishing side means calling this before handing the message to
  /// Publisher::publish().
  void emit(
    uint8_t event_type, const std::array<uint8_t, 16> & client_gid, int64_t sequence_number,
    const typename ServiceT::Request * request, const typename ServiceT::Response * response)
  {
    typename EventPublisher::SharedPtr publisher;
    rclcpp::Clock::SharedPtr clock;
    int state = RCL_SERVICE_INTROSPECTION_OFF;
    {
      std::lock_guard<std::mutex> lock(mtx_);
      state = state_.load(std::memory_order_relaxed);
      if (state == RCL_SERVICE_INTROSPECTION_OFF) {
        return;
      }
      publisher = publisher_;
      clock = clock_;
    }
    if (!publisher || !clock) {
      return;
    }

    ipc_shared_ptr<EventT> event = publisher->borrow_loaned_message();
    event->info.event_type = event_type;
    event->info.stamp = clock->now();
    event->info.client_gid = client_gid;
    event->info.sequence_number = sequence_number;
    event->request.clear();
    event->response.clear();

    if (state == RCL_SERVICE_INTROSPECTION_CONTENTS) {
      if (request != nullptr) {
        event->request.push_back(*request);
      }
      if (response != nullptr) {
        event->response.push_back(*response);
      }
    }

    publisher->publish(std::move(event));
  }

public:
  /// @brief Enable, reconfigure or disable introspection. Mirrors rclcpp's semantics: passing
  /// RCL_SERVICE_INTROSPECTION_OFF stops event publication, and passing a different state switches
  /// modes. The event publisher is created on the first enabling call and kept afterwards, so
  /// toggling does not tear the topic down.
  /// @param node The node that owns the service or client.
  /// @param resolved_service_name The service name after remapping/expansion.
  /// @param clock Clock used to stamp events.
  /// @param qos QoS of the event publisher.
  /// @param state Introspection state to apply.
  void configure(
    const ServiceNodeVariant & node, const std::string & resolved_service_name,
    rclcpp::Clock::SharedPtr clock, const rclcpp::QoS & qos,
    rcl_service_introspection_state_t state)
  {
    std::lock_guard<std::mutex> lock(mtx_);

    if (state == RCL_SERVICE_INTROSPECTION_OFF) {
      state_.store(RCL_SERVICE_INTROSPECTION_OFF, std::memory_order_relaxed);
      return;
    }

    clock_ = std::move(clock);

    if (!publisher_) {
      // TransientLocal durability is not allowed for services, and events follow the same rule.
      const rclcpp::QoS event_qos = rclcpp::QoS(qos).durability_volatile();
      const std::string topic_name = create_service_event_topic_name(resolved_service_name);
      std::visit(
        [this, &topic_name, &event_qos](auto * n) {
          publisher_ = std::make_shared<EventPublisher>(
            n, topic_name, event_qos, agnocast::PublisherOptions{}, PublisherRole::Default);
        },
        node);
    }

    state_.store(state, std::memory_order_relaxed);
  }

  /// @brief Whether any event should be published. Checked before doing any work on the hot path.
  bool is_enabled() const
  {
    return state_.load(std::memory_order_relaxed) != RCL_SERVICE_INTROSPECTION_OFF;
  }

  /// @brief Publish a REQUEST_SENT event (client side, before the request is published).
  /// @param client_gid GID of the calling client, from make_service_client_gid().
  /// @param sequence_number The Agnocast seqno of the call.
  /// @param request Request payload; only read in CONTENTS mode.
  void emit_request_sent(
    const std::array<uint8_t, 16> & client_gid, int64_t sequence_number,
    const typename ServiceT::Request * request)
  {
    emit(EventInfoT::REQUEST_SENT, client_gid, sequence_number, request, nullptr);
  }

  /// @brief Publish a REQUEST_RECEIVED event (server side, before the user callback runs).
  void emit_request_received(
    const std::array<uint8_t, 16> & client_gid, int64_t sequence_number,
    const typename ServiceT::Request * request)
  {
    emit(EventInfoT::REQUEST_RECEIVED, client_gid, sequence_number, request, nullptr);
  }

  /// @brief Publish a RESPONSE_SENT event (server side, before the response is published).
  void emit_response_sent(
    const std::array<uint8_t, 16> & client_gid, int64_t sequence_number,
    const typename ServiceT::Response * response)
  {
    emit(EventInfoT::RESPONSE_SENT, client_gid, sequence_number, nullptr, response);
  }

  /// @brief Publish a RESPONSE_RECEIVED event (client side, before the future is fulfilled).
  void emit_response_received(
    const std::array<uint8_t, 16> & client_gid, int64_t sequence_number,
    const typename ServiceT::Response * response)
  {
    emit(EventInfoT::RESPONSE_RECEIVED, client_gid, sequence_number, nullptr, response);
  }
};

#endif  // AGNOCAST_SERVICE_INTROSPECTION_SUPPORTED

}  // namespace internal

}  // namespace agnocast
