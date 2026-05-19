#pragma once

#include "agnocast/agnocast_ioctl.hpp"
#include "agnocast/agnocast_mq.hpp"
#include "agnocast/agnocast_public_api.hpp"
#include "agnocast/agnocast_smart_pointer.hpp"
#include "agnocast/agnocast_subscription.hpp"
#include "rclcpp/rclcpp.hpp"

#include <mqueue.h>

#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <utility>

namespace agnocast
{

class Node;

// Callback-style subscription whose message type is not known at compile time.
//
// GenericSubscription is the type-erased counterpart of agnocast::Subscription<MessageT>: it
// registers a callback with the agnocast executor exactly like a typed subscription, and the
// executor dispatches it on every publish. The only difference is the payload type: it is
// delivered as ipc_shared_ptr<const void> instead of ipc_shared_ptr<const MessageT>. The
// message type is supplied at construction time as a "<pkg>/msg/<MsgName>" string.
//
// The executor dispatch path is already type-erased (CallbackInfo in agnocast_callback_info.hpp
// stores a TypeErasedCallback plus a message_creator), so a generic callback subscription is
// simply register_callback<void>() plus a callback taking ipc_shared_ptr<const void>; no
// executor changes are required.
//
// Consumers that need to inspect the payload pair type_name() with the raw pointer
// (msg.get()) and pass them to the runtime introspection helpers in agnocast_introspection.hpp
// (resolve_field_offset / dump_message_yaml / ...). GenericSubscription itself performs no
// introspection -- it only delivers the payload and remembers the type name.
//
// Design notes:
// - BridgeRequestPolicy is skipped entirely (no MessageT, no bridge auto-spawn). External-ECU
//   reachability must come from a pre-existing typed a2r/r2a bridge pair.
// - QoS overriding via parameters is not supported (only the typed BasicSubscription path is).
class GenericSubscription : public SubscriptionBase
{
public:
  // The payload is type-erased; consumers interpret it via agnocast_introspection.hpp using
  // type_name(). It is `const void` because subscribers must not mutate received messages.
  using Callback = std::function<void(const ipc_shared_ptr<const void> &)>;
  using SharedPtr = std::shared_ptr<GenericSubscription>;

  AGNOCAST_PUBLIC
  GenericSubscription(
    rclcpp::Node * node, const std::string & topic_name, const std::string & type_name,
    const rclcpp::QoS & qos, Callback callback,
    SubscriptionOptions options = SubscriptionOptions{});

  AGNOCAST_PUBLIC
  GenericSubscription(
    agnocast::Node * node, const std::string & topic_name, const std::string & type_name,
    const rclcpp::QoS & qos, Callback callback,
    SubscriptionOptions options = SubscriptionOptions{});

  GenericSubscription(const GenericSubscription &) = delete;
  GenericSubscription & operator=(const GenericSubscription &) = delete;
  GenericSubscription(GenericSubscription &&) = delete;
  GenericSubscription & operator=(GenericSubscription &&) = delete;

  ~GenericSubscription() override;

  // The message type name supplied at construction ("<pkg>/msg/<MsgName>"). Pass this to the
  // agnocast_introspection.hpp helpers to interpret a delivered payload.
  const std::string & type_name() const { return type_name_; }

private:
  std::pair<mqd_t, std::string> mq_subscription_;
  uint32_t callback_info_id_;
  std::string type_name_;

  template <typename NodeT>
  void constructor_impl(
    NodeT * node, const rclcpp::QoS & qos, Callback callback,
    const rclcpp::CallbackGroup::SharedPtr & callback_group, const SubscriptionOptions & options);
};

}  // namespace agnocast
