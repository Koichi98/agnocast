#pragma once

#include "agnocast/agnocast_ioctl.hpp"
#include "agnocast/agnocast_mq.hpp"
#include "agnocast/agnocast_public_api.hpp"
#include "agnocast/agnocast_smart_pointer.hpp"
#include "agnocast/agnocast_subscription.hpp"
#include "agnocast/agnocast_utils.hpp"
#include "rclcpp/rclcpp.hpp"

#include <mqueue.h>

#include <cstdint>
#include <functional>
#include <string>
#include <utility>

namespace agnocast
{

class Node;

// Type-erased subscription used by introspection-based CLI tools (e.g. ros2 topic
// hz_agnocast / delay_agnocast). It registers a subscriber with the kernel module without
// any compile-time MessageT and exposes the raw payload pointer to the caller. The caller is
// responsible for interpreting the payload (or ignoring it, in the hz case).
//
// Design choices:
// - Registers as a callback-style subscriber (is_take_sub=false) so the kmod *does* dispatch
//   MQ notifications on each publish. (The kmod deliberately skips MQ notification for
//   take-style subscribers — see agnocast_kmod/agnocast_ioctl.c near `is_take_sub` check.)
// - Skips BridgeRequestPolicy entirely (no MessageT, no bridge auto-spawn). External-ECU
//   reachability must come from a pre-existing typed a2r/r2a bridge pair.
// - Exposes mq_fd() so the caller can poll/epoll for events.
// - Provides drain() which uses AGNOCAST_RECEIVE_MSG_CMD to pull all pending entries,
//   invoke a user callback with each (payload, entry_id), then immediately release the
//   kernel-side reference. The callback must NOT retain the payload pointer beyond the call.
class GenericSubscription : public SubscriptionBase
{
  std::pair<mqd_t, std::string> mq_subscription_;

  void initialize_with(
    const rclcpp::QoS & qos, const SubscriptionOptions & options, const std::string & node_name);

public:
  using EntryCallback = std::function<void(const void * payload, int64_t entry_id)>;

  AGNOCAST_PUBLIC
  GenericSubscription(
    rclcpp::Node * node, const std::string & topic_name, const rclcpp::QoS & qos,
    SubscriptionOptions options = SubscriptionOptions{});

  AGNOCAST_PUBLIC
  GenericSubscription(
    agnocast::Node * node, const std::string & topic_name, const rclcpp::QoS & qos,
    SubscriptionOptions options = SubscriptionOptions{});

  GenericSubscription(const GenericSubscription &) = delete;
  GenericSubscription & operator=(const GenericSubscription &) = delete;
  GenericSubscription(GenericSubscription &&) = delete;
  GenericSubscription & operator=(GenericSubscription &&) = delete;

  ~GenericSubscription() override;

  // File descriptor of the publisher-notification message queue. The caller can poll/epoll on
  // this fd; when readable, drain it with mq_receive and then call drain().
  mqd_t mq_fd() const { return mq_subscription_.first; }

  // Pull every pending entry from the kernel queue. For each entry, invokes `cb` with the
  // raw SHM payload pointer and the entry id. After `cb` returns, the entry's kernel-side
  // reference is released. The callback MUST NOT retain the payload pointer beyond the call
  // since the kernel may reclaim the slot once the reference is released.
  // Returns the number of entries processed.
  AGNOCAST_PUBLIC
  size_t drain(const EntryCallback & cb);
};

}  // namespace agnocast
