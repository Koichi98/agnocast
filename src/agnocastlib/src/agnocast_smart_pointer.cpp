#include "agnocast/agnocast_smart_pointer.hpp"

namespace agnocast
{

void release_subscriber_reference(
  const std::string & topic_name, const topic_local_id_t pubsub_id, const int64_t entry_id)
{
  struct ioctl_update_entry_args entry_args = {};
  entry_args.topic_name = {topic_name.c_str(), topic_name.size()};
  entry_args.pubsub_id = pubsub_id;
  entry_args.entry_id = entry_id;
  if (ioctl(agnocast_fd, AGNOCAST_RELEASE_SUB_REF_CMD, &entry_args) >= 0) {
    return;
  }

  // A stale release is not a fatal condition. It happens when a message reference outlives the
  // Subscription that delivered it: ~SubscriptionBase issues AGNOCAST_REMOVE_SUBSCRIBER_CMD, which
  // clears this subscriber's bit on every entry of the topic, so the later release finds no bit to
  // clear and the kernel module returns -EINVAL. EBADF likewise means agnocast_fd was already
  // closed during shutdown. The shared-memory entry is already reclaimable in both cases, so warn
  // once and continue -- terminating here would kill an application whose only mistake was holding
  // a message longer than its subscription.
  if (errno == EINVAL || errno == EBADF) {
    static std::atomic<bool> warned{false};
    if (!warned.exchange(true, std::memory_order_relaxed)) {
      RCLCPP_WARN(
        logger,
        "AGNOCAST_RELEASE_SUB_REF_CMD failed (%s) for topic %s: a message reference outlived its "
        "Subscription. Declare the Subscription member before any member that caches messages, so "
        "that it is destroyed last. This warning is printed only once.",
        strerror(errno), topic_name.c_str());
    }
    return;
  }

  RCLCPP_ERROR(logger, "AGNOCAST_RELEASE_SUB_REF_CMD failed: %s", strerror(errno));
  close(agnocast_fd);
  exit(EXIT_FAILURE);
}

}  // namespace agnocast
