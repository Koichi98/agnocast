#include "agnocast/agnocast_generic_subscription.hpp"

#include "agnocast/agnocast.hpp"
#include "agnocast/node/agnocast_node.hpp"

#include <array>
#include <mutex>

namespace agnocast
{

void GenericSubscription::initialize_with(
  const rclcpp::QoS & qos, const SubscriptionOptions & options, const std::string & node_name)
{
  validate_subscription_qos(qos);

  // is_take_sub=false: registers as a callback-style subscriber so that publishers will
  // notify our MQ on each publish. The kmod (agnocast_ioctl.c, see the publish path)
  // explicitly skips take-style subscribers when emitting MQ notifications, so generic
  // event-driven consumption requires the callback-style registration even though we do
  // not run a typed callback through the executor.
  union ioctl_add_subscriber_args add_subscriber_args = initialize(
    qos, /*is_take_sub=*/false, options.ignore_local_publications, /*is_bridge=*/false,
    node_name);

  id_ = add_subscriber_args.ret_id;

  open_mq_for_subscription(topic_name_, id_, mq_subscription_);
}

GenericSubscription::GenericSubscription(
  rclcpp::Node * node, const std::string & topic_name, const rclcpp::QoS & qos,
  SubscriptionOptions options)
: SubscriptionBase(node, topic_name)
{
  initialize_with(qos, options, node->get_fully_qualified_name());
}

GenericSubscription::GenericSubscription(
  agnocast::Node * node, const std::string & topic_name, const rclcpp::QoS & qos,
  SubscriptionOptions options)
: SubscriptionBase(node, topic_name)
{
  initialize_with(qos, options, node->get_fully_qualified_name());
}

GenericSubscription::~GenericSubscription()
{
  // Best-effort: drain anything pending so kmod doesn't keep references around. Ignore
  // payloads; the kernel reference is released by drain() automatically.
  drain([](const void *, int64_t) {});
  remove_mq(mq_subscription_);
  // SubscriptionBase destructor issues AGNOCAST_REMOVE_SUBSCRIBER_CMD.
}

size_t GenericSubscription::drain(const EntryCallback & cb)
{
  size_t total = 0;
  bool call_again = true;
  while (call_again) {
    std::array<publisher_shm_info, MAX_PUBLISHER_NUM> pub_shm_infos{};

    union ioctl_receive_msg_args receive_args = {};
    receive_args.topic_name = {topic_name_.c_str(), topic_name_.size()};
    receive_args.subscriber_id = id_;
    receive_args.pub_shm_info_addr = reinterpret_cast<uint64_t>(pub_shm_infos.data());
    receive_args.pub_shm_info_size = MAX_PUBLISHER_NUM;

    {
      std::lock_guard<std::mutex> lock(mmap_mtx);

      if (ioctl(agnocast_fd, AGNOCAST_RECEIVE_MSG_CMD, &receive_args) < 0) {
        RCLCPP_ERROR(logger, "AGNOCAST_RECEIVE_MSG_CMD failed: %s", strerror(errno));
        close(agnocast_fd);
        exit(EXIT_FAILURE);
      }

      for (uint32_t i = 0; i < receive_args.ret_pub_shm_num; i++) {
        map_read_only_area(
          pub_shm_infos[i].pid, pub_shm_infos[i].shm_addr, pub_shm_infos[i].shm_size);
      }
    }

    for (uint16_t i = 0; i < receive_args.ret_entry_num; i++) {
      const int64_t entry_id = receive_args.ret_entry_ids[i];
      const auto * payload = reinterpret_cast<const void *>(receive_args.ret_entry_addrs[i]);
      cb(payload, entry_id);
      release_subscriber_reference(topic_name_, id_, entry_id);
      ++total;
    }

    call_again = receive_args.ret_call_again;
  }
  return total;
}

}  // namespace agnocast
