#include "agnocast/agnocast_generic_subscription.hpp"

#include "agnocast/agnocast.hpp"
#include "agnocast/agnocast_callback_info.hpp"
#include "agnocast/node/agnocast_node.hpp"

#include <mutex>
#include <utility>

namespace agnocast
{

template <typename NodeT>
void GenericSubscription::constructor_impl(
  NodeT * node, const rclcpp::QoS & qos, Callback callback,
  const rclcpp::CallbackGroup::SharedPtr & callback_group, const SubscriptionOptions & options)
{
  validate_subscription_qos(qos);

  // is_take_sub=false: register as a callback-style subscriber so publishers notify our MQ and
  // the executor dispatches the callback on each publish. is_bridge=false: GenericSubscription
  // never participates in bridging.
  union ioctl_add_subscriber_args add_subscriber_args = initialize(
    qos, /*is_take_sub=*/false, options.ignore_local_publications, /*is_bridge=*/false,
    node->get_fully_qualified_name());
  id_ = add_subscriber_args.ret_id;

  mqd_t mq = open_mq_for_subscription(topic_name_, id_, mq_subscription_);

  const bool is_transient_local = qos.durability() == rclcpp::DurabilityPolicy::TransientLocal;

  // register_callback<void> instantiates the type-erased dispatch path: the kernel payload
  // pointer is wrapped in ipc_shared_ptr<void> by the message_creator and handed to `callback`,
  // which converts it to ipc_shared_ptr<const void>. The executor itself needs no changes --
  // CallbackInfo already stores a fully type-erased callback.
  callback_info_id_ = agnocast::register_callback<void>(
    std::move(callback), topic_name_, id_, is_transient_local, mq, callback_group);
}

GenericSubscription::GenericSubscription(
  rclcpp::Node * node, const std::string & topic_name, const std::string & type_name,
  const rclcpp::QoS & qos, Callback callback, SubscriptionOptions options)
: SubscriptionBase(node, topic_name), type_name_(type_name)
{
  rclcpp::CallbackGroup::SharedPtr callback_group = get_valid_callback_group(node, options);
  constructor_impl(node, qos, std::move(callback), callback_group, options);
}

GenericSubscription::GenericSubscription(
  agnocast::Node * node, const std::string & topic_name, const std::string & type_name,
  const rclcpp::QoS & qos, Callback callback, SubscriptionOptions options)
: SubscriptionBase(node, topic_name), type_name_(type_name)
{
  rclcpp::CallbackGroup::SharedPtr callback_group = get_valid_callback_group(node, options);
  constructor_impl(node, qos, std::move(callback), callback_group, options);
}

GenericSubscription::~GenericSubscription()
{
  // Mirror BasicSubscription::~BasicSubscription: drop the callback info before closing the mq
  // so that a later-reused fd number cannot collide with a stale id2_callback_info entry when
  // the executor's epoll is updated.
  {
    std::lock_guard<std::mutex> lock(id2_callback_info_mtx);
    id2_callback_info.erase(callback_info_id_);
  }
  remove_mq(mq_subscription_);
  // SubscriptionBase destructor issues AGNOCAST_REMOVE_SUBSCRIBER_CMD.
}

}  // namespace agnocast
