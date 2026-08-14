#pragma once

#include "agnocast/agnocast_callback_info.hpp"
#include "agnocast/agnocast_ioctl.hpp"
#include "agnocast/agnocast_public_api.hpp"
#include "agnocast/agnocast_smart_pointer.hpp"
#include "agnocast/agnocast_tracepoint_wrapper.h"
#include "agnocast/agnocast_utils.hpp"
#include "rclcpp/detail/qos_parameters.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/serialized_message.hpp"
#include "rcpputils/shared_library.hpp"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"

#include <mqueue.h>
#include <unistd.h>

#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <type_traits>

namespace agnocast
{
class Node;

extern std::mutex mmap_mtx;

void map_read_only_area(const pid_t pid, const uint64_t shm_addr, const uint64_t shm_size);

// Get the default callback group from an agnocast::Node for tracepoint use.
// Defined in .cpp to avoid circular inclusion between agnocast_subscription.hpp and
// agnocast_node.hpp.
rclcpp::CallbackGroup::SharedPtr get_default_callback_group_for_tracepoint(agnocast::Node * node);
const void * get_node_base_address(Node * node);
const void * get_node_base_address(rclcpp::Node * node);

// rclcpp overload of the above, so callback-less subscription construction can be written once
// for both node types. rclcpp::Node needs no out-of-line definition to break an include cycle.
inline rclcpp::CallbackGroup::SharedPtr get_default_callback_group_for_tracepoint(
  rclcpp::Node * node)
{
  return node->get_node_base_interface()->get_default_callback_group();
}

/**
 * @brief Options for configuring an Agnocast subscription.
 */
AGNOCAST_PUBLIC
struct SubscriptionOptions
{
  /// Callback group for the subscription (nullptr = default group).
  rclcpp::CallbackGroup::SharedPtr callback_group{nullptr};
  /// If true, messages from publishers in the same process are ignored.
  bool ignore_local_publications{false};
  /// QoS parameter override options (same semantics as rclcpp).
  rclcpp::QosOverridingOptions qos_overriding_options{};
};

/**
 * @brief Role of a subscription with respect to the ROS<->Agnocast bridge.
 *
 * Encodes two properties of a subscription:
 *   - whether it is used by the bridge implementation itself
 *   - whether it should issue an R2A bridge request on construction
 *
 *   | Role            | kmod `is_bridge` | bridge request issued |
 *   |-----------------|------------------|-----------------------|
 *   | Default         | false            | yes (R2A)             |
 *   | AgnocastOnly    | false            | no                    |
 *   | BridgeInternal  | true             | no                    |
 */
enum class SubscriptionRole : uint8_t {
  /// User-created subscription; issues an R2A bridge request.
  Default,
  /// Used internally by the service/client implementation; no bridge request.
  /// Not intended for direct use by application code.
  AgnocastOnly,
  /// Used by the bridge implementation itself; marked as bridge in kmod and
  /// issues no bridge request.
  /// Not intended for direct use by application code.
  BridgeInternal,
};

// These are cut out of the class for information hiding.
void close_notify_eventfd(int notify_eventfd);
// No longer called: publish notification moved to eventfd. Removed together with the rest of the
// MQ machinery in a follow-up.
mqd_t open_mq_for_subscription(
  const std::string & topic_name, const topic_local_id_t subscriber_id,
  std::pair<mqd_t, std::string> & mq_subscription);
void remove_mq(const std::pair<mqd_t, std::string> & mq_subscription);
uint32_t get_publisher_count_core(const std::string & topic_name);

template <typename NodeT>
rclcpp::CallbackGroup::SharedPtr get_valid_callback_group(
  NodeT * node, const SubscriptionOptions & options)
{
  rclcpp::CallbackGroup::SharedPtr callback_group = options.callback_group;

  if (callback_group) {
    if (!node->get_node_base_interface()->callback_group_in_node(callback_group)) {
      RCLCPP_ERROR(logger, "Cannot create agnocast subscription, callback group not in node.");
      close(agnocast_fd);
      exit(EXIT_FAILURE);
    }
  } else {
    callback_group = node->get_node_base_interface()->get_default_callback_group();
  }

  return callback_group;
}

class SubscriptionBase
{
protected:
  topic_local_id_t id_{-1};
  const std::string topic_name_;
  int notify_eventfd_ = -1;  // publish-notification eventfd (-1 for callback-less subscriptions)
  // Effective QoS after any qos_overriding_options are applied. Set by init_base().
  rclcpp::QoS actual_qos_{1};
  void initialize(
    const rclcpp::QoS & qos, const bool is_take_sub, const bool ignore_local_publications,
    SubscriptionRole role, const std::string & node_name, const std::string & type_name);

  template <typename NodeT>
  rclcpp::QoS init_base(
    NodeT * node, const rclcpp::QoS & qos, const std::string & type_name, bool is_take_sub,
    const SubscriptionOptions & options, SubscriptionRole role);

public:
  SubscriptionBase(rclcpp::Node * node, const std::string & topic_name);
  SubscriptionBase(agnocast::Node * node, const std::string & topic_name);

  /**
   * @brief Return the fully-resolved topic name.
   * @return Null-terminated topic name string.
   */
  AGNOCAST_PUBLIC
  const char * get_topic_name() const { return topic_name_.c_str(); }

  uint32_t get_publisher_count() const { return get_publisher_count_core(topic_name_); }

  /**
   * @brief Return the effective QoS of this subscription.
   *
   * Mirrors `rclcpp::SubscriptionBase::get_actual_qos()`. This is the QoS passed at construction
   * with any `SubscriptionOptions::qos_overriding_options` applied.
   */
  AGNOCAST_PUBLIC
  rclcpp::QoS get_actual_qos() const { return actual_qos_; }

  virtual ~SubscriptionBase()
  {
    if (id_ >= 0) {
      // NOTE: Unmapping memory when a subscriber is destroyed is not implemented. Multiple
      // subscribers
      // may share the same mmap region, requiring reference counting in kmod. Since leaving the
      // memory mapped should not cause any functional issues, this is left as future work.
      struct ioctl_remove_subscriber_args remove_subscriber_args
      {
      };
      remove_subscriber_args.topic_name = {topic_name_.c_str(), topic_name_.size()};
      remove_subscriber_args.subscriber_id = id_;
      if (ioctl(agnocast_fd, AGNOCAST_REMOVE_SUBSCRIBER_CMD, &remove_subscriber_args) < 0) {
        RCLCPP_WARN(logger, "Failed to remove subscriber (id=%d) from kernel.", id_);
      }
    }

    close_notify_eventfd(notify_eventfd_);
  }
};

/**
 * @brief Agnocast subscription for a compile-time known message type.
 *
 * Delivers messages via a callback that is invoked each time a publisher
 * writes to the topic. Allocate instances with
 * `agnocast::create_subscription<MessageT>()` or construct directly.
 *
 * @tparam MessageT  ROS message type.
 */
AGNOCAST_PUBLIC
template <typename MessageT>
class Subscription : public SubscriptionBase
{
  // Empty when the subscription was constructed without a callback. That case is what the kmod
  // calls a "take subscription": no eventfd is registered and publishers skip it when signalling.
  std::optional<uint32_t> callback_info_id_;

  // Cached pointer from the most recent take(allow_same_message=true) call.
  // When the same entry is returned again, a copy sharing the same control_block is returned
  // so that the kernel-side reference is not released until all userspace copies are destroyed.
  agnocast::ipc_shared_ptr<const MessageT> last_taken_ptr_;
  std::mutex last_taken_ptr_mtx_;

  // Returns rosidl message name for MessageT, or empty string if MessageT is not a rosidl message
  // type.
  static std::string get_message_type_name()
  {
    if constexpr (rosidl_generator_traits::is_message<MessageT>::value) {
      return rosidl_generator_traits::name<MessageT>();
    }
    return std::string{};
  }

  // A 4th positional argument of type SubscriptionOptions selects the callback-less overload
  // below, so it must not be deduced as a callback here.
  template <typename Func>
  static constexpr bool is_callback_arg_v =
    !std::is_same_v<std::decay_t<Func>, agnocast::SubscriptionOptions>;

  template <typename NodeT>
  void constructor_impl_no_callback(
    NodeT * node, const std::string & type_name, const rclcpp::QoS & qos,
    agnocast::SubscriptionOptions options, SubscriptionRole role)
  {
    const rclcpp::QoS actual_qos = init_base(node, qos, type_name, true, options, role);

    {
      auto default_cbg = get_default_callback_group_for_tracepoint(node);
      auto dummy_cb = []() {};
      std::string dummy_cb_symbols = "dummy_take" + topic_name_;
      TRACEPOINT(
        agnocast_subscription_init, static_cast<const void *>(this), get_node_base_address(node),
        static_cast<const void *>(&dummy_cb), static_cast<const void *>(default_cbg.get()),
        dummy_cb_symbols.c_str(), topic_name_.c_str(), actual_qos.depth(), 0);
    }
  }

  template <typename NodeT, typename Func>
  void constructor_impl(
    NodeT * node, const std::string & type_name, const rclcpp::QoS & qos, Func && callback,
    agnocast::SubscriptionOptions options, SubscriptionRole role)
  {
    rclcpp::CallbackGroup::SharedPtr callback_group = get_valid_callback_group(node, options);

    const void * callback_addr = static_cast<const void *>(&callback);
    const char * callback_symbol = tracetools::get_symbol(callback);

    const rclcpp::QoS actual_qos = init_base(node, qos, type_name, false, options, role);

    const bool is_transient_local =
      actual_qos.durability() == rclcpp::DurabilityPolicy::TransientLocal;
    callback_info_id_ = agnocast::register_callback<MessageT>(
      std::forward<Func>(callback), topic_name_, id_, is_transient_local, notify_eventfd_,
      callback_group);

    {
      uint64_t pid_callback_info_id = (static_cast<uint64_t>(getpid()) << 32) | *callback_info_id_;
      TRACEPOINT(
        agnocast_subscription_init, static_cast<const void *>(this), get_node_base_address(node),
        callback_addr, static_cast<const void *>(callback_group.get()), callback_symbol,
        topic_name_.c_str(), actual_qos.depth(), pid_callback_info_id);
    }
  }

public:
  using SharedPtr = std::shared_ptr<Subscription<MessageT>>;

  template <typename Func, std::enable_if_t<is_callback_arg_v<Func>, int> = 0>
  Subscription(
    rclcpp::Node * node, const std::string & topic_name, const rclcpp::QoS & qos, Func && callback,
    agnocast::SubscriptionOptions options, SubscriptionRole role = SubscriptionRole::Default)
  : SubscriptionBase(node, topic_name)
  {
    constructor_impl(
      node, get_message_type_name(), qos, std::forward<Func>(callback), options, role);
  }

  template <typename Func, std::enable_if_t<is_callback_arg_v<Func>, int> = 0>
  Subscription(
    agnocast::Node * node, const std::string & topic_name, const rclcpp::QoS & qos,
    Func && callback, agnocast::SubscriptionOptions options,
    SubscriptionRole role = SubscriptionRole::Default)
  : SubscriptionBase(node, topic_name)
  {
    constructor_impl(
      node, get_message_type_name(), qos, std::forward<Func>(callback), options, role);
  }

  /// Construct without a callback. Messages are then retrieved by polling take(), and the
  /// subscription registers no eventfd so publishers skip it when signalling. Mirrors the
  /// rclcpp idiom of a subscription whose callback group is never spun.
  Subscription(
    rclcpp::Node * node, const std::string & topic_name, const rclcpp::QoS & qos,
    agnocast::SubscriptionOptions options = agnocast::SubscriptionOptions(),
    SubscriptionRole role = SubscriptionRole::Default)
  : SubscriptionBase(node, topic_name)
  {
    constructor_impl_no_callback(node, get_message_type_name(), qos, options, role);
  }

  /// @copydoc Subscription(rclcpp::Node*, const std::string&, const rclcpp::QoS&,
  ///          agnocast::SubscriptionOptions, SubscriptionRole)
  Subscription(
    agnocast::Node * node, const std::string & topic_name, const rclcpp::QoS & qos,
    agnocast::SubscriptionOptions options = agnocast::SubscriptionOptions(),
    SubscriptionRole role = SubscriptionRole::Default)
  : SubscriptionBase(node, topic_name)
  {
    constructor_impl_no_callback(node, get_message_type_name(), qos, options, role);
  }

  template <typename Func, typename U = MessageT, std::enable_if_t<std::is_void_v<U>, int> = 0>
  Subscription(
    rclcpp::Node * node, const std::string & topic_name, const std::string & type_name,
    const rclcpp::QoS & qos, Func && callback, agnocast::SubscriptionOptions options,
    SubscriptionRole role)
  : SubscriptionBase(node, topic_name)
  {
    constructor_impl(node, type_name, qos, std::forward<Func>(callback), options, role);
  }

  template <typename Func, typename U = MessageT, std::enable_if_t<std::is_void_v<U>, int> = 0>
  Subscription(
    agnocast::Node * node, const std::string & topic_name, const std::string & type_name,
    const rclcpp::QoS & qos, Func && callback, agnocast::SubscriptionOptions options,
    SubscriptionRole role)
  : SubscriptionBase(node, topic_name)
  {
    constructor_impl(node, type_name, qos, std::forward<Func>(callback), options, role);
  }

  /**
   * @brief Retrieve the latest message from the topic without going through the callback.
   *
   * Mirrors `rclcpp::Subscription::take()` in that it is available on every subscription, not
   * just callback-less ones. Note the same caveat as rclcpp: if this subscription also has a
   * callback that is being spun by an executor, the callback and take() consume from the same
   * per-subscriber cursor and will steal messages from each other. Use one or the other.
   *
   * @param allow_same_message  If true, may return the same message as the previous call
   *                            (useful for always having the latest value). If false, returns
   *                            only new messages since the last take.
   * @return Shared pointer to the message, or empty if unavailable.
   */
  AGNOCAST_PUBLIC
  agnocast::ipc_shared_ptr<const MessageT> take(bool allow_same_message = false)
  {
    publisher_shm_info pub_shm_infos[MAX_PUBLISHER_NUM]{};

    union ioctl_take_msg_args take_args;
    take_args.topic_name = {topic_name_.c_str(), topic_name_.size()};
    take_args.subscriber_id = id_;
    take_args.allow_same_message = allow_same_message;
    take_args.pub_shm_info_addr = reinterpret_cast<uint64_t>(pub_shm_infos);
    take_args.pub_shm_info_size = MAX_PUBLISHER_NUM;

    {
      std::lock_guard<std::mutex> lock(mmap_mtx);

      if (ioctl(agnocast_fd, AGNOCAST_TAKE_MSG_CMD, &take_args) < 0) {
        RCLCPP_ERROR(logger, "AGNOCAST_TAKE_MSG_CMD failed: %s", strerror(errno));
        close(agnocast_fd);
        exit(EXIT_FAILURE);
      }

      for (uint32_t i = 0; i < take_args.ret_pub_shm_num; i++) {
        const pid_t pid = pub_shm_infos[i].pid;
        const uint64_t addr = pub_shm_infos[i].shm_addr;
        const uint64_t size = pub_shm_infos[i].shm_size;
        map_read_only_area(pid, addr, size);
      }
    }

    if (take_args.ret_addr == 0) {
      TRACEPOINT(agnocast_take, static_cast<void *>(this), 0, 0);
      return agnocast::ipc_shared_ptr<const MessageT>();
    }

    TRACEPOINT(
      agnocast_take, static_cast<void *>(this), reinterpret_cast<void *>(take_args.ret_addr),
      take_args.ret_entry_id);

    if (allow_same_message) {
      // Declared outside the lock scope so that its destructor (which may call ioctl to release
      // the kernel reference) runs after the mutex is released, avoiding unnecessary contention.
      agnocast::ipc_shared_ptr<const MessageT> old_ptr;
      {
        std::lock_guard<std::mutex> lock(last_taken_ptr_mtx_);

        // When the kernel returned the same entry as last time, return a copy of the cached
        // pointer (sharing the same control_block) instead of creating a new one.
        // This keeps the kernel-side reference alive until all copies are destroyed.
        if (last_taken_ptr_ && last_taken_ptr_.get_entry_id() == take_args.ret_entry_id) {
          return last_taken_ptr_;
        }

        MessageT * ptr = reinterpret_cast<MessageT *>(take_args.ret_addr);
        auto result =
          agnocast::ipc_shared_ptr<const MessageT>(ptr, topic_name_, id_, take_args.ret_entry_id);
        old_ptr = std::move(last_taken_ptr_);
        last_taken_ptr_ = result;
        return result;
      }
    }

    MessageT * ptr = reinterpret_cast<MessageT *>(take_args.ret_addr);
    return agnocast::ipc_shared_ptr<const MessageT>(ptr, topic_name_, id_, take_args.ret_entry_id);
  }

  ~Subscription()
  {
    // Nothing to erase when constructed without a callback: no entry was ever registered and
    // no eventfd was opened.
    if (!callback_info_id_.has_value()) {
      return;
    }

    // Remove from callback info map to prevent stale references on re-subscription and to avoid
    // fd reuse conflicts. ~SubscriptionBase() closes the eventfd once this body returns, after
    // which the OS may reuse the same fd number for a new subscription. If the old entry remained
    // in id2_callback_info, adding the new fd to epoll (EPOLL_CTL_ADD) could fail with EEXIST
    // because epoll would still associate that fd number with the stale entry.
    std::lock_guard<std::mutex> lock(id2_callback_info_mtx);
    id2_callback_info.erase(*callback_info_id_);
  }
};

// NOTE on Subscription<void>:
//   Subscription<void> is the subscription counterpart of TypeErasedPublisher. We do not create a
//   separate TypeErasedSubscription class because it would share most of its code with
//   Subscription<MessageT>, making a void specialization the cleaner choice.

/**
 * @brief Backwards-compatible spelling of a callback-less Subscription<MessageT>.
 *
 * Retained so existing call sites keep compiling. `Subscription<MessageT>` now supports both
 * callback delivery and take(), mirroring `rclcpp::Subscription`, so prefer constructing a
 * `Subscription<MessageT>` without a callback instead of using this class.
 *
 * @tparam MessageT  ROS message type.
 */
AGNOCAST_PUBLIC
template <typename MessageT>
class TakeSubscription : public Subscription<MessageT>
{
public:
  using SharedPtr = std::shared_ptr<TakeSubscription<MessageT>>;

  TakeSubscription(
    rclcpp::Node * node, const std::string & topic_name, const rclcpp::QoS & qos,
    agnocast::SubscriptionOptions options = agnocast::SubscriptionOptions(),
    SubscriptionRole role = SubscriptionRole::Default)
  : Subscription<MessageT>(node, topic_name, qos, options, role)
  {
  }

  TakeSubscription(
    agnocast::Node * node, const std::string & topic_name, const rclcpp::QoS & qos,
    agnocast::SubscriptionOptions options = agnocast::SubscriptionOptions(),
    SubscriptionRole role = SubscriptionRole::Default)
  : Subscription<MessageT>(node, topic_name, qos, options, role)
  {
  }
};

/**
 * @brief Agnocast polling subscriber for a compile-time known message type.
 *
 * Wraps a callback-less Subscription<MessageT> and exposes a simple take_data() API
 * that always returns the most recent message (or an empty pointer if nothing
 * has been published yet).
 *
 * @tparam MessageT  ROS message type.
 */
AGNOCAST_PUBLIC
template <typename MessageT>
class PollingSubscriber
{
  typename Subscription<MessageT>::SharedPtr subscriber_;

public:
  using SharedPtr = std::shared_ptr<PollingSubscriber<MessageT>>;

  explicit PollingSubscriber(
    rclcpp::Node * node, const std::string & topic_name, const rclcpp::QoS & qos = rclcpp::QoS{1},
    agnocast::SubscriptionOptions options = agnocast::SubscriptionOptions(),
    SubscriptionRole role = SubscriptionRole::Default)
  {
    subscriber_ = std::make_shared<Subscription<MessageT>>(node, topic_name, qos, options, role);
  };

  explicit PollingSubscriber(
    agnocast::Node * node, const std::string & topic_name, const rclcpp::QoS & qos = rclcpp::QoS{1},
    agnocast::SubscriptionOptions options = agnocast::SubscriptionOptions(),
    SubscriptionRole role = SubscriptionRole::Default)
  {
    subscriber_ = std::make_shared<Subscription<MessageT>>(node, topic_name, qos, options, role);
  };

  /// @deprecated Use take_data() instead.
  const agnocast::ipc_shared_ptr<const MessageT> takeData() { return subscriber_->take(true); };
  /// @brief Retrieve the latest message. Always returns the most recent message even if already
  /// retrieved. Returns an empty pointer if no message has been published yet.
  /// @return Shared pointer to the latest message.
  AGNOCAST_PUBLIC
  const agnocast::ipc_shared_ptr<const MessageT> take_data() { return subscriber_->take(true); };
};

/// @brief Mirrors `rclcpp::GenericSubscription` semantics: the topic type is supplied
/// as a runtime string (e.g. "std_msgs/msg/String") rather than a compile-time
/// template argument. The typesupport library is loaded eagerly and held by the
/// subscription callback.
///
/// Messages are delivered to the callback as serialized data, outside of Agnocast shared memory.
///
/// The supported callback signatures are:
///   - `void(std::shared_ptr<rclcpp::SerializedMessage>)` (and `const` / `const`-T variants)
///   - `void(std::unique_ptr<rclcpp::SerializedMessage>)` (and `const` / `const`-T variants)
///   - `void(rclcpp::SerializedMessage &)` (and `const` variants)
AGNOCAST_PUBLIC
class GenericSubscription : public Subscription<void>
{
  struct TypeSupportBundle
  {
    std::shared_ptr<rcpputils::SharedLibrary> library;
    const rosidl_message_type_support_t * handle{nullptr};
  };

  static TypeSupportBundle load_typesupport_impl(const std::string & topic_type);

  static bool serialize_message(
    const void * raw, const rosidl_message_type_support_t * type_support,
    rclcpp::SerializedMessage & out);

  template <typename Func>
  static auto get_subscription_callback(Func && callback, const std::string & topic_type)
  {
    using F = std::decay_t<Func>;
    static_assert(
      std::is_invocable_v<F, std::shared_ptr<rclcpp::SerializedMessage>> ||
        std::is_invocable_v<F, std::unique_ptr<rclcpp::SerializedMessage>> ||
        std::is_invocable_v<F, rclcpp::SerializedMessage &>,
      "This callback type cannot be handled as a GenericCallback. "
      "Callback must be invocable with one of the following arguments "
      "(or any types implicitly convertible from them, e.g., const variants): "
      "std::unique_ptr<rclcpp::SerializedMessage>, "
      "std::shared_ptr<rclcpp::SerializedMessage>, or "
      "rclcpp::SerializedMessage &.");

    TypeSupportBundle ts_bundle = load_typesupport_impl(topic_type);

    return [callback = std::forward<Func>(callback),
            ts_bundle = std::move(ts_bundle)](ipc_shared_ptr<void> && message) {
      if constexpr (std::is_invocable_v<F, std::shared_ptr<rclcpp::SerializedMessage>>) {
        auto serialized = std::make_shared<rclcpp::SerializedMessage>();
        if (!serialize_message(message.get(), ts_bundle.handle, *serialized)) {
          return;
        }
        message.reset();
        callback(std::move(serialized));
      } else if constexpr (std::is_invocable_v<F, std::unique_ptr<rclcpp::SerializedMessage>>) {
        auto serialized = std::make_unique<rclcpp::SerializedMessage>();
        if (!serialize_message(message.get(), ts_bundle.handle, *serialized)) {
          return;
        }
        message.reset();
        callback(std::move(serialized));
      } else {
        rclcpp::SerializedMessage serialized;
        if (!serialize_message(message.get(), ts_bundle.handle, serialized)) {
          return;
        }
        message.reset();
        callback(serialized);
      }
    };
  }

public:
  using SharedPtr = std::shared_ptr<GenericSubscription>;

  template <typename Func>
  GenericSubscription(
    rclcpp::Node * node, const std::string & topic_name, const std::string & topic_type,
    const rclcpp::QoS & qos, Func && callback,
    agnocast::SubscriptionOptions options = agnocast::SubscriptionOptions(),
    SubscriptionRole role = SubscriptionRole::Default)
  : Subscription<void>(
      node, topic_name, topic_type, qos,
      get_subscription_callback(std::forward<Func>(callback), topic_type), options, role)
  {
  }

  template <typename Func>
  GenericSubscription(
    agnocast::Node * node, const std::string & topic_name, const std::string & topic_type,
    const rclcpp::QoS & qos, Func && callback,
    agnocast::SubscriptionOptions options = agnocast::SubscriptionOptions(),
    SubscriptionRole role = SubscriptionRole::Default)
  : Subscription<void>(
      node, topic_name, topic_type, qos,
      get_subscription_callback(std::forward<Func>(callback), topic_type), options, role)
  {
  }
};

}  // namespace agnocast
