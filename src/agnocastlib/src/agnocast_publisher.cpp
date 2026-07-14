#include "agnocast/agnocast_publisher.hpp"

#include "agnocast/internal/type_registry_writer.hpp"
#include "agnocast/node/agnocast_node.hpp"

#include <sys/types.h>

#include <array>
#include <mutex>
#include <string>
#include <unordered_map>

namespace agnocast
{

// Keep the initial-exec TLS model here: it avoids the following infinite recursion that causes a
// SIGSEGV:
// 1. heaphook malloc() is called.
// 2. agnocast_get_borrowed_publisher_num() is called and accesses a thread_local variable.
// 3. __tls_get_addr() is called to resolve the address.
// 4. _dl_resize_dtv() is called to resize the DTV region. This occurs when new .so libraries are
//    loaded via dlopen() and the number of managed TLS variables increases.
// 5. _dl_resize_dtv() calls malloc(), which loops back to step 1.
__attribute__((tls_model("initial-exec"))) thread_local uint32_t borrowed_publisher_num = 0;

extern "C" uint32_t agnocast_get_borrowed_publisher_num()
{
  return borrowed_publisher_num;
}

void increment_borrowed_publisher_num()
{
  borrowed_publisher_num++;
}

void decrement_borrowed_publisher_num()
{
  if (borrowed_publisher_num == 0) {
    RCLCPP_ERROR(
      logger,
      "The number of publish() called exceeds the number of borrow_loaned_message() called.");
    close(agnocast_fd);
    exit(EXIT_FAILURE);
  }
  borrowed_publisher_num--;
}

topic_local_id_t initialize_publisher(
  const std::string & topic_name, const std::string & node_name, const rclcpp::QoS & qos,
  const bool is_bridge, const std::string & type_name)
{
  validate_ld_preload();

  // Announce to the per-IPC-namespace discovery agent before the kmod call so
  // the registry line is in place whenever a later snapshot sees the
  // ioctl-side endpoint. Empty `type_name` (e.g. service types) skips this.
  if (!type_name.empty()) {
    internal::TypeRegistryWriter::instance().register_type(topic_name, type_name, "pub", node_name);
  }

  union ioctl_add_publisher_args pub_args = {};
  pub_args.topic_name = {topic_name.c_str(), topic_name.size()};
  pub_args.node_name = {node_name.c_str(), node_name.size()};
  pub_args.qos_depth = qos.depth();
  pub_args.qos_is_transient_local = qos.durability() == rclcpp::DurabilityPolicy::TransientLocal;
  pub_args.is_bridge = is_bridge;

  RCLCPP_INFO(
    logger,
    "[agnocast-dbg] AGNOCAST_ADD_PUBLISHER_CMD request: pid=%d topic='%s' node='%s' "
    "qos_depth=%u qos_transient_local=%d is_bridge=%d",
    getpid(), topic_name.c_str(), node_name.c_str(), pub_args.qos_depth,
    pub_args.qos_is_transient_local, is_bridge);

  if (ioctl(agnocast_fd, AGNOCAST_ADD_PUBLISHER_CMD, &pub_args) < 0) {
    RCLCPP_ERROR(
      logger, "[agnocast-dbg] AGNOCAST_ADD_PUBLISHER_CMD failed: pid=%d topic='%s' node='%s': %s",
      getpid(), topic_name.c_str(), node_name.c_str(), strerror(errno));
    close(agnocast_fd);
    exit(EXIT_FAILURE);
  }

  RCLCPP_INFO(
    logger,
    "[agnocast-dbg] AGNOCAST_ADD_PUBLISHER_CMD success: pid=%d topic='%s' node='%s' "
    "topic_local_id=%d is_bridge=%d",
    getpid(), topic_name.c_str(), node_name.c_str(), pub_args.ret_id, is_bridge);

  return pub_args.ret_id;
}

union ioctl_publish_msg_args publish_core(
  [[maybe_unused]] const void * publisher_handle /* for CARET */, const std::string & topic_name,
  const topic_local_id_t publisher_id, const uint64_t msg_virtual_address,
  std::unordered_map<topic_local_id_t, std::tuple<mqd_t, bool>> & opened_mqs)
{
  std::array<topic_local_id_t, MAX_SUBSCRIBER_NUM> subscriber_ids_buffer{};

  union ioctl_publish_msg_args publish_msg_args = {};
  publish_msg_args.topic_name = {topic_name.c_str(), topic_name.size()};
  publish_msg_args.publisher_id = publisher_id;
  publish_msg_args.msg_virtual_address = msg_virtual_address;
  // The kernel writes subscriber IDs directly to this buffer via copy_to_user,
  // unlike ret_* fields which are copied back through the union.
  publish_msg_args.subscriber_ids_buffer_addr =
    reinterpret_cast<uint64_t>(subscriber_ids_buffer.data());
  publish_msg_args.subscriber_ids_buffer_size = MAX_SUBSCRIBER_NUM;

  if (ioctl(agnocast_fd, AGNOCAST_PUBLISH_MSG_CMD, &publish_msg_args) < 0) {
    RCLCPP_ERROR(logger, "AGNOCAST_PUBLISH_MSG_CMD failed: %s", strerror(errno));
    close(agnocast_fd);
    exit(EXIT_FAILURE);
  }

  TRACEPOINT(agnocast_publish, publisher_handle, publish_msg_args.ret_entry_id);

  for (uint32_t i = 0; i < publish_msg_args.ret_subscriber_num; i++) {
    const topic_local_id_t subscriber_id = subscriber_ids_buffer[i];
    mqd_t mq = 0;
    if (opened_mqs.find(subscriber_id) != opened_mqs.end()) {
      std::tuple<mqd_t, bool> & t = opened_mqs[subscriber_id];
      mq = std::get<0>(t);
      // The boolean in the tuple indicates whether the mq is used in this publication round.
      // An unused mq means that its corresponding subscribers have exited, so we close such mqs
      // later.
      std::get<1>(t) = true;
    } else {
      const std::string mq_name = create_mq_name_for_agnocast_publish(topic_name, subscriber_id);
      mq = mq_open(mq_name.c_str(), O_WRONLY | O_NONBLOCK);
      if (mq == -1) {
        // Right after a subscriber is added, its message queue has not been created yet. Therefore,
        // the `mq_open` call above might fail. In that case, we just continue.
        RCLCPP_INFO_STREAM(
          logger, "[agnocast-dbg] mq_open (publisher side) failed: pid=" << getpid() << " topic='"
                                               << topic_name << "' subscriber_id=" << subscriber_id
                                               << " entry_id=" << publish_msg_args.ret_entry_id
                                               << " mq_name='" << mq_name
                                               << "': " << strerror(errno));
        continue;
      }
      RCLCPP_INFO_STREAM(
        logger, "[agnocast-dbg] mq_open (publisher side) success: pid=" << getpid() << " topic='"
                                             << topic_name << "' subscriber_id=" << subscriber_id
                                             << " mq_name='" << mq_name << "' mqd=" << mq);
      opened_mqs.insert({subscriber_id, {mq, true}});
    }

    struct MqMsgAgnocast mq_msg = {};

    // Count sends per (topic, subscriber) so the first one always reports. ret_entry_id is a
    // global counter, not a per-topic sequence, so throttling on it alone silences low-rate
    // topics entirely and makes "no log" indistinguishable from "never sent".
    uint64_t send_count = 0;
    {
      static std::mutex send_count_mutex;
      static std::unordered_map<std::string, uint64_t> send_counts;
      std::lock_guard<std::mutex> lock(send_count_mutex);
      send_count = ++send_counts[topic_name + "#" + std::to_string(subscriber_id)];
    }
    const bool report = !dbg_topic_filter_is_unset() || send_count == 1 || send_count % 100 == 0;

    // Although the size of the struct is 1, we deliberately send a zero-length message
    if (mq_send(mq, reinterpret_cast<char *>(&mq_msg), 0 /*msg_len*/, 0) == -1) {
      const int send_errno = errno;
      if (send_errno != EAGAIN) {
        RCLCPP_ERROR_STREAM(
          logger, "[agnocast-dbg] mq_send failed #" << send_count << ": pid=" << getpid()
                                               << " topic='" << topic_name
                                               << "' subscriber_id=" << subscriber_id
                                               << " entry_id=" << publish_msg_args.ret_entry_id
                                               << ": " << strerror(send_errno));
      } else if (is_dbg_target_topic(topic_name) && report) {
        // EAGAIN means a wake-up is already queued and unread. The queue is depth-1, so this is
        // benign when the subscriber is merely behind -- but permanent EAGAIN from #1 onward means
        // the subscriber never drained the doorbell at all.
        RCLCPP_INFO_STREAM(
          logger, "[agnocast-dbg] mq_send EAGAIN #" << send_count << " (doorbell still full): pid="
                                               << getpid() << " topic='" << topic_name
                                               << "' subscriber_id=" << subscriber_id
                                               << " entry_id=" << publish_msg_args.ret_entry_id);
      }
    } else if (is_dbg_target_topic(topic_name) && report) {
      RCLCPP_INFO_STREAM(
        logger, "[agnocast-dbg] mq_send success #" << send_count << ": pid=" << getpid()
                                             << " topic='" << topic_name
                                             << "' subscriber_id=" << subscriber_id
                                             << " entry_id=" << publish_msg_args.ret_entry_id);
    }
  }

  // Close mqs that are no longer needed and update `opened_mqs`
  for (auto it = opened_mqs.begin(); it != opened_mqs.end();) {
    bool & keep = std::get<1>(it->second);
    if (!keep) {
      mqd_t mq = std::get<0>(it->second);
      if (mq_close(mq) == -1) {
        RCLCPP_ERROR_STREAM(
          logger, "mq_close failed for topic '" << topic_name << "' (subscriber_id=" << it->first
                                                << "): " << strerror(errno));
      }
      it = opened_mqs.erase(it);
    } else {
      // Update the value for the next publication round
      keep = false;
      ++it;
    }
  }

  return publish_msg_args;
}

uint32_t get_subscription_count_core(const std::string & topic_name)
{
  union ioctl_get_subscriber_num_args args = {};
  args.topic_name = {topic_name.c_str(), topic_name.size()};
  if (ioctl(agnocast_fd, AGNOCAST_GET_SUBSCRIBER_NUM_CMD, &args) < 0) {
    RCLCPP_ERROR(logger, "AGNOCAST_GET_SUBSCRIBER_NUM_CMD failed: %s", strerror(errno));
    close(agnocast_fd);
    exit(EXIT_FAILURE);
  }

  uint32_t inter_count = args.ret_other_process_subscriber_num;
  // If an A2R bridge exists, exclude the agnocast subscriber created by the bridge
  if (args.ret_a2r_bridge_exist && inter_count > 0) {
    inter_count--;
  }

  uint32_t ros2_count = args.ret_ros2_subscriber_num;
  // If an R2A bridge exists, exclude the ROS 2 subscriber created by the bridge
  if (args.ret_r2a_bridge_exist && ros2_count > 0) {
    ros2_count--;
  }

  const uint32_t result = inter_count + ros2_count;

  // [AGN_DEBUG] Report the breakdown whenever the reported count changes for a topic. Callers such
  // as autoware_image_projection_based_fusion and autoware_tensorrt_yolox gate publishing (and even
  // tear down their input subscription) on this value, so a count of 0 while real subscribers exist
  // silently kills a whole pipeline.
  {
    static std::mutex agn_debug_mtx;
    static std::unordered_map<std::string, uint32_t> agn_debug_last;
    std::lock_guard<std::mutex> lock(agn_debug_mtx);
    const auto it = agn_debug_last.find(topic_name);
    if (it == agn_debug_last.end() || it->second != result) {
      agn_debug_last[topic_name] = result;
      RCLCPP_INFO(
        logger,
        "[AGN_DEBUG] get_subscription_count('%s') = %u  "
        "(raw: same_process=%u other_process=%u ros2=%u, a2r_bridge=%d r2a_bridge=%d, "
        "after exclusion: inter=%u ros2=%u). "
        "NOTE: same_process agnocast subscribers are NOT included in this count.",
        topic_name.c_str(), result, args.ret_same_process_subscriber_num,
        args.ret_other_process_subscriber_num, args.ret_ros2_subscriber_num,
        static_cast<int>(args.ret_a2r_bridge_exist), static_cast<int>(args.ret_r2a_bridge_exist),
        inter_count, ros2_count);
    }
  }

  return result;
}

uint32_t get_intra_subscription_count_core(const std::string & topic_name)
{
  union ioctl_get_subscriber_num_args get_subscriber_count_args = {};
  get_subscriber_count_args.topic_name = {topic_name.c_str(), topic_name.size()};
  if (ioctl(agnocast_fd, AGNOCAST_GET_SUBSCRIBER_NUM_CMD, &get_subscriber_count_args) < 0) {
    RCLCPP_ERROR(logger, "AGNOCAST_GET_SUBSCRIBER_NUM_CMD failed: %s", strerror(errno));
    close(agnocast_fd);
    exit(EXIT_FAILURE);
  }

  return get_subscriber_count_args.ret_same_process_subscriber_num;
}

}  // namespace agnocast
