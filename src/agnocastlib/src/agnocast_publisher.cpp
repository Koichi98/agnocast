#include "agnocast/agnocast_publisher.hpp"

#include "agnocast/node/agnocast_node.hpp"

#include <sys/types.h>

#include <array>

namespace agnocast
{

thread_local uint32_t borrowed_publisher_num = 0;

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
  const bool is_bridge)
{
  validate_ld_preload();

  union ioctl_add_publisher_args pub_args = {};
  pub_args.topic_name = {topic_name.c_str(), topic_name.size()};
  pub_args.node_name = {node_name.c_str(), node_name.size()};
  pub_args.qos_depth = qos.depth();
  pub_args.qos_is_transient_local = qos.durability() == rclcpp::DurabilityPolicy::TransientLocal;
  pub_args.is_bridge = is_bridge;
  if (ioctl(agnocast_fd, AGNOCAST_ADD_PUBLISHER_CMD, &pub_args) < 0) {
    RCLCPP_ERROR(logger, "AGNOCAST_ADD_PUBLISHER_CMD failed: %s", strerror(errno));
    close(agnocast_fd);
    exit(EXIT_FAILURE);
  }

  return pub_args.ret_id;
}

union ioctl_publish_msg_args publish_core(
  [[maybe_unused]] const void * publisher_handle /* for CARET */, const std::string & topic_name,
  const topic_local_id_t publisher_id, const uint64_t msg_virtual_address)
{
  union ioctl_publish_msg_args publish_msg_args = {};
  publish_msg_args.topic_name = {topic_name.c_str(), topic_name.size()};
  publish_msg_args.publisher_id = publisher_id;
  publish_msg_args.msg_virtual_address = msg_virtual_address;

  if (ioctl(agnocast_fd, AGNOCAST_PUBLISH_MSG_CMD, &publish_msg_args) < 0) {
    RCLCPP_ERROR(logger, "AGNOCAST_PUBLISH_MSG_CMD failed: %s", strerror(errno));
    close(agnocast_fd);
    exit(EXIT_FAILURE);
  }

  TRACEPOINT(agnocast_publish, publisher_handle, publish_msg_args.ret_entry_id);

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

  return inter_count + ros2_count;
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
