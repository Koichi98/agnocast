#include "agnocast/agnocast.hpp"
#include "agnocast/node/agnocast_node.hpp"

#include <sys/eventfd.h>

namespace agnocast
{

SubscriptionBase::SubscriptionBase(rclcpp::Node * node, const std::string & topic_name)
: id_(0), topic_name_(node->get_node_topics_interface()->resolve_topic_name(topic_name))
{
  validate_ld_preload();
}

SubscriptionBase::SubscriptionBase(
  agnocast::Node * node, const std::string & topic_name)  // NOLINT(modernize-pass-by-value)
: id_(0), topic_name_(node->get_node_topics_interface()->resolve_topic_name(topic_name))
{
  validate_ld_preload();
}

union ioctl_add_subscriber_args SubscriptionBase::initialize(
  const rclcpp::QoS & qos, const bool is_take_sub, const bool ignore_local_publications,
  const bool is_bridge, const std::string & node_name)
{
  int efd = -1;
  if (!is_take_sub) {
    efd = eventfd(0, EFD_NONBLOCK | EFD_CLOEXEC);
    if (efd == -1) {
      RCLCPP_ERROR(logger, "eventfd creation failed: %s", strerror(errno));
      close(agnocast_fd);
      exit(EXIT_FAILURE);
    }
  }

  union ioctl_add_subscriber_args add_subscriber_args = {};
  add_subscriber_args.topic_name = {topic_name_.c_str(), topic_name_.size()};
  add_subscriber_args.node_name = {node_name.c_str(), node_name.size()};
  add_subscriber_args.qos_depth = static_cast<uint32_t>(qos.depth());
  add_subscriber_args.qos_is_transient_local =
    qos.durability() == rclcpp::DurabilityPolicy::TransientLocal;
  add_subscriber_args.qos_is_reliable = qos.reliability() == rclcpp::ReliabilityPolicy::Reliable;
  add_subscriber_args.is_take_sub = is_take_sub;
  add_subscriber_args.ignore_local_publications = ignore_local_publications;
  add_subscriber_args.is_bridge = is_bridge;
  add_subscriber_args.eventfd = efd;
  if (ioctl(agnocast_fd, AGNOCAST_ADD_SUBSCRIBER_CMD, &add_subscriber_args) < 0) {
    RCLCPP_ERROR(logger, "AGNOCAST_ADD_SUBSCRIBER_CMD failed: %s", strerror(errno));
    if (efd >= 0) {
      close(efd);
    }
    close(agnocast_fd);
    exit(EXIT_FAILURE);
  }

  notify_eventfd_ = efd;

  return add_subscriber_args;
}

uint32_t get_publisher_count_core(const std::string & topic_name)
{
  union ioctl_get_publisher_num_args args = {};
  args.topic_name = {topic_name.c_str(), topic_name.size()};
  if (ioctl(agnocast_fd, AGNOCAST_GET_PUBLISHER_NUM_CMD, &args) < 0) {
    RCLCPP_ERROR(logger, "AGNOCAST_GET_PUBLISHER_NUM_CMD failed: %s", strerror(errno));
    close(agnocast_fd);
    exit(EXIT_FAILURE);
  }

  uint32_t count = args.ret_publisher_num;
  // If an R2A bridge exists, exclude the agnocast publisher created by the bridge
  if (args.ret_r2a_bridge_exist && count > 0) {
    count--;
  }

  uint32_t ros2_count = args.ret_ros2_publisher_num;
  // If an A2R bridge exists, exclude the ROS 2 publisher created by the bridge
  if (args.ret_a2r_bridge_exist && ros2_count > 0) {
    ros2_count--;
  }

  return count + ros2_count;
}

void close_notify_eventfd(int eventfd)
{
  if (eventfd >= 0) {
    close(eventfd);
  }
}

rclcpp::CallbackGroup::SharedPtr get_default_callback_group_for_tracepoint(agnocast::Node * node)
{
  return node->get_node_base_interface()->get_default_callback_group();
}

}  // namespace agnocast
