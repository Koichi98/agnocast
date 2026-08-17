#include "agnocast/node/node_interfaces/node_graph.hpp"

#include "agnocast/agnocast_ioctl.hpp"
#include "agnocast/agnocast_publisher.hpp"
#include "agnocast/agnocast_subscription.hpp"
#include "agnocast/agnocast_utils.hpp"
#include "agnocast/internal/ros2_node_registry_reader.hpp"

#include <sys/ioctl.h>
#include <unistd.h>

#include <algorithm>
#include <optional>
#include <stdexcept>

namespace agnocast::node_interfaces
{

NodeGraph::NodeGraph(NodeBase::SharedPtr node_base) : node_base_(std::move(node_base))
{
}

std::map<std::string, std::vector<std::string>> NodeGraph::get_topic_names_and_types(
  bool no_demangle) const
{
  (void)no_demangle;
  throw std::runtime_error("NodeGraph::get_topic_names_and_types is not supported in agnocast.");
}

std::map<std::string, std::vector<std::string>> NodeGraph::get_service_names_and_types() const
{
  throw std::runtime_error("NodeGraph::get_service_names_and_types is not supported in agnocast.");
}

std::map<std::string, std::vector<std::string>> NodeGraph::get_service_names_and_types_by_node(
  const std::string & node_name, const std::string & namespace_) const
{
  (void)node_name;
  (void)namespace_;
  throw std::runtime_error(
    "NodeGraph::get_service_names_and_types_by_node is not supported in agnocast.");
}

std::map<std::string, std::vector<std::string>> NodeGraph::get_client_names_and_types_by_node(
  const std::string & node_name, const std::string & namespace_) const
{
  (void)node_name;
  (void)namespace_;
  throw std::runtime_error(
    "NodeGraph::get_client_names_and_types_by_node is not supported in agnocast.");
}

std::map<std::string, std::vector<std::string>> NodeGraph::get_publisher_names_and_types_by_node(
  const std::string & node_name, const std::string & namespace_, bool no_demangle) const
{
  (void)node_name;
  (void)namespace_;
  (void)no_demangle;
  throw std::runtime_error(
    "NodeGraph::get_publisher_names_and_types_by_node is not supported in agnocast.");
}

std::map<std::string, std::vector<std::string>> NodeGraph::get_subscriber_names_and_types_by_node(
  const std::string & node_name, const std::string & namespace_, bool no_demangle) const
{
  (void)node_name;
  (void)namespace_;
  (void)no_demangle;
  throw std::runtime_error(
    "NodeGraph::get_subscriber_names_and_types_by_node is not supported in agnocast.");
}

namespace
{
std::vector<std::string> query_agnocast_node_names(const bool exclude_ros2_nodes)
{
  // Sized to the kernel-side limit so that the kernel never has to report -ENOBUFS.
  std::vector<char> buffer(static_cast<size_t>(MAX_NODE_NUM) * NODE_NAME_BUFFER_SIZE);

  union ioctl_get_node_names_args get_node_names_args = {};
  get_node_names_args.node_name_buffer_addr = reinterpret_cast<uint64_t>(buffer.data());
  get_node_names_args.node_name_buffer_size = static_cast<uint32_t>(buffer.size());
  get_node_names_args.exclude_ros2_nodes = exclude_ros2_nodes;
  if (ioctl(agnocast_fd, AGNOCAST_GET_NODE_NAMES_CMD, &get_node_names_args) < 0) {
    RCLCPP_ERROR(logger, "AGNOCAST_GET_NODE_NAMES_CMD failed: %s", strerror(errno));
    close(agnocast_fd);
    exit(EXIT_FAILURE);
  }

  std::vector<std::string> node_names;
  node_names.reserve(get_node_names_args.ret_node_num);
  const char * current_ptr = buffer.data();

  for (uint32_t i = 0; i < get_node_names_args.ret_node_num; ++i) {
    node_names.emplace_back(current_ptr);
    current_ptr += node_names.back().length() + 1;
  }

  return node_names;
}

std::optional<std::vector<std::string>> read_ros2_node_names_of_this_scope()
{
  try {
    return internal::read_ros2_node_names(get_self_ipc_ns_inode(), get_ros_domain_id());
  } catch (const std::exception & e) {
    RCLCPP_WARN_ONCE(
      logger, "Failed to read the IPC namespace inode (%s), so ROS 2 nodes are not reported.",
      e.what());
    return std::nullopt;
  }
}
}  // namespace

// The graph is served from two sources that do not overlap: the discovery agent contributes the
// nodes DDS knows about, and the kmod the Agnocast-only ones (`agnocast::Node`, which no
// participant ever announces). Splitting them this way -- rather than taking the union of two
// overlapping lists -- is what lets duplicate node names survive: `rclcpp` reports a name once per
// node that carries it, and merging by name would collapse those into one entry.
//
// Both sources are scoped to this IPC namespace and ROS_DOMAIN_ID, which is also the scope the
// agent runs in (one agent per (namespace, domain)).
//
// Without an agent there is no DDS-side list, and the second-best answer is every Agnocast node
// regardless of whether DDS also announces it: a node visible through one interface beats a node
// missing from both. The result therefore holds fewer node names once an agent starts, if the
// process' own nodes are the only Agnocast ones around.
std::vector<std::string> NodeGraph::get_node_names() const
{
  const std::optional<std::vector<std::string>> ros2_node_names =
    read_ros2_node_names_of_this_scope();

  std::vector<std::string> node_names = query_agnocast_node_names(ros2_node_names.has_value());
  if (ros2_node_names) {
    node_names.insert(node_names.end(), ros2_node_names->begin(), ros2_node_names->end());
  }

  // rclcpp reports the calling node unconditionally; the kmod only knows the nodes that own an
  // endpoint, so a node that has not created one yet has to add itself. The name check keeps a node
  // that does own an endpoint from being counted twice -- at the cost of hiding the rare case where
  // another process happens to run a node of the same name.
  const std::string self_name = node_base_->get_fully_qualified_name();
  if (std::find(node_names.begin(), node_names.end(), self_name) == node_names.end()) {
    node_names.push_back(self_name);
  }

  return node_names;
}

std::vector<std::tuple<std::string, std::string, std::string>>
NodeGraph::get_node_names_with_enclaves() const
{
  throw std::runtime_error("NodeGraph::get_node_names_with_enclaves is not supported in agnocast.");
}

std::vector<std::pair<std::string, std::string>> NodeGraph::get_node_names_and_namespaces() const
{
  throw std::runtime_error(
    "NodeGraph::get_node_names_and_namespaces is not supported in agnocast.");
}

// Counts agnocast and ROS 2 endpoints, excluding the ones created by bridges.
// agnocast::Node::count_publishers()/count_subscribers() delegate here, so the resolution and the
// bridge bookkeeping live in one place.
//
// Note that count_subscribers() does not see agnocast subscribers in the caller's own process:
// get_subscription_count_core() reports ret_other_process_subscriber_num, because its original
// caller (agnocast::Publisher::get_subscription_count()) asks how many peers receive a published
// message. count_publishers() has no such split.
size_t NodeGraph::count_publishers(const std::string & topic_name) const
{
  return get_publisher_count_core(node_base_->resolve_topic_or_service_name(topic_name, false));
}

size_t NodeGraph::count_subscribers(const std::string & topic_name) const
{
  return get_subscription_count_core(node_base_->resolve_topic_or_service_name(topic_name, false));
}

const rcl_guard_condition_t * NodeGraph::get_graph_guard_condition() const
{
  throw std::runtime_error("NodeGraph::get_graph_guard_condition is not supported in agnocast.");
}

// No-op rather than throwing: agnocast has no graph events, so there is nothing to notify.
// rclcpp utilities call these unconditionally when entities are created or the context shuts
// down, and throwing there would break otherwise working code.
void NodeGraph::notify_graph_change()
{
}

void NodeGraph::notify_shutdown()
{
}

rclcpp::Event::SharedPtr NodeGraph::get_graph_event()
{
  throw std::runtime_error("NodeGraph::get_graph_event is not supported in agnocast.");
}

void NodeGraph::wait_for_graph_change(
  rclcpp::Event::SharedPtr event, std::chrono::nanoseconds timeout)
{
  (void)event;
  (void)timeout;
  throw std::runtime_error("NodeGraph::wait_for_graph_change is not supported in agnocast.");
}

size_t NodeGraph::count_graph_users() const
{
  throw std::runtime_error("NodeGraph::count_graph_users is not supported in agnocast.");
}

std::vector<rclcpp::TopicEndpointInfo> NodeGraph::get_publishers_info_by_topic(
  const std::string & topic_name, bool no_mangle) const
{
  (void)topic_name;
  (void)no_mangle;
  throw std::runtime_error("NodeGraph::get_publishers_info_by_topic is not supported in agnocast.");
}

std::vector<rclcpp::TopicEndpointInfo> NodeGraph::get_subscriptions_info_by_topic(
  const std::string & topic_name, bool no_mangle) const
{
  (void)topic_name;
  (void)no_mangle;
  throw std::runtime_error(
    "NodeGraph::get_subscriptions_info_by_topic is not supported in agnocast.");
}

// rclcpp 28+ (Jazzy) added these methods to NodeGraphInterface.
#if RCLCPP_VERSION_MAJOR >= 28
size_t NodeGraph::count_clients(const std::string & service_name) const
{
  (void)service_name;
  throw std::runtime_error("NodeGraph::count_clients is not supported in agnocast.");
}

size_t NodeGraph::count_services(const std::string & service_name) const
{
  (void)service_name;
  throw std::runtime_error("NodeGraph::count_services is not supported in agnocast.");
}
#endif

}  // namespace agnocast::node_interfaces
