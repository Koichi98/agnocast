#include "agnocast/bridge/performance/agnocast_performance_bridge_loader.hpp"

#include "agnocast/agnocast_publisher.hpp"
#include "agnocast/agnocast_subscription.hpp"
#include "agnocast/agnocast_utils.hpp"
#include "agnocast/bridge/agnocast_bridge_node.hpp"
#include "agnocast/bridge/performance/agnocast_generic_service.hpp"
#include "rclcpp/version.h"

#include <ament_index_cpp/get_package_prefix.hpp>

#include <dlfcn.h>
#include <rcl/service.h>
#include <rmw/rmw.h>

#include <algorithm>
#include <cstdlib>
#include <memory>
#include <mutex>
#include <queue>
#include <sstream>
#include <utility>

namespace agnocast
{

PerformanceBridgeLoader::PerformanceBridgeLoader(const rclcpp::Logger & logger) : logger_(logger)
{
}

PerformanceBridgeLoader::~PerformanceBridgeLoader()
{
  for (auto & pair : loaded_libraries_) {
    if (pair.second != nullptr) {
      dlclose(pair.second);
    }
  }
  loaded_libraries_.clear();
}

PerformancePubsubBridgeResult PerformanceBridgeLoader::create_r2a_pubsub_bridge(
  rclcpp::Node::SharedPtr node, const std::string & topic_name, const std::string & message_type,
  const rclcpp::QoS & qos)
{
  void * symbol = get_bridge_factory_symbol(message_type, "create_r2a_pubsub_bridge", false);
  if (symbol == nullptr) {
    // Fall back to the generic bridge, which is independent of plugins.
    RCLCPP_DEBUG(
      logger_, "No plugin found for topic '%s' (type: %s). Using generic bridge.",
      topic_name.c_str(), message_type.c_str());
    return create_r2a_pubsub_bridge_generic(node, topic_name, message_type, qos);
  }

  auto factory = reinterpret_cast<R2APubsubBridgeFactory>(symbol);
  return factory(std::move(node), topic_name, qos);
}

PerformancePubsubBridgeResult PerformanceBridgeLoader::create_a2r_pubsub_bridge(
  rclcpp::Node::SharedPtr node, const std::string & topic_name, const std::string & message_type,
  const rclcpp::QoS & qos)
{
  void * symbol = get_bridge_factory_symbol(message_type, "create_a2r_pubsub_bridge", false);
  if (symbol == nullptr) {
    // Fall back to the generic bridge, which is independent of plugins.
    RCLCPP_DEBUG(
      logger_, "No plugin found for topic '%s' (type: %s). Using generic bridge.",
      topic_name.c_str(), message_type.c_str());
    return create_a2r_pubsub_bridge_generic(node, topic_name, message_type, qos);
  }

  auto factory = reinterpret_cast<A2RPubsubBridgeFactory>(symbol);
  return factory(std::move(node), topic_name, qos);
}

PerformanceServiceBridgeResult PerformanceBridgeLoader::create_r2a_service_bridge(
  rclcpp::Node::SharedPtr node, const std::string & service_name, const std::string & service_type,
  const rclcpp::QoS & qos)
{
  void * symbol = get_bridge_factory_symbol(service_type, "create_r2a_service_bridge", true);
  if (symbol == nullptr) {
    // Fall back to the generic (plugin-free) service bridge.
    RCLCPP_DEBUG(
      logger_, "No plugin found for service '%s' (type: %s). Using generic bridge.",
      service_name.c_str(), service_type.c_str());
    return create_r2a_service_bridge_generic(node, service_name, service_type, qos);
  }

  auto factory = reinterpret_cast<R2AServiceBridgeFactory>(symbol);
  return factory(std::move(node), service_name, qos);
}

namespace
{
// Shared state for one generic R2A service bridge. Requests are forwarded to the Agnocast service
// one-in-flight, so the single response on the bridge's response topic maps unambiguously to the
// pending request without reading back a per-message correlation id.
class GenericR2AServiceState
{
public:
  std::shared_ptr<agnocast::GenericPublisher> req_pub;
  std::shared_ptr<agnocast::GenericSubscription> res_sub;
  std::shared_ptr<agnocast::bridge::GenericService> ros_srv;
  std::string node_fqn;

  void enqueue(std::shared_ptr<rmw_request_id_t> req_id, rclcpp::SerializedMessage && req)
  {
    std::lock_guard<std::mutex> lock(mtx_);
    pending_.emplace(std::move(req_id), std::move(req));
    if (!in_flight_) {
      dispatch_locked();
    }
  }

  void on_response(const rclcpp::SerializedMessage & res)
  {
    std::shared_ptr<rmw_request_id_t> req_id;
    {
      std::lock_guard<std::mutex> lock(mtx_);
      if (!in_flight_ || pending_.empty()) {
        return;  // unexpected response; ignore
      }
      req_id = pending_.front().first;
      pending_.pop();
      in_flight_ = false;
    }

    // Deserialize the response payload and send it back over ROS.
    auto ros_response = ros_srv->create_response();
    const rmw_ret_t ret = rmw_deserialize(
      &res.get_rcl_serialized_message(), ros_srv->response_message_typesupport(),
      ros_response.get());
    if (ret == RMW_RET_OK) {
      ros_srv->send_response(*req_id, ros_response);
    }

    std::lock_guard<std::mutex> lock(mtx_);
    if (!in_flight_ && !pending_.empty()) {
      dispatch_locked();
    }
  }

private:
  void dispatch_locked()
  {
    in_flight_ = true;
    req_pub->publish_service_request(pending_.front().second, ++seq_, node_fqn);
  }

  std::mutex mtx_;
  std::queue<std::pair<std::shared_ptr<rmw_request_id_t>, rclcpp::SerializedMessage>> pending_;
  bool in_flight_ = false;
  int64_t seq_ = 0;
};
}  // namespace

PerformanceServiceBridgeResult PerformanceBridgeLoader::create_r2a_service_bridge_generic(
  const rclcpp::Node::SharedPtr & node, const std::string & service_name,
  const std::string & service_type, const rclcpp::QoS & qos)
{
  const std::string request_type = service_type + "_Request";
  const std::string response_type = service_type + "_Response";
  const std::string request_topic = create_service_request_topic_name(service_name);
  const std::string node_fqn = node->get_fully_qualified_name();
  const std::string response_topic = create_service_response_topic_name(service_name, node_fqn);

  auto srv_cb_group = node->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  auto sub_cb_group = node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  auto state = std::make_shared<GenericR2AServiceState>();
  state->node_fqn = node_fqn;

  state->req_pub = std::make_shared<agnocast::GenericPublisher>(
    node.get(), request_topic, request_type, qos, agnocast::PublisherOptions{},
    agnocast::PublisherRole::BridgeInternal);

  // ROS-side type-erased service server: deferred response (forward request, respond later).
  std::weak_ptr<GenericR2AServiceState> weak_state = state;
  agnocast::bridge::GenericService::DeferredCallback deferred_cb =
    [weak_state](
      const std::shared_ptr<agnocast::bridge::GenericService> & service,
      std::shared_ptr<rmw_request_id_t> req_id, std::shared_ptr<void> request) {
      auto st = weak_state.lock();
      if (!st) {
        return;
      }
      // Guard the executor thread: an exception escaping here would terminate the bridge daemon.
      try {
        rclcpp::SerializedMessage serialized;
        const rmw_ret_t ret = rmw_serialize(
          request.get(), service->request_message_typesupport(),
          &serialized.get_rcl_serialized_message());
        if (ret != RMW_RET_OK) {
          return;
        }
        st->enqueue(std::move(req_id), std::move(serialized));
      } catch (const std::exception & e) {
        RCLCPP_ERROR(
          rclcpp::get_logger("agnocast_generic_service_bridge"),
          "R2A generic service request forwarding failed: %s", e.what());
      }
    };

  rcl_service_options_t service_options = rcl_service_get_default_options();
  service_options.qos = qos.get_rmw_qos_profile();
  state->ros_srv = agnocast::bridge::GenericService::make_shared(
    node->get_node_base_interface()->get_shared_rcl_node_handle(), service_name, service_type,
    std::move(deferred_cb), service_options);
  node->get_node_services_interface()->add_service(
    std::dynamic_pointer_cast<rclcpp::ServiceBase>(state->ros_srv), srv_cb_group);

  // Agnocast-side response subscription (response payload at wire offset 0).
  agnocast::SubscriptionOptions sub_opts;
  sub_opts.ignore_local_publications = true;
  sub_opts.callback_group = sub_cb_group;
  state->res_sub = std::make_shared<agnocast::GenericSubscription>(
    node.get(), response_topic, response_type, qos,
    [weak_state](const rclcpp::SerializedMessage & res) {
      if (auto st = weak_state.lock()) {
        st->on_response(res);
      }
    },
    sub_opts, agnocast::SubscriptionRole::BridgeInternal);

  PerformanceServiceBridgeResult result;
  result.entity_handle = state;
  result.ros_srv_cb_group = srv_cb_group;
  result.agno_client_cb_group = sub_cb_group;
  return result;
}

std::string PerformanceBridgeLoader::convert_type_to_snake_case(const std::string & message_type)
{
  std::string result = message_type;
  std::replace(result.begin(), result.end(), '/', '_');
  return result;
}

std::vector<std::string> PerformanceBridgeLoader::generate_library_paths()
{
  std::vector<std::string> paths;
  const std::string lib_name = "libagnocast_bridge_plugins.so";

  // 1. Check environment variable AGNOCAST_BRIDGE_PLUGINS_PATH (colon-separated)
  const char * env_path = std::getenv("AGNOCAST_BRIDGE_PLUGINS_PATH");
  if (env_path != nullptr) {
    std::string env_str(env_path);
    std::istringstream iss(env_str);
    std::string path;
    while (std::getline(iss, path, ':')) {
      if (!path.empty()) {
        std::string full_path;
        full_path.reserve(path.size() + 1 + lib_name.size());
        full_path += path;
        full_path += '/';
        full_path += lib_name;
        paths.push_back(std::move(full_path));
      }
    }
  }

  // 2. Check user-generated agnocast_bridge_plugins package
  try {
    const std::string user_prefix = ament_index_cpp::get_package_prefix("agnocast_bridge_plugins");
    paths.push_back(user_prefix + "/lib/agnocast_bridge_plugins/" + lib_name);
  } catch (const ament_index_cpp::PackageNotFoundError &) {
    // Package not found, continue
  }

  return paths;
}

void * PerformanceBridgeLoader::load_library_from_paths(
  const std::vector<std::string> & paths, std::string & last_error)
{
  last_error.clear();

  if (paths.empty()) {
    return nullptr;
  }

  for (const auto & path : paths) {
    // Check cache first
    if (loaded_libraries_.find(path) != loaded_libraries_.end()) {
      return loaded_libraries_[path];
    }

    // Try to load
    void * handle = dlopen(path.c_str(), RTLD_LAZY);
    if (handle != nullptr) {
      loaded_libraries_[path] = handle;
      return handle;
    }
    // Capture error immediately before any subsequent call clears it.
    const char * err = dlerror();
    if (err != nullptr) {
      last_error = err;
    }
  }

  return nullptr;
}

void * PerformanceBridgeLoader::get_bridge_factory_symbol(
  const std::string & type_name, const std::string & symbol_name_prefix, bool is_service)
{
  const char * type_label = is_service ? "service" : "message";
  std::string snake_type = convert_type_to_snake_case(type_name);
  std::vector<std::string> lib_paths = generate_library_paths();

  std::string last_dlopen_error;
  void * handle = load_library_from_paths(lib_paths, last_dlopen_error);
  if (handle == nullptr) {
    if (lib_paths.empty()) {
      if (is_service) {
        RCLCPP_ERROR(
          logger_,
          "No plugin paths available for service bridge. Have you generated bridge plugins?");
      }
    } else {
      std::string tried_paths;
      for (const auto & path : lib_paths) {
        tried_paths += "\n  - " + path;
      }
      if (is_service) {
        RCLCPP_ERROR(
          logger_,
          "Failed to load plugin for service type '%s'.\n"
          "Tried paths:%s\n"
          "Last error: %s",
          type_name.c_str(), tried_paths.c_str(), last_dlopen_error.c_str());
      } else {
        RCLCPP_WARN(
          logger_,
          "Failed to load plugin for message type '%s'. Falling back to generic bridge.\n"
          "Tried paths:%s\n"
          "Last error: %s",
          type_name.c_str(), tried_paths.c_str(), last_dlopen_error.c_str());
      }
    }
    return nullptr;
  }

  const std::string symbol_name = symbol_name_prefix + "_" + snake_type;

  dlerror();
  void * symbol = dlsym(handle, symbol_name.c_str());

  const char * dlsym_error = dlerror();
  if (dlsym_error != nullptr) {
    if (is_service) {
      RCLCPP_ERROR(
        logger_, "Failed to find symbol '%s' for %s type '%s': %s", symbol_name.c_str(), type_label,
        type_name.c_str(), dlsym_error);
    }
    return nullptr;
  }

  if (symbol == nullptr) {
    if (is_service) {
      RCLCPP_ERROR(
        logger_,
        "Symbol '%s' was found for %s type '%s' but returned NULL, which is invalid for a factory "
        "function.",
        symbol_name.c_str(), type_label, type_name.c_str());
    }
    return nullptr;
  }

  return symbol;
}

PerformancePubsubBridgeResult PerformanceBridgeLoader::create_r2a_pubsub_bridge_generic(
  const rclcpp::Node::SharedPtr & node, const std::string & topic_name,
  const std::string & message_type, const rclcpp::QoS & qos)
{
  auto agno_pub = std::make_shared<agnocast::GenericPublisher>(
    node.get(), topic_name, message_type,
    rclcpp::QoS(agnocast::DEFAULT_QOS_DEPTH).transient_local(), agnocast::PublisherOptions{},
    agnocast::PublisherRole::BridgeInternal);

  auto cb_group = node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  rclcpp::SubscriptionOptions opts;
  opts.ignore_local_publications = true;
  opts.callback_group = cb_group;

  // Compatibility note:
  // - Humble expects the callback taking std::shared_ptr<SerializedMessage>.
  // - Jazzy treats the callback as AnySubscriptionCallback, but if we receive the message as
  //   std::shared_ptr<SerializedMessage>, rclcpp deep-copies the message.
  // Keep the Humble/Jazzy split here for performance optimization.
  auto sub = node->create_generic_subscription(
    topic_name, message_type, qos,
#if RCLCPP_VERSION_MAJOR >= 28
    [agno_pub](const rclcpp::SerializedMessage & serialized_msg) {
      agno_pub->publish(serialized_msg);
    },
#else
    [agno_pub](const std::shared_ptr<rclcpp::SerializedMessage> & serialized_msg) {
      agno_pub->publish(*serialized_msg);
    },
#endif
    opts);

  PerformancePubsubBridgeResult result;
  result.entity_handle = sub;
  result.callback_group = cb_group;
  return result;
}

PerformancePubsubBridgeResult PerformanceBridgeLoader::create_a2r_pubsub_bridge_generic(
  const rclcpp::Node::SharedPtr & node, const std::string & topic_name,
  const std::string & message_type, const rclcpp::QoS & qos)
{
  auto ros_pub = node->create_generic_publisher(
    topic_name, message_type,
    rclcpp::QoS(agnocast::DEFAULT_QOS_DEPTH).reliable().transient_local());

  auto cb_group = node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  agnocast::SubscriptionOptions sub_opts;
  sub_opts.ignore_local_publications = true;
  sub_opts.callback_group = cb_group;

  auto agno_sub = std::make_shared<agnocast::GenericSubscription>(
    node.get(), topic_name, message_type, qos,
    [ros_pub](const rclcpp::SerializedMessage & serialized_msg) {
      ros_pub->publish(serialized_msg);
    },
    sub_opts, agnocast::SubscriptionRole::BridgeInternal);

  return {agno_sub, cb_group};
}

}  // namespace agnocast
