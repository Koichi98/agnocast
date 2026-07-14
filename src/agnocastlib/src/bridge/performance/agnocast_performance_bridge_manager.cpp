
#include "agnocast/bridge/performance/agnocast_performance_bridge_manager.hpp"

#include "agnocast/agnocast_callback_isolated_executor.hpp"
#include "agnocast/agnocast_ioctl.hpp"
#include "agnocast/agnocast_mq.hpp"
#include "agnocast/agnocast_utils.hpp"
#include "agnocast/bridge/agnocast_bridge_utils.hpp"

#include <mqueue.h>
#include <sys/prctl.h>
#include <unistd.h>

#include <algorithm>
#include <cstdlib>
#include <mutex>
#include <string>
#include <unordered_map>

namespace agnocast
{

namespace
{
// Verbose per-poll bridge diagnostics. Off by default; enable with
// `export AGNOCAST_BRIDGE_DEBUG=1` to log the a2r/r2a activation gate every poll.
// Transition events (request received / bridge created / bridge removed) are logged
// unconditionally at INFO regardless of this flag.
//
// NOTE: the bridge manager runs in a forked daemon process whose stdout/stderr are normally
// redirected to /dev/null under `ros2 launch` (spawn_daemon_process in agnocast.cpp). That
// redirect is skipped while AGNOCAST_BRIDGE_DEBUG is set, so these RCLCPP logs reach the
// launch-captured stdout/stderr (and thus cloud log collection).
bool bridge_debug_enabled()
{
  static const bool enabled = []() {
    const char * e = std::getenv("AGNOCAST_BRIDGE_DEBUG");
    if (e == nullptr) {
      return false;
    }
    const std::string v(e);
    return v != "0" && v != "off" && v != "false";
  }();
  return enabled;
}
}  // namespace

PerformanceBridgeManager::PerformanceBridgeManager()
: logger_(rclcpp::get_logger("agnocast_performance_bridge_manager")),
  self_ipc_ns_inode_(get_self_ipc_ns_inode()),
  event_loop_(logger_),
  loader_(logger_)
{
  if (rclcpp::ok()) {
    rclcpp::shutdown();
  }

  rclcpp::InitOptions init_options{};
  init_options.shutdown_on_signal = false;
  rclcpp::init(0, nullptr, init_options);
}

PerformanceBridgeManager::~PerformanceBridgeManager()
{
  if (executor_) {
    executor_->cancel();
  }

  if (executor_thread_.joinable()) {
    executor_thread_.join();
  }

  if (rclcpp::ok()) {
    rclcpp::shutdown();
  }
}

void PerformanceBridgeManager::run()
{
  constexpr int EVENT_LOOP_TIMEOUT_MS = 1000;

  std::string proc_name = "agno_pbr_" + std::to_string(getpid());
  prctl(PR_SET_NAME, proc_name.c_str(), 0, 0, 0);

  start_ros_execution();

  event_loop_.set_mq_handler([this](int fd) { this->on_mq_request(fd); });
  event_loop_.set_signal_handler([this]() { this->on_signal(); });
  event_loop_.set_socket_handler([this]() { return this->on_socket_request(); });

  while (!shutdown_requested_) {
    if (!event_loop_.spin_once(EVENT_LOOP_TIMEOUT_MS)) {
      RCLCPP_ERROR(logger_, "Event loop spin failed.");
      break;
    }

    check_and_create_pubsub_bridges();
    check_and_remove_pubsub_bridges();
    check_and_remove_service_bridges();
    check_and_remove_request_cache();
    check_and_request_shutdown();
  }
}

void PerformanceBridgeManager::start_ros_execution()
{
  std::string node_name = "agnocast_bridge_node_performance";
  container_node_ = std::make_shared<rclcpp::Node>(node_name);

  // We must not use single-threaded executors because of how service bridges work. Service bridges
  // require two callback groups to execute concurrently. If a single-threaded executor is used, it
  // can deadlock. See the service bridge implementation for details.
  executor_ = std::make_shared<agnocast::CallbackIsolatedAgnocastExecutor>();
  executor_->add_node(container_node_);

  executor_thread_ = std::thread([this]() {
    try {
      this->executor_->spin();
    } catch (const std::exception & e) {
      if (ioctl(agnocast_fd, AGNOCAST_NOTIFY_BRIDGE_SHUTDOWN_CMD) < 0) {
        RCLCPP_ERROR(logger_, "Failed to notify bridge shutdown: %s", strerror(errno));
      }
      shutdown_requested_ = true;
      RCLCPP_ERROR(logger_, "Executor Thread CRASHED: %s", e.what());
    }
  });
}

void PerformanceBridgeManager::on_mq_request(int fd)
{
  MqMsgPerformanceBridge msg{};

  ssize_t bytes_read = mq_receive(fd, reinterpret_cast<char *>(&msg), sizeof(msg), nullptr);
  if (bytes_read < 0) {
    if (errno != EAGAIN) {
      RCLCPP_WARN_STREAM(
        logger_, "mq_receive failed for mq_name='" << event_loop_.get_mq_name() << "' (fd=" << fd
                                                   << "): " << strerror(errno));
    }
    return;
  }

  if (msg.is_service) {
    create_service_bridge_if_needed(msg.srv_target, msg.direction);
  } else {
    std::string topic_name = static_cast<const char *>(msg.pubsub_target.topic_name);
    topic_local_id_t target_id = msg.pubsub_target.target_id;
    std::string message_type = static_cast<const char *>(msg.pubsub_target.message_type);

    request_cache_[topic_name][target_id] = msg;

    RCLCPP_INFO(
      logger_,
      "[bridge-dbg] request received: topic='%s' type='%s' dir=%s target_id=%d",
      topic_name.c_str(), message_type.c_str(),
      msg.direction == BridgeDirection::AGNOCAST_TO_ROS2 ? "A2R" : "R2A", target_id);

    create_pubsub_bridge_if_needed(
      topic_name, request_cache_[topic_name], message_type, msg.direction);
  }
}

void PerformanceBridgeManager::on_signal()
{
  if (ioctl(agnocast_fd, AGNOCAST_NOTIFY_BRIDGE_SHUTDOWN_CMD) < 0) {
    RCLCPP_ERROR(logger_, "Failed to notify bridge shutdown: %s", strerror(errno));
  }
  shutdown_requested_ = true;
  if (executor_) {
    executor_->cancel();
  }
}

std::string PerformanceBridgeManager::on_socket_request() const
{
  return R"({"type":"performance","ipc_ns":)" + std::to_string(self_ipc_ns_inode_) + R"(,"pid":)" +
         std::to_string(getpid()) + "}";
}

void PerformanceBridgeManager::check_and_create_pubsub_bridges()
{
  for (auto cache_it = request_cache_.begin(); cache_it != request_cache_.end();) {
    const auto & topic_name = cache_it->first;
    auto & requests = cache_it->second;

    if (requests.empty()) {
      cache_it = request_cache_.erase(cache_it);
      continue;
    }

    const std::string message_type =
      static_cast<const char *>(requests.begin()->second.pubsub_target.message_type);

    create_pubsub_bridge_if_needed(
      topic_name, requests, message_type, BridgeDirection::ROS2_TO_AGNOCAST);
    create_pubsub_bridge_if_needed(
      topic_name, requests, message_type, BridgeDirection::AGNOCAST_TO_ROS2);

    if (requests.empty()) {
      cache_it = request_cache_.erase(cache_it);
    } else {
      ++cache_it;
    }
  }
}

void PerformanceBridgeManager::check_and_remove_pubsub_bridges()
{
  auto r2a_it = active_pubsub_r2a_bridges_.begin();
  while (r2a_it != active_pubsub_r2a_bridges_.end()) {
    const std::string & topic_name = r2a_it->first;
    auto result = get_agnocast_subscriber_count(topic_name);
    bool is_demanded_by_ros2 = has_external_ros2_publisher(container_node_.get(), topic_name);
    if (result.count == -1) {
      RCLCPP_ERROR(
        logger_, "Failed to get subscriber count for topic '%s'. Requesting shutdown.",
        topic_name.c_str());
      if (ioctl(agnocast_fd, AGNOCAST_NOTIFY_BRIDGE_SHUTDOWN_CMD) < 0) {
        RCLCPP_ERROR(logger_, "Failed to notify bridge shutdown: %s", strerror(errno));
      }
      shutdown_requested_ = true;
      return;
    }

    if (result.count <= 0 || !is_demanded_by_ros2) {
      RCLCPP_INFO(
        logger_,
        "[bridge-dbg] R2A bridge REMOVED topic='%s' (agnocast_sub_count=%d "
        "has_external_ros2_publisher=%s)",
        topic_name.c_str(), result.count, is_demanded_by_ros2 ? "true" : "false");
      if (r2a_it->second.callback_group) {
        executor_->stop_callback_group(r2a_it->second.callback_group);
      }
      r2a_it = active_pubsub_r2a_bridges_.erase(r2a_it);
    } else {
      if (!update_ros2_publisher_num(container_node_.get(), topic_name)) {
        RCLCPP_ERROR(
          logger_, "Failed to update ROS 2 publisher count for topic '%s'.", topic_name.c_str());
      }
      ++r2a_it;
    }
  }

  auto a2r_it = active_pubsub_a2r_bridges_.begin();
  while (a2r_it != active_pubsub_a2r_bridges_.end()) {
    const std::string & topic_name = a2r_it->first;
    auto result = get_agnocast_publisher_count(topic_name);
    bool is_demanded_by_ros2 = has_external_ros2_subscriber(container_node_.get(), topic_name);
    if (result.count == -1) {
      RCLCPP_ERROR(
        logger_, "Failed to get publisher count for topic '%s'. Requesting shutdown.",
        topic_name.c_str());
      if (ioctl(agnocast_fd, AGNOCAST_NOTIFY_BRIDGE_SHUTDOWN_CMD) < 0) {
        RCLCPP_ERROR(logger_, "Failed to notify bridge shutdown: %s", strerror(errno));
      }
      shutdown_requested_ = true;
      return;
    }

    if (result.count <= 0 || !is_demanded_by_ros2) {
      RCLCPP_INFO(
        logger_,
        "[bridge-dbg] A2R bridge REMOVED topic='%s' (agnocast_pub_count=%d "
        "has_external_ros2_subscriber=%s)",
        topic_name.c_str(), result.count, is_demanded_by_ros2 ? "true" : "false");
      if (a2r_it->second.callback_group) {
        executor_->stop_callback_group(a2r_it->second.callback_group);
      }
      a2r_it = active_pubsub_a2r_bridges_.erase(a2r_it);
    } else {
      if (!update_ros2_subscriber_num(container_node_.get(), topic_name)) {
        RCLCPP_ERROR(
          logger_, "Failed to update ROS 2 subscriber count for topic '%s'.", topic_name.c_str());
      }
      ++a2r_it;
    }
  }
}

void PerformanceBridgeManager::check_and_remove_service_bridges()
{
  auto r2a_srv_it = active_r2a_service_bridges_.begin();
  while (r2a_srv_it != active_r2a_service_bridges_.end()) {
    const std::string & service_name = r2a_srv_it->first;

    std::string reason;
    if (is_agnocast_service_alive(service_name, reason)) {
      ++r2a_srv_it;
      continue;
    }

    RCLCPP_WARN(
      logger_, "Removing R2A service bridge for '%s': %s", service_name.c_str(), reason.c_str());

    if (r2a_srv_it->second.result.ros_srv_cb_group) {
      executor_->stop_callback_group(r2a_srv_it->second.result.ros_srv_cb_group);
    }
    if (r2a_srv_it->second.result.agno_client_cb_group) {
      executor_->stop_callback_group(r2a_srv_it->second.result.agno_client_cb_group);
    }
    r2a_srv_it = active_r2a_service_bridges_.erase(r2a_srv_it);
  }
}

void PerformanceBridgeManager::check_and_remove_request_cache()
{
  for (auto cache_it = request_cache_.begin(); cache_it != request_cache_.end();) {
    const auto & topic_name = cache_it->first;
    auto & requests = cache_it->second;

    remove_invalid_requests(topic_name, requests);

    if (requests.empty()) {
      cache_it = request_cache_.erase(cache_it);
    } else {
      ++cache_it;
    }
  }
}

void PerformanceBridgeManager::check_and_request_shutdown()
{
  struct ioctl_check_and_request_bridge_shutdown_args args = {};
  if (ioctl(agnocast_fd, AGNOCAST_CHECK_AND_REQUEST_BRIDGE_SHUTDOWN_CMD, &args) < 0) {
    RCLCPP_ERROR(logger_, "Failed to check bridge shutdown from kernel module.");
    return;
  }

  if (args.ret_should_shutdown) {
    shutdown_requested_ = true;
  }
}

namespace
{
// The gate is re-evaluated on every poll, so an unchanging verdict was being reprinted forever:
// 15,881 gate lines in one 2-minute run collapsed to 310 distinct messages (98% duplicates), and
// that alone pushed the log past the 8 MiB cap the CI runner truncates at. Print a gate verdict
// only when it differs from the last one for that topic+direction; a transition is the only thing
// worth seeing anyway.
void log_gate_if_changed(const rclcpp::Logger & logger, const std::string & key, std::string msg)
{
  static std::mutex mutex;
  static std::unordered_map<std::string, std::string> last_msg;
  {
    std::lock_guard<std::mutex> lock(mutex);
    auto & prev = last_msg[key];
    if (prev == msg) {
      return;
    }
    prev = msg;
  }
  RCLCPP_INFO(logger, "%s", msg.c_str());
}
}  // namespace

bool PerformanceBridgeManager::should_create_pubsub_bridge(
  const std::string & topic_name, BridgeDirection direction) const
{
  if (direction == BridgeDirection::ROS2_TO_AGNOCAST) {
    if (active_pubsub_r2a_bridges_.count(topic_name) > 0) {
      return false;
    }

    const auto stats = get_agnocast_subscriber_count(topic_name);
    if (stats.count <= 0) {
      if (bridge_debug_enabled()) {
        log_gate_if_changed(
          logger_, "R2A|" + topic_name,
          "[bridge-dbg] R2A gate topic='" + topic_name +
            "' agnocast_sub_count=" + std::to_string(stats.count) + " (<=0, skip)");
      }
      return false;
    }

    const bool has_ext_pub = has_external_ros2_publisher(container_node_.get(), topic_name);
    if (bridge_debug_enabled()) {
      log_gate_if_changed(
        logger_, "R2A|" + topic_name,
        "[bridge-dbg] R2A gate topic='" + topic_name +
          "' agnocast_sub_count=" + std::to_string(stats.count) +
          " has_external_ros2_publisher=" + (has_ext_pub ? "true" : "false"));
    }
    return has_ext_pub;
  }
  if (active_pubsub_a2r_bridges_.count(topic_name) > 0) {
    return false;
  }

  const auto stats = get_agnocast_publisher_count(topic_name);
  if (stats.count <= 0) {
    if (bridge_debug_enabled()) {
      log_gate_if_changed(
        logger_, "A2R|" + topic_name,
        "[bridge-dbg] A2R gate topic='" + topic_name +
          "' agnocast_pub_count=" + std::to_string(stats.count) + " (<=0, skip)");
    }
    return false;
  }

  const bool has_ext_sub = has_external_ros2_subscriber(container_node_.get(), topic_name);
  if (bridge_debug_enabled()) {
    log_gate_if_changed(
      logger_, "A2R|" + topic_name,
      "[bridge-dbg] A2R gate topic='" + topic_name +
        "' agnocast_pub_count=" + std::to_string(stats.count) +
        " has_external_ros2_subscriber=" + (has_ext_sub ? "true" : "false"));
  }
  return has_ext_sub;
}

void PerformanceBridgeManager::create_pubsub_bridge_if_needed(
  const std::string & topic_name, RequestMap & requests, const std::string & message_type,
  BridgeDirection direction)
{
  if (!should_create_pubsub_bridge(topic_name, direction)) {
    return;
  }

  topic_local_id_t qos_source_id = -1;
  for (const auto & [id, req] : requests) {
    if (req.direction == direction) {
      qos_source_id = id;
      break;
    }
  }
  if (qos_source_id == -1) {
    if (bridge_debug_enabled()) {
      RCLCPP_INFO(
        logger_,
        "[bridge-dbg] %s create for topic='%s' skipped: no matching request in cache",
        direction == BridgeDirection::ROS2_TO_AGNOCAST ? "R2A" : "A2R", topic_name.c_str());
    }
    return;
  }

  try {
    const bool is_r2a = (direction == BridgeDirection::ROS2_TO_AGNOCAST);

    PerformancePubsubBridgeResult result;
    if (is_r2a) {
      auto qos = get_subscriber_qos(topic_name, qos_source_id);
      RCLCPP_INFO(
        logger_,
        "[bridge-dbg] R2A create for topic='%s' using qos_source_id=%d: "
        "depth=%zu reliability=%s durability=%s",
        topic_name.c_str(), qos_source_id, qos.depth(),
        qos.reliability() == rclcpp::ReliabilityPolicy::Reliable ? "reliable" : "best_effort",
        qos.durability() == rclcpp::DurabilityPolicy::TransientLocal ? "transient_local"
                                                                      : "volatile");
      result = loader_.create_r2a_pubsub_bridge(container_node_, topic_name, message_type, qos);
    } else {
      auto qos = get_publisher_qos(topic_name, qos_source_id);
      RCLCPP_INFO(
        logger_,
        "[bridge-dbg] A2R create for topic='%s' using qos_source_id=%d: "
        "depth=%zu reliability=%s durability=%s",
        topic_name.c_str(), qos_source_id, qos.depth(),
        qos.reliability() == rclcpp::ReliabilityPolicy::Reliable ? "reliable" : "best_effort",
        qos.durability() == rclcpp::DurabilityPolicy::TransientLocal ? "transient_local"
                                                                      : "volatile");
      result = loader_.create_a2r_pubsub_bridge(container_node_, topic_name, message_type, qos);
    }

    if (result.entity_handle) {
      RCLCPP_INFO(
        logger_,
        "[bridge-dbg] %s pubsub bridge CREATED for topic='%s' (type='%s')",
        is_r2a ? "R2A" : "A2R", topic_name.c_str(), message_type.c_str());
      if (is_r2a) {
        if (!update_ros2_publisher_num(container_node_.get(), topic_name)) {
          RCLCPP_ERROR(
            logger_, "Failed to update ROS 2 publisher count for topic '%s'.", topic_name.c_str());
        }
        active_pubsub_r2a_bridges_[topic_name] = result;
      } else {
        if (!update_ros2_subscriber_num(container_node_.get(), topic_name)) {
          RCLCPP_ERROR(
            logger_, "Failed to update ROS 2 subscriber count for topic '%s'.", topic_name.c_str());
        }
        // The A2R bridge subscription is an agnocast subscription, so its callback group must be
        // hosted by an agnocast-capable child executor. The plugin creates the group with
        // automatically_add_to_executor_with_node()==false and registers the agnocast subscription
        // before returning, so adding the group here (after the subscription exists) guarantees the
        // executor classifies it correctly instead of racing the monitoring poll.
        if (result.callback_group) {
          executor_->add_callback_group(result.callback_group, container_node_->get_node_base_interface());
        }
        active_pubsub_a2r_bridges_[topic_name] = result;
      }
    } else {
      RCLCPP_WARN(
        logger_,
        "[bridge-dbg] %s bridge factory returned null handle for topic='%s' (type='%s'); "
        "bridge NOT created",
        is_r2a ? "R2A" : "A2R", topic_name.c_str(), message_type.c_str());
    }

  } catch (const std::exception & e) {
    RCLCPP_WARN(
      logger_, "Failed to create bridge for '%s': %s. Removing invalid request ID %d.",
      topic_name.c_str(), e.what(), qos_source_id);
    requests.erase(qos_source_id);
  } catch (...) {
    RCLCPP_WARN(
      logger_, "Unknown error creating bridge for '%s'. Removing invalid request ID %d.",
      topic_name.c_str(), qos_source_id);
    requests.erase(qos_source_id);
  }
}

void PerformanceBridgeManager::create_service_bridge_if_needed(
  const ServiceBridgeTargetInfoWithType & target, BridgeDirection direction)
{
  std::string service_name = static_cast<const char *>(target.service_name);
  std::string service_type = static_cast<const char *>(target.service_type);
  std::string shadow_node_namespace = static_cast<const char *>(target.shadow_node_namespace);
  std::string shadow_node_name = static_cast<const char *>(target.shadow_node_name);

  if (direction == BridgeDirection::AGNOCAST_TO_ROS2) {
    // A2R service bridge is not implemented yet.
    return;
  }

  try {
    // Check that the target bridge does not already exist.
    if (active_r2a_service_bridges_.count(service_name) > 0) {
      return;
    }

    // Check that the target service does not already exist in ROS 2.
    const auto services = container_node_->get_service_names_and_types();
    bool exists = std::any_of(services.begin(), services.end(), [&service_name](const auto & s) {
      return s.first == service_name;
    });
    if (exists) {
      RCLCPP_WARN(
        logger_,
        "Found a ROS 2 service with the same name while creating the R2A service bridge: '%s'",
        service_name.c_str());
    }

    auto service_qos = get_service_qos(service_name);

    std::shared_ptr<rcl_node_t> shadow_node;
    if (target.create_shadow_node && !shadow_node_name.empty()) {
      shadow_node = find_or_create_shadow_node(
        active_r2a_service_bridges_, shadow_node_namespace, shadow_node_name);
    }

    PerformanceServiceBridgeResult result =
      loader_.create_r2a_service_bridge(container_node_, service_name, service_type, service_qos);
    if (result.entity_handle) {
      active_r2a_service_bridges_.emplace(
        service_name, R2AServiceBridgeItem(std::move(result), std::move(shadow_node)));
    }
  } catch (const std::exception & e) {
    RCLCPP_WARN(
      logger_, "Failed to create service bridge for '%s': %s", service_name.c_str(), e.what());
  } catch (...) {
    RCLCPP_WARN(logger_, "Unknown error creating service bridge for '%s'", service_name.c_str());
  }
}

void PerformanceBridgeManager::remove_invalid_requests(
  const std::string & topic_name, RequestMap & request_map)
{
  for (auto req_it = request_map.begin(); req_it != request_map.end();) {
    const auto target_id = req_it->first;
    const auto & msg = req_it->second;

    // Verify liveness by attempting to retrieve QoS.
    // If the target no longer exists, an exception is thrown.
    try {
      if (msg.direction == BridgeDirection::ROS2_TO_AGNOCAST) {
        get_subscriber_qos(topic_name, target_id);
      } else {
        get_publisher_qos(topic_name, target_id);
      }
      ++req_it;
    } catch (...) {
      req_it = request_map.erase(req_it);
    }
  }
}

}  // namespace agnocast
