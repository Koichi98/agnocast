#include "agnocast/bridge/performance/agnocast_generic_service.hpp"

#include "rcl/service.h"
#include "rclcpp/exceptions.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/typesupport_helpers.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"
#include "rosidl_typesupport_introspection_cpp/identifier.hpp"

#include <cstdint>
#include <stdexcept>
#include <tuple>

namespace agnocast::bridge
{

namespace
{
rclcpp::Logger logger()
{
  return rclcpp::get_logger("agnocast_generic_service");
}

// Parse "package/srv/Type" -> (package, "Type"). Mirrors rclcpp's type-string parsing.
std::pair<std::string, std::string> split_service_type(const std::string & service_type)
{
  const auto front = service_type.find_first_of('/');
  const auto back = service_type.find_last_of('/');
  if (
    front == std::string::npos || back == std::string::npos || front == 0 ||
    back == service_type.length() - 1) {
    throw std::runtime_error("Service type is not of the form package/srv/Type: " + service_type);
  }
  return {service_type.substr(0, front), service_type.substr(back + 1)};
}

// Humble's rclcpp lacks get_service_typesupport_handle(); resolve the generated symbol directly
// from the already-loaded rosidl_typesupport_cpp library (same scheme upstream uses).
const rosidl_service_type_support_t * resolve_service_typesupport(
  const std::string & package_name, const std::string & type_name,
  rcpputils::SharedLibrary & library)
{
  const std::string symbol_name = "rosidl_typesupport_cpp__get_service_type_support_handle__" +
                                  package_name + "__srv__" + type_name;
  using GetTsFn = const rosidl_service_type_support_t * (*)();
  auto get_ts = reinterpret_cast<GetTsFn>(library.get_symbol(symbol_name));
  return get_ts();
}

// Humble's rosidl_service_type_support_t has no request_typesupport/response_typesupport members
// (added later upstream). Resolve the request/response *message* typesupports directly by symbol.
// The service's request/response messages are "<pkg>/srv/<Type>_Request" / "<Type>_Response", whose
// typesupport lives in the same library.
const rosidl_message_type_support_t * resolve_srv_message_typesupport(
  const std::string & package_name, const std::string & message_type_name,
  rcpputils::SharedLibrary & library)
{
  const std::string symbol_name = "rosidl_typesupport_cpp__get_message_type_support_handle__" +
                                  package_name + "__srv__" + message_type_name;
  using GetTsFn = const rosidl_message_type_support_t * (*)();
  auto get_ts = reinterpret_cast<GetTsFn>(library.get_symbol(symbol_name));
  return get_ts();
}

const rosidl_typesupport_introspection_cpp::MessageMembers * introspection_members(
  const rosidl_message_type_support_t * message_ts)
{
  const rosidl_message_type_support_t * intro =
    message_ts->func(message_ts, rosidl_typesupport_introspection_cpp::typesupport_identifier);
  return static_cast<const rosidl_typesupport_introspection_cpp::MessageMembers *>(intro->data);
}
}  // namespace

GenericService::GenericService(
  std::shared_ptr<rcl_node_t> node_handle, const std::string & service_name,
  const std::string & service_type, DeferredCallback callback,
  rcl_service_options_t & service_options)
: ServiceBase(node_handle), callback_(std::move(callback))
{
  const rosidl_service_type_support_t * service_ts = nullptr;
  try {
    const auto [package_name, type_name] = split_service_type(service_type);
    ts_lib_ = rclcpp::get_typesupport_library(service_type, "rosidl_typesupport_cpp");
    service_ts = resolve_service_typesupport(package_name, type_name, *ts_lib_);

    request_ts_ = resolve_srv_message_typesupport(package_name, type_name + "_Request", *ts_lib_);
    response_ts_ = resolve_srv_message_typesupport(package_name, type_name + "_Response", *ts_lib_);
    request_members_ = introspection_members(request_ts_);
    response_members_ = introspection_members(response_ts_);
  } catch (const std::exception & err) {
    RCLCPP_ERROR(logger(), "Invalid service type '%s': %s", service_type.c_str(), err.what());
    throw;
  }

  // rcl owns the service handle; fini it on destruction.
  service_handle_ = std::shared_ptr<rcl_service_t>(
    new rcl_service_t, [handle = node_handle_, service_name](rcl_service_t * service) {
      if (rcl_service_fini(service, handle.get()) != RCL_RET_OK) {
        RCLCPP_ERROR(
          logger(), "Error destroying rcl service handle: %s", rcl_get_error_string().str);
        rcl_reset_error();
      }
      delete service;
    });
  *service_handle_.get() = rcl_get_zero_initialized_service();

  const rcl_ret_t ret = rcl_service_init(
    service_handle_.get(), node_handle.get(), service_ts, service_name.c_str(), &service_options);
  if (ret != RCL_RET_OK) {
    rclcpp::exceptions::throw_from_rcl_error(ret, "could not create generic service");
  }
}

std::shared_ptr<void> GenericService::create_request()
{
  void * request = new uint8_t[request_members_->size_of_];
  request_members_->init_function(request, rosidl_runtime_cpp::MessageInitialization::ZERO);
  return std::shared_ptr<void>(request, [this](void * p) {
    request_members_->fini_function(p);
    delete[] reinterpret_cast<uint8_t *>(p);
  });
}

std::shared_ptr<void> GenericService::create_response()
{
  void * response = new uint8_t[response_members_->size_of_];
  response_members_->init_function(response, rosidl_runtime_cpp::MessageInitialization::ZERO);
  return std::shared_ptr<void>(response, [this](void * p) {
    response_members_->fini_function(p);
    delete[] reinterpret_cast<uint8_t *>(p);
  });
}

std::shared_ptr<rmw_request_id_t> GenericService::create_request_header()
{
  return std::make_shared<rmw_request_id_t>();
}

void GenericService::handle_request(
  std::shared_ptr<rmw_request_id_t> request_header, std::shared_ptr<void> request)
{
  // Deferred: hand off the request; the response is sent later via send_response().
  callback_(this->shared_from_this(), std::move(request_header), std::move(request));
}

void GenericService::send_response(rmw_request_id_t & req_id, SharedResponse & response)
{
  const rcl_ret_t ret = rcl_send_response(get_service_handle().get(), &req_id, response.get());
  if (ret == RCL_RET_TIMEOUT) {
    RCLCPP_WARN(logger(), "send_response timed out");
    return;
  }
  if (ret != RCL_RET_OK) {
    rclcpp::exceptions::throw_from_rcl_error(ret, "failed to send response");
  }
}

}  // namespace agnocast::bridge
