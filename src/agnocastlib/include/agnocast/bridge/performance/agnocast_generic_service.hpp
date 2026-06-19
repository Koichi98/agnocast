#pragma once

// Vendored, trimmed port of rclcpp::GenericService (introduced upstream in Kilted/Rolling, PR
// ros2/rclcpp#2617). ROS 2 Humble's rclcpp has no type-erased service server, which the generic
// (plugin-free) R2A service bridge needs. This port targets Humble's rclcpp::ServiceBase and is
// reduced to the single deferred-response callback the bridge uses: the request is handed to the
// callback and the response is sent later (asynchronously) via send_response(), because the
// response travels back through Agnocast out-of-line.
//
// When building on Kilted/Rolling, prefer the upstream rclcpp::GenericService instead of this copy.

#include "rcl/service.h"
#include "rclcpp/exceptions.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/qos.hpp"
#include "rclcpp/service.hpp"
#include "rclcpp/visibility_control.hpp"
#include "rcpputils/shared_library.hpp"
#include "rmw/types.h"
#include "rosidl_runtime_c/service_type_support_struct.h"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"

#include <functional>
#include <memory>
#include <string>

namespace agnocast::bridge
{

class GenericService : public rclcpp::ServiceBase,
                       public std::enable_shared_from_this<GenericService>
{
public:
  using SharedRequest = std::shared_ptr<void>;
  using SharedResponse = std::shared_ptr<void>;

  // Deferred-response callback: invoked with (this service, request id, deserialized request).
  // The handler keeps the request id and calls send_response() once the response is ready.
  using DeferredCallback = std::function<void(
    std::shared_ptr<GenericService>, std::shared_ptr<rmw_request_id_t>, SharedRequest)>;

  RCLCPP_SMART_PTR_DEFINITIONS(GenericService)

  GenericService(
    std::shared_ptr<rcl_node_t> node_handle, const std::string & service_name,
    const std::string & service_type, DeferredCallback callback,
    rcl_service_options_t & service_options);

  GenericService() = delete;
  ~GenericService() override = default;

  // --- rclcpp::ServiceBase overrides (driven by the executor) ---
  std::shared_ptr<void> create_request() override;
  std::shared_ptr<rmw_request_id_t> create_request_header() override;
  void handle_request(
    std::shared_ptr<rmw_request_id_t> request_header, std::shared_ptr<void> request) override;

  // Allocate a (zero-initialized) response message of the service's response type.
  std::shared_ptr<void> create_response();

  // Send a response for a previously-received request id.
  void send_response(rmw_request_id_t & req_id, SharedResponse & response);

  // The request/response message typesupports (rosidl_typesupport_cpp), for the bridge to
  // rmw_serialize the request and rmw_deserialize the response.
  const rosidl_message_type_support_t * request_message_typesupport() const { return request_ts_; }
  const rosidl_message_type_support_t * response_message_typesupport() const
  {
    return response_ts_;
  }

private:
  RCLCPP_DISABLE_COPY(GenericService)

  DeferredCallback callback_;
  std::shared_ptr<rcpputils::SharedLibrary> ts_lib_;
  const rosidl_typesupport_introspection_cpp::MessageMembers * request_members_;
  const rosidl_typesupport_introspection_cpp::MessageMembers * response_members_;
  const rosidl_message_type_support_t * request_ts_;
  const rosidl_message_type_support_t * response_ts_;
};

}  // namespace agnocast::bridge
