#include "agnocast/agnocast.hpp"
#include "agnocast/vendor/rclcpp/generic_client.hpp"
#include "agnocast_sample_interfaces/srv/sum_int_array.hpp"
#include "rclcpp/rclcpp.hpp"

#include <thread>

using namespace std::chrono_literals;

constexpr size_t ARRAY_SIZE = 100;

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto node = rclcpp::Node::make_shared("minimal_generic_client");
  std::thread spin_thread([node]() mutable {
    agnocast::CallbackIsolatedAgnocastExecutor executor;
    executor.add_node(node);
    executor.spin();
  });

  auto callback_group = node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  auto client = agnocast::vendor_rclcpp::GenericClient::create_generic_client(
    node.get(), "sum_int_array", "agnocast_sample_interfaces/srv/SumIntArray",
    rclcpp::ServicesQoS(), callback_group);

  while (!client->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(node->get_logger(), "Interrupted while waiting for the service. Exiting.");
      spin_thread.join();
      rclcpp::shutdown();
      return 0;
    }
    RCLCPP_INFO(node->get_logger(), "Service not available, waiting again...");
  }

  using Request = agnocast_sample_interfaces::srv::SumIntArray::Request;
  using Response = agnocast_sample_interfaces::srv::SumIntArray::Response;

  // Send first request with a callback
  {
    Request request1;
    for (size_t i = 1; i <= ARRAY_SIZE; ++i) {
      request1.data.push_back(i);
    }
    const void * request1_ptr = &request1;
    client->async_send_request(
      request1_ptr,
      [node = node.get()](agnocast::vendor_rclcpp::GenericClient::SharedFuture future) {
        auto response_ptr = future.get();
        auto * response = static_cast<Response *>(response_ptr.get());
        RCLCPP_INFO(node->get_logger(), "Result1: %ld", response->sum);
      });
  }

  // Send second request and wait synchronously
  {
    Request request2;
    for (size_t i = 0; i < ARRAY_SIZE; ++i) {
      request2.data.push_back(i);
    }
    const void * request2_ptr = &request2;
    auto future_and_id = client->async_send_request(request2_ptr, [](auto) {});
    auto response2_ptr = future_and_id.get();
    auto * response2 = static_cast<Response *>(response2_ptr.get());
    RCLCPP_INFO(node->get_logger(), "Result2: %ld", response2->sum);
  }

  spin_thread.join();
  rclcpp::shutdown();
  return 0;
}
