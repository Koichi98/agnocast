#include "agnocast/agnocast.hpp"
#include "agnocast_sample_interfaces/srv/sum_int_array.hpp"
#include "rclcpp/rclcpp.hpp"

class MinimalGenericService : public rclcpp::Node
{
  using Request = agnocast_sample_interfaces::srv::SumIntArray::Request;
  using Response = agnocast_sample_interfaces::srv::SumIntArray::Response;

  rclcpp::CallbackGroup::SharedPtr callback_group_;
  agnocast::GenericService::SharedPtr service_;

public:
  explicit MinimalGenericService() : Node("minimal_generic_server")
  {
    callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    service_ = std::make_shared<agnocast::GenericService>(
      this, "sum_int_array", "agnocast_sample_interfaces/srv/SumIntArray",
      [this](
        agnocast::ipc_shared_ptr<void> && request, agnocast::ipc_shared_ptr<void> && response) {
        auto * request_ptr = static_cast<Request *>(request.get());
        auto * response_ptr = static_cast<Response *>(response.get());
        response_ptr->sum = 0;
        for (int64_t value : request_ptr->data) {
          response_ptr->sum += value;
        }
        RCLCPP_INFO(this->get_logger(), "Sending back response: [%ld]", response_ptr->sum);
      },
      rclcpp::ServicesQoS(), callback_group_);
  }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  agnocast::CallbackIsolatedAgnocastExecutor executor;
  auto node = std::make_shared<MinimalGenericService>();
  executor.add_node(node);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}
