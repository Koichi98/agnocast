#include "agnocast/vendor/rclcpp/generic_service.hpp"
#include "agnocast_sample_interfaces/srv/sum_int_array.hpp"
#include "rclcpp/rclcpp.hpp"

class MinimalGenericService : public rclcpp::Node
{
  using Request = agnocast_sample_interfaces::srv::SumIntArray::Request;
  using Response = agnocast_sample_interfaces::srv::SumIntArray::Response;

  rclcpp::CallbackGroup::SharedPtr callback_group_;
  std::shared_ptr<agnocast::vendor_rclcpp::GenericService> service_;

public:
  explicit MinimalGenericService() : Node("minimal_generic_server")
  {
    callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    service_ = agnocast::vendor_rclcpp::GenericService::create_generic_service(
      this, "sum_int_array", "agnocast_sample_interfaces/srv/SumIntArray",
      [this](std::shared_ptr<void> request, std::shared_ptr<void> response) {
        auto req = std::static_pointer_cast<Request>(request);
        auto res = std::static_pointer_cast<Response>(response);
        res->sum = 0;
        for (int64_t value : req->data) {
          res->sum += value;
        }
        RCLCPP_INFO(this->get_logger(), "Sending back response: [%ld]", res->sum);
      },
      rclcpp::ServicesQoS(), callback_group_);
  }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  rclcpp::executors::SingleThreadedExecutor executor;
  auto node = std::make_shared<MinimalGenericService>();
  executor.add_node(node);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}
