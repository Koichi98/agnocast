// Publishes ComplexMessage at 10 Hz on /complex_topic.
// Demonstrates a moderately complex message structure (Header + nested-message sequence +
// fixed-size array + string + string array + bool) that the type-erased generic_listener
// can render via introspection.

#include "agnocast/agnocast.hpp"
#include "agnocast_sample_interfaces/msg/complex_message.hpp"
#include "rclcpp/rclcpp.hpp"

#include <chrono>
#include <memory>
#include <string>

using namespace std::chrono_literals;

class ComplexPublisher : public rclcpp::Node
{
  int64_t count_;
  rclcpp::TimerBase::SharedPtr timer_;
  agnocast::Publisher<agnocast_sample_interfaces::msg::ComplexMessage>::SharedPtr publisher_;

  void timer_callback()
  {
    auto msg = publisher_->borrow_loaned_message();

    msg->header.stamp = this->get_clock()->now();
    msg->header.frame_id = "complex_publisher";

    msg->id = count_;
    msg->label = "tick-" + std::to_string(count_);
    msg->active = (count_ % 2 == 0);

    // Nested-message sequence: a couple of readings whose values track count_.
    msg->readings.clear();
    for (int i = 0; i < 3; ++i) {
      agnocast_sample_interfaces::msg::NamedValue nv;
      nv.name = "sensor_" + std::to_string(i);
      nv.value = static_cast<double>(count_) + 0.1 * i;
      msg->readings.push_back(nv);
    }

    // Fixed-size 3x3-like array (9 elements).
    for (size_t i = 0; i < msg->covariance.size(); ++i) {
      msg->covariance[i] = static_cast<double>(count_) * 0.01 + static_cast<double>(i);
    }

    // String sequence.
    msg->tags.clear();
    msg->tags.push_back("alpha");
    msg->tags.push_back("beta");
    msg->tags.push_back("gamma");

    publisher_->publish(std::move(msg));
    RCLCPP_INFO(this->get_logger(), "publish message: id=%ld", count_++);
  }

public:
  ComplexPublisher() : Node("complex_publisher"), count_(0)
  {
    publisher_ = agnocast::create_publisher<agnocast_sample_interfaces::msg::ComplexMessage>(
      this, "/complex_topic", 1);
    timer_ = this->create_wall_timer(100ms, std::bind(&ComplexPublisher::timer_callback, this));
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  agnocast::SingleThreadedAgnocastExecutor executor;
  auto node = std::make_shared<ComplexPublisher>();
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
