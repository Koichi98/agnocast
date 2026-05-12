// Helper publisher used to exercise delay_agnocast end-to-end.
// Publishes sensor_msgs/msg/Imu at 10 Hz with header.stamp = current wall clock.

#include "agnocast/agnocast.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"

#include <chrono>
#include <memory>

using namespace std::chrono_literals;

class ImuTestPublisher : public rclcpp::Node
{
public:
  ImuTestPublisher() : Node("agnocast_stats_imu_test_publisher")
  {
    publisher_ = agnocast::create_publisher<sensor_msgs::msg::Imu>(this, "/agnocast_stats/imu", 1);
    timer_ = this->create_wall_timer(100ms, [this]() {
      auto msg = publisher_->borrow_loaned_message();
      const auto now = this->get_clock()->now();
      msg->header.stamp = now;
      msg->header.frame_id = "agnocast_stats";
      // Other fields left default-initialised.
      publisher_->publish(std::move(msg));
    });
  }

private:
  agnocast::Publisher<sensor_msgs::msg::Imu>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  agnocast::SingleThreadedAgnocastExecutor executor;
  auto node = std::make_shared<ImuTestPublisher>();
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
