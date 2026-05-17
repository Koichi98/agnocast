#include "agnocast/agnocast.hpp"

#include "std_msgs/msg/int64.hpp"

#include <chrono>
#include <functional>
#include <iostream>
#include <memory>
#include <string>

#include <unistd.h>

using namespace std::chrono_literals;

// Standalone publisher built on agnocast::Node (the pure-Agnocast node, not
// rclcpp::Node). It is driven by an Agnocast wall timer: once the expected
// number of subscribers have connected it publishes a fixed number of messages,
// one per timer tick, then shuts the process down.
//
// In "forever" mode (used by the stress test) it keeps publishing past the
// target count and never shuts itself down, so it can soak under load.
class TestAgnocastNodePublisher : public agnocast::Node
{
  enum class State {
    WaitingForConnection,  // Waiting for subscriber connections
    Publishing             // Normal publishing
  };

  agnocast::Publisher<std_msgs::msg::Int64>::SharedPtr publisher_;
  agnocast::TimerBase::SharedPtr timer_;
  std::string topic_name_;
  int64_t pub_num_;
  size_t planned_sub_count_;
  bool forever_;
  int64_t count_ = 0;
  State state_ = State::WaitingForConnection;

  void timer_callback()
  {
    switch (state_) {
      case State::WaitingForConnection: {
        const size_t total_sub_count =
          publisher_->get_subscription_count() + publisher_->get_intra_subscription_count();
        if (total_sub_count >= planned_sub_count_) {
          state_ = State::Publishing;
        }
        break;
      }

      case State::Publishing:
        publish_message();
        break;
    }
  }

  void publish_message()
  {
    agnocast::ipc_shared_ptr<std_msgs::msg::Int64> message = publisher_->borrow_loaned_message();
    message->data = count_;
    publisher_->publish(std::move(message));
    RCLCPP_INFO(get_logger(), "Publishing %ld.", count_);
    count_++;

    if (count_ == pub_num_) {
      RCLCPP_INFO(get_logger(), "All messages published. Shutting down.");
      std::cout << std::flush;

      if (!forever_) {
        sleep(3);  // HACK: give subscribers time to receive before shutting down
        agnocast::shutdown();
      }
    }
  }

public:
  TestAgnocastNodePublisher(
    const std::string & node_name, const std::string & topic_name, int64_t pub_num,
    size_t planned_sub_count, bool forever)
  : Node(node_name),
    topic_name_(topic_name),
    pub_num_(pub_num),
    planned_sub_count_(planned_sub_count),
    forever_(forever)
  {
    publisher_ =
      create_publisher<std_msgs::msg::Int64>(topic_name_, rclcpp::QoS(rclcpp::KeepLast(10)));
    timer_ = create_wall_timer(100ms, std::bind(&TestAgnocastNodePublisher::timer_callback, this));
  }
};

int main(int argc, char ** argv)
{
  agnocast::init(argc, argv);

  std::string node_name = "test_agnocast_node_publisher";
  std::string topic_name = "/test_topic";
  int64_t pub_num = 5;
  size_t planned_sub_count = 1;
  bool forever = false;

  // Parse plain command-line arguments. ROS arguments (after "--ros-args") are
  // left to agnocast::init(), so stop scanning once they begin.
  for (int i = 1; i < argc; i++) {
    const std::string arg = argv[i];
    if (arg == "--ros-args") {
      break;
    }
    if (arg == "--node-name" && i + 1 < argc) {
      node_name = argv[++i];
    } else if (arg == "--topic" && i + 1 < argc) {
      topic_name = argv[++i];
    } else if (arg == "--pub-num" && i + 1 < argc) {
      pub_num = std::stoll(argv[++i]);
    } else if (arg == "--planned-sub-count" && i + 1 < argc) {
      planned_sub_count = static_cast<size_t>(std::stoull(argv[++i]));
    } else if (arg == "--forever") {
      forever = true;
    }
  }

  agnocast::AgnocastOnlySingleThreadedExecutor executor;
  auto node = std::make_shared<TestAgnocastNodePublisher>(
    node_name, topic_name, pub_num, planned_sub_count, forever);
  executor.add_node(node);
  executor.spin();

  return 0;
}
