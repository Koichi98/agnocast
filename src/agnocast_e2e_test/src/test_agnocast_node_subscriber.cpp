#include "agnocast/agnocast.hpp"

#include "std_msgs/msg/int64.hpp"

#include <functional>
#include <iostream>
#include <memory>
#include <string>

#include <unistd.h>

// Standalone subscriber built on agnocast::Node (the pure-Agnocast node, not
// rclcpp::Node). It logs every received message and shuts the process down once
// it has received the terminating message id the expected number of times (once
// per upstream publisher).
//
// In "forever" mode (used by the stress test) it keeps receiving and never
// shuts itself down, so it can soak under load.
class TestAgnocastNodeSubscriber : public agnocast::Node
{
  agnocast::Subscription<std_msgs::msg::Int64>::SharedPtr subscription_;
  int64_t target_end_id_;
  int target_end_count_;
  bool forever_;
  int received_end_count_ = 0;

  void callback(const agnocast::ipc_shared_ptr<std_msgs::msg::Int64> & message)
  {
    RCLCPP_INFO(get_logger(), "Receiving %ld.", message->data);

    if (message->data == target_end_id_) {
      received_end_count_++;

      if (received_end_count_ >= target_end_count_) {
        RCLCPP_INFO(get_logger(), "All messages received. Shutting down.");
        std::cout << std::flush;

        if (!forever_) {
          sleep(3);  // HACK: wait for other nodes before shutting down
          agnocast::shutdown();
        }
      }
    }
  }

public:
  TestAgnocastNodeSubscriber(
    const std::string & node_name, const std::string & topic_name, int64_t target_end_id,
    int target_end_count, bool forever)
  : Node(node_name),
    target_end_id_(target_end_id),
    target_end_count_(target_end_count),
    forever_(forever)
  {
    subscription_ = create_subscription<std_msgs::msg::Int64>(
      topic_name, rclcpp::QoS(rclcpp::KeepLast(10)),
      std::bind(&TestAgnocastNodeSubscriber::callback, this, std::placeholders::_1));
  }
};

int main(int argc, char ** argv)
{
  agnocast::init(argc, argv);

  std::string node_name = "test_agnocast_node_subscriber";
  std::string topic_name = "/test_topic";
  int64_t target_end_id = 4;
  int target_end_count = 1;
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
    } else if (arg == "--target-end-id" && i + 1 < argc) {
      target_end_id = std::stoll(argv[++i]);
    } else if (arg == "--target-end-count" && i + 1 < argc) {
      target_end_count = std::stoi(argv[++i]);
    } else if (arg == "--forever") {
      forever = true;
    }
  }

  agnocast::AgnocastOnlySingleThreadedExecutor executor;
  auto node = std::make_shared<TestAgnocastNodeSubscriber>(
    node_name, topic_name, target_end_id, target_end_count, forever);
  executor.add_node(node);
  executor.spin();

  return 0;
}
