#include "agnocast/agnocast.hpp"
#include "agnocast_sample_interfaces/msg/dynamic_size_array.hpp"

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <vector>

using namespace std::chrono_literals;
const long long MESSAGE_SIZE = 1000ll * 1024;

class AgnocastPubSubTest : public agnocast::Node
{
  using MessageT = agnocast_sample_interfaces::msg::DynamicSizeArray;

  int64_t count_;
  agnocast::TimerBase::SharedPtr timer_;
  std::vector<agnocast::Publisher<MessageT>::SharedPtr> publishers_;
  std::vector<agnocast::Subscription<MessageT>::SharedPtr> subscriptions_;

  void timer_callback()
  {
    for (size_t topic_index = 0; topic_index < publishers_.size(); ++topic_index) {
      auto message = publishers_[topic_index]->borrow_loaned_message();
      message->id = count_;
      message->data.reserve(MESSAGE_SIZE / sizeof(uint64_t));
      for (size_t i = 0; i < MESSAGE_SIZE / sizeof(uint64_t); i++) {
        message->data.push_back(i + count_);
      }

      publishers_[topic_index]->publish(std::move(message));
    }

    // RCLCPP_INFO(get_logger(), "publish message: id=%ld to %zu topics", count_++,
    // publishers_.size());
  }

  void subscription_callback(
    const agnocast::ipc_shared_ptr<MessageT> & message, const size_t topic_index)
  {
    // RCLCPP_INFO(get_logger(), "subscribe topic_index=%zu message: id=%ld", topic_index,
    // message->id);
  }

public:
  AgnocastPubSubTest() : Node("agnocast_pubsub_test")
  {
    count_ = 0;

    const int topic_count_param = declare_parameter<int>("publish_count", 1);
    const int topic_count = std::max(topic_count_param, 1);
    const std::string pub_topic_prefix =
      declare_parameter<std::string>("pub_topic_prefix", "/my_pub_topic");
    const std::string sub_topic_prefix =
      declare_parameter<std::string>("sub_topic_prefix", "/my_sub_topic");

    publishers_.reserve(topic_count);
    subscriptions_.reserve(topic_count);

    for (int i = 0; i < topic_count; ++i) {
      const std::string pub_topic_name = pub_topic_prefix + "_" + std::to_string(i);
      auto publisher = create_publisher<MessageT>(pub_topic_name, 1);

      publishers_.push_back(publisher);
      RCLCPP_INFO(get_logger(), "created agnocast publisher for topic: %s", pub_topic_name.c_str());
    }

    for (int i = 0; i < topic_count; ++i) {
      const std::string sub_topic_name = sub_topic_prefix + "_" + std::to_string(i);
      auto subscription = create_subscription<MessageT>(
        sub_topic_name, 1, [this, i](const agnocast::ipc_shared_ptr<MessageT> & message) {
          this->subscription_callback(message, static_cast<size_t>(i));
        });

      subscriptions_.push_back(subscription);
      RCLCPP_INFO(
        get_logger(), "created agnocast subscriber for topic: %s", sub_topic_name.c_str());
    }

    timer_ = create_wall_timer(100ms, std::bind(&AgnocastPubSubTest::timer_callback, this));
  }
};

int main(int argc, char * argv[])
{
  agnocast::init(argc, argv);

  agnocast::AgnocastOnlySingleThreadedExecutor executor;
  auto node = std::make_shared<AgnocastPubSubTest>();
  executor.add_node(node);
  executor.spin();

  return 0;
}
