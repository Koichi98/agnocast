// Agnocast tracepoint verification node (rclcpp::Node version)
//
// rclcpp::Node ベースで agnocast トレースポイントを発火させる検証専用ノード。
// agnocast::Node 版 (tracepoint_verifier.cpp) との比較用。
//
// rclcpp::Node 版で発火するトレースポイント:
//   - agnocast_publisher_init
//   - agnocast_subscription_init
//   - agnocast_construct_executor
//   - agnocast_publish
//   - agnocast_create_callable
//   - agnocast_callable_start / agnocast_callable_end
//   - agnocast_take
//
// rclcpp::Node 版では発火しないトレースポイント:
//   - agnocast_init           (rclcpp::init を使用するため)
//   - agnocast_node_init      (agnocast::Node 固有)
//   - agnocast_timer_init     (agnocast::Node の create_wall_timer 固有)
//   - agnocast_add_callback_group (AgnocastOnlyExecutor 固有)
//   - agnocast_create_timer_callable (agnocast timer 固有)
//
// 使い方:
//   $ ros2 launch agnocast_sample_application tracepoint_verifier_rclcpp.launch.xml
//   (約3秒で自動終了)
//   $ babeltrace ./trace_dir/ | awk -F: '{print $4}' | sort | uniq -c | sort -rn

#include "agnocast/agnocast.hpp"
#include "agnocast_sample_interfaces/msg/dynamic_size_array.hpp"

#include <atomic>
#include <chrono>

using namespace std::chrono_literals;
using std::placeholders::_1;

class TracepointVerifierRclcpp : public rclcpp::Node
{
  // --- Publisher ---
  // agnocast_publisher_init
  agnocast::Publisher<agnocast_sample_interfaces::msg::DynamicSizeArray>::SharedPtr pub_;

  // --- Callback subscription ---
  // agnocast_subscription_init (callback型)
  // 実行時: agnocast_create_callable, agnocast_callable_start/end
  agnocast::Subscription<agnocast_sample_interfaces::msg::DynamicSizeArray>::SharedPtr sub_cb_;

  // --- Polling subscription ---
  // agnocast_subscription_init (polling型)
  // 実行時: agnocast_take
  agnocast::PollingSubscriber<agnocast_sample_interfaces::msg::DynamicSizeArray>::SharedPtr
    sub_poll_;

  // --- Timer (rclcpp standard) ---
  // rclcpp の標準タイマー (agnocast_timer_init は発火しない)
  rclcpp::TimerBase::SharedPtr timer_;

  std::atomic<int> publish_count_{0};
  std::atomic<int> cb_recv_count_{0};
  std::atomic<int> poll_recv_count_{0};

  static constexpr int MAX_PUBLISH = 20;

  void timer_callback()
  {
    int count = publish_count_.load();
    if (count >= MAX_PUBLISH) {
      return;
    }

    // agnocast_publish
    auto message = pub_->borrow_loaned_message();
    message->id = count;
    message->data.push_back(static_cast<uint64_t>(count));
    pub_->publish(std::move(message));

    RCLCPP_INFO(get_logger(), "[timer] published id=%d", count);
    publish_count_++;

    // agnocast_take
    auto polled = sub_poll_->take_data();
    if (polled) {
      poll_recv_count_++;
      RCLCPP_INFO(get_logger(), "[poll]  take_data id=%ld", polled->id);
    }
  }

  // agnocast_create_callable, agnocast_callable_start/end
  void subscription_callback(
    const agnocast::ipc_shared_ptr<agnocast_sample_interfaces::msg::DynamicSizeArray> & message)
  {
    cb_recv_count_++;
    RCLCPP_INFO(get_logger(), "[cb]    received id=%ld", message->id);
  }

public:
  explicit TracepointVerifierRclcpp() : rclcpp::Node("tracepoint_verifier_rclcpp", "/verify_ns")
  {
    RCLCPP_INFO(get_logger(), "=== Agnocast Tracepoint Verifier (rclcpp::Node) ===");
    RCLCPP_INFO(get_logger(), "node_name:  %s", get_name());
    RCLCPP_INFO(get_logger(), "namespace:  %s", get_namespace());
    RCLCPP_INFO(get_logger(), "fqn:        %s", get_fully_qualified_name());

    // agnocast_publisher_init
    pub_ = agnocast::create_publisher<agnocast_sample_interfaces::msg::DynamicSizeArray>(
      this, "/verify_topic", 1);

    // agnocast_subscription_init (callback型)
    // minimal_subscriber と同様に callback_group を明示指定
    rclcpp::CallbackGroup::SharedPtr cb_group =
      create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    agnocast::SubscriptionOptions agnocast_sub_options;
    agnocast_sub_options.callback_group = cb_group;

    sub_cb_ = agnocast::create_subscription<agnocast_sample_interfaces::msg::DynamicSizeArray>(
      this, "/verify_topic", rclcpp::QoS(rclcpp::KeepLast(5)),
      std::bind(&TracepointVerifierRclcpp::subscription_callback, this, _1), agnocast_sub_options);

    // agnocast_subscription_init (polling型)
    sub_poll_ = agnocast::create_subscription<agnocast_sample_interfaces::msg::DynamicSizeArray>(
      this, "/verify_topic", 5);

    // rclcpp 標準タイマー
    timer_ =
      this->create_wall_timer(200ms, std::bind(&TracepointVerifierRclcpp::timer_callback, this));

    RCLCPP_INFO(
      get_logger(), "All tracepoints initialized. Publishing %d messages...", MAX_PUBLISH);
  }

  bool is_done() const
  {
    return publish_count_.load() >= MAX_PUBLISH && cb_recv_count_.load() >= MAX_PUBLISH;
  }

  void print_summary() const
  {
    RCLCPP_INFO(get_logger(), "=== Summary ===");
    RCLCPP_INFO(get_logger(), "  published:     %d", publish_count_.load());
    RCLCPP_INFO(get_logger(), "  callback recv: %d", cb_recv_count_.load());
    RCLCPP_INFO(get_logger(), "  polling recv:  %d", poll_recv_count_.load());
    RCLCPP_INFO(get_logger(), "================");
  }
};

int main(int argc, char ** argv)
{
  // rclcpp::init を使用 (agnocast_init トレースポイントは発火しない)
  rclcpp::init(argc, argv);

  // agnocast_construct_executor
  // rclcpp::Node 用の MultiThreadedAgnocastExecutor を使用
  agnocast::MultiThreadedAgnocastExecutor executor;

  auto node = std::make_shared<TracepointVerifierRclcpp>();
  executor.add_node(node);

  // エグゼキュータの処理（spin）を別スレッドで開始
  std::thread spinner_thread([&executor]() { executor.spin(); });

  // 終了判定ループ
  auto start = std::chrono::steady_clock::now();
  constexpr auto TIMEOUT = 20s;

  while (std::chrono::steady_clock::now() - start < TIMEOUT) {
    std::this_thread::sleep_for(50ms);

    if (node->is_done()) {
      // callback が全て届くまで少し待つ
      std::this_thread::sleep_for(500ms);
      break;
    }
  }

  executor.cancel();
  spinner_thread.join();

  node->print_summary();
  RCLCPP_INFO(node->get_logger(), "Tracepoint verification complete.");

  rclcpp::shutdown();
  return 0;
}
