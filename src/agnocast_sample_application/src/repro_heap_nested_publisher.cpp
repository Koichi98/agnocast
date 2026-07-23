// Repro: publish a message whose nested sequence (`data`) is backed by the process
// heap instead of Agnocast shared memory, by building the message OUTSIDE the
// borrow_loaned_message()..publish() window and then move-assigning it into the
// loaned message.
//
// Because the heaphook only routes allocations to shared memory while a loaned
// message is borrowed (agnocast_get_borrowed_publisher_num() > 0), the `built.data`
// vector below is allocated on the ordinary heap. The move-assign then leaves the
// shared-memory message struct pointing at that heap buffer.
//
// A cross-process consumer that walks `data` (e.g. the A2R bridge serializing the
// message in the performance bridge daemon) dereferences a pointer that is only
// valid in this publisher process -> SIGSEGV.
//
// Correct counterpart: no_rclcpp_talker (builds in place after borrow).

#include "agnocast/agnocast.hpp"
#include "agnocast_sample_interfaces/msg/dynamic_size_array.hpp"

using namespace std::chrono_literals;
const long long MESSAGE_SIZE = 1000ll * 1024;

class ReproHeapPublisher : public agnocast::Node
{
  int64_t count_;
  agnocast::Publisher<agnocast_sample_interfaces::msg::DynamicSizeArray>::SharedPtr pub_;
  agnocast::TimerBase::SharedPtr timer_;

  void timer_callback()
  {
    // (1) Build the message BEFORE borrowing -> borrowed_publisher_num == 0 ->
    //     heaphook routes these allocations to the ordinary heap (NOT shared memory).
    agnocast_sample_interfaces::msg::DynamicSizeArray built;
    built.id = count_;
    built.data.reserve(MESSAGE_SIZE / sizeof(uint64_t));
    for (size_t i = 0; i < MESSAGE_SIZE / sizeof(uint64_t); i++) {
      built.data.push_back(i + count_);
    }

    // (2) Borrow the loaned (shared-memory) message and move-assign the heap-built
    //     one into it. The move only transfers the vector's begin/end/cap pointers,
    //     so `message->data` now points at `built`'s heap buffer (process-private).
    auto message = pub_->borrow_loaned_message();
    *message = std::move(built);

    // (3) Publish. The shared-memory struct is valid cross-process, but its `data`
    //     pointer references this process's heap -> other processes crash on access.
    pub_->publish(std::move(message));
    RCLCPP_INFO(get_logger(), "publish (heap-backed data) message: id=%ld", count_++);
  }

public:
  explicit ReproHeapPublisher() : Node("repro_heap_publisher")
  {
    count_ = 0;
    pub_ =
      this->create_publisher<agnocast_sample_interfaces::msg::DynamicSizeArray>("/my_topic", 1);
    timer_ = agnocast::create_timer(
      this, std::make_shared<rclcpp::Clock>(RCL_STEADY_TIME), rclcpp::Duration(100ms),
      std::bind(&ReproHeapPublisher::timer_callback, this));
  }
};

int main(int argc, char ** argv)
{
  agnocast::init(argc, argv);
  agnocast::AgnocastOnlySingleThreadedExecutor executor;
  auto node = std::make_shared<ReproHeapPublisher>();
  executor.add_node(node);
  executor.spin();
  return 0;
}
