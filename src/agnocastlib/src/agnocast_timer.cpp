#include "agnocast/agnocast_timer.hpp"

#include "agnocast/agnocast_timer_info.hpp"
#include "rclcpp/logging.hpp"

#include <typeinfo>

namespace agnocast
{

TimerBase::~TimerBase()
{
  unregister_timer_info(timer_id_);
}

void TimerBase::reset()
{
  auto timer_info = timer_info_.lock();
  if (!timer_info) {
    return;
  }
  timer_info->reset();
  canceled_.store(false);

  trigger_on_reset_callback(1);
}

std::chrono::nanoseconds TimerBase::time_until_trigger()
{
  if (canceled_.load()) {
    return std::chrono::nanoseconds::max();
  }

  auto timer_info = timer_info_.lock();
  if (!timer_info) {
    return std::chrono::nanoseconds::max();
  }
  const int64_t now_ns = timer_info->clock->now().nanoseconds();
  const int64_t next_ns = timer_info->next_call_time_ns.load();
  return std::chrono::nanoseconds(next_ns - now_ns);
}

void TimerBase::set_period(std::chrono::nanoseconds period)
{
  auto timer_info = timer_info_.lock();
  if (!timer_info) {
    throw std::runtime_error("set_period called on an invalidated timer (timer_info expired)");
  }
  timer_info->set_period(period);
}

void TimerBase::set_on_reset_callback(std::function<void(size_t)> callback)
{
  if (!callback) {
    throw std::invalid_argument("The callback passed to set_on_reset_callback is not callable.");
  }

  auto new_callback = [callback = std::move(callback), this](size_t reset_calls) {
    try {
      callback(reset_calls);
    } catch (const std::exception & exception) {
      RCLCPP_ERROR_STREAM(
        rclcpp::get_logger("timer" + std::to_string(timer_id_)),
        "agnocast::TimerBase@"
          << this << " caught " << typeid(exception).name()
          << " exception in user-provided callback for the 'on reset' callback: "
          << exception.what());
    } catch (...) {
      RCLCPP_ERROR_STREAM(
        rclcpp::get_logger("timer" + std::to_string(timer_id_)),
        "agnocast::TimerBase@" << this << " caught unhandled exception in user-provided callback "
                               << "for the 'on reset' callback");
    }
  };

  std::lock_guard<std::recursive_mutex> lock(callback_mutex_);
  on_reset_callback_ = std::move(new_callback);
  if (reset_counter) {
    trigger_on_reset_callback(reset_counter);
    reset_counter = 0;
  }
}

void TimerBase::clear_on_reset_callback()
{
  std::lock_guard<std::recursive_mutex> lock(callback_mutex_);
  on_reset_callback_ = nullptr;
}

void TimerBase::trigger_on_reset_callback(size_t reset_count)
{
  std::function<void(size_t)> callback_to_run = nullptr;

  {
    std::lock_guard<std::recursive_mutex> lock(callback_mutex_);
    callback_to_run = on_reset_callback_;
  }

  if (callback_to_run) {
    callback_to_run(reset_count);
  } else {
    reset_counter++;
  }
}

}  // namespace agnocast
