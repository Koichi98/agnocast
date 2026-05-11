#include "agnocast/agnocast_timer.hpp"

#include "agnocast/agnocast_timer_info.hpp"

#include <sys/timerfd.h>
#include <unistd.h>

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
  const int64_t now_ns = timer_info->clock->now().nanoseconds();
  timer_info->next_call_time_ns.store(now_ns + period_.count(), std::memory_order_relaxed);

  std::shared_lock fd_lock(timer_info->fd_mutex);

  if (timer_info->timer_fd != -1) {
    struct itimerspec spec = {};
    const auto period_count = period_.count();
    if (period_count == 0) {
      // Workaround: timerfd_settime() disarms the timer when both it_value and it_interval
      // are zero. Use 1ns to keep the timer armed and achieve "always ready" semantics.
      spec.it_interval.tv_sec = 0;
      spec.it_interval.tv_nsec = 1;
    } else {
      spec.it_interval.tv_sec = period_count / NANOSECONDS_PER_SECOND;
      spec.it_interval.tv_nsec = period_count % NANOSECONDS_PER_SECOND;
    }
    spec.it_value = spec.it_interval;

    if (timerfd_settime(timer_info->timer_fd, 0, &spec, nullptr) == -1) {
      const int saved_errno = errno;
      throw std::runtime_error(
        "timerfd_settime failed for timer_id=" + std::to_string(timer_info->timer_id) +
        ", period=" + std::to_string(period_count) + "ns: " + +std::strerror(saved_errno));
    }
  }
  canceled_.store(false);

  trigger_on_reset_callback();
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
}  // namespace agnocast
