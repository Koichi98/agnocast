#include "agnocast/agnocast_timer.hpp"

#include "agnocast/agnocast_timer_info.hpp"

namespace agnocast
{

TimerBase::~TimerBase()
{
  unregister_timer_info(timer_id_);
}

void TimerBase::cancel()
{
  cancel_timer(timer_id_);
}

bool TimerBase::is_canceled() const
{
  return is_timer_canceled(timer_id_);
}

void TimerBase::reset()
{
  reset_timer(timer_id_);
}

void TimerBase::set_period(std::chrono::nanoseconds period)
{
  set_timer_period(timer_id_, period);
  period_ = period;
}

std::chrono::nanoseconds TimerBase::time_until_trigger() const
{
  return agnocast::time_until_trigger(timer_id_);
}

}  // namespace agnocast
