#include "agnocast/agnocast_epoll_update_dispatcher.hpp"

#include "agnocast/agnocast_utils.hpp"

#include <sys/eventfd.h>
#include <unistd.h>

#include <atomic>

namespace agnocast
{

EventFdWrapper::EventFdWrapper() : fd_{eventfd(0, EFD_NONBLOCK | EFD_CLOEXEC)}
{
  if (fd_ == -1) {
    RCLCPP_ERROR(logger, "Failed to allocate eventfd: %s", strerror(errno));
    exit(EXIT_FAILURE);
  }
}

EventFdWrapper::~EventFdWrapper()
{
  if (fd_ != -1) {
    close(fd_);
  }
}

EventFdWrapper::EventFdWrapper(EventFdWrapper && other) noexcept : fd_(other.fd_)
{
  other.fd_ = -1;
}

EventFdWrapper & EventFdWrapper::operator=(EventFdWrapper && other) noexcept
{
  if (this != &other) {
    if (fd_ != -1) {
      ::close(fd_);
    }
    fd_ = other.fd_;
    other.fd_ = -1;
  }
  return *this;
}

void EventFdWrapper::notify() const
{
  uint64_t val = 1;
  write(fd_, &val, sizeof(uint64_t));
}

void EventFdWrapper::clear() const
{
  uint64_t val = 0;
  read(fd_, &val, sizeof(uint64_t));
}

EpollUpdateTracker EpollUpdateDispatcher::create_tracker()
{
  int new_id = next_tracker_id_.fetch_add(1, std::memory_order_relaxed);

  auto context = std::make_shared<TrackerContext>();

  {
    std::lock_guard<std::mutex> lock(mutex_);
    trackers_.emplace(new_id, context);
  }

  return {new_id, context};
}

void EpollUpdateDispatcher::notify_all()
{
  std::vector<std::shared_ptr<TrackerContext>> active_contexts;

  {
    std::lock_guard<std::mutex> lock(mutex_);
    active_contexts.reserve(trackers_.size());
    for (const auto & [id, context] : trackers_) {
      active_contexts.push_back(context);
    }
  }

  for (const auto & context : active_contexts) {
    context->need_update.store(true, std::memory_order_release);
    context->event_fd.notify();
  }
}

void EpollUpdateDispatcher::unregister(int tracker_id)
{
  std::lock_guard<std::mutex> lock(mutex_);
  trackers_.erase(tracker_id);
}

EpollUpdateTracker::EpollUpdateTracker(EpollUpdateTracker && other) noexcept
: id_(other.id_), context_(std::move(other.context_))
{
  other.id_ = 0;
}

EpollUpdateTracker & EpollUpdateTracker::operator=(EpollUpdateTracker && other) noexcept
{
  if (this != &other) {
    if (id_ != 0) {
      EpollUpdateDispatcher::get_instance().unregister(id_);
    }
    id_ = other.id_;
    context_ = std::move(other.context_);

    other.id_ = 0;
  }
  return *this;
}

EpollUpdateTracker::~EpollUpdateTracker()
{
  if (id_ != 0) {
    EpollUpdateDispatcher::get_instance().unregister(id_);
  }
}

bool EpollUpdateTracker::need_update() const
{
  if (!context_) {
    return false;
  }
  return context_->need_update.exchange(false, std::memory_order_acquire);
}

EpollUpdateEventSource::EpollUpdateEventSource(std::shared_ptr<TrackerContext> tracker)
: tracker_(std::move(tracker))
{
  if (!tracker_) {
    RCLCPP_ERROR(logger, "Invalid tracker context");
    exit(EXIT_FAILURE);
  }
}

bool EpollUpdateEventSource::handle(EpollEventLocalID /*event_local_id*/)
{
  tracker_->event_fd.clear();
  return false;
}

}  // namespace agnocast
