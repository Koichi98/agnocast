#pragma once

#include "agnocast/agnocast_epoll.hpp"
#include "sys/epoll.h"

#include <atomic>
#include <memory>
#include <mutex>
#include <unordered_map>

namespace agnocast
{

class EventFdWrapper
{
public:
  EventFdWrapper();

  ~EventFdWrapper();

  EventFdWrapper(const EventFdWrapper &) = delete;
  EventFdWrapper & operator=(const EventFdWrapper &) = delete;

  EventFdWrapper(EventFdWrapper && other) noexcept;
  EventFdWrapper & operator=(EventFdWrapper && other) noexcept;

  void notify() const;
  void clear() const;

  [[nodiscard]] int fd() const { return fd_; }

private:
  int fd_{-1};
};

class EpollUpdateTracker;

struct TrackerContext
{
  std::atomic<bool> need_update{true};
  EventFdWrapper event_fd;
};

class EpollUpdateDispatcher
{
public:
  static EpollUpdateDispatcher & get_instance()
  {
    static EpollUpdateDispatcher instance;
    return instance;
  }

  void notify_all();

  EpollUpdateTracker create_tracker();

private:
  EpollUpdateDispatcher() = default;
  friend class EpollUpdateTracker;

  void unregister(int tracker_id);

  std::atomic<int> next_tracker_id_{1};

  std::mutex mutex_;
  std::unordered_map<int, std::shared_ptr<TrackerContext>> trackers_;
};

class EpollUpdateTracker
{
public:
  EpollUpdateTracker(const EpollUpdateTracker &) = delete;
  EpollUpdateTracker & operator=(const EpollUpdateTracker &) = delete;

  EpollUpdateTracker(EpollUpdateTracker && other) noexcept;
  EpollUpdateTracker & operator=(EpollUpdateTracker && other) noexcept;

  ~EpollUpdateTracker();

  [[nodiscard]] bool need_update() const;

  [[nodiscard]] int get_event_fd() const { return context_ ? context_->event_fd.fd() : -1; }

  [[nodiscard]] std::shared_ptr<TrackerContext> get_tracker_context() const { return context_; }

private:
  friend class EpollUpdateDispatcher;

  EpollUpdateTracker(int tracker_id, std::shared_ptr<TrackerContext> context)
  : id_(tracker_id), context_(std::move(context))
  {
  }

  int id_;
  std::shared_ptr<TrackerContext> context_;
};

class EpollUpdateEventSource : public EpollEventSource
{
  std::shared_ptr<TrackerContext> tracker_;

public:
  explicit EpollUpdateEventSource(std::shared_ptr<TrackerContext> tracker);

  [[nodiscard]] EpollEventType get_type() const override { return EpollEventType::EpollUpdate; }

  void prepare_epoll(Epoll & epoll, const CallbackGroupValidator & validate_callback_group) override
  {
    (void)epoll;
    (void)validate_callback_group;
  }

  bool handle(EpollEventLocalID /*event_local_id*/) override;
};

}  // namespace agnocast
