#pragma once

#include "rclcpp/callback_group.hpp"

#include <functional>
#include <memory>

namespace agnocast
{

struct AgnocastExecutable
{
  std::shared_ptr<std::function<void()>> callable;
  rclcpp::CallbackGroup::SharedPtr callback_group{nullptr};
};

}  // namespace agnocast
