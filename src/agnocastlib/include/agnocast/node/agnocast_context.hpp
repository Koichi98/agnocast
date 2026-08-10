#pragma once

#include "agnocast/agnocast_public_api.hpp"
#include "agnocast/node/agnocast_arguments.hpp"

#include <mutex>
#include <string>

namespace agnocast
{

class Context
{
  struct CommandLineParams
  {
    std::string node_name;
  };

public:
  CommandLineParams command_line_params;

  void init(int argc, char const * const * argv);

  // Mark the context as initialized without parsing command-line arguments and
  // without touching the process-global rcl logging configuration.
  //
  // Used when an agnocast::Node lives in a process that never calls agnocast::init(),
  // e.g. a component container whose main() only calls rclcpp::init(). There, rclcpp
  // owns the logging configuration and the command line, so agnocast must not claim
  // either; it only needs agnocast::ok() to report the context as alive so that the
  // Agnocast-only executors it spawns internally (clock thread, tf listener) keep
  // spinning. get_parsed_arguments() keeps returning nullptr in this mode, which
  // callers already handle.
  //
  // Does nothing once shutdown() has run: a node created during teardown must not
  // revive the context. Only an explicit init() may do that.
  //
  // @return whether the context is initialized after the call.
  bool init_without_arguments();

  void shutdown();
  bool is_initialized() const { return initialized_; }

  const rcl_arguments_t * get_parsed_arguments() const
  {
    return parsed_arguments_.is_valid() ? parsed_arguments_.get() : nullptr;
  }

private:
  bool initialized_ = false;
  // Set by shutdown(), cleared by init(). Distinguishes "not initialized yet" from
  // "already torn down", which init_without_arguments() must not undo.
  bool shutdown_called_ = false;
  ParsedArguments parsed_arguments_;
};

extern Context g_context;
extern std::mutex g_context_mtx;

/// @brief Initialize Agnocast. Must be called once before creating any agnocast::Node.
/// This is the counterpart of rclcpp::init() for agnocast::Node.
/// @param argc Number of command-line arguments.
/// @param argv Command-line argument array.
AGNOCAST_PUBLIC
void init(int argc, char const * const * argv);

/// @brief Shut down Agnocast. Should be called before process exit in agnocast::Node processes.
/// This is the counterpart of rclcpp::shutdown() for agnocast::Node.
AGNOCAST_PUBLIC
void shutdown();

/// @brief Check whether Agnocast context is valid (initialized and not shutdown).
/// This is the counterpart of rclcpp::ok() for agnocast::Node.
AGNOCAST_PUBLIC
bool ok();

}  // namespace agnocast
